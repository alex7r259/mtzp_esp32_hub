#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

/* ================= HARDWARE ================= */
#define RS485_RX 16
#define RS485_TX 17
#define RS485_DE 4
#define LED_BUILTIN 2

#define MTZP_ADDR 0x12  // По умолчанию в документации указан 0x12! (диапазон 0x01-0xFF)
#define UART_BAUD 115200  // Проверьте в МТЗП! Обычно 9600, 19200, 38400, 115200

/* ================= WIFI ================= */
const char* AP_SSID = "MTZP";
const char* AP_PASS = "12345678";

/* ================= SLIP ================= */
#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

/* ================= TIMEOUTS ================= */
#define SLIP_TIMEOUT_MS 500      // Общий таймаут ответа (достаточно 500мс)
#define BYTE_TIMEOUT_MS 100      // Между байтами (увеличено для длинных кабелей)
#define DE_SWITCH_DELAY_US 100   // Задержка переключения направления RS485

/* ================= ОТЛАДКА ================= */
#define DEBUG_SERIAL Serial
#define DEBUG_ENABLED true

void debugHex(const char* label, const uint8_t* data, int len) {
  if (!DEBUG_ENABLED) return;
  DEBUG_SERIAL.print(label);
  for (int i = 0; i < len; i++) {
    DEBUG_SERIAL.printf(" %02X", data[i]);
  }
  DEBUG_SERIAL.println();
}

void debugLog(const char* message) {
  if (!DEBUG_ENABLED) return;
  DEBUG_SERIAL.println(message);
}

void debugLogf(const char* format, ...) {
  if (!DEBUG_ENABLED) return;
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  DEBUG_SERIAL.println(buffer);
}

/* ================= CRC16 (Точная реализация из документации МТЗП) ================= */
const uint16_t TableCRC_1021[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

// Точная реализация CRC как в документации МТЗП (стр. 22)
uint16_t crc16_mtzp(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < len; i++) {
    uint8_t index = (crc >> 8) ^ data[i];
    crc = (crc << 8) ^ TableCRC_1021[index];
  }

  return crc;
}

/* ================= MTZP ERROR CODES ================= */
const char* getMtzpError(uint8_t errorCode) {
  switch(errorCode) {
    case 0x01: return "Неверная команда";
    case 0x02: return "Недоступный адрес регистра";
    case 0x03: return "Недопустимое значение";
    default: return "Неизвестная ошибка";
  }
}

/* ================= SLIP ================= */
void slipSend(const uint8_t* data, uint16_t len) {
  // Переключаем RS485 на передачу
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(DE_SWITCH_DELAY_US);

  // Отправляем начало пакета
  Serial2.write(SLIP_END);

  // Отправляем данные с экранированием
  for (uint16_t i = 0; i < len; i++) {
    if (data[i] == SLIP_END) {
      Serial2.write(SLIP_ESC);
      Serial2.write(SLIP_ESC_END);
    } else if (data[i] == SLIP_ESC) {
      Serial2.write(SLIP_ESC);
      Serial2.write(SLIP_ESC_ESC);
    } else {
      Serial2.write(data[i]);
    }
  }

  // Отправляем конец пакета
  Serial2.write(SLIP_END);
  Serial2.flush();

  // Переключаем RS485 на приём
  delayMicroseconds(DE_SWITCH_DELAY_US);
  digitalWrite(RS485_DE, LOW);
}

int slipRecv(uint8_t* buffer, uint16_t maxLen, uint32_t timeoutMs) {
  uint32_t start = millis();
  uint32_t lastByteTime = start;
  bool escape = false;
  uint16_t idx = 0;
  bool inPacket = false;

  while (millis() - start < timeoutMs) {
    if (Serial2.available()) {
      lastByteTime = millis();
      uint8_t ch = Serial2.read();

      // Ждём начала пакета
      if (!inPacket && ch == SLIP_END) {
        inPacket = true;
        continue;
      }

      // Пропускаем байты до начала пакета
      if (!inPacket) continue;

      // Конец пакета
      if (ch == SLIP_END) {
        if (idx > 0) {  // Игнорируем пустые пакеты
          debugLogf("SLIP: Получен пакет %d байт", idx);
          return idx;
        }
        continue;
      }

      // Обработка escape-последовательностей
      if (ch == SLIP_ESC) {
        escape = true;
        continue;
      }

      if (escape) {
        if (ch == SLIP_ESC_END) ch = SLIP_END;
        else if (ch == SLIP_ESC_ESC) ch = SLIP_ESC;
        escape = false;
      }

      // Сохраняем байт в буфер
      if (idx < maxLen) {
        buffer[idx++] = ch;
      } else {
        debugLog("ОШИБКА: Переполнение буфера SLIP");
        debugHex("Частичный пакет:", buffer, idx);
        return -2;
      }
    }

    // Проверяем таймаут между байтами
    if (inPacket && idx > 0 && (millis() - lastByteTime > BYTE_TIMEOUT_MS)) {
      debugLogf("ОШИБКА: Таймаут между байтами (получено %d байт)", idx);
      debugHex("Частичный пакет:", buffer, idx);
      return -3;
    }

    delay(1);
  }

  if (inPacket) {
    debugLogf("ОШИБКА: Общий таймаут (получено %d байт)", idx);
    debugHex("Частичный пакет:", buffer, idx);
  } else {
    debugLog("ОШИБКА: Общий таймаут (пакет не начался)");
  }
  return -1;
}

/* ================= MTZP Протокол (SLIP) ================= */

bool mtzpRead(uint16_t reg, uint16_t& val) {
  // Формируем пакет для чтения параметра (команда 0x01)
  uint8_t frame[6] = {
    MTZP_ADDR,           // Адрес устройства
    0x01,                // Команда чтения параметра (бит 7 = 0 - запрос от master)
    (uint8_t)(reg >> 8), // Старший байт номера параметра
    (uint8_t)(reg),      // Младший байт номера параметра
  };

  // Вычисляем CRC для первых 4 байт
  uint16_t crc = crc16_mtzp(frame, 4);
  frame[4] = crc >> 8;    // Старший байт CRC
  frame[5] = crc & 0xFF;  // Младший байт CRC

  debugHex("TX READ:", frame, 6);

  // Отправляем
  slipSend(frame, 6);

  // Принимаем ответ (ожидаем 8 байт: Адрес + Команда + Номер + Значение + CRC)
  uint8_t reply[64];  // Увеличенный буфер для надёжности
  int len = slipRecv(reply, sizeof(reply), SLIP_TIMEOUT_MS);

  if (len < 8) {
    debugLogf("ОШИБКА: Короткий ответ %d байт (ожидалось 8)", len);
    if (len > 0) {
      debugHex("Полученные данные:", reply, len);
    }
    return false;
  }

  debugHex("RX READ:", reply, len);

  // Проверяем адрес
  if (reply[0] != MTZP_ADDR) {
    debugLogf("ОШИБКА: Неверный адрес в ответе: ожидали 0x%02X, получили 0x%02X",
              MTZP_ADDR, reply[0]);
    return false;
  }

  // Проверяем, что это ответ от slave (бит 7 = 1)
  if ((reply[1] & 0x80) == 0) {
    debugLog("ОШИБКА: Получен запрос вместо ответа (бит 7 = 0)");
    return false;
  }

  // Проверяем бит ошибки (бит 6 = 1)
  if (reply[1] & 0x40) {
    const char* errMsg = getMtzpError(reply[2]);
    debugLogf("ОШИБКА МТЗП: %s (код 0x%02X)", errMsg, reply[2]);
    return false;
  }

  // ИСПРАВЛЕНО: Проверяем код команды (биты 0-3, маска 0x0F)
  uint8_t responseCmd = reply[1] & 0x0F;
  if (responseCmd != 0x01) {
    debugLogf("ОШИБКА: Неверный код команды в ответе: ожидали 0x01, получили 0x%02X",
              responseCmd);
    return false;
  }

  // Проверяем CRC ответа
  uint16_t recvCrc = (reply[len-2] << 8) | reply[len-1];
  uint16_t calcCrc = crc16_mtzp(reply, len-2);

  if (recvCrc != calcCrc) {
    debugLogf("ОШИБКА CRC: получено 0x%04X, вычислено 0x%04X", recvCrc, calcCrc);
    return false;
  }

  // Проверяем номер параметра в ответе
  uint16_t respReg = (reply[2] << 8) | reply[3];
  if (respReg != reg) {
    debugLogf("ОШИБКА: Неверный номер регистра в ответе: ожидали %d, получили %d",
              reg, respReg);
    return false;
  }

  // Извлекаем значение (байты 4 и 5)
  val = (reply[4] << 8) | reply[5];
  debugLogf("Успешное чтение: reg=%d, val=%d (0x%04X)", reg, val, val);

  return true;
}

bool mtzpWrite(uint16_t reg, uint16_t val) {
  // Команда записи = 0x03 (согласно документации стр. 23)
  uint8_t frame[8] = {
    MTZP_ADDR,           // Адрес устройства
    0x03,                // Команда записи параметра
    (uint8_t)(reg >> 8), // Старший байт номера параметра
    (uint8_t)(reg),      // Младший байт номера параметра
    (uint8_t)(val >> 8), // Старший байт значения
    (uint8_t)(val),      // Младший байт значения
  };

  // Вычисляем CRC для первых 6 байт
  uint16_t crc = crc16_mtzp(frame, 6);
  frame[6] = crc >> 8;    // Старший байт CRC
  frame[7] = crc & 0xFF;  // Младший байт CRC

  debugHex("TX WRITE:", frame, 8);

  // Отправляем
  slipSend(frame, 8);

  // НОВОЕ: Проверка широковещательного адреса (стр. 21)
  if (MTZP_ADDR == 0x00) {
    debugLog("Широковещательная команда - ответа не будет");
    return true;  // Считаем успешной
  }

  // Принимаем ответ (ожидаем 6 байт: Адрес + Команда + Номер + CRC)
  uint8_t reply[64];  // Увеличенный буфер
  int len = slipRecv(reply, sizeof(reply), SLIP_TIMEOUT_MS);

  if (len < 6) {
    debugLogf("ОШИБКА: Короткий ответ %d байт (ожидалось 6)", len);
    if (len > 0) {
      debugHex("Полученные данные:", reply, len);
    }
    return false;
  }

  debugHex("RX WRITE:", reply, len);

  // Проверяем адрес
  if (reply[0] != MTZP_ADDR) {
    debugLogf("ОШИБКА: Неверный адрес в ответе: ожидали 0x%02X, получили 0x%02X",
              MTZP_ADDR, reply[0]);
    return false;
  }

  // Проверяем, что это ответ от slave (бит 7 = 1)
  if ((reply[1] & 0x80) == 0) {
    debugLog("ОШИБКА: Получен запрос вместо ответа (бит 7 = 0)");
    return false;
  }

  // Проверяем бит ошибки (бит 6 = 1)
  if (reply[1] & 0x40) {
    const char* errMsg = getMtzpError(reply[2]);
    debugLogf("ОШИБКА МТЗП: %s (код 0x%02X)", errMsg, reply[2]);
    return false;
  }

  // ИСПРАВЛЕНО: Проверяем код команды (биты 0-3, маска 0x0F)
  uint8_t responseCmd = reply[1] & 0x0F;
  if (responseCmd != 0x03) {
    debugLogf("ОШИБКА: Неверный код команды в ответе: ожидали 0x03, получили 0x%02X",
              responseCmd);
    return false;
  }

  // Проверяем CRC ответа
  uint16_t recvCrc = (reply[len-2] << 8) | reply[len-1];
  uint16_t calcCrc = crc16_mtzp(reply, len-2);

  if (recvCrc != calcCrc) {
    debugLogf("ОШИБКА CRC: получено 0x%04X, вычислено 0x%04X", recvCrc, calcCrc);
    return false;
  }

  // Проверяем номер параметра в ответе
  uint16_t respReg = (reply[2] << 8) | reply[3];
  if (respReg != reg) {
    debugLogf("ОШИБКА: Неверный номер регистра в ответе: ожидали %d, получили %d",
              reg, respReg);
    return false;
  }

  debugLogf("Успешная запись: reg=%d, val=%d (0x%04X)", reg, val, val);

  return true;
}

/* ================= НОВАЯ ФУНКЦИЯ: Чтение группы параметров ================= */
bool mtzpReadMultiple(uint16_t* regs, uint8_t count, uint16_t* values) {
  if (count < 1 || count > 16) {
    debugLogf("ОШИБКА: Неверное количество параметров: %d (допустимо 1-16)", count);
    return false;
  }

  // Формируем пакет: Адрес + Команда + Количество + Номера*N + CRC
  uint8_t frameLen = 3 + count * 2 + 2;
  uint8_t frame[frameLen];

  frame[0] = MTZP_ADDR;
  frame[1] = 0x02;  // Команда чтения группы параметров
  frame[2] = count;

  // Добавляем номера параметров
  for (uint8_t i = 0; i < count; i++) {
    frame[3 + i*2] = regs[i] >> 8;
    frame[4 + i*2] = regs[i] & 0xFF;
  }

  // Вычисляем CRC
  uint16_t crc = crc16_mtzp(frame, frameLen - 2);
  frame[frameLen - 2] = crc >> 8;
  frame[frameLen - 1] = crc & 0xFF;

  debugHex("TX READ MULTIPLE:", frame, frameLen);

  // Отправляем
  slipSend(frame, frameLen);

  // Принимаем ответ: Адрес + Команда + Количество + (Номер+Данные)*N + CRC
  // Максимум: 1 + 1 + 1 + (2+2)*16 + 2 = 69 байт
  uint8_t reply[128];
  int len = slipRecv(reply, sizeof(reply), SLIP_TIMEOUT_MS);

  int expectedLen = 3 + count * 4 + 2;  // Адрес + Команда + Количество + Данные + CRC
  if (len < expectedLen) {
    debugLogf("ОШИБКА: Короткий ответ %d байт (ожидалось %d)", len, expectedLen);
    if (len > 0) {
      debugHex("Полученные данные:", reply, len);
    }
    return false;
  }

  debugHex("RX READ MULTIPLE:", reply, len);

  // Проверяем адрес
  if (reply[0] != MTZP_ADDR) {
    debugLogf("ОШИБКА: Неверный адрес в ответе: ожидали 0x%02X, получили 0x%02X",
              MTZP_ADDR, reply[0]);
    return false;
  }

  // Проверяем ответ от slave
  if ((reply[1] & 0x80) == 0) {
    debugLog("ОШИБКА: Получен запрос вместо ответа");
    return false;
  }

  // Проверяем бит ошибки
  if (reply[1] & 0x40) {
    const char* errMsg = getMtzpError(reply[2]);
    debugLogf("ОШИБКА МТЗП: %s (код 0x%02X)", errMsg, reply[2]);
    return false;
  }

  // Проверяем код команды
  uint8_t responseCmd = reply[1] & 0x0F;
  if (responseCmd != 0x02) {
    debugLogf("ОШИБКА: Неверный код команды в ответе: ожидали 0x02, получили 0x%02X",
              responseCmd);
    return false;
  }

  // Проверяем CRC
  uint16_t recvCrc = (reply[len-2] << 8) | reply[len-1];
  uint16_t calcCrc = crc16_mtzp(reply, len-2);

  if (recvCrc != calcCrc) {
    debugLogf("ОШИБКА CRC: получено 0x%04X, вычислено 0x%04X", recvCrc, calcCrc);
    return false;
  }

  // Проверяем количество
  if (reply[2] != count) {
    debugLogf("ОШИБКА: Неверное количество в ответе: ожидали %d, получили %d",
              count, reply[2]);
    return false;
  }

  // Извлекаем значения
  for (uint8_t i = 0; i < count; i++) {
    uint16_t respReg = (reply[3 + i*4] << 8) | reply[4 + i*4];
    values[i] = (reply[5 + i*4] << 8) | reply[6 + i*4];

    // Проверяем соответствие номеров
    if (respReg != regs[i]) {
      debugLogf("ПРЕДУПРЕЖДЕНИЕ: Номер регистра %d не совпадает: ожидали %d, получили %d",
                i, regs[i], respReg);
    }

    debugLogf("  [%d] reg=%d, val=%d (0x%04X)", i, respReg, values[i], values[i]);
  }

  debugLog("Успешное чтение группы параметров");
  return true;
}

WebServer server(80);

/* ================= API ================= */
void handleRead() {
  if (!server.hasArg("reg")) {
    server.send(400, "application/json", "{\"error\":\"Missing 'reg' parameter\"}");
    return;
  }

  uint16_t reg = server.arg("reg").toInt();
  uint16_t val = 0;
  bool ok = mtzpRead(reg, val);

  StaticJsonDocument<256> doc;
  doc["reg"] = reg;
  doc["value"] = val;
  doc["ok"] = ok;
  doc["hex_value"] = String(val, HEX);

  String payload;
  serializeJson(doc, payload);
  server.send(ok ? 200 : 500, "application/json", payload);
}

void handleWrite() {
  if (!server.hasArg("reg") || !server.hasArg("val")) {
    server.send(400, "application/json",
                "{\"error\":\"Missing 'reg' or 'val' parameter\"}");
    return;
  }

  uint16_t reg = server.arg("reg").toInt();
  uint16_t val = server.arg("val").toInt();
  bool ok = mtzpWrite(reg, val);

  StaticJsonDocument<256> doc;
  doc["reg"] = reg;
  doc["value"] = val;
  doc["ok"] = ok;

  String payload;
  serializeJson(doc, payload);
  server.send(ok ? 200 : 500, "application/json", payload);
}

void handleReadMultiple() {
  if (!server.hasArg("regs")) {
    server.send(400, "application/json",
                "{\"error\":\"Missing 'regs' parameter (comma-separated)\"}");
    return;
  }

  // Парсим список регистров
  String regsStr = server.arg("regs");
  uint16_t regs[16];
  uint16_t values[16];
  uint8_t count = 0;

  int startIdx = 0;
  for (int i = 0; i <= regsStr.length(); i++) {
    if (i == regsStr.length() || regsStr[i] == ',') {
      if (count >= 16) break;
      String regStr = regsStr.substring(startIdx, i);
      regs[count++] = regStr.toInt();
      startIdx = i + 1;
    }
  }

  if (count == 0) {
    server.send(400, "application/json",
                "{\"error\":\"No valid register numbers provided\"}");
    return;
  }

  bool ok = mtzpReadMultiple(regs, count, values);

  DynamicJsonDocument doc(1024);
  doc["ok"] = ok;
  doc["count"] = count;

  JsonArray dataArray = doc.createNestedArray("data");
  for (uint8_t i = 0; i < count; i++) {
    JsonObject item = dataArray.createNestedObject();
    item["reg"] = regs[i];
    item["value"] = values[i];
    item["hex_value"] = String(values[i], HEX);
  }

  String payload;
  serializeJson(doc, payload);
  server.send(ok ? 200 : 500, "application/json", payload);
}

void handleTest() {
  // Тестовая функция для проверки связи
  // Читаем регистр 10 (серийный номер) и регистр 11 (дата изготовления)
  uint16_t serialNum = 0;
  uint16_t mfgDate = 0;

  bool ok1 = mtzpRead(10, serialNum);
  bool ok2 = mtzpRead(11, mfgDate);

  StaticJsonDocument<512> doc;
  doc["test_read_serial"] = ok1;
  doc["test_read_date"] = ok2;
  doc["serial_number"] = serialNum;
  doc["manufacturing_date"] = mfgDate;
  doc["message"] = (ok1 && ok2) ? "Связь с МТЗП установлена" : "Ошибка связи с МТЗП";

  // Дополнительная информация
  doc["device_address"] = String("0x") + String(MTZP_ADDR, HEX);
  doc["baud_rate"] = UART_BAUD;

  String payload;
  serializeJson(doc, payload);
  server.send((ok1 || ok2) ? 200 : 500, "application/json", payload);
}

void handleStatus() {
  // Статус системы
  StaticJsonDocument<512> doc;
  doc["uptime_ms"] = millis();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.softAPgetStationNum();
  doc["device_address"] = String("0x") + String(MTZP_ADDR, HEX);
  doc["baud_rate"] = UART_BAUD;
  doc["slip_timeout_ms"] = SLIP_TIMEOUT_MS;
  doc["byte_timeout_ms"] = BYTE_TIMEOUT_MS;

  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

/* ================= SETUP / LOOP ================= */
void setup() {
  DEBUG_SERIAL.begin(115200);
  delay(1000);

  DEBUG_SERIAL.println("\n\n=== MTZP ESP32 Шлюз v2.0 ===");
  DEBUG_SERIAL.println("Исправленная версия с улучшенной обработкой протокола");

  // НОВОЕ: Валидация адреса устройства
  if (MTZP_ADDR == 0x00) {
    DEBUG_SERIAL.println("ВНИМАНИЕ: Адрес 0x00 - широковещательный режим");
    DEBUG_SERIAL.println("Устройство не будет отвечать на запросы!");
  } else if (MTZP_ADDR > 0xFF) {
    DEBUG_SERIAL.printf("ОШИБКА: Неверный адрес МТЗП 0x%02X (допустимо 0x01-0xFF)\n",
                       MTZP_ADDR);
    while(1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  // Инициализация UART для RS485
  Serial2.begin(UART_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);  // Режим приёма по умолчанию

  DEBUG_SERIAL.printf("RS485 инициализирован: %d бод\n", UART_BAUD);
  DEBUG_SERIAL.printf("Адрес МТЗП: 0x%02X\n", MTZP_ADDR);
  DEBUG_SERIAL.printf("Таймауты: общий=%dмс, между байтами=%dмс\n",
                     SLIP_TIMEOUT_MS, BYTE_TIMEOUT_MS);

  // Настройка WiFi точки доступа
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress myIP = WiFi.softAPIP();
  DEBUG_SERIAL.print("AP IP адрес: ");
  DEBUG_SERIAL.println(myIP);

  // Настройка веб-сервера
  server.on("/api/read", HTTP_GET, handleRead);
  server.on("/api/write", HTTP_POST, handleWrite);
  server.on("/api/read_multiple", HTTP_GET, handleReadMultiple);
  server.on("/api/test", HTTP_GET, handleTest);
  server.on("/api/status", HTTP_GET, handleStatus);

  // Статический контент (HTML интерфейс)
  server.on("/", []() {
    String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>MTZP Web Interface v2.0</title>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
          font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
          padding: 20px;
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          min-height: 100vh;
        }
        .container {
          max-width: 1200px;
          margin: 0 auto;
        }
        h1 {
          color: white;
          margin-bottom: 30px;
          text-align: center;
          text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .card {
          background: white;
          border-radius: 10px;
          padding: 25px;
          margin: 15px 0;
          box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        .card h2 {
          color: #333;
          margin-bottom: 20px;
          border-bottom: 2px solid #667eea;
          padding-bottom: 10px;
        }
        input, button, select {
          padding: 12px 20px;
          margin: 8px 5px;
          border: 2px solid #ddd;
          border-radius: 6px;
          font-size: 14px;
          transition: all 0.3s;
        }
        input:focus {
          outline: none;
          border-color: #667eea;
        }
        button {
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          color: white;
          border: none;
          cursor: pointer;
          font-weight: bold;
        }
        button:hover {
          transform: translateY(-2px);
          box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
        }
        button:active {
          transform: translateY(0);
        }
        .success {
          color: #27ae60;
          padding: 15px;
          background: #d5f4e6;
          border-radius: 6px;
          margin-top: 15px;
        }
        .error {
          color: #e74c3c;
          padding: 15px;
          background: #fadbd8;
          border-radius: 6px;
          margin-top: 15px;
        }
        .status-badge {
          display: inline-block;
          padding: 5px 15px;
          border-radius: 20px;
          font-size: 12px;
          font-weight: bold;
          margin-left: 10px;
        }
        .status-online {
          background: #27ae60;
          color: white;
        }
        .status-offline {
          background: #e74c3c;
          color: white;
        }
        .info-grid {
          display: grid;
          grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
          gap: 15px;
          margin-top: 20px;
        }
        .info-item {
          padding: 15px;
          background: #f8f9fa;
          border-radius: 6px;
          border-left: 4px solid #667eea;
        }
        .info-label {
          font-size: 12px;
          color: #666;
          margin-bottom: 5px;
        }
        .info-value {
          font-size: 18px;
          font-weight: bold;
          color: #333;
        }
        code {
          background: #f4f4f4;
          padding: 2px 6px;
          border-radius: 3px;
          font-family: 'Courier New', monospace;
        }
        .examples {
          background: #f8f9fa;
          padding: 15px;
          border-radius: 6px;
          margin-top: 15px;
          font-size: 13px;
        }
        .examples h3 {
          margin-bottom: 10px;
          color: #667eea;
        }
      </style>
    </head>
    <body>
      <div class="container">
        <h1>🔧 MTZP Web Interface v2.0</h1>

        <div class="card">
          <h2>📊 Статус системы <span class="status-badge status-offline" id="statusBadge">Проверка...</span></h2>
          <div class="info-grid" id="statusGrid">
            <div class="info-item">
              <div class="info-label">Адрес устройства</div>
              <div class="info-value" id="deviceAddr">-</div>
            </div>
            <div class="info-item">
              <div class="info-label">Скорость обмена</div>
              <div class="info-value" id="baudRate">-</div>
            </div>
            <div class="info-item">
              <div class="info-label">Время работы</div>
              <div class="info-value" id="uptime">-</div>
            </div>
            <div class="info-item">
              <div class="info-label">Свободная память</div>
              <div class="info-value" id="freeHeap">-</div>
            </div>
          </div>
        </div>

        <div class="card">
          <h2>📖 Чтение регистра</h2>
          <input type="number" id="readReg" placeholder="Номер регистра (0-276)" min="0" max="276" value="10">
          <button onclick="readRegister()">Прочитать</button>
          <div id="readResult"></div>
          <div class="examples">
            <h3>Примеры регистров:</h3>
            • Рег. 10: Серийный номер<br>
            • Рег. 33-35: Напряжения фаз A, B, C<br>
            • Рег. 41-43: Токи фаз A, B, C
          </div>
        </div>

        <div class="card">
          <h2>📝 Запись регистра</h2>
          <input type="number" id="writeReg" placeholder="Номер регистра" min="0" max="276">
          <input type="number" id="writeVal" placeholder="Значение (0-65535)" min="0" max="65535">
          <button onclick="writeRegister()">Записать</button>
          <div id="writeResult"></div>
        </div>

        <div class="card">
          <h2>📚 Чтение группы регистров</h2>
          <input type="text" id="multiRegs" placeholder="Номера через запятую (напр: 10,11,33)" style="width: 100%;">
          <button onclick="readMultiple()">Прочитать группу</button>
          <div id="multiResult"></div>
          <div class="examples">
            <h3>Примеры:</h3>
            • <code>10,11</code> - Серийный номер и дата<br>
            • <code>33,34,35</code> - Напряжения трёх фаз<br>
            • <code>41,42,43</code> - Токи трёх фаз
          </div>
        </div>

        <div class="card">
          <h2>🔍 Тест связи</h2>
          <button onclick="testConnection()">Проверить связь с МТЗП</button>
          <div id="testResult"></div>
        </div>
      </div>

      <script>
        // Обновление статуса при загрузке
        window.onload = function() {
          updateStatus();
          setInterval(updateStatus, 5000); // Обновляем каждые 5 секунд
        };

        function updateStatus() {
          fetch('/api/status')
            .then(r => r.json())
            .then(data => {
              document.getElementById('deviceAddr').textContent = data.device_address;
              document.getElementById('baudRate').textContent = data.baud_rate + ' bps';
              document.getElementById('uptime').textContent = Math.floor(data.uptime_ms / 1000) + ' сек';
              document.getElementById('freeHeap').textContent = Math.floor(data.free_heap / 1024) + ' КБ';

              const badge = document.getElementById('statusBadge');
              badge.textContent = 'Онлайн';
              badge.className = 'status-badge status-online';
            })
            .catch(err => {
              const badge = document.getElementById('statusBadge');
              badge.textContent = 'Офлайн';
              badge.className = 'status-badge status-offline';
            });
        }

        function readRegister() {
          let reg = document.getElementById('readReg').value;
          fetch('/api/read?reg=' + reg)
            .then(r => r.json())
            .then(data => {
              let result = document.getElementById('readResult');
              if (data.ok) {
                result.innerHTML = `<div class="success">
                  ✅ Регистр ${data.reg}: <strong>${data.value}</strong> (0x${data.hex_value})</div>`;
              } else {
                result.innerHTML = `<div class="error">❌ Ошибка чтения регистра ${data.reg}</div>`;
              }
            })
            .catch(err => {
              document.getElementById('readResult').innerHTML =
                `<div class="error">❌ Ошибка сети: ${err.message}</div>`;
            });
        }

        function writeRegister() {
          let reg = document.getElementById('writeReg').value;
          let val = document.getElementById('writeVal').value;
          fetch('/api/write?reg=' + reg + '&val=' + val, {method: 'POST'})
            .then(r => r.json())
            .then(data => {
              let result = document.getElementById('writeResult');
              if (data.ok) {
                result.innerHTML = `<div class="success">
                  ✅ Записано: регистр ${data.reg} = <strong>${data.value}</strong></div>`;
              } else {
                result.innerHTML = `<div class="error">❌ Ошибка записи</div>`;
              }
            })
            .catch(err => {
              document.getElementById('writeResult').innerHTML =
                `<div class="error">❌ Ошибка сети: ${err.message}</div>`;
            });
        }

        function readMultiple() {
          let regs = document.getElementById('multiRegs').value;
          fetch('/api/read_multiple?regs=' + regs)
            .then(r => r.json())
            .then(data => {
              let result = document.getElementById('multiResult');
              if (data.ok) {
                let html = '<div class="success">✅ Успешно прочитано ' + data.count + ' регистров:<br><br>';
                data.data.forEach(item => {
                  html += `• Рег. ${item.reg}: <strong>${item.value}</strong> (0x${item.hex_value})<br>`;
                });
                html += '</div>';
                result.innerHTML = html;
              } else {
                result.innerHTML = `<div class="error">❌ Ошибка чтения группы регистров</div>`;
              }
            })
            .catch(err => {
              document.getElementById('multiResult').innerHTML =
                `<div class="error">❌ Ошибка сети: ${err.message}</div>`;
            });
        }

        function testConnection() {
          let btn = event.target;
          btn.disabled = true;
          btn.textContent = 'Проверка...';

          fetch('/api/test')
            .then(r => r.json())
            .then(data => {
              let result = document.getElementById('testResult');
              if (data.test_read_serial && data.test_read_date) {
                result.innerHTML = `<div class="success">
                  ✅ ${data.message}<br><br>
                  📋 Серийный номер: <strong>${data.serial_number}</strong><br>
                  📅 Дата изготовления: <strong>${data.manufacturing_date}</strong><br>
                  🔌 Адрес: <code>${data.device_address}</code><br>
                  ⚡ Скорость: <code>${data.baud_rate} bps</code>
                </div>`;
              } else {
                result.innerHTML = `<div class="error">
                  ❌ ${data.message}<br>
                  Проверьте подключение и настройки RS485
                </div>`;
              }
              btn.disabled = false;
              btn.textContent = 'Проверить связь с МТЗП';
            })
            .catch(err => {
              document.getElementById('testResult').innerHTML =
                `<div class="error">❌ Ошибка сети: ${err.message}</div>`;
              btn.disabled = false;
              btn.textContent = 'Проверить связь с МТЗП';
            });
        }
      </script>
    </body>
    </html>
    )rawliteral";

    server.send(200, "text/html", html);
  });

  server.begin();
  DEBUG_SERIAL.println("HTTP сервер запущен");
  DEBUG_SERIAL.println("Откройте http://" + myIP.toString() + " в браузере");
  DEBUG_SERIAL.println("\nДоступные API endpoints:");
  DEBUG_SERIAL.println("  GET  /api/read?reg=<номер>");
  DEBUG_SERIAL.println("  POST /api/write?reg=<номер>&val=<значение>");
  DEBUG_SERIAL.println("  GET  /api/read_multiple?regs=<номер1,номер2,...>");
  DEBUG_SERIAL.println("  GET  /api/test");
  DEBUG_SERIAL.println("  GET  /api/status");
  DEBUG_SERIAL.println("\nГотов к работе!\n");
}

void loop() {
  server.handleClient();

  // Опционально: периодический опрос состояния (отключен по умолчанию)
  /*
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll > 10000) { // Каждые 10 секунд
    lastPoll = millis();

    // Пример: чтение текущих токов
    uint16_t currentA = 0, currentB = 0, currentC = 0;
    if (mtzpRead(41, currentA) && mtzpRead(42, currentB) && mtzpRead(43, currentC)) {
      debugLogf("Токи: A=%dА, B=%dА, C=%dА", currentA, currentB, currentC);
    }
  }
  */
}
