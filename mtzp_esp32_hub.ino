#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Preferences.h>

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
#define BYTE_TIMEOUT_MS 10      // Между байтами (увеличено для длинных кабелей)

/* ================= ОТЛАДКА ================= */
#define DEBUG_SERIAL Serial
#define DEBUG_ENABLED true

#define ADC_PIN       34
#define SAMPLES       32

// Коэффициент делителя (220k + 100k) / 100k
const float DIVIDER_RATIO = 3.2f;
const float VREF          = 3.3f;     // можно уточнить при калибровке

Preferences preferences;
uint8_t mtzpAddress = MTZP_ADDR;
uint32_t mtzpBaudRate = UART_BAUD;

const uint32_t allowedBaudRates[] = {9600, 19200, 38400, 57600, 115200};
const size_t allowedBaudRatesCount = sizeof(allowedBaudRates) / sizeof(allowedBaudRates[0]);

float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(ADC_PIN);
    delay(1);
  }
  float adc = sum / (float)SAMPLES;
  float v_adc = (adc / 4095.0f) * VREF;
  return v_adc * DIVIDER_RATIO;
}

// Простая, но достаточно точная кривая для большинства 18650
int voltageToPercent(float v) {
  if (v >= 4.20) return 100;
  if (v <= 3.27) return 0;

  // Основные точки (примерно соответствует большинству качественных 18650)
  if      (v >= 4.06) return map(v, 4.06, 4.20,  85, 100);
  else if (v >= 3.80) return map(v, 3.80, 4.06,  50,  85);
  else if (v >= 3.60) return map(v, 3.60, 3.80,  20,  50);
  else if (v >= 3.42) return map(v, 3.42, 3.60,   5,  20);
  else                return map(v, 3.27, 3.42,   0,   5);
}

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

bool isAllowedBaudRate(uint32_t baudRate) {
  for (size_t i = 0; i < allowedBaudRatesCount; i++) {
    if (allowedBaudRates[i] == baudRate) {
      return true;
    }
  }
  return false;
}

void loadSettings() {
  mtzpAddress = preferences.getUChar("addr", MTZP_ADDR);
  mtzpBaudRate = preferences.getUInt("baud", UART_BAUD);

  if (mtzpAddress > 0xFF) {
    mtzpAddress = MTZP_ADDR;
  }

  if (!isAllowedBaudRate(mtzpBaudRate)) {
    mtzpBaudRate = UART_BAUD;
  }
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
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(20);

  // END = начало сообщения
  Serial2.write(SLIP_END);

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

  Serial2.flush();
  delayMicroseconds(20);
  digitalWrite(RS485_DE, LOW);
}

int slipRecv(uint8_t* buffer, uint16_t maxLen, uint32_t timeoutMs) {
  uint32_t start = millis();
  uint32_t lastByte = 0;
  bool inPacket = false;
  bool escape = false;
  uint16_t idx = 0;

  while (millis() - start < timeoutMs) {
    if (Serial2.available()) {
      uint8_t ch = Serial2.read();
      lastByte = millis();

      if (ch == SLIP_END) {
        if (inPacket && idx > 0) {
          // конец предыдущего сообщения
          return idx;
        }
        // начало нового
        inPacket = true;
        idx = 0;
        escape = false;
        lastByte = millis();   // ← ДОБАВИТЬ
        continue;
      }

      if (!inPacket) continue;

      if (ch == SLIP_ESC) {
        escape = true;
        continue;
      }

      if (escape) {
        if (ch == SLIP_ESC_END) ch = SLIP_END;
        else if (ch == SLIP_ESC_ESC) ch = SLIP_ESC;
        escape = false;
      }

      if (idx < maxLen) buffer[idx++] = ch;
      else return -2;
    }

    // 🔴 КРИТИЧНО: конец кадра по тишине
    if (inPacket && idx > 0 &&
        millis() - lastByte > BYTE_TIMEOUT_MS) {
      return idx;
    }

    delay(1);
  }

  return -1;
}


/* ================= MTZP Протокол (SLIP) ================= */

bool mtzpRead(uint16_t reg, uint16_t& val) {

  while (Serial2.available()) {
    Serial2.read();
  }
  // Формируем пакет для чтения параметра (команда 0x01)
  uint8_t frame[6] = {
    mtzpAddress,         // Адрес устройства
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
  if (reply[0] != mtzpAddress) {
    debugLogf("ОШИБКА: Неверный адрес в ответе: ожидали 0x%02X, получили 0x%02X",
              mtzpAddress, reply[0]);
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

  while (Serial2.available()) {
    Serial2.read();
  }
  // Команда записи = 0x03 (согласно документации стр. 23)
  uint8_t frame[8] = {
    mtzpAddress,         // Адрес устройства
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
  if (mtzpAddress == 0x00) {
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
  if (reply[0] != mtzpAddress) {
    debugLogf("ОШИБКА: Неверный адрес в ответе: ожидали 0x%02X, получили 0x%02X",
              mtzpAddress, reply[0]);
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

  frame[0] = mtzpAddress;
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
  if (reply[0] != mtzpAddress) {
    debugLogf("ОШИБКА: Неверный адрес в ответе: ожидали 0x%02X, получили 0x%02X",
              mtzpAddress, reply[0]);
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

/* ================= НОВАЯ ФУНКЦИЯ: ЧТЕНИЕ УЧАСТКА ПАМЯТИ (команда 0x08) ================= */
bool mtzpReadMemory(uint32_t startAddr, uint16_t byteCount, uint8_t* outBuffer, uint16_t& bytesRead) {
    bytesRead = 0;
    
    if (byteCount == 0 || byteCount > 256) {
        debugLog("Недопустимое количество байт");
        return false;
    }

    uint8_t req[9] = {0};  // можно 9, без лишнего нулевого байта
    req[0] = mtzpAddress;
    req[1] = 0x08;
    req[2] = (uint8_t)byteCount;

    req[3] = (startAddr >> 24) & 0xFF;
    req[4] = (startAddr >> 16) & 0xFF;
    req[5] = (startAddr >>  8) & 0xFF;
    req[6] = (startAddr      ) & 0xFF;

    uint16_t crc = crc16_mtzp(req, 7);
    req[7] = crc >> 8;
    req[8] = crc & 0xFF;

    debugHex("TX 0x08:", req, 9);
    slipSend(req, 9);

    uint8_t resp[512] = {0};
    int len = slipRecv(resp, sizeof(resp), 1500);

    if (len < 9) {   // минимум заголовок + хотя бы 1 байт данных + CRC
        debugLogf("Слишком короткий ответ: %d байт", len);
        return false;
    }

    debugHex("RX 0x08:", resp, len);

    if (resp[0] != mtzpAddress || resp[1] != 0x88) {
        debugLogf("Неверный заголовок: %02X %02X", resp[0], resp[1]);
        return false;
    }

    uint16_t gotCrc = (resp[len-2] << 8) | resp[len-1];
    uint16_t expCrc = crc16_mtzp(resp, len-2);
    if (gotCrc != expCrc) {
        debugLogf("CRC ошибка: rx=%04X calc=%04X", gotCrc, expCrc);
        return false;
    }

    uint16_t reportedCount = resp[2];

    if (reportedCount != byteCount) {
        debugLogf("Несоответствие длины данных: запросили %d, устройство сказало %d",
                  byteCount, reportedCount);
        // можно return false, если строго; или продолжить с reportedCount
    }

    // Основное исправление: копируем столько, сколько реально пришло
    bytesRead = reportedCount;

    // Защита от переполнения буфера вызывающей стороны
    if (len < 7 + bytesRead + 2) {
        debugLogf("Ответ короче ожидаемого: len=%d, нужно минимум %d", len, 7 + bytesRead + 2);
        bytesRead = 0;
        return false;
    }

    // Копируем данные
    memcpy(outBuffer, resp + 7, bytesRead);

    debugHex("COPIED to outBuffer:", outBuffer, bytesRead);

    return true;
}

WebServer server(80);

/* ================= API ================= */
void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleOptions() {
  addCorsHeaders();
  server.send(204);
}

void handleBat() {
  float voltage = readBatteryVoltage();
  int percent = voltageToPercent(voltage);

  // Возвращаем чистый JSON с процентом
  String json = "{\"percent\":" + String(percent) + "}";
  server.send(200, "application/json", json);
}

void handleRead() {
  addCorsHeaders();
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
  addCorsHeaders();
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
  addCorsHeaders();
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
  addCorsHeaders();
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
  doc["device_address"] = String("0x") + String(mtzpAddress, HEX);
  doc["baud_rate"] = mtzpBaudRate;

  String payload;
  serializeJson(doc, payload);
  server.send((ok1 || ok2) ? 200 : 500, "application/json", payload);
}

void handleStatus() {
  addCorsHeaders();
  // Статус системы
  StaticJsonDocument<512> doc;
  doc["uptime_ms"] = millis();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.softAPgetStationNum();
  doc["device_address"] = String("0x") + String(mtzpAddress, HEX);
  doc["baud_rate"] = mtzpBaudRate;
  doc["slip_timeout_ms"] = SLIP_TIMEOUT_MS;
  doc["byte_timeout_ms"] = BYTE_TIMEOUT_MS;

  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

void handleConfigGet() {
  addCorsHeaders();
  StaticJsonDocument<256> doc;
  doc["ok"] = true;
  doc["address"] = mtzpAddress;
  doc["baud_rate"] = mtzpBaudRate;

  JsonArray baudArray = doc.createNestedArray("allowed_baud_rates");
  for (size_t i = 0; i < allowedBaudRatesCount; i++) {
    baudArray.add(allowedBaudRates[i]);
  }

  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

void handleConfigSet() {
  addCorsHeaders();
  if (!server.hasArg("addr") || !server.hasArg("baud")) {
    server.send(400, "application/json",
                "{\"error\":\"Missing 'addr' or 'baud' parameter\"}");
    return;
  }

  int addrValue = server.arg("addr").toInt();
  uint32_t baudValue = server.arg("baud").toInt();

  if (addrValue < 0 || addrValue > 255) {
    server.send(400, "application/json",
                "{\"error\":\"Invalid address (0-255)\"}");
    return;
  }

  if (!isAllowedBaudRate(baudValue)) {
    server.send(400, "application/json",
                "{\"error\":\"Invalid baud rate\"}");
    return;
  }

  mtzpAddress = static_cast<uint8_t>(addrValue);
  mtzpBaudRate = baudValue;
  preferences.putUChar("addr", mtzpAddress);
  preferences.putUInt("baud", mtzpBaudRate);

  Serial2.end();
  delay(50);
  Serial2.begin(mtzpBaudRate, SERIAL_8N1, RS485_RX, RS485_TX);

  StaticJsonDocument<256> doc;
  doc["ok"] = true;
  doc["address"] = mtzpAddress;
  doc["baud_rate"] = mtzpBaudRate;

  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

void handleRestart() {
  addCorsHeaders();
  server.send(200, "application/json", "{\"ok\":true}");
  delay(200);
  ESP.restart();
}

void handleJournal() {
    addCorsHeaders();

    if (!server.hasArg("type")) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"type обязателен\"}");
        return;
    }

    String type = server.arg("type");
    int startIdx = server.hasArg("idx") ? server.arg("idx").toInt() : 1;   // с какой записи начинать (по умолчанию с самой свежей)
    int count = server.hasArg("count") ? server.arg("count").toInt() : 10; // сколько читать (по умолчанию 10)

    if (startIdx < 1 || count < 1 || count > 50) {  // лимит на разумное количество
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"неверные idx или count (1..50)\"}");
        return;
    }

    // Структура журнала
    struct Journal {
        uint32_t baseAddr;
        uint16_t recSize;
        uint16_t regTotal;
        uint16_t regLast;
        uint16_t maxRecords;
    } j;

    if      (type == "alarm")     { j = {0x00009C0, 64, 100, 101, 200}; }
    else if (type == "fault")     { j = {0x003BC0, 64, 102, 103, 100}; }
    else if (type == "setchange") { j = {0x0054C0, 12, 104, 105, 200}; }
    else if (type == "comm")      { j = {0x005E20,  8, 106, 107, 100}; }
    else if (type == "power")     { j = {0x006140,  8, 108, 109, 100}; }
    else if (type == "diag")      { j = {0x006460,  8, 110, 111, 100}; }
    else if (type == "powerlog")  { j = {0x006780, 10, 112, 113, 600}; }
    else {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"неизвестный тип\"}");
        return;
    }

    // Читаем метаданные один раз
    uint16_t totalRecords = 0;
    uint16_t lastPointer  = 0;

    if (!mtzpRead(j.regTotal, totalRecords) || !mtzpRead(j.regLast, lastPointer)) {
        server.send(500, "application/json", "{\"ok\":false,\"error\":\"ошибка чтения метаданных журнала\"}");
        return;
    }

    if (totalRecords == 0) {
        server.send(200, "application/json", "{\"ok\":true,\"warning\":\"журнал пуст\",\"total\":0}");
        return;
    }

    // Ограничиваем count, чтобы не выйти за пределы
    if (startIdx + count - 1 > totalRecords) {
        count = totalRecords - startIdx + 1;
    }

    // Готовим массив для ответа
    StaticJsonDocument<4096> doc;  // большой буфер — 10 записей по 64 байта + служебное
    doc["ok"] = true;
    doc["type"] = type;
    doc["total_records"] = totalRecords;
    doc["last_pointer"] = lastPointer;
    doc["requested_start"] = startIdx;
    doc["requested_count"] = count;

    JsonArray records = doc.createNestedArray("records");

    // Читаем записи пачкой
    for (int i = 0; i < count; i++) {
        int currentIdx = startIdx + i;

        // Позиция в циклическом буфере
        int32_t pos = (int32_t)lastPointer - (currentIdx - 1);
        if (pos <= 0) pos += j.maxRecords;

        uint32_t byteOffset = (uint32_t)(pos - 1) * j.recSize;
        uint32_t readAddr = j.baseAddr + byteOffset;

        uint8_t buf[128] = {0};
        uint16_t bytesRead = 0;

        if (!mtzpReadMemory(readAddr, j.recSize, buf, bytesRead)) {
            // Если ошибка чтения одной записи — продолжаем, но отметим
            JsonObject rec = records.createNestedObject();
            rec["idx"] = currentIdx;
            rec["error"] = "чтение не удалось";
            continue;
        }

        JsonObject rec = records.createNestedObject();
        rec["idx"] = currentIdx;
        rec["address"] = String("0x") + String(readAddr, HEX);
        rec["bytes_read"] = bytesRead;

        String hexStr;
        for (uint16_t k = 0; k < j.recSize; k++) {
            if (k > 0) hexStr += " ";
            if (buf[k] < 16) hexStr += "0";
            hexStr += String(buf[k], HEX);
        }
        rec["raw_hex"] = hexStr;
    }

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
}

/* ================= SETUP / LOOP ================= */
void setup() {
  DEBUG_SERIAL.begin(115200);
  delay(1000);

  DEBUG_SERIAL.println("\n\n=== MTZP ESP32 Шлюз v2.0 ===");
  DEBUG_SERIAL.println("Исправленная версия с улучшенной обработкой протокола");

  preferences.begin("mtzp", false);
  loadSettings();

  // НОВОЕ: Валидация адреса устройства
  if (mtzpAddress == 0x00) {
    DEBUG_SERIAL.println("ВНИМАНИЕ: Адрес 0x00 - широковещательный режим");
    DEBUG_SERIAL.println("Устройство не будет отвечать на запросы!");
  } else if (mtzpAddress > 0xFF) {
    DEBUG_SERIAL.printf("ОШИБКА: Неверный адрес МТЗП 0x%02X (допустимо 0x01-0xFF)\n",
                       mtzpAddress);
    while(1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }
  
  if (!LittleFS.begin(true)) {   // true = форматировать при ошибке монтирования
    DEBUG_SERIAL.println("Ошибка монтирования LittleFS!");
    // Можно добавить индикацию ошибки, например мигание LED
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  } else {
    DEBUG_SERIAL.println("LittleFS успешно смонтирован");
    DEBUG_SERIAL.printf("Общий размер: %d байт\n", LittleFS.totalBytes());
    DEBUG_SERIAL.printf("Использовано:  %d байт\n", LittleFS.usedBytes());
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Инициализация UART для RS485
  Serial2.begin(mtzpBaudRate, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);  // Режим приёма по умолчанию

  DEBUG_SERIAL.printf("RS485 инициализирован: %d бод\n", mtzpBaudRate);
  DEBUG_SERIAL.printf("Адрес МТЗП: 0x%02X\n", mtzpAddress);
  DEBUG_SERIAL.printf("Таймауты: общий=%dмс, между байтами=%dмс\n",
                     SLIP_TIMEOUT_MS, BYTE_TIMEOUT_MS);

  // Настройка WiFi точки доступа
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress apIP = WiFi.softAPIP();
  DEBUG_SERIAL.print("AP IP адрес: ");
  DEBUG_SERIAL.println(apIP);

  // Настройка веб-сервера
  server.on("/api/read", HTTP_GET, handleRead);
  server.on("/api/write", HTTP_POST, handleWrite);
  server.on("/api/read_multiple", HTTP_GET, handleReadMultiple);
  server.on("/api/test", HTTP_GET, handleTest);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_GET, handleConfigGet);
  server.on("/api/config", HTTP_POST, handleConfigSet);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/read", HTTP_OPTIONS, handleOptions);
  server.on("/api/write", HTTP_OPTIONS, handleOptions);
  server.on("/api/read_multiple", HTTP_OPTIONS, handleOptions);
  server.on("/api/journal", HTTP_GET, handleJournal);
  server.on("/api/test", HTTP_OPTIONS, handleOptions);
  server.on("/bat", HTTP_GET, handleBat);
  server.on("/api/status", HTTP_OPTIONS, handleOptions);
  server.on("/api/config", HTTP_OPTIONS, handleOptions);
  server.on("/api/restart", HTTP_OPTIONS, handleOptions);
  server.on("/", HTTP_GET, []() {
    if (LittleFS.exists("/index.html")) {
      File file = LittleFS.open("/index.html", "r");
      if (file) {
        server.streamFile(file, "text/html");
        file.close();
      } else {
        server.send(500, "text/plain", "Failed to open index.html");
      }
    } else {
      server.send(404, "text/plain", "index.html not found in LittleFS");
    }
  });
  server.on("/set", HTTP_GET, []() {
    if (LittleFS.exists("/set.html")) {
      File file = LittleFS.open("/set.html", "r");
      if (file) {
        server.streamFile(file, "text/html");
        file.close();
      } else {
        server.send(500, "text/plain", "Failed to open set.html");
      }
    } else {
      server.send(404, "text/plain", "set.html not found in LittleFS");
    }
  });
  server.on("/logs", HTTP_GET, []() {
    if (LittleFS.exists("/logs.html")) {
        File file = LittleFS.open("/logs.html", "r");
        server.streamFile(file, "text/html");
        file.close();
    } else {
        server.send(404, "text/plain", "logs.html not found");
    }
  });

  server.begin();
  DEBUG_SERIAL.println("HTTP сервер запущен");
  DEBUG_SERIAL.println("Откройте http://" + apIP.toString() + " в браузере");
  DEBUG_SERIAL.println("\nДоступные API endpoints:");
  DEBUG_SERIAL.println("  GET  /api/read?reg=<номер>");
  DEBUG_SERIAL.println("  POST /api/write?reg=<номер>&val=<значение>");
  DEBUG_SERIAL.println("  GET  /api/read_multiple?regs=<номер1,номер2,...>");
  DEBUG_SERIAL.println("  GET  /api/test");
  DEBUG_SERIAL.println("  GET  /api/status");
  DEBUG_SERIAL.println("  GET  /api/config");
  DEBUG_SERIAL.println("  POST /api/config?addr=<адрес>&baud=<скорость>");
  DEBUG_SERIAL.println("  POST /api/restart");
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
