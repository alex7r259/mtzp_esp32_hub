#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

/* ================= HARDWARE ================= */
#define RS485_RX 16
#define RS485_TX 17
#define RS485_DE 4

#define MTZP_ADDR 0x01
#define UART_BAUD 19200

/* ================= WIFI ================= */
const char* AP_SSID = "MTZP";
const char* AP_PASS = "12345678";

/* ================= SLIP ================= */
#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

/* ================= CRC16 ================= */
static const uint16_t crc16_table[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040
};

uint16_t crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    uint8_t idx = (crc ^ data[i]) & 0xFF;
    crc = (crc >> 8) ^ crc16_table[idx];
  }
  return crc;
}

/* ================= SLIP ================= */
void slipSend(const uint8_t* data, uint16_t len) {
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
  Serial2.write(SLIP_END);
}

int slipRecv(uint8_t* buffer, uint16_t maxLen, uint32_t timeoutMs) {
  uint32_t start = millis();
  bool escape = false;
  uint16_t idx = 0;
  while (millis() - start < timeoutMs) {
    if (!Serial2.available()) {
      continue;
    }
    uint8_t ch = Serial2.read();
    if (ch == SLIP_END && idx > 0) {
      return idx;
    }
    if (ch == SLIP_ESC) {
      escape = true;
      continue;
    }
    if (escape) {
      ch = (ch == SLIP_ESC_END) ? SLIP_END : SLIP_ESC;
      escape = false;
    }
    if (idx < maxLen) {
      buffer[idx++] = ch;
    }
  }
  return -1;
}

/* ================= MTZP ================= */
bool mtzpRead(uint16_t reg, uint16_t& val) {
  uint8_t frame[7] = {MTZP_ADDR, 0x01, 0x02, (uint8_t)(reg >> 8), (uint8_t)reg};
  uint16_t crc = crc16(frame, 5);
  frame[5] = crc & 0xFF;
  frame[6] = crc >> 8;

  digitalWrite(RS485_DE, HIGH);
  slipSend(frame, 7);
  Serial2.flush();
  digitalWrite(RS485_DE, LOW);

  uint8_t reply[16];
  int len = slipRecv(reply, 16, 300);
  if (len < 7) {
    return false;
  }
  val = (reply[5] << 8) | reply[6];
  return true;
}

bool mtzpWrite(uint16_t reg, uint16_t val) {
  uint8_t frame[9] = {
    MTZP_ADDR,
    0x02,
    0x04,
    (uint8_t)(reg >> 8),
    (uint8_t)reg,
    (uint8_t)(val >> 8),
    (uint8_t)val
  };
  uint16_t crc = crc16(frame, 7);
  frame[7] = crc & 0xFF;
  frame[8] = crc >> 8;

  digitalWrite(RS485_DE, HIGH);
  slipSend(frame, 9);
  Serial2.flush();
  digitalWrite(RS485_DE, LOW);

  uint8_t reply[16];
  int len = slipRecv(reply, 16, 300);
  if (len < 7) {
    return false;
  }
  return true;
}

WebServer server(80);

/* ================= HTML UI ================= */
const char MAIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>MTZP</title>
<style>
body{font-family:Arial;background:#111;color:#eee;margin:0;padding:20px}
.card{background:#1e1e1e;padding:15px;border-radius:8px;margin-bottom:15px}
input,button{padding:8px;font-size:16px;margin:5px 0;width:100%}
button{background:#0a84ff;color:white;border:0;border-radius:5px}
button[disabled]{background:#555}
details{margin:6px 0}
summary{cursor:pointer}
.menu-item{margin:4px 0}
.menu-link{cursor:pointer;text-decoration:underline;color:#9bd1ff}
.menu-link:hover{color:#cfe8ff}
.menu-meta{font-size:12px;color:#aaa}
.tabs{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:10px}
.tab{padding:6px 10px;border-radius:6px;background:#2a2a2a;cursor:pointer}
.tab.active{background:#0a84ff}
.menu-value{margin-left:6px;font-weight:bold}
.menu-edit{display:flex;gap:6px;align-items:center;margin-top:4px}
.menu-edit input{flex:1}
</style></head>
<body>
<h2>MTZP Web Panel</h2>
<div class=card>
<h3>Меню устройства</h3>
<div class=tabs id=menuTabs></div>
<div id=menu></div>
</div>
<script>
const registerTable = [
  {"id":0,"name":"Регистр команд","format":"DEC","scale":0},
  {"id":1,"name":"RS485 Адрес","format":"HEX","scale":0},
  {"id":2,"name":"RS485 Скорость, kbps","format":"DEC","scale":1},
  {"id":3,"name":"RS485 Таймаут, мсек","format":"DEC","scale":0},
  {"id":4,"name":"Регистр пароля","format":"DEC","scale":0},
  {"id":5,"name":"RS485 MODBUS (0=Slip/1=Modbus)","format":"DEC","scale":0},
  {"id":6,"name":"RS485 Пак./сек.","format":"DEC","scale":0},
  {"id":7,"name":"Регистр команд МПУ","format":"HEX","scale":0},
  {"id":8,"name":"МПУ Таймаут, мсек","format":"DEC","scale":0},
  {"id":9,"name":"МПУ Пак./сек.","format":"DEC","scale":0},
  {"id":10,"name":"МТЗП-1200 номер","format":"DEC","scale":0},
  {"id":11,"name":"Месяц изготовлен (Soft)","format":"DEC","scale":2},
  {"id":12,"name":"Регистр состояния","format":"HEX","scale":0},
  {"id":13,"name":"Циклов main, ед.","format":"DEC","scale":0},
  {"id":14,"name":"Циклов АЦП, ед.","format":"DEC","scale":0},
  {"id":15,"name":"Циклов Input, ед.","format":"DEC","scale":0},
  {"id":16,"name":"RTC миллисекунды","format":"DEC","scale":0},
  {"id":17,"name":"RTC секунды","format":"DEC","scale":0},
  {"id":18,"name":"RTC минуты","format":"DEC","scale":0},
  {"id":19,"name":"RTC часы","format":"DEC","scale":0},
  {"id":20,"name":"RTC дата","format":"DEC","scale":0},
  {"id":21,"name":"RTC месяц","format":"DEC","scale":0},
  {"id":22,"name":"RTC год","format":"DEC","scale":0},
  {"id":23,"name":"Регистр реле","format":"HEX","scale":0},
  {"id":24,"name":"Внешние входы 1","format":"OCT","scale":0},
  {"id":25,"name":"Внешние входы 2","format":"OCT","scale":0},
  {"id":26,"name":"Внутренние входы","format":"HEX","scale":0},
  {"id":27,"name":"Входы пульта МПУ","format":"HEX","scale":0},
  {"id":28,"name":"Регистр аварий - 1","format":"HEX","scale":0},
  {"id":29,"name":"Регистр аварий - 2","format":"HEX","scale":0},
  {"id":30,"name":"Регистр неисправностей - 1","format":"HEX","scale":0},
  {"id":31,"name":"Регистр неисправностей - 2","format":"HEX","scale":0},
  {"id":32,"name":"Темпер. воздуха, °C","format":"SDC","scale":1},
  {"id":33,"name":"Напряжен. фазы A, В","format":"DEC","scale":0},
  {"id":34,"name":"Напряжен. фазы B, В","format":"DEC","scale":0},
  {"id":35,"name":"Напряжен. фазы C, В","format":"DEC","scale":0},
  {"id":36,"name":"Макс. напряжение, В","format":"DEC","scale":0},
  {"id":37,"name":"Мин. напряжение, В","format":"DEC","scale":0},
  {"id":38,"name":"Несимметрия Uфаз, %","format":"DEC","scale":1},
  {"id":39,"name":"Порог несим. Uфаз, В","format":"DEC","scale":0},
  {"id":40,"name":"Индикация напряж.","format":"BIN","scale":2},
  {"id":41,"name":"Ток фазы A, A","format":"DEC","scale":0},
  {"id":42,"name":"Ток фазы B, A","format":"DEC","scale":0},
  {"id":43,"name":"Ток фазы C, A","format":"DEC","scale":0},
  {"id":44,"name":"Максимальный ток, A","format":"DEC","scale":0},
  {"id":45,"name":"Минимальный ток, A","format":"DEC","scale":0},
  {"id":46,"name":"Несимметрия Iфаз, %","format":"DEC","scale":1},
  {"id":47,"name":"Порог несим. Iфаз, А","format":"DEC","scale":0},
  {"id":48,"name":"Перегрузка по Iф, %","format":"DEC","scale":1},
  {"id":49,"name":"R изоляции ф. A, кОм","format":"DEC","scale":0},
  {"id":50,"name":"R изоляции ф. B, кОм","format":"DEC","scale":0},
  {"id":51,"name":"R изоляции ф. C, кОм","format":"DEC","scale":0},
  {"id":52,"name":"Мин. Rизоляции, кОм","format":"DEC","scale":0},
  {"id":53,"name":"Паралл. нагрузка","format":"SWT","scale":1},
  {"id":54,"name":"Мощность фазы A, кВт","format":"SDC","scale":1},
  {"id":55,"name":"Мощность фазы B, кВт","format":"SDC","scale":1},
  {"id":56,"name":"Мощность фазы C, кВт","format":"SDC","scale":1},
  {"id":57,"name":"Мощность активн., кВт","format":"SDC","scale":1},
  {"id":58,"name":"Мощность полная, кВА","format":"DEC","scale":1},
  {"id":59,"name":"Коэф. мощности","format":"REL","scale":3},
  {"id":60,"name":"Напряжение нуля, В","format":"DEC","scale":0},
  {"id":61,"name":"Ток нулевой, mA","format":"DEC","scale":0},
  {"id":62,"name":"Угол между V и I, °","format":"SDC","scale":0},
  {"id":63,"name":"Оперативное пит., В","format":"DEC","scale":1},
  {"id":64,"name":"Линейное напряж., В","format":"DEC","scale":0},
  {"id":65,"name":"Номинал. Uпит., В","format":"DEC","scale":1},
  {"id":66,"name":"Номинал. Uлин., В","format":"DEC","scale":0},
  {"id":67,"name":"Напряжение Vtst, В","format":"DEC","scale":1},
  {"id":68,"name":"Напряжение 17V, В","format":"DEC","scale":2},
  {"id":69,"name":"Напряжение 05V, В","format":"DEC","scale":2},
  {"id":70,"name":"Пульсация Iмакс, %","format":"DEC","scale":1},
  {"id":71,"name":"Ток пуска, A","format":"DEC","scale":0},
  {"id":72,"name":"Напряжение пуска, В","format":"DEC","scale":0},
  {"id":73,"name":"Время пуска, мсек","format":"DEC","scale":0},
  {"id":74,"name":"Порог изм. Iпуска, А","format":"DEC","scale":0},
  {"id":75,"name":"T фильтра парам., мсек","format":"DEC","scale":0},
  {"id":76,"name":"Код Аварии/Неиспр.","format":"DEC","scale":0},
  {"id":77,"name":"Счетчик Аварий","format":"DEC","scale":0},
  {"id":78,"name":"Счетчик Неиспр.","format":"DEC","scale":0},
  {"id":79,"name":"Циклов вкл./откл. КА","format":"DEC","scale":0},
  {"id":80,"name":"Энергия активная, кВт/ч","format":"DEC","scale":0},
  {"id":81,"name":"Энергия активная, МВт/ч","format":"DEC","scale":0},
  {"id":82,"name":"Энергия полная, кВА/ч","format":"DEC","scale":0},
  {"id":83,"name":"Энергия полная, МВА/ч","format":"DEC","scale":0},
  {"id":84,"name":"Отключение пит., мин/сек","format":"DEC","scale":2},
  {"id":85,"name":"Отключение пит., день/час","format":"DEC","scale":2},
  {"id":86,"name":"Включение пит., мин/сек","format":"DEC","scale":2},
  {"id":87,"name":"Включение пит., день/час","format":"DEC","scale":2},
  {"id":88,"name":"Отключение КА мин/сек","format":"DEC","scale":2},
  {"id":89,"name":"Отключение КА, день/час","format":"DEC","scale":2},
  {"id":90,"name":"Включение КА, мин/сек","format":"DEC","scale":2},
  {"id":91,"name":"Включение КА, день/час","format":"DEC","scale":2},
  {"id":92,"name":"Работа присоед., мин/сек","format":"DEC","scale":2},
  {"id":93,"name":"Работа присоед., день/час","format":"DEC","scale":2},
  {"id":94,"name":"Работа прис/сум., мин/сек","format":"DEC","scale":2},
  {"id":95,"name":"Работа прис/сум., день/час","format":"DEC","scale":2},
  {"id":96,"name":"Наработка блока, мин/сек","format":"DEC","scale":2},
  {"id":97,"name":"Наработка блока, день/час","format":"DEC","scale":2},
  {"id":98,"name":"Работа блока сум, мин/сек","format":"DEC","scale":2},
  {"id":99,"name":"Работа блока сум, день/час","format":"DEC","scale":2},
  {"id":100,"name":"Всего Аварий","format":"DEC","scale":0},
  {"id":101,"name":"Текущий протокол (аварии)","format":"DEC","scale":0},
  {"id":102,"name":"Всего Неисправн.","format":"DEC","scale":0},
  {"id":103,"name":"Текущий протокол (неиспр.)","format":"DEC","scale":0},
  {"id":104,"name":"Всего уставок","format":"DEC","scale":0},
  {"id":105,"name":"Текущий протокол (уставки)","format":"DEC","scale":0},
  {"id":106,"name":"Всего коммутаций","format":"DEC","scale":0},
  {"id":107,"name":"Текущий протокол (коммутаций)","format":"DEC","scale":0},
  {"id":108,"name":"Всего Вкл/Откл","format":"DEC","scale":0},
  {"id":109,"name":"Текущий протокол (питание)","format":"DEC","scale":0},
  {"id":110,"name":"Всего диагностик","format":"DEC","scale":0},
  {"id":111,"name":"Текущий протокол (диагностика)","format":"DEC","scale":0},
  {"id":112,"name":"Всего мощности","format":"DEC","scale":0},
  {"id":113,"name":"Текущий протокол (мощность)","format":"DEC","scale":0},
  {"id":114,"name":"Период записи P, сек.","format":"DEC","scale":0},
  {"id":115,"name":"МТЗ1 Р. управлен.","format":"HEX","scale":0},
  {"id":116,"name":"МТЗ1 Р. состояния","format":"HEX","scale":0},
  {"id":117,"name":"МТЗ1 Уставка, A","format":"DEC","scale":0},
  {"id":118,"name":"МТЗ1 Задержка, мсек","format":"DEC","scale":0},
  {"id":119,"name":"МТЗ1 Уставка ЛЗШ, A","format":"DEC","scale":0},
  {"id":120,"name":"МТЗ1 Задержка ЛЗШ, мсек","format":"DEC","scale":0},
  {"id":121,"name":"ЛЗШ1 Задержка, мсек","format":"DEC","scale":0},
  {"id":122,"name":"МТЗ2 Р. управлен.","format":"HEX","scale":0},
  {"id":123,"name":"МТЗ2 Р. состояния","format":"HEX","scale":0},
  {"id":124,"name":"МТЗ2 Уставка, A","format":"DEC","scale":0},
  {"id":125,"name":"МТЗ2 Задержка, мсек","format":"DEC","scale":0},
  {"id":126,"name":"МТЗ3 Р. управлен.","format":"HEX","scale":0},
  {"id":127,"name":"МТЗ3 Р. состояния","format":"HEX","scale":0},
  {"id":128,"name":"МТЗ3 Уставка, A","format":"DEC","scale":0},
  {"id":129,"name":"МТЗ3 Задержка, мсек","format":"DEC","scale":0},
  {"id":130,"name":"МТЗ3 Тип характеристики","format":"DEC","scale":0},
  {"id":131,"name":"УМТЗ Р. управлен.","format":"HEX","scale":0},
  {"id":132,"name":"УМТЗ Р. состояния","format":"HEX","scale":0},
  {"id":133,"name":"УМТЗ Уставка, A","format":"DEC","scale":0},
  {"id":134,"name":"УМТЗ Задержка, мсек","format":"DEC","scale":0},
  {"id":135,"name":"УМТЗ Время работы, мсек","format":"DEC","scale":0},
  {"id":136,"name":"ЗМТ Р. управлен.","format":"HEX","scale":0},
  {"id":137,"name":"ЗМТ Р. состояния","format":"HEX","scale":0},
  {"id":138,"name":"ЗМТ Уставка, A","format":"DEC","scale":0},
  {"id":139,"name":"ЗМТ Задержка, мсек","format":"DEC","scale":0},
  {"id":140,"name":"ЗМТ Включение, мсек","format":"DEC","scale":0},
  {"id":141,"name":"ЗМН Р. управления","format":"HEX","scale":0},
  {"id":142,"name":"ЗМН Р. состояния","format":"HEX","scale":0},
  {"id":143,"name":"ЗМН Max Уставка, В","format":"DEC","scale":0},
  {"id":144,"name":"ЗМН Max Задержка, мсек","format":"DEC","scale":0},
  {"id":145,"name":"ЗМН Min Уставка, В","format":"DEC","scale":0},
  {"id":146,"name":"ЗМН Min Задержка, мсек","format":"DEC","scale":0},
  {"id":147,"name":"Uс.шин Уставка, В","format":"DEC","scale":0},
  {"id":148,"name":"Uс.шин Задержка, мсек","format":"DEC","scale":0},
  {"id":149,"name":"ЗНФ Р. управления","format":"HEX","scale":0},
  {"id":150,"name":"ЗНФ Р. состояния","format":"HEX","scale":0},
  {"id":151,"name":"ЗНФ Уставка, %","format":"DEC","scale":1},
  {"id":152,"name":"ЗНФ Задержка, мсек","format":"DEC","scale":0},
  {"id":153,"name":"ЗНФ Включение, мсек","format":"DEC","scale":0},
  {"id":154,"name":"ЗОФ Р. управления","format":"HEX","scale":0},
  {"id":155,"name":"ЗОФ Р. состояния","format":"HEX","scale":0},
  {"id":156,"name":"ЗОФ Уставка, %","format":"DEC","scale":1},
  {"id":157,"name":"ЗОФ Задержка, мсек","format":"DEC","scale":0},
  {"id":158,"name":"ЗОФ Включение, мсек","format":"DEC","scale":0},
  {"id":159,"name":"БКИ Р. управления","format":"HEX","scale":0},
  {"id":160,"name":"БКИ Р. состояния","format":"HEX","scale":0},
  {"id":161,"name":"БКИ Уставка, кОм","format":"DEC","scale":0},
  {"id":162,"name":"БКИ Задержка, мсек","format":"DEC","scale":0},
  {"id":163,"name":"БКИ Включение, мсек","format":"DEC","scale":0},
  {"id":164,"name":"НЗЗ Р. управления","format":"HEX","scale":0},
  {"id":165,"name":"НЗЗ Р. состояния","format":"HEX","scale":0},
  {"id":166,"name":"НЗЗ Уставка по I, mA","format":"DEC","scale":0},
  {"id":167,"name":"НЗЗ Уставка по U, В","format":"DEC","scale":0},
  {"id":168,"name":"НЗЗ Задержка, мсек","format":"DEC","scale":0},
  {"id":169,"name":"НЗЗ Угол Мин., °","format":"SDC","scale":0},
  {"id":170,"name":"НЗЗ Угол Макс., °","format":"SDC","scale":0},
  {"id":171,"name":"EXT Р. управления-1","format":"HEX","scale":0},
  {"id":172,"name":"EXT Р. управления-2","format":"HEX","scale":0},
  {"id":173,"name":"EXT Р. состояния","format":"HEX","scale":0},
  {"id":174,"name":"EXT1 Задержка, мсек","format":"DEC","scale":0},
  {"id":175,"name":"EXT2 Задержка, мсек","format":"DEC","scale":0},
  {"id":176,"name":"EXT3 Задержка, мсек","format":"DEC","scale":0},
  {"id":177,"name":"EXT4 Задержка, мсек","format":"DEC","scale":0},
  {"id":178,"name":"EXT KZ Задержка, мсек","format":"DEC","scale":0},
  {"id":179,"name":"INT Р. управления","format":"HEX","scale":0},
  {"id":180,"name":"INT Р. состояния","format":"HEX","scale":0},
  {"id":181,"name":"INT1 Задержка, мсек","format":"DEC","scale":0},
  {"id":182,"name":"INT2 Задержка, мсек","format":"DEC","scale":0},
  {"id":183,"name":"INT3 Задержка, мсек","format":"DEC","scale":0},
  {"id":184,"name":"INT4 Задержка, мсек","format":"DEC","scale":0},
  {"id":185,"name":"SWT Р. управления","format":"HEX","scale":0},
  {"id":186,"name":"SWT Р. состояния","format":"HEX","scale":0},
  {"id":187,"name":"SWT_E Задержка, мсек","format":"DEC","scale":0},
  {"id":188,"name":"Время реле ОТКЛ, мсек","format":"DEC","scale":0},
  {"id":189,"name":"Время реле ВКЛ, мсек","format":"DEC","scale":0},
  {"id":190,"name":"READY Задержка, мсек","format":"DEC","scale":0},
  {"id":191,"name":"Причина отключ.","format":"DEC","scale":0},
  {"id":192,"name":"Причина включен.","format":"DEC","scale":0},
  {"id":193,"name":"Время отключения, мсек","format":"DEC","scale":1},
  {"id":194,"name":"Время включения, мсек","format":"DEC","scale":1},
  {"id":195,"name":"УРО Р. управления","format":"HEX","scale":0},
  {"id":196,"name":"УРО Р. состояния","format":"HEX","scale":0},
  {"id":197,"name":"УРО Уставка по I, A","format":"DEC","scale":0},
  {"id":198,"name":"УРО Задержка, мсек","format":"DEC","scale":0},
  {"id":199,"name":"Диаг. управление","format":"HEX","scale":0},
  {"id":200,"name":"Диаг. Р. состояния","format":"HEX","scale":0},
  {"id":201,"name":"Диаг. БКИ Уставка, кОм","format":"DEC","scale":0},
  {"id":202,"name":"Диаг. ДТ Уставка, A","format":"DEC","scale":0},
  {"id":203,"name":"Диаг. ТНП Уставка, mA","format":"DEC","scale":0},
  {"id":204,"name":"Время диагност., мсек","format":"DEC","scale":0},
  {"id":205,"name":"АПВ управление","format":"HEX","scale":0},
  {"id":206,"name":"АПВ Р. состояния","format":"HEX","scale":0},
  {"id":207,"name":"АПВ Время откл., сек.","format":"DEC","scale":0},
  {"id":208,"name":"АПВ Время вкл., мсек","format":"DEC","scale":0},
  {"id":209,"name":"АПВ Интервал вкл, мсек","format":"DEC","scale":0},
  {"id":210,"name":"АВР управление","format":"HEX","scale":0},
  {"id":211,"name":"АВР Р. состояния","format":"DEC","scale":0},
  {"id":212,"name":"АВР Задержка, мсек","format":"DEC","scale":0},
  {"id":213,"name":"Ctrl управление","format":"HEX","scale":0},
  {"id":214,"name":"Ctrl Р. состоян-1","format":"HEX","scale":0},
  {"id":215,"name":"Ctrl Р. состоян-2","format":"HEX","scale":0},
  {"id":216,"name":"Ctrl ПДУ1 Время, мсек","format":"DEC","scale":0},
  {"id":217,"name":"Ctrl ПДУ2 Время, мсек","format":"DEC","scale":0},
  {"id":218,"name":"Ctrl МПУ Время, мсек","format":"DEC","scale":0},
  {"id":219,"name":"Ctrl STOP Время, мсек","format":"DEC","scale":0},
  {"id":220,"name":"Конфигурация БВР","format":"HEX","scale":1},
  {"id":221,"name":"Конфигурация ДТ","format":"HEX","scale":1},
  {"id":222,"name":"Конфигурация ТНП","format":"HEX","scale":1},
  {"id":223,"name":"Конфигурация МОД","format":"HEX","scale":1},
  {"id":224,"name":"Конфигурация ДП1","format":"HEX","scale":1},
  {"id":225,"name":"Конфигурация ДП2","format":"HEX","scale":1},
  {"id":226,"name":"Конфигурация ДП3","format":"HEX","scale":1},
  {"id":227,"name":"Конфигурация ДП4","format":"HEX","scale":1},
  {"id":228,"name":"Конфигурация ДП5","format":"HEX","scale":1},
  {"id":229,"name":"Конфигурация ДП6","format":"HEX","scale":1},
  {"id":230,"name":"R изоляции Ron, кОм","format":"DEC","scale":0},
  {"id":231,"name":"Ток отключения по МТЗ, А","format":"DEC","scale":0},
  {"id":232,"name":"Уставка защиты МТЗ, А","format":"DEC","scale":0},
  {"id":233,"name":"Коррекция по U, ед","format":"DEC","scale":0},
  {"id":234,"name":"Коррекция по I, ед","format":"DEC","scale":0},
  {"id":235,"name":"Коррекция по P, ед","format":"DEC","scale":0},
  {"id":236,"name":"Коррекция по R, ед","format":"DEC","scale":0},
  {"id":237,"name":"Коррекция по I_0, ед","format":"DEC","scale":0},
  {"id":238,"name":"Коррекция по Vab, ед","format":"DEC","scale":0},
  {"id":239,"name":"Коррекция V400, ед","format":"DEC","scale":0},
  {"id":240,"name":"Коррекция Temp, ед","format":"DEC","scale":0},
  {"id":241,"name":"Сдвиг нуля Temp, °C","format":"SDC","scale":1},
  {"id":242,"name":"Коррекция V 17, ед","format":"DEC","scale":0},
  {"id":243,"name":"Коррекция по Ron, ед","format":"DEC","scale":0},
  {"id":244,"name":"Время прерывания, uS","format":"DEC","scale":0},
  {"id":245,"name":"Время прерывания, uS","format":"DEC","scale":0},
  {"id":246,"name":"Режим отладки","format":"HEX","scale":0},
  {"id":247,"name":"Тестовый байт0","format":"HEX","scale":0},
  {"id":248,"name":"Тестовый байт1","format":"HEX","scale":0},
  {"id":249,"name":"Тестовый байт2","format":"SDC","scale":0},
  {"id":250,"name":"Тестовый байт3","format":"SDC","scale":0},
  {"id":251,"name":"Тестовый байт4","format":"SDC","scale":0},
  {"id":252,"name":"Период обновлен.,мсек","format":"DEC","scale":0},
  {"id":253,"name":"Указатель макс.","format":"DEC","scale":0},
  {"id":254,"name":"Указатель внешн.","format":"DEC","scale":0},
  {"id":255,"name":"Переменная1","format":"SDC","scale":0},
  {"id":256,"name":"Переменная2","format":"SDC","scale":0},
  {"id":257,"name":"Переменная3","format":"SDC","scale":0},
  {"id":258,"name":"Переменная4","format":"SDC","scale":0},
  {"id":259,"name":"ADC 0 (UA lo)","format":"DEC","scale":0},
  {"id":260,"name":"ADC 1 (UA hi)","format":"DEC","scale":0},
  {"id":261,"name":"ADC 2 (UB lo)","format":"DEC","scale":0},
  {"id":262,"name":"ADC 3 (UB hi)","format":"DEC","scale":0},
  {"id":263,"name":"ADC 4 (UC lo)","format":"DEC","scale":0},
  {"id":264,"name":"ADC 5 (UC hi)","format":"DEC","scale":0},
  {"id":265,"name":"ADC 6 (IA lo)","format":"DEC","scale":0},
  {"id":266,"name":"ADC 7 (IA hi)","format":"DEC","scale":0},
  {"id":267,"name":"ADC 8 (IC lo)","format":"DEC","scale":0},
  {"id":268,"name":"ADC 9 (IC hi)","format":"DEC","scale":0},
  {"id":269,"name":"ADC 10 (3U_0)","format":"DEC","scale":0},
  {"id":270,"name":"ADC 11 (3I_0)","format":"DEC","scale":0},
  {"id":271,"name":"ADC 12 (U_AB)","format":"DEC","scale":0},
  {"id":272,"name":"ADC 13 (Vtst)","format":"DEC","scale":0},
  {"id":273,"name":"ADC 14 (Temp)","format":"DEC","scale":0},
  {"id":274,"name":"ADC 15 (free)","format":"DEC","scale":0},
  {"id":275,"name":"Параметров FRAM","format":"DEC","scale":0},
  {"id":276,"name":"Параметров всего","format":"DEC","scale":0}
];

const registerIndex = new Map(registerTable.map(item => [item.id, item]));

const menuTree = [
  {
    "label":"1. Текущие параметры",
    "children":[
      {
        "label":"1.1. Фазные токи",
        "children":[
          {"label":"1.1.1. Ток фазы А, А","regId":41},
          {"label":"1.1.2. Ток фазы В, А","regId":42},
          {"label":"1.1.3. Ток фазы С, А","regId":43},
          {"label":"1.1.4. Максимальный ток, А","regId":44},
          {"label":"1.1.5. Минимальный ток, А","regId":45},
          {"label":"1.1.6. Несимметрия тока ΔI, %","regId":46},
          {"label":"1.1.7. Перегрузка, %","regId":48},
          {"label":"1.1.8. К трансформации, ед.","regId":233},
          {"label":"1.1.9. Порог несимметрии, А","regId":47}
        ]
      },
      {
        "label":"1.2. Фазные напряжения",
        "children":[
          {"label":"1.2.1. Напряжение фазы А, В","regId":33},
          {"label":"1.2.2. Напряжение фазы В, В","regId":34},
          {"label":"1.2.3. Напряжение фазы С, В","regId":35},
          {"label":"1.2.4. Напряжение максимальное, В","regId":36},
          {"label":"1.2.5. Напряжение минимальное, В","regId":37},
          {"label":"1.2.6. Несимметрия напряжения ΔU, %","regId":38},
          {"label":"1.2.7. Порог несимметрии, В","regId":39},
          {"label":"1.2.8. Индикация линейного напряжения, да/нет","regId":40}
        ]
      },
      {
        "label":"1.3. Сопротивление изоляции",
        "children":[
          {"label":"1.3.1. Сопротивление фазы А, кОм","regId":49},
          {"label":"1.3.2. Сопротивление фазы В, кОм","regId":50},
          {"label":"1.3.3. Сопротивление фазы С, кОм","regId":51},
          {"label":"1.3.4. Сопротивление минимальное, кОм","regId":52},
          {"label":"1.3.5. Параллельная нагрузка, да/нет","regId":53}
        ]
      },
      {
        "label":"1.4. Потребляемая мощность",
        "children":[
          {"label":"1.4.1. Мощность фазы А, кВт","regId":54},
          {"label":"1.4.2. Мощность фазы В, кВт","regId":55},
          {"label":"1.4.3. Мощность фазы С, кВт","regId":56},
          {"label":"1.4.4. Мощность активная, кВт","regId":57},
          {"label":"1.4.5. Мощность полная, кВА","regId":58},
          {"label":"1.4.6. Коэффициент мощности","regId":59},
          {"label":"1.4.7. Энергия активная, кВт/ч","regId":80},
          {"label":"1.4.8. Энергия активная, МВт/ч","regId":81},
          {"label":"1.4.9. Энергия полная, кВт/ч","regId":82},
          {"label":"1.4.10. Энергия полная, МВт/ч","regId":83}
        ]
      },
      {
        "label":"1.5. Линейное напряжение",
        "children":[
          {"label":"1.5.1. Оперативное питание, В","regId":63},
          {"label":"1.5.2. Линейное напряжение, В","regId":64},
          {"label":"1.5.3. Номинальное напряжение питания, В","regId":65},
          {"label":"1.5.4. Номинальное напряжение линейное, В","regId":66}
        ]
      },
      {
        "label":"1.6. Время наработки",
        "children":[
          {"label":"1.6.1. Наработка КА, минуты и секунды","regId":92},
          {"label":"1.6.2. Наработка КА, дни и часы","regId":93},
          {"label":"1.6.3. Суммарная наработка КА, минуты и секунды","regId":94},
          {"label":"1.6.4. Суммарная наработка КА, дни и часы","regId":95},
          {"label":"1.6.5. Наработка МТЗП, минуты и секунды","regId":96},
          {"label":"1.6.6. Наработка МТЗП, дни и часы","regId":97},
          {"label":"1.6.7. Суммарная наработка МТЗП, минуты и секунды","regId":98},
          {"label":"1.6.8. Суммарная наработка МТЗП, дни и часы","regId":99}
        ]
      },
      {
        "label":"1.7. Прочие параметры",
        "children":[
          {"label":"1.7.1. Температура в блоке БЗУ, ºС","regId":32},
          {"label":"1.7.2. Напряжение нуля, В","regId":60},
          {"label":"1.7.3. Ток нуля, мА","regId":61},
          {"label":"1.7.4. Угол между током и напряжением нуля, º","regId":62},
          {"label":"1.7.5. Пульсация максимального тока, %","regId":70},
          {"label":"1.7.6. Ток пуска, А","regId":71},
          {"label":"1.7.7. Напряжение пуска, В","regId":72},
          {"label":"1.7.8. Время пуска, мС","regId":73},
          {"label":"1.7.9. Порог измерения тока пуска, А","regId":74}
        ]
      },
      {"label":"1.8. Ввод пароля","regId":4}
    ]
  },
  {
    "label":"2. Уставки защит",
    "children":[
      {
        "label":"2.1. МТЗ – 1",
        "children":[
          {"label":"2.1.1.1. Защита введена, да/нет","regId":115},
          {"label":"2.1.1.2. Максимальный ток, А","regId":44},
          {"label":"2.1.1.3. Уставка по току, А","regId":117},
          {"label":"2.1.1.4. Уставка по времени, мС","regId":118},
          {"label":"2.1.1.5. Отключение КА, да/нет","regId":115},
          {"label":"2.1.1.6. Фиксация, да/нет","regId":115},
          {"label":"2.1.1.7. Вход ЛЗШ, да/нет","regId":115},
          {"label":"2.1.1.8. Внешний вход, да/нет","regId":115},
          {"label":"2.1.1.9. Порог Rmin, да/нет","regId":115},
          {"label":"2.1.1.10. Уставка ЛЗШ по току, А","regId":119},
          {"label":"2.1.1.11. Уставка ЛЗШ по времени, мС","regId":120},
          {"label":"2.1.1.12. Выход ЛЗШ, да/нет","regId":115},
          {"label":"2.1.1.13. Время ЛЗШ по выходу, мС","regId":121}
        ]
      },
      {
        "label":"2.1. МТЗ – 2",
        "children":[
          {"label":"2.1.2.1. Защита введена, да/нет","regId":122},
          {"label":"2.1.2.2. Максимальный ток, А","regId":44},
          {"label":"2.1.2.3. Уставка по току, А","regId":124},
          {"label":"2.1.2.4. Уставка по времени, мС","regId":125},
          {"label":"2.1.2.5. Отключение КА, да/нет","regId":122},
          {"label":"2.1.2.6. Фиксация, да/нет","regId":122}
        ]
      },
      {
        "label":"2.1. МТЗ – 3",
        "children":[
          {"label":"2.1.3.1. Защита введена, да/нет","regId":126},
          {"label":"2.1.3.2. Максимальный ток, А","regId":44},
          {"label":"2.1.3.3. Уставка по току, А","regId":128},
          {"label":"2.1.3.4. Уставка по времени, мС","regId":129},
          {"label":"2.1.3.5. Тип характеристики","regId":130},
          {"label":"2.1.3.6. Отключение КА, да/нет","regId":126},
          {"label":"2.1.3.7. Фиксация, да/нет","regId":126}
        ]
      },
      {
        "label":"2.1. УМТЗ",
        "children":[
          {"label":"2.1.4.1. Защита введена, да/нет","regId":131},
          {"label":"2.1.4.2. Максимальный ток, А","regId":44},
          {"label":"2.1.4.3. Уставка по току, А","regId":133},
          {"label":"2.1.4.4. Уставка по времени, мС","regId":134},
          {"label":"2.1.4.5. Время работы, мС","regId":135},
          {"label":"2.1.4.6. Отключение КА, да/нет","regId":131},
          {"label":"2.1.4.7. Фиксация, да/нет","regId":131}
        ]
      },
      {
        "label":"2.2. ЗМТ",
        "children":[
          {"label":"2.2.1. Защита введена, да/нет","regId":136},
          {"label":"2.2.2. Минимальный ток, А","regId":45},
          {"label":"2.2.3. Уставка по току, А","regId":138},
          {"label":"2.2.4. Уставка по времени, мС","regId":139},
          {"label":"2.2.5. Время включения, мС","regId":140},
          {"label":"2.2.6. Отключение КА, да/нет","regId":136},
          {"label":"2.2.7. Фиксация, да/нет","regId":136}
        ]
      },
      {
        "label":"2.3. ЗММН",
        "children":[
          {
            "label":"2.3.1. ЗММН максимум",
            "children":[
              {"label":"2.3.1.1. Защита введена, да/нет","regId":141},
              {"label":"2.3.1.2. Линейное напряжение, В","regId":64},
              {"label":"2.3.1.3. Уставка по напряжению, В","regId":143},
              {"label":"2.3.1.4. Уставка по времени, мС","regId":144}
            ]
          },
          {
            "label":"2.3.2. ЗММН минимум",
            "children":[
              {"label":"2.3.2.1. Защита введена, да/нет","regId":141},
              {"label":"2.3.2.2. Линейное напряжение, В","regId":64},
              {"label":"2.3.2.3. Уставка по напряжению, В","regId":145},
              {"label":"2.3.2.4. Уставка по времени, мС","regId":146}
            ]
          },
          {
            "label":"2.3.3. ЗММН Секция шин",
            "children":[
              {"label":"2.3.3.1. Линейное напряжение, В","regId":64},
              {"label":"2.3.3.2. Уставка по напряжению, В","regId":147},
              {"label":"2.3.3.3. Уставка по времени, мС","regId":148}
            ]
          },
          {"label":"2.3.4. Отключение КА, да/нет","regId":141},
          {"label":"2.3.5. Фиксация, да/нет","regId":141}
        ]
      },
      {
        "label":"2.4. ЗНФ",
        "children":[
          {"label":"2.4.1. Защита введена, да/нет","regId":149},
          {"label":"2.4.2. Несимметрия по току ΔI, %","regId":46},
          {"label":"2.4.3. Уставка по несимметрии, %","regId":151},
          {"label":"2.4.4. Уставка по времени, мС","regId":152},
          {"label":"2.4.5. Время включения, мС","regId":153},
          {"label":"2.4.6. Отключение КА, да/нет","regId":149},
          {"label":"2.4.7. Фиксация, да/нет","regId":149}
        ]
      },
      {
        "label":"2.5. ЗОФ",
        "children":[
          {"label":"2.5.1. Защита введена, да/нет","regId":154},
          {"label":"2.5.2. Несимметрия по напряжению ΔU, %","regId":38},
          {"label":"2.5.3. Уставка по несимметрии, %","regId":156},
          {"label":"2.5.4. Уставка по времени, мС","regId":157},
          {"label":"2.5.5. Время включения, мС","regId":158},
          {"label":"2.5.6. Отключение КА, да/нет","regId":154},
          {"label":"2.5.7. Фиксация, да/нет","regId":154}
        ]
      },
      {
        "label":"2.6. БКИ",
        "children":[
          {"label":"2.6.1. Защита введена, да/нет","regId":159},
          {"label":"2.6.2. Минимальное R изоляции, кОм","regId":52},
          {"label":"2.6.3. Уставка по сопротивлению, кОм","regId":161},
          {"label":"2.6.4. Уставка по времени, мС","regId":162},
          {"label":"2.6.5. Время включения, мС","regId":163},
          {"label":"2.6.6. Фиксация, да/нет","regId":159},
          {"label":"2.6.7. Предварительная защита введена, да/нет","regId":159},
          {"label":"2.6.8. Предварительная фиксация, да/нет","regId":159}
        ]
      },
      {
        "label":"2.7. НЗЗ",
        "children":[
          {"label":"2.7.1. Защита введена, да/нет","regId":164},
          {"label":"2.7.2. Канал по току введен, да/нет","regId":164},
          {"label":"2.7.3. Канал по напряжению введен, да/нет","regId":164},
          {"label":"2.7.4. Канал по углу введен, да/нет","regId":164},
          {"label":"2.7.5. Напряжение нуля, В","regId":60},
          {"label":"2.7.6. Ток нуля, мА","regId":61},
          {"label":"2.7.7. Угол между током и напряжением нуля, º","regId":62},
          {"label":"2.7.8. Уставка по току, мА","regId":166},
          {"label":"2.7.9. Уставка по напряжению, В","regId":167},
          {"label":"2.7.10. Уставка по времени, мС","regId":168},
          {"label":"2.7.11. Угол минимальный, º","regId":169},
          {"label":"2.7.12. Угол максимальный, º","regId":170},
          {"label":"2.7.13. Отключение КА, да/нет","regId":164},
          {"label":"2.7.14. Фиксация, да/нет","regId":164}
        ]
      },
      {
        "label":"2.8. Внешние защиты",
        "children":[
          {
            "label":"2.8.1. Внешняя защита – 1",
            "children":[
              {"label":"2.8.1.1. Защита введена, да/нет","regId":171},
              {"label":"2.8.1.2. Уставка по времени, мС","regId":174},
              {"label":"2.8.1.3. Отключение КА, да/нет","regId":171},
              {"label":"2.8.1.4. Фиксация, да/нет","regId":171},
              {"label":"2.8.1.5. Порог Rmin, да/нет","regId":171}
            ]
          },
          {
            "label":"2.8.2. Внешняя защита - 2",
            "children":[
              {"label":"2.8.2.1. Защита введена, да/нет","regId":171},
              {"label":"2.8.2.2. Уставка по времени, мС","regId":175},
              {"label":"2.8.2.3. Отключение КА, да/нет","regId":171},
              {"label":"2.8.2.4. Фиксация, да/нет","regId":171},
              {"label":"2.8.2.5. Порог Rmin, да/нет","regId":171}
            ]
          },
          {
            "label":"2.8.3. Внешняя защита - 3",
            "children":[
              {"label":"2.8.3.1. Защита введена, да/нет","regId":171},
              {"label":"2.8.3.2. Уставка по времени, мС","regId":176},
              {"label":"2.8.3.3. Отключение КА, да/нет","regId":171},
              {"label":"2.8.3.4. Фиксация, да/нет","regId":171},
              {"label":"2.8.3.5. Порог Rmin, да/нет","regId":171}
            ]
          },
          {
            "label":"2.8.4. Внешняя защита - 4",
            "children":[
              {"label":"2.8.4.1. Защита введена, да/нет","regId":171},
              {"label":"2.8.4.2. Уставка по времени, мС","regId":177},
              {"label":"2.8.4.3. Отключение КА, да/нет","regId":171},
              {"label":"2.8.4.4. Фиксация, да/нет","regId":171},
              {"label":"2.8.4.5. Порог Rmin, да/нет","regId":171}
            ]
          },
          {
            "label":"2.8.5. Защита по замыканию",
            "children":[
              {"label":"2.8.5.1. Защита введена, да/нет","regId":172},
              {"label":"2.8.5.2. Уставка по времени, мС","regId":178},
              {"label":"2.8.5.3. Отключение КА, да/нет","regId":172},
              {"label":"2.8.5.4. Фиксация, да/нет","regId":172},
              {"label":"2.8.5.5. Учёт вход1 (ПДУ1), да/нет","regId":172},
              {"label":"2.8.5.6. Учёт вход2 (ПДУ2), да/нет","regId":172},
              {"label":"2.8.5.7. Учёт вход3 (EXT 1), да/нет","regId":172},
              {"label":"2.8.5.8. Учёт вход4 (EXT 2), да/нет","regId":172},
              {"label":"2.8.5.9. Учёт вход5 (EXT 3), да/нет","regId":172},
              {"label":"2.8.5.10. Учёт вход6 (ЛЗШ), да/нет","regId":172},
              {"label":"2.8.5.11. Учёт вход7 (EXT 4), да/нет","regId":172}
            ]
          }
        ]
      },
      {
        "label":"2.9. Местные защиты",
        "children":[
          {
            "label":"2.9.1. Местная защита – 1",
            "children":[
              {"label":"2.9.1.1. Защита введена, да/нет","regId":179},
              {"label":"2.9.1.2. Уставка по времени, мС","regId":181},
              {"label":"2.9.1.3. Отключение КА, да/нет","regId":179},
              {"label":"2.9.1.4. Фиксация, да/нет","regId":179},
              {"label":"2.9.1.5. Инверсия, да/нет","regId":179}
            ]
          },
          {
            "label":"2.9.2. Местная защита - 2",
            "children":[
              {"label":"2.9.2.1. Защита введена, да/нет","regId":179},
              {"label":"2.9.2.2. Уставка по времени, мС","regId":182},
              {"label":"2.9.2.3. Отключение КА, да/нет","regId":179},
              {"label":"2.9.2.4. Фиксация, да/нет","regId":179},
              {"label":"2.9.2.5. Инверсия, да/нет","regId":179}
            ]
          },
          {
            "label":"2.9.3. Местная защита - 3",
            "children":[
              {"label":"2.9.3.1. Защита введена, да/нет","regId":179},
              {"label":"2.9.3.2. Уставка по времени, мС","regId":183},
              {"label":"2.9.3.3. Отключение КА, да/нет","regId":179},
              {"label":"2.9.3.4. Фиксация, да/нет","regId":179},
              {"label":"2.9.3.5. Инверсия, да/нет","regId":179}
            ]
          },
          {
            "label":"2.9.4. Местная защита - 4",
            "children":[
              {"label":"2.9.4.1. Защита введена, да/нет","regId":179},
              {"label":"2.9.4.2. Уставка по времени, мС","regId":184},
              {"label":"2.9.4.3. Отключение КА, да/нет","regId":179},
              {"label":"2.9.4.4. Фиксация, да/нет","regId":179}
            ]
          }
        ]
      },
      {
        "label":"2.10. Контроль КА",
        "children":[
          {"label":"2.10.1. Защита введена, да/нет","regId":185},
          {"label":"2.10.2. Уставка по времени, мС","regId":187},
          {"label":"2.10.3. Фиксация, да/нет","regId":185}
        ]
      },
      {
        "label":"2.11. Защита УРОВ",
        "children":[
          {"label":"2.11.1. Защита введена, да/нет","regId":195},
          {"label":"2.11.2. Максимальный ток, А","regId":44},
          {"label":"2.11.3. Уставка по току, А","regId":197},
          {"label":"2.11.4. Уставка по времени, мС","regId":198},
          {"label":"2.11.5. Канал по току введен, да/нет","regId":195},
          {"label":"2.11.6. Отключение КА, да/нет","regId":195},
          {"label":"2.11.7. Фиксация, да/нет","regId":195},
          {"label":"2.11.8. Учёт внешней защиты 1, да/нет","regId":195},
          {"label":"2.11.9. Учёт внешней защиты 2, да/нет","regId":195},
          {"label":"2.11.10. Учёт внешней защиты 3, да/нет","regId":195},
          {"label":"2.11.11. Учёт внешней защиты 4, да/нет","regId":195},
          {"label":"2.11.12. Учёт ЗММН минимум, да/нет","regId":195},
          {"label":"2.11.13. Учёт ЗММН максимум, да/нет","regId":195},
          {"label":"2.11.14. Учёт ЗОФ, да/нет","regId":195},
          {"label":"2.11.15. Учёт ЗНФ, да/нет","regId":195},
          {"label":"2.11.16. Учёт НЗЗ, да/нет","regId":195},
          {"label":"2.11.17. Учёт ЗМТ, да/нет","regId":195}
        ]
      },
      {"label":"2.12. Ввод пароля","regId":4}
    ]
  },
  {
    "label":"3. Контроль сигналов",
    "children":[
      {
        "label":"3.1. Релейные выходы",
        "children":[
          {"label":"3.1.1. Реле К1 (Включение КА / Вперед), да/нет","regId":23},
          {"label":"3.1.2. Реле К2 (Управление КН / Назад), да/нет","regId":23},
          {"label":"3.1.3. Реле К3 (Отключение КА), да/нет","regId":23},
          {"label":"3.1.4. Реле К4 (УРОВ), да/нет","regId":23},
          {"label":"3.1.5. Реле К5 (Секция Шин), да/нет","regId":23},
          {"label":"3.1.6. Реле К6 (Пуск МТЗ), да/нет","regId":23},
          {"label":"3.1.7. Реле К7 (МТЗ - 1), да/нет","regId":23},
          {"label":"3.1.8. Реле К8 (Авария), да/нет","regId":23},
          {"label":"3.1.9. Реле К9 (Авария), да/нет","regId":23},
          {"label":"3.1.10. Реле К10 (Положение КА), да/нет","regId":23},
          {"label":"3.1.11. Реле К11 (УРОВ), да/нет","regId":23},
          {"label":"3.1.12. Реле К12 (Пуск МТЗ), да/нет","regId":23}
        ]
      },
      {
        "label":"3.2. Внешние входы 1",
        "children":[
          {"label":"3.2.1. Внешний вход 1_бит0 (ПДУ1), да/нет","regId":24},
          {"label":"3.2.2. Внешний вход 1_бит1 (ПДУ1), да/нет","regId":24},
          {"label":"3.2.3. Внешний вход 1_бит2 (ПДУ1), да/нет","regId":24},
          {"label":"3.2.4. Внешний вход 2_бит0 (ПДУ2), да/нет","regId":24},
          {"label":"3.2.5. Внешний вход 2_бит1 (ПДУ2), да/нет","regId":24},
          {"label":"3.2.6. Внешний вход 2_бит2 (ПДУ2), да/нет","regId":24},
          {"label":"3.2.7. Внешний вход 3_бит0 (АГЗ), да/нет","regId":24},
          {"label":"3.2.8. Внешний вход 3_бит1 (АГЗ), да/нет","regId":24},
          {"label":"3.2.9. Внешний вход 3_бит2 (АГЗ), да/нет","regId":24},
          {"label":"3.2.10. Внешний вход 4_бит0 (ВЗ-2), да/нет","regId":24},
          {"label":"3.2.11. Внешний вход 4_бит1 (ВЗ-2), да/нет","regId":24},
          {"label":"3.2.12. Внешний вход 4_бит2 (ВЗ-2), да/нет","regId":24}
        ]
      },
      {
        "label":"3.3. Внешние входы 2",
        "children":[
          {"label":"3.3.1. Внешний вход 5_бит0 (ВЗ-3), да/нет","regId":25},
          {"label":"3.3.2. Внешний вход 5_бит1 (ВЗ-3), да/нет","regId":25},
          {"label":"3.3.3. Внешний вход 5_бит2 (ВЗ-3), да/нет","regId":25},
          {"label":"3.3.4. Внешний вход 6_бит0 (ЛЗШ), да/нет","regId":25},
          {"label":"3.3.5. Внешний вход 6_бит1 (ЛЗШ), да/нет","regId":25},
          {"label":"3.3.6. Внешний вход 6_бит2 (ЛЗШ), да/нет","regId":25},
          {"label":"3.3.7. Внешний вход 7_бит0 (Терм), да/нет","regId":25},
          {"label":"3.3.8. Внешний вход 7_бит1 (Терм), да/нет","regId":25},
          {"label":"3.3.9. Внешний вход 7_бит2 (Терм), да/нет","regId":25}
        ]
      },
      {
        "label":"3.4. Внутренние входы",
        "children":[
          {"label":"3.4.1. Внутренний вход 1 (РПО), да/нет","regId":26},
          {"label":"3.4.2. Внутренний вход 2 (РПВ), да/нет","regId":26},
          {"label":"3.4.3. Внутренний вход 3 (Контроль БП), да/нет","regId":26},
          {"label":"3.4.4. Внутренний вход 4 (Дуговая защита), да/нет","regId":26},
          {"label":"3.4.5. Внутренний вход 5 (МТЗ прямого действия), да/нет","regId":26},
          {"label":"3.4.6. Внутренний вход 6 (ОТКЛ), да/нет","regId":26},
          {"label":"3.4.7. Внутренний вход 7 (ВКЛ), да/нет","regId":26},
          {"label":"3.4.8. Внутренний вход 8 (СБРОС), да/нет","regId":26},
          {"label":"3.4.9. Внутренний вход 9 (ТЕСТ МТЗ), да/нет","regId":26},
          {"label":"3.4.10. Внутренний вход 10 (ТЕСТ БКИ), да/нет","regId":26},
          {"label":"3.4.11. Внутренний вход 11 (ТЕСТ ТНП), да/нет","regId":26},
          {"label":"3.4.12. Внутренний вход 12 (ЛЗШ), да/нет","regId":26},
          {"label":"3.4.13. Внутренний вход 13 (АВР-1), да/нет","regId":26},
          {"label":"3.4.14. Внутренний вход 14 (АВР-2), да/нет","regId":26},
          {"label":"3.4.15. Внутренний вход 15 (Резерв), да/нет","regId":26}
        ]
      },
      {"label":"3.5. Включение управления выходами, да/нет","regId":29},
      {"label":"3.6. Ввод пароля","regId":4}
    ]
  },
  {
    "label":"4. Настройки",
    "children":[
      {
        "label":"4.1. Присоединение",
        "children":[
          {"label":"4.1.1. Номинальное напряжение питания, В","regId":65},
          {"label":"4.1.2. Номинальное напряжение линейное, В","regId":66},
          {"label":"4.1.3. К трансформации, ед.","regId":233},
          {"label":"4.1.4. Измерение тока В, да/нет","regId":234},
          {"label":"4.1.5. Порог несимметрии, А","regId":47},
          {"label":"4.1.6. Порог несимметрии, В","regId":39},
          {"label":"4.1.7. Параллельная нагрузка, да/нет","regId":53},
          {"label":"4.1.8. Порог измерения тока пуска, А","regId":74},
          {"label":"4.1.9. Индикация линейного напряжения, да/нет","regId":40}
        ]
      },
      {
        "label":"4.2. Режим управления",
        "children":[
          {"label":"4.2.1. Телеуправление, да/нет","regId":213},
          {"label":"4.2.2. МПУ, да/нет","regId":213},
          {"label":"4.2.3. ПДУ - 1, да/нет","regId":213},
          {"label":"4.2.4. ПДУ - 2, да/нет","regId":213},
          {"label":"4.2.5. Контактор, да/нет","regId":213},
          {"label":"4.2.6. Реверсивный блок, да/нет","regId":213},
          {"label":"4.2.7. ПДУ-1 + ПДУ-2, да/нет","regId":213},
          {"label":"4.2.8. Время МПУ, мС","regId":218},
          {"label":"4.2.9. Время «СТОП», мС","regId":219},
          {"label":"4.2.10. Время ПДУ1, мС","regId":216},
          {"label":"4.2.11. Время ПДУ2, мС","regId":217}
        ]
      },
      {
        "label":"4.3. Блок управления КА",
        "children":[
          {"label":"4.3.1. Время реле ОТКЛ, мС","regId":188},
          {"label":"4.3.2. Время реле ВКЛ, мС","regId":189},
          {"label":"4.3.3. Время ГОТОВ, мС","regId":190},
          {"label":"4.3.4. Время отключения КА, мС","regId":193},
          {"label":"4.3.5. Время включения КА, мС","regId":194}
        ]
      },
      {
        "label":"4.4. Настройки АПВ",
        "children":[
          {"label":"4.4.1. АПВ введен, да/нет","regId":205},
          {"label":"4.4.2. Без ограничения времени, да/нет","regId":205},
          {"label":"4.4.3. Время отключения, сек","regId":207},
          {"label":"4.4.4. Время включения, мС","regId":208},
          {"label":"4.4.5. Интервал включения, мС","regId":209}
        ]
      },
      {
        "label":"4.5. Настройки АВР",
        "children":[
          {"label":"4.5.1. АВР введен, да/нет","regId":210},
          {"label":"4.5.2. Внешний вход, да/нет","regId":210},
          {"label":"4.5.3. Порог Rmin, да/нет","regId":210},
          {"label":"4.5.4. Время включения, мС","regId":212}
        ]
      },
      {
        "label":"4.6. Часы RTC",
        "children":[
          {"label":"4.6.1. Часы RTC, секунды","regId":17},
          {"label":"4.6.2. Часы RTC, минуты","regId":18},
          {"label":"4.6.3. Часы RTC, часы","regId":19},
          {"label":"4.6.4. Часы RTC, дата","regId":20},
          {"label":"4.6.5. Часы RTC, месяц","regId":21},
          {"label":"4.6.6. Часы RTC, год","regId":22}
        ]
      },
      {
        "label":"4.7. Телеуправление RS485",
        "children":[
          {"label":"4.7.1. Адрес в сети","regId":1},
          {"label":"4.7.2. Скорость, kbps","regId":2},
          {"label":"4.7.3. Таймаут, мС","regId":3},
          {"label":"4.7.4. Тип протокола","regId":5},
          {"label":"4.7.5. Пакетов в секунду","regId":6}
        ]
      },
      {"label":"4.8. Обновление, мС","regId":252},
      {
        "label":"4.9. Управление профилями",
        "children":[
          {"label":"4.9.1. Сохранить настройки, да/нет","regId":0},
          {"label":"4.9.2. Загрузить настройки, да/нет","regId":0},
          {"label":"4.9.3. Загрузить заводские настройки, да/нет","regId":0},
          {"label":"4.9.4. Юстировка каналов измерений, да/нет","regId":0},
          {"label":"4.9.5. Обнуление счётчиков, да/нет","regId":0}
        ]
      },
      {
        "label":"4.10. Самодиагностика",
        "children":[
          {
            "label":"4.10.1. Тест датчиков тока",
            "children":[
              {"label":"4.10.1.1. Самодиагностика, да/нет","regId":199},
              {"label":"4.10.1.2. При включенном КА, да/нет","regId":199},
              {"label":"4.10.1.3. Отключение КА, да/нет","regId":199},
              {"label":"4.10.1.4. Уставка, А","regId":202}
            ]
          },
          {
            "label":"4.10.2. Тест канала БКИ",
            "children":[
              {"label":"4.10.2.1. Самодиагностика, да/нет","regId":199},
              {"label":"4.10.2.2. Уставка, кОм","regId":201}
            ]
          },
          {
            "label":"4.10.3. Тест канала НЗЗ",
            "children":[
              {"label":"4.10.3.1. Самодиагностика, да/нет","regId":199},
              {"label":"4.10.3.2. При включенном КА, да/нет","regId":199},
              {"label":"4.10.3.3. Отключение КА, да/нет","regId":199},
              {"label":"4.10.3.4. Уставка диагностики, мА","regId":203}
            ]
          },
          {"label":"4.10.4. Время диагностики, мС","regId":204}
        ]
      },
      {"label":"4.11. Ввод пароля","regId":4}
    ]
  },
  {
    "label":"5. Журнал событий",
    "children":[
      {"label":"5.1. Журналы Аварий и Неисправностей","children":[
        {"label":"5.1.1. Протокол Х из 200","regId":101},
        {"label":"5.1.2. Событие: Отключение КА","regId":191},
        {"label":"5.1.3. Источник: Авария","regId":28},
        {"label":"5.1.4. Время: ХХ:ХХ:ХХ","regId":17},
        {"label":"5.1.5. Дата: ХХ.ХХ.ХХХХ","regId":20},
        {"label":"5.1.6. Авария: Наименование","regId":28},
        {"label":"5.1.7. Время точное, мС","regId":16},
        {"label":"5.1.8. Ток фазы А, А","regId":41},
        {"label":"5.1.9. Ток фазы В, А","regId":42},
        {"label":"5.1.10. Ток фазы С, А","regId":43},
        {"label":"5.1.11. Максимальный ток, А","regId":44},
        {"label":"5.1.12. Напряжение фазы А, В","regId":33},
        {"label":"5.1.13. Напряжение фазы В, В","regId":34},
        {"label":"5.1.14. Напряжение фазы С, В","regId":35},
        {"label":"5.1.15. Сопротивление фазы А, кОм","regId":49},
        {"label":"5.1.16. Сопротивление фазы В, кОм","regId":50},
        {"label":"5.1.17. Сопротивление фазы С, кОм","regId":51},
        {"label":"5.1.18. Напряжение нуля, В","regId":60},
        {"label":"5.1.19. Ток нуля, мА","regId":61},
        {"label":"5.1.20. Угол между током и напряжением нуля, º","regId":62},
        {"label":"5.1.21. Линейное напряжение, В","regId":64},
        {"label":"5.1.22. Температура в блоке БЗУ, ºС","regId":32},
        {"label":"5.1.23. Ток пуска, А","regId":71},
        {"label":"5.1.24. Напряжение пуска, В","regId":72},
        {"label":"5.1.25. Время пуска, мС","regId":73},
        {"label":"5.1.26. Релейные выходы","regId":23},
        {"label":"5.1.27. Внешние входы 1","regId":24},
        {"label":"5.1.28. Внешние входы 2","regId":25},
        {"label":"5.1.29. Внутренние входы","regId":26},
        {"label":"5.1.30. Регистр аварий","regId":28},
        {"label":"5.1.31. Регистр неисправностей","regId":30}
      ]},
      {"label":"5.2. Изменение уставок","children":[
        {"label":"5.2.1. Протокол Х из 200","regId":105},
        {"label":"5.2.2. Номер параметра: ХХХ","regId":104},
        {"label":"5.2.3. Наименование параметра","regId":104},
        {"label":"5.2.4. Время: ХХ:ХХ:ХХ","regId":17},
        {"label":"5.2.5. Дата: ХХ.ХХ.ХХХХ","regId":20},
        {"label":"5.2.6. Источник: МПУ","regId":27},
        {"label":"5.2.7. Старое значение","regId":104},
        {"label":"5.2.8. Новое значение","regId":104}
      ]},
      {"label":"5.3. Включения/Отключения КА","children":[
        {"label":"5.3.1. Протокол Х из 200","regId":107},
        {"label":"5.3.2. Событие: Отключение","regId":191},
        {"label":"5.3.3. Источник: Питание","regId":108},
        {"label":"5.3.4. Время: ХХ:ХХ:ХХ","regId":17},
        {"label":"5.3.5. Дата: ХХ.ХХ.ХХХХ","regId":20}
      ]},
      {"label":"5.4. Включения/Отключения питания МТЗП","children":[
        {"label":"5.4.1. Протокол Х из 200","regId":109},
        {"label":"5.4.2. Событие: Отключение","regId":108},
        {"label":"5.4.3. Источник: Питание","regId":108},
        {"label":"5.4.4. Время: ХХ:ХХ:ХХ","regId":17},
        {"label":"5.4.5. Дата: ХХ.ХХ.ХХХХ","regId":20}
      ]},
      {"label":"5.5. Самодиагностика МТЗП","children":[
        {"label":"5.5.1. Протокол Х из 200","regId":111},
        {"label":"5.5.2. Событие: Диагностика МТЗ","regId":200},
        {"label":"5.5.3. Источник: ПТУ","regId":199},
        {"label":"5.5.4. Время: ХХ:ХХ:ХХ","regId":17},
        {"label":"5.5.5. Дата: ХХ.ХХ.ХХХХ","regId":20}
      ]},
      {"label":"5.6. Потребляемая мощность","children":[
        {"label":"5.6.1. Протокол Х из 117","regId":113},
        {"label":"5.6.2. Мощность активная, кВт","regId":57},
        {"label":"5.6.3. Коэффициент мощности","regId":59},
        {"label":"5.6.4. Время: ХХ:ХХ:ХХ","regId":17},
        {"label":"5.6.5. Дата: ХХ.ХХ.ХХХХ","regId":20}
      ]},
      {"label":"5.7. Запись мощности, сек","regId":114},
      {"label":"5.8. Сброс журналов, да/нет","regId":0},
      {"label":"5.9. Ввод пароля","regId":4}
    ]
  },
  {
    "label":"6. Информация об изделии",
    "children":[
      {"label":"Тип блока: МТЗП – 2T","regId":10},
      {"label":"Серийный номер","regId":10},
      {"label":"Месяц выпуска","regId":11},
      {"label":"Версия программы","regId":11},
      {"label":"Время","regId":17},
      {"label":"Дата","regId":20}
    ]
  }
];

function isEditableLabel(label){
  const editableHints = [
    'да/нет',
    'Уставка',
    'уставка',
    'Порог',
    'порог',
    'Время',
    'время',
    'Номинал',
    'номинал',
    'Конфигурация',
    'управление',
    'Управление',
    'Обновление',
    'Сохранить',
    'Загрузить',
    'Юстировка',
    'Обнуление'
  ];
  return editableHints.some(hint => label.includes(hint));
}

function renderMenuItem(item){
  const container = document.createElement('div');
  if(item.children && item.children.length){
    const details = document.createElement('details');
    const summary = document.createElement('summary');
    summary.textContent = item.label;
    details.appendChild(summary);
    item.children.forEach(child => {
      details.appendChild(renderMenuItem(child));
    });
    container.appendChild(details);
  }else{
    const line = document.createElement('div');
    line.className = 'menu-item';
    if(typeof item.regId === 'number'){
      const link = document.createElement('span');
      link.className = 'menu-link';
      link.textContent = item.label;
      line.appendChild(link);
      const meta = registerIndex.get(item.regId);
      if(meta){
        const metaSpan = document.createElement('span');
        metaSpan.className = 'menu-meta';
        metaSpan.textContent = ` (рег. ${meta.id}, ${meta.format}, scale ${meta.scale})`;
        line.appendChild(metaSpan);
      }
      const valueSpan = document.createElement('span');
      valueSpan.className = 'menu-value';
      valueSpan.textContent = '...';
      valueSpan.dataset.regId = String(item.regId);
      line.appendChild(valueSpan);
      if(isEditableLabel(item.label)){
        const edit = document.createElement('div');
        edit.className = 'menu-edit';
        const input = document.createElement('input');
        input.placeholder = 'Новое значение';
        input.dataset.regId = String(item.regId);
        const button = document.createElement('button');
        button.textContent = 'Записать';
        button.addEventListener('click', () => writeRegisterValue(item.regId, input.value));
        edit.appendChild(input);
        edit.appendChild(button);
        line.appendChild(edit);
      }
    }else{
      line.textContent = item.label;
    }
    container.appendChild(line);
  }
  return container;
}

const menuRoot = document.getElementById('menu');
const menuTabs = document.getElementById('menuTabs');
let activeMenuIndex = 0;

function renderTabs(){
  menuTabs.innerHTML = '';
  menuTree.forEach((item, index) => {
    const tab = document.createElement('div');
    tab.className = 'tab' + (index === activeMenuIndex ? ' active' : '');
    tab.textContent = item.label;
    tab.addEventListener('click', () => {
      activeMenuIndex = index;
      renderMenuPage();
    });
    menuTabs.appendChild(tab);
  });
}

function renderMenuPage(){
  renderTabs();
  menuRoot.innerHTML = '';
  menuRoot.appendChild(renderMenuItem(menuTree[activeMenuIndex]));
  refreshVisibleValues();
}

function formatValue(raw, format, scale){
  const scaleFactor = Math.pow(10, scale);
  if(format==='HEX') return `0x${raw.toString(16).toUpperCase()}`;
  if(format==='OCT') return `0o${raw.toString(8)}`;
  if(format==='BIN') return `0b${raw.toString(2)}`;
  if(format==='REL') return (raw / scaleFactor).toFixed(scale);
  if(format==='SDC') return (raw / scaleFactor).toFixed(scale);
  if(format==='SWT') return raw ? '1' : '0';
  return (raw / scaleFactor).toFixed(scale);
}

function parseValue(text, format, scale){
  if(format==='HEX') return parseInt(text, 16);
  if(format==='OCT') return parseInt(text, 8);
  if(format==='BIN') return parseInt(text, 2);
  if(format==='SWT') return text === '1' || text.toLowerCase() === 'true' ? 1 : 0;
  const scaleFactor = Math.pow(10, scale);
  const num = Number(text);
  return Math.round(num * scaleFactor);
}

function writeRegisterValue(regId, valueText){
  const meta = registerIndex.get(regId);
  if(!meta) return;
  const raw = parseValue(valueText, meta.format || 'DEC', meta.scale || 0);
  fetch(`/api/write?reg=${regId}&val=${raw}`)
    .then(r=>r.json())
    .then(() => refreshVisibleValues());
}

function getVisibleRegIds(){
  const ids = new Set();
  menuRoot.querySelectorAll('[data-reg-id]').forEach(el => {
    ids.add(Number(el.dataset.regId));
  });
  return Array.from(ids);
}

function refreshVisibleValues(){
  const ids = getVisibleRegIds();
  ids.forEach(regId => {
    fetch(`/api/read?reg=${regId}`)
      .then(r=>r.json())
      .then(d => {
        const meta = registerIndex.get(regId);
        const formatted = meta ? formatValue(d.value, meta.format || 'DEC', meta.scale || 0) : d.value;
        menuRoot.querySelectorAll(`.menu-value[data-reg-id="${regId}"]`)
          .forEach(span => span.textContent = formatted);
      });
  });
}

renderMenuPage();
setInterval(refreshVisibleValues, 2000);
</script>
</body></html>
)rawliteral";

/* ================= API ================= */
void handleRoot() {
  server.send_P(200, "text/html", MAIN_HTML);
}

void handleRead() {
  uint16_t reg = server.arg("reg").toInt();
  uint16_t val = 0;
  bool ok = mtzpRead(reg, val);
  StaticJsonDocument<64> doc;
  doc["reg"] = reg;
  doc["value"] = val;
  doc["ok"] = ok;
  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

void handleWrite() {
  uint16_t reg = server.arg("reg").toInt();
  uint16_t val = server.arg("val").toInt();
  bool ok = mtzpWrite(reg, val);
  StaticJsonDocument<64> doc;
  doc["reg"] = reg;
  doc["value"] = val;
  doc["ok"] = ok;
  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

/* ================= SETUP / LOOP ================= */
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/api/read", handleRead);
  server.on("/api/write", handleWrite);
  server.begin();
}

void loop() {
  server.handleClient();
}
