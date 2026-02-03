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
</style></head>
<body>
<h2>MTZP Web Panel</h2>
<div class=card>
<h3>Read register</h3>
<label>Register</label>
<select id=rreg></select>
<label>Format</label>
<select id=rformat>
  <option value="DEC">DEC</option>
  <option value="HEX">HEX</option>
  <option value="OCT">OCT</option>
  <option value="BIN">BIN</option>
  <option value="SDC">SDC</option>
  <option value="REL">REL</option>
  <option value="SWT">SWT</option>
</select>
<label>Scale (decimal digits)</label>
<input id=rscale type=number value="0" min="0" max="4">
<button onclick=readReg()>Read</button>
<pre id=rout></pre>
</div>
<div class=card>
<h3>Write register</h3>
<label>Register</label>
<select id=wreg></select>
<label>Format</label>
<select id=wformat>
  <option value="DEC">DEC</option>
  <option value="HEX">HEX</option>
  <option value="OCT">OCT</option>
  <option value="BIN">BIN</option>
  <option value="SDC">SDC</option>
  <option value="REL">REL</option>
  <option value="SWT">SWT</option>
</select>
<label>Scale (decimal digits)</label>
<input id=wscale type=number value="0" min="0" max="4">
<label>Value</label>
<input id=wval placeholder="Value">
<button onclick=writeReg()>Write</button>
<pre id=wout></pre>
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

function fillRegisters(selectEl){
  registerTable.forEach(item => {
    const opt=document.createElement('option');
    opt.value=item.id;
    opt.textContent=`${item.id} — ${item.name}`;
    selectEl.appendChild(opt);
  });
}
fillRegisters(rreg);
fillRegisters(wreg);

function applyRegisterDefaults(selectEl, formatEl, scaleEl){
  const reg = Number(selectEl.value);
  const meta = registerIndex.get(reg);
  if (!meta) return;
  formatEl.value = meta.format || 'DEC';
  scaleEl.value = typeof meta.scale === 'number' ? meta.scale : 0;
}

rreg.addEventListener('change', () => applyRegisterDefaults(rreg, rformat, rscale));
wreg.addEventListener('change', () => applyRegisterDefaults(wreg, wformat, wscale));
applyRegisterDefaults(rreg, rformat, rscale);
applyRegisterDefaults(wreg, wformat, wscale);

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

function readReg(){
 const reg=rreg.value;
 const format=rformat.value;
 const scale=parseInt(rscale.value||'0',10);
 fetch(`/api/read?reg=${reg}`)
 .then(r=>r.json()).then(d=>{
   const formatted = formatValue(d.value, format, scale);
   rout.innerText=JSON.stringify({...d, formatted},null,2);
 });
}
function writeReg(){
 const reg=wreg.value;
 const format=wformat.value;
 const scale=parseInt(wscale.value||'0',10);
 const raw = parseValue(wval.value, format, scale);
 fetch(`/api/write?reg=${reg}&val=${raw}`)
 .then(r=>r.json()).then(d=>wout.innerText=JSON.stringify(d,null,2));
}
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
