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
const registerMax = 276;
function fillRegisters(selectEl){
  for(let i=0;i<=registerMax;i++){
    const opt=document.createElement('option');
    opt.value=i;
    opt.textContent=`${i}`;
    selectEl.appendChild(opt);
  }
}
fillRegisters(rreg);
fillRegisters(wreg);

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
