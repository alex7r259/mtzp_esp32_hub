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
const char MAIN_HTML[] PROGMEM = R"MTZP_HTML(
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>МТЗП</title>
    <style>
        :root {
            --primary: #2196f3;
            --primary-dark: #1976d2;
            --secondary: #03a9f4;
            --success: #4caf50;
            --warning: #ff9800;
            --danger: #f44336;
            --light: #f5f7fa;
            --gray: #e0e0e0;
            --dark-gray: #757575;
            --text: #333333;
            --border: #d1d1d1;
            --card-bg: #ffffff;
            --header-bg: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            -webkit-tap-highlight-color: transparent;
        }

        html {
            font-size: 14px;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, sans-serif;
            background: var(--light);
            color: var(--text);
            line-height: 1.4;
            padding: 0;
            margin: 0;
            min-height: 100vh;
        }

        /* Header */
        .header {
            background: var(--header-bg);
            color: white;
            padding: 12px 16px;
            position: sticky;
            top: 0;
            z-index: 100;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }

        .header-top {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
        }

        .header-title {
            font-size: 1.2rem;
            font-weight: 600;
        }

        .header-subtitle {
            font-size: 0.85rem;
            opacity: 0.9;
        }

        .status-badge {
            display: inline-flex;
            align-items: center;
            gap: 6px;
            padding: 4px 10px;
            background: rgba(255,255,255,0.2);
            border-radius: 12px;
            font-size: 0.8rem;
            margin-top: 6px;
        }

        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: var(--success);
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 0.6; }
            50% { opacity: 1; }
        }

        /* Bottom Navigation */
        .bottom-nav {
            position: fixed;
            bottom: 0;
            left: 0;
            right: 0;
            background: white;
            display: flex;
            justify-content: space-around;
            padding: 8px 0;
            border-top: 1px solid var(--border);
            z-index: 100;
            box-shadow: 0 -2px 10px rgba(0,0,0,0.05);
        }

        .nav-item {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 4px;
            padding: 6px 12px;
            color: var(--dark-gray);
            text-decoration: none;
            font-size: 0.75rem;
            flex: 1;
            max-width: 80px;
            border-radius: 8px;
            transition: all 0.2s;
        }

        .nav-item.active {
            color: var(--primary);
            background: rgba(33, 150, 243, 0.08);
        }

        .nav-icon {
            font-size: 1.2rem;
            width: 24px;
            height: 24px;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        /* Main Content */
        .main-content {
            padding: 16px;
            padding-bottom: 70px;
            max-width: 100%;
            overflow-x: hidden;
        }

        /* Quick Actions */
        .quick-actions {
            display: flex;
            gap: 8px;
            margin-bottom: 16px;
            flex-wrap: nowrap;
            overflow-x: auto;
            -webkit-overflow-scrolling: touch;
            padding: 4px 0;
        }

        .quick-action {
            flex: 0 0 auto;
            display: flex;
            align-items: center;
            gap: 6px;
            padding: 8px 12px;
            background: white;
            border: 1px solid var(--border);
            border-radius: 8px;
            font-size: 0.85rem;
            white-space: nowrap;
        }

        .quick-action.primary {
            background: var(--primary);
            color: white;
            border-color: var(--primary);
        }

        /* Cards */
        .section-title {
            font-size: 1rem;
            font-weight: 600;
            margin: 20px 0 12px 0;
            padding-left: 8px;
            border-left: 3px solid var(--primary);
            color: var(--primary-dark);
        }

        .register-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(100%, 1fr));
            gap: 10px;
            margin-bottom: 20px;
        }

        .register-card {
            background: var(--card-bg);
            border-radius: 10px;
            padding: 14px;
            border: 1px solid var(--border);
            position: relative;
            transition: transform 0.2s;
        }

        .register-card:active {
            transform: scale(0.98);
        }

        .card-header {
            display: flex;
            justify-content: space-between;
            align-items: flex-start;
            margin-bottom: 10px;
        }

        .register-name {
            font-size: 0.9rem;
            font-weight: 500;
            color: var(--text);
            flex: 1;
            padding-right: 8px;
        }

        .register-id {
            font-size: 0.75rem;
            color: var(--dark-gray);
            background: var(--light);
            padding: 2px 8px;
            border-radius: 10px;
            white-space: nowrap;
        }

        .value-row {
            display: flex;
            align-items: center;
            justify-content: space-between;
            margin: 8px 0;
        }

        .value-display {
            font-size: 1.6rem;
            font-weight: 700;
            color: var(--primary-dark);
            flex: 1;
        }

        .value-unit {
            font-size: 0.85rem;
            color: var(--dark-gray);
            margin-left: 4px;
        }

        .card-footer {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-top: 10px;
            padding-top: 10px;
            border-top: 1px solid var(--border);
            font-size: 0.8rem;
        }

        .register-format {
            color: var(--dark-gray);
        }

        .register-status {
            display: flex;
            align-items: center;
            gap: 4px;
        }

        .status-dot.small {
            width: 6px;
            height: 6px;
        }

        /* Alerts */
        .alert {
            padding: 10px 14px;
            border-radius: 8px;
            margin: 10px 0;
            display: flex;
            align-items: center;
            gap: 10px;
            font-size: 0.85rem;
            animation: slideIn 0.3s ease;
        }

        @keyframes slideIn {
            from {
                transform: translateY(-10px);
                opacity: 0;
            }
            to {
                transform: translateY(0);
                opacity: 1;
            }
        }

        .alert.success {
            background: #e8f5e9;
            color: #2e7d32;
            border-left: 3px solid #4caf50;
        }

        .alert.error {
            background: #ffebee;
            color: #c62828;
            border-left: 3px solid #f44336;
        }

        /* Modal */
        .modal-overlay {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: rgba(0,0,0,0.5);
            display: flex;
            align-items: center;
            justify-content: center;
            z-index: 1000;
            padding: 16px;
            opacity: 0;
            visibility: hidden;
            transition: all 0.3s;
            -webkit-backdrop-filter: blur(4px);
            backdrop-filter: blur(4px);
        }

        .modal-overlay.active {
            opacity: 1;
            visibility: visible;
        }

        .modal {
            background: white;
            border-radius: 12px;
            width: 100%;
            max-width: 400px;
            max-height: 80vh;
            overflow-y: auto;
            transform: translateY(20px);
            transition: transform 0.3s;
        }

        .modal-overlay.active .modal {
            transform: translateY(0);
        }

        .modal-header {
            padding: 16px;
            border-bottom: 1px solid var(--border);
            display: flex;
            justify-content: space-between;
            align-items: center;
            position: sticky;
            top: 0;
            background: white;
            border-radius: 12px 12px 0 0;
        }

        .modal-title {
            font-size: 1.1rem;
            font-weight: 600;
        }

        .modal-close {
            background: none;
            border: none;
            font-size: 1.5rem;
            color: var(--dark-gray);
            padding: 0;
            width: 32px;
            height: 32px;
            display: flex;
            align-items: center;
            justify-content: center;
            border-radius: 50%;
        }

        .modal-close:active {
            background: var(--gray);
        }

        .modal-body {
            padding: 16px;
        }

        .modal-section {
            margin-bottom: 20px;
        }

        .modal-label {
            display: block;
            margin-bottom: 6px;
            color: var(--dark-gray);
            font-size: 0.85rem;
        }

        .modal-value {
            font-size: 1.4rem;
            font-weight: 600;
            color: var(--primary-dark);
            padding: 10px;
            background: var(--light);
            border-radius: 8px;
            text-align: center;
            margin-bottom: 10px;
        }

        .form-group {
            margin-bottom: 16px;
        }

        .form-input {
            width: 100%;
            padding: 12px;
            border: 2px solid var(--border);
            border-radius: 8px;
            font-size: 1rem;
            transition: border-color 0.2s;
        }

        .form-input:focus {
            outline: none;
            border-color: var(--primary);
        }

        .form-actions {
            display: flex;
            gap: 10px;
            margin-top: 20px;
        }

        .btn {
            flex: 1;
            padding: 12px;
            border: none;
            border-radius: 8px;
            font-size: 0.9rem;
            font-weight: 500;
            cursor: pointer;
            transition: all 0.2s;
        }

        .btn:active {
            transform: scale(0.98);
        }

        .btn-primary {
            background: var(--primary);
            color: white;
        }

        .btn-secondary {
            background: var(--gray);
            color: var(--text);
        }

        /* Loading */
        .loading {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
            color: var(--dark-gray);
        }

        .spinner {
            width: 16px;
            height: 16px;
            border: 2px solid var(--border);
            border-top-color: var(--primary);
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }

        @keyframes spin {
            to { transform: rotate(360deg); }
        }

        /* Responsive */
        @media (min-width: 360px) {
            .register-grid {
                grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
            }
        }

        @media (min-width: 768px) {
            .register-grid {
                grid-template-columns: repeat(auto-fill, minmax(200px, 1fr));
            }
            
            .bottom-nav {
                display: none;
            }
            
            .main-content {
                padding-bottom: 20px;
            }
        }

        .no-select {
            -webkit-touch-callout: none;
            -webkit-user-select: none;
            -khtml-user-select: none;
            -moz-user-select: none;
            -ms-user-select: none;
            user-select: none;
        }
    </style>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css">
</head>
<body class="no-select">
    <div class="header">
        <div class="header-top">
            <div class="header-title">МТЗП-2T</div>
            <div class="status-badge">
                <div class="status-dot"></div>
                <span>Подключено</span>
            </div>
        </div>
        <div class="header-subtitle">Панель управления защитой</div>
    </div>

    <div class="main-content" id="content">
        <div id="alerts"></div>

        <div class="quick-actions">
            <button class="quick-action" id="btn-refresh">
                <i class="fas fa-sync-alt"></i>
                <span>Обновить</span>
            </button>
            <button class="quick-action" id="btn-save">
                <i class="fas fa-save"></i>
                <span>Сохранить</span>
            </button>
            <button class="quick-action" id="btn-export">
                <i class="fas fa-download"></i>
                <span>Экспорт</span>
            </button>
            <button class="quick-action" id="btn-import">
                <i class="fas fa-upload"></i>
                <span>Импорт</span>
            </button>
            <button class="quick-action primary" id="btn-scan">
                <i class="fas fa-search"></i>
                <span>Сканировать</span>
            </button>
            <input type="file" id="import-file" accept="application/json" style="display:none" />
        </div>

        <div id="content-area">
            <div class="loading" style="padding: 40px; text-align: center;">
                <div class="spinner"></div>
                <span>Загрузка...</span>
            </div>
        </div>
    </div>

    <div class="bottom-nav">
        <a href="#" class="nav-item active" data-tab="dashboard">
            <div class="nav-icon">
                <i class="fas fa-tachometer-alt"></i>
            </div>
            <span>Прибор</span>
        </a>
        <a href="#" class="nav-item" data-tab="protection">
            <div class="nav-icon">
                <i class="fas fa-shield-alt"></i>
            </div>
            <span>Защиты</span>
        </a>
        <a href="#" class="nav-item" data-tab="signals">
            <div class="nav-icon">
                <i class="fas fa-bell"></i>
            </div>
            <span>Сигналы</span>
        </a>
        <a href="#" class="nav-item" data-tab="settings">
            <div class="nav-icon">
                <i class="fas fa-cog"></i>
            </div>
            <span>Настройки</span>
        </a>
        <a href="#" class="nav-item" data-tab="info">
            <div class="nav-icon">
                <i class="fas fa-info-circle"></i>
            </div>
            <span>Инфо</span>
        </a>
        <a href="#" class="nav-item" data-tab="all">
            <div class="nav-icon">
                <i class="fas fa-list"></i>
            </div>
            <span>Все</span>
        </a>
    </div>

    <div class="modal-overlay" id="editModal">
        <div class="modal">
            <div class="modal-header">
                <div class="modal-title" id="modalTitle">Редактирование</div>
                <button class="modal-close" id="modalClose">&times;</button>
            </div>
            <div class="modal-body">
                <div class="modal-section">
                    <div class="modal-label">Текущее значение</div>
                    <div class="modal-value" id="currentValue">--</div>
                </div>
                <div class="form-group">
                    <label class="modal-label" id="newValueLabel">Новое значение</label>
                    <input type="text" class="form-input" id="newValue" placeholder="Введите значение">
                </div>
                <div class="form-actions">
                    <button class="btn btn-secondary" id="btnCancel">Отмена</button>
                    <button class="btn btn-primary" id="btnSave">Сохранить</button>
                </div>
            </div>
        </div>
    </div>

    <div class="modal-overlay" id="infoModal">
        <div class="modal">
            <div class="modal-header">
                <div class="modal-title">Информация</div>
                <button class="modal-close" id="infoModalClose">&times;</button>
            </div>
            <div class="modal-body">
                <div class="modal-section">
                    <div class="modal-label">Название</div>
                    <div style="font-weight: 500; margin-bottom: 10px;" id="infoName">--</div>
                    <div class="modal-label">ID регистра</div>
                    <div style="font-family: monospace; font-size: 1.1rem; margin-bottom: 10px;" id="infoId">--</div>
                    <div class="modal-label">Формат</div>
                    <div style="margin-bottom: 10px;" id="infoFormat">--</div>
                    <div class="modal-label">Масштаб</div>
                    <div style="margin-bottom: 10px;" id="infoScale">--</div>
                    <div class="modal-label">Текущее значение</div>
                    <div class="modal-value" id="infoValue">--</div>
                </div>
            </div>
        </div>
    </div>

    <script>
        const CONFIG = {
            refreshInterval: 5000,
            deviceAddress: 0x01,
            editableRegisters: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 117, 118, 119, 120, 121, 124, 125, 128, 129, 130, 133, 134, 135, 138, 139, 140, 143, 144, 145, 146, 147, 148, 151, 152, 153, 156, 157, 158, 161, 162, 163, 166, 167, 168, 169, 170, 174, 175, 176, 177, 178, 181, 182, 183, 184, 187, 188, 189, 190, 197, 198, 201, 202, 203, 204, 207, 208, 209, 212, 216, 217, 218, 219, 252]
        };

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

        const tabContent = {
            dashboard: {
                title: "Текущие параметры",
                groups: [
                    { title: "Напряжения", registers: [33, 34, 35, 36, 37] },
                    { title: "Токи", registers: [41, 42, 43, 44, 45, 46, 47, 48] },
                    { title: "Мощность", registers: [54, 55, 56, 57, 58, 59, 80, 81, 82, 83] },
                    { title: "Датчики", registers: [32, 60, 61, 62, 63, 64, 65, 66] },
                    { title: "Наработка", registers: [92, 93, 94, 95, 96, 97, 98, 99] }
                ]
            },
            protection: {
                title: "Настройки защит",
                groups: [
                    { title: "МТЗ-1", registers: [115, 117, 118, 119, 120, 121] },
                    { title: "МТЗ-2", registers: [122, 124, 125] },
                    { title: "МТЗ-3", registers: [126, 128, 129, 130] },
                    { title: "УМТЗ", registers: [131, 133, 134, 135] },
                    { title: "ЗМТ", registers: [136, 138, 139, 140] },
                    { title: "ЗМН", registers: [141, 143, 144, 145, 146, 147, 148] },
                    { title: "ЗНФ", registers: [149, 151, 152, 153] },
                    { title: "ЗОФ", registers: [154, 156, 157, 158] },
                    { title: "БКИ", registers: [159, 161, 162, 163] },
                    { title: "НЗЗ", registers: [164, 166, 167, 168, 169, 170] },
                    { title: "EXT", registers: [171, 172, 173, 174, 175, 176, 177, 178] },
                    { title: "INT", registers: [179, 181, 182, 183, 184] },
                    { title: "УРОВ", registers: [195, 197, 198] }
                ]
            },
            signals: {
                title: "Состояние сигналов",
                groups: [
                    { title: "Реле", registers: [23] },
                    { title: "Внешние входы", registers: [24, 25] },
                    { title: "Внутренние входы", registers: [26, 27] },
                    { title: "Аварии/Неиспр.", registers: [28, 29, 30, 31] }
                ]
            },
            settings: {
                title: "Настройки системы",
                groups: [
                    { title: "RS485", registers: [1, 2, 3, 5, 6] },
                    { title: "RTC", registers: [16, 17, 18, 19, 20, 21, 22] },
                    { title: "Управление", registers: [213, 214, 215, 216, 217, 218, 219] },
                    { title: "КА", registers: [188, 189, 190, 193, 194] },
                    { title: "АПВ/АВР", registers: [205, 207, 208, 209, 210, 211, 212] },
                    { title: "Конфигурация", registers: [220, 221, 222, 223, 224, 225, 226, 227, 228, 229] },
                    { title: "Диагностика", registers: [199, 200, 201, 202, 203, 204] },
                    { title: "Сервис", registers: [252, 253, 254] }
                ]
            },
            info: {
                title: "Информация",
                groups: [
                    { title: "Изделие", registers: [10, 11] },
                    { title: "Состояние", registers: [12, 13, 14, 15] },
                    { title: "Статистика", registers: [76, 77, 78, 79, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114] }
                ]
            },
            all: {
                title: "Все параметры",
                groups: [
                    { title: "Все регистры", registers: registerTable.map(item => item.id) }
                ]
            }
        };

        let currentTab = 'dashboard';
        let currentRegister = null;
        let valuesCache = new Map();
        let refreshTimer = null;

        const contentArea = document.getElementById('content-area');
        const navItems = document.querySelectorAll('.nav-item');
        const editModal = document.getElementById('editModal');
        const infoModal = document.getElementById('infoModal');
        const alertsContainer = document.getElementById('alerts');
        const importFile = document.getElementById('import-file');

        function formatValue(value, format, scale) {
            if (value === undefined || value === null) return '--';
            const numValue = Number(value);
            if (isNaN(numValue)) return '--';
            switch(format) {
                case 'HEX': return '0x' + numValue.toString(16).toUpperCase().padStart(4, '0');
                case 'OCT': return '0o' + numValue.toString(8);
                case 'BIN': return '0b' + numValue.toString(2).padStart(16, '0');
                case 'SDC':
                case 'REL':
                    return (numValue / Math.pow(10, scale)).toFixed(scale);
                case 'SWT':
                    return numValue ? 'ВКЛ' : 'ВЫКЛ';
                default:
                    return (numValue / Math.pow(10, scale)).toFixed(scale);
            }
        }

        function parseValue(value, format, scale) {
            const trimmed = value.trim();
            const scaleFactor = Math.pow(10, scale);
            switch(format) {
                case 'HEX': return parseInt(trimmed.replace(/^0x/i, ''), 16);
                case 'OCT': return parseInt(trimmed.replace(/^0o/i, ''), 8);
                case 'BIN': return parseInt(trimmed.replace(/^0b/i, ''), 2);
                case 'SWT': return ['1', 'вкл', 'true', 'on', 'да'].includes(trimmed.toLowerCase()) ? 1 : 0;
                case 'SDC':
                case 'REL':
                default:
                    return Math.round(parseFloat(trimmed) * scaleFactor);
            }
        }

        function createRegisterCard(regId) {
            const reg = registerIndex.get(regId);
            if (!reg) return '';
            const isEditable = CONFIG.editableRegisters.includes(regId);
            const cachedValue = valuesCache.get(regId);
            const unit = reg.unit || '';
            return `
                <div class="register-card" data-reg-id="${regId}" onclick="${isEditable ? 'openEditModal(' + regId + ')' : 'openInfoModal(' + regId + ')'}">
                    <div class="card-header">
                        <div class="register-name">${reg.name}</div>
                        <div class="register-id">#${reg.id}</div>
                    </div>
                    <div class="value-row">
                        <div class="value-display" id="value-${regId}">
                            ${cachedValue !== undefined ? formatValue(cachedValue, reg.format, reg.scale) : '<span class="loading"><div class="spinner"></div></span>'}
                        </div>
                        <div class="value-unit">${unit}</div>
                    </div>
                    <div class="card-footer">
                        <div class="register-format">${reg.format}</div>
                        <div class="register-status">
                            <div class="status-dot small" style="background:${cachedValue !== undefined ? '#4caf50' : '#ff9800'}"></div>
                            <span>${cachedValue !== undefined ? 'акт.' : 'загр.'}</span>
                        </div>
                    </div>
                </div>
            `;
        }

        function loadTab(tabId) {
            const tab = tabContent[tabId];
            if (!tab) return;
            currentTab = tabId;
            let html = '';
            tab.groups.forEach(group => {
                html += `<div class="section-title">${group.title}</div>`;
                html += '<div class="register-grid">';
                group.registers.forEach(regId => {
                    html += createRegisterCard(regId);
                });
                html += '</div>';
            });
            contentArea.innerHTML = html;
            navItems.forEach(item => {
                item.classList.toggle('active', item.dataset.tab === tabId);
            });
            refreshTabValues();
        }

        async function refreshTabValues() {
            const tab = tabContent[currentTab];
            if (!tab) return;
            const regIds = [];
            tab.groups.forEach(group => regIds.push(...group.registers));
            for (const regId of regIds) {
                await updateRegisterValue(regId);
            }
        }

        async function updateRegisterValue(regId) {
            try {
                const response = await fetch(`/api/read?reg=${regId}`);
                const data = await response.json();
                if (data.ok) {
                    valuesCache.set(regId, data.value);
                    const reg = registerIndex.get(regId);
                    const valueElement = document.getElementById(`value-${regId}`);
                    if (valueElement) {
                        valueElement.textContent = formatValue(data.value, reg?.format || 'DEC', reg?.scale || 0);
                        const card = valueElement.closest('.register-card');
                        if (card) {
                            const statusDot = card.querySelector('.status-dot.small');
                            if (statusDot) {
                                statusDot.style.background = '#4caf50';
                                statusDot.parentElement.querySelector('span').textContent = 'акт.';
                            }
                        }
                    }
                }
            } catch (error) {
                console.error(`Network error for register ${regId}:`, error);
            }
        }

        function openEditModal(regId) {
            const reg = registerIndex.get(regId);
            if (!reg || !CONFIG.editableRegisters.includes(regId)) return;
            currentRegister = regId;
            const currentVal = valuesCache.get(regId);
            document.getElementById('modalTitle').textContent = reg.name;
            document.getElementById('currentValue').textContent = formatValue(currentVal, reg.format, reg.scale) + (reg.unit ? ' ' + reg.unit : '');
            document.getElementById('newValueLabel').textContent = `Новое значение (${reg.unit || '-'})`;
            document.getElementById('newValue').value = currentVal !== undefined ? (currentVal / Math.pow(10, reg.scale)) : '';
            editModal.classList.add('active');
            document.getElementById('newValue').focus();
        }

        function openInfoModal(regId) {
            const reg = registerIndex.get(regId);
            if (!reg) return;
            const currentVal = valuesCache.get(regId);
            document.getElementById('infoName').textContent = reg.name;
            document.getElementById('infoId').textContent = '#' + reg.id;
            document.getElementById('infoFormat').textContent = reg.format;
            document.getElementById('infoScale').textContent = reg.scale;
            document.getElementById('infoValue').textContent = formatValue(currentVal, reg.format, reg.scale) + (reg.unit ? ' ' + reg.unit : '');
            infoModal.classList.add('active');
        }

        async function saveEditedValue() {
            if (!currentRegister) return;
            const input = document.getElementById('newValue');
            const value = input.value.trim();
            const reg = registerIndex.get(currentRegister);
            if (!reg || !value) return;
            try {
                const scaledValue = parseValue(value, reg.format, reg.scale);
                const response = await fetch(`/api/write?reg=${currentRegister}&val=${scaledValue}`, { method: 'POST' });
                const result = await response.json();
                if (result.ok) {
                    showAlert('Значение сохранено', 'success');
                    valuesCache.set(currentRegister, scaledValue);
                    const valueElement = document.getElementById(`value-${currentRegister}`);
                    if (valueElement) {
                        valueElement.textContent = formatValue(scaledValue, reg.format, reg.scale);
                    }
                    closeEditModal();
                } else {
                    showAlert('Ошибка сохранения', 'error');
                }
            } catch (error) {
                console.error('Error saving value:', error);
                showAlert('Ошибка сети', 'error');
            }
        }

        function closeEditModal() {
            editModal.classList.remove('active');
            currentRegister = null;
        }

        function closeInfoModal() {
            infoModal.classList.remove('active');
        }

        function showAlert(message, type) {
            const alert = document.createElement('div');
            alert.className = `alert ${type}`;
            alert.innerHTML = `
                <i class="fas fa-${type === 'success' ? 'check-circle' : 'exclamation-circle'}"></i>
                <span>${message}</span>
                <button onclick="this.parentElement.remove()" style="margin-left: auto; background: none; border: none; color: inherit;">
                    <i class="fas fa-times"></i>
                </button>
            `;
            alertsContainer.appendChild(alert);
            setTimeout(() => {
                if (alert.parentElement) {
                    alert.remove();
                }
            }, 3000);
        }

        function refreshAll() {
            showAlert('Обновление данных...', 'success');
            refreshTabValues();
        }

        function saveSettings() {
            fetch('/api/write?reg=0&val=1')
                .then(response => response.json())
                .then(data => {
                    if (data.ok) {
                        showAlert('Настройки сохранены', 'success');
                    } else {
                        showAlert('Ошибка сохранения', 'error');
                    }
                })
                .catch(() => {
                    showAlert('Ошибка сети', 'error');
                });
        }

        async function exportSettings() {
            showAlert('Экспорт настроек...', 'success');
            const data = {};
            for (const reg of registerTable) {
                try {
                    const response = await fetch(`/api/read?reg=${reg.id}`);
                    const payload = await response.json();
                    if (payload.ok) {
                        data[reg.id] = payload.value;
                    }
                } catch (error) {
                    console.error('Export error', reg.id, error);
                }
            }
            const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            const link = document.createElement('a');
            link.href = url;
            link.download = 'mtzp_settings.json';
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(url);
            showAlert('Экспорт завершен', 'success');
        }

        async function importSettings(file) {
            try {
                const text = await file.text();
                const data = JSON.parse(text);
                const entries = Object.entries(data);
                for (const [regId, value] of entries) {
                    await fetch(`/api/write?reg=${regId}&val=${value}`, { method: 'POST' });
                }
                showAlert('Импорт завершен', 'success');
                refreshTabValues();
            } catch (error) {
                console.error('Import error', error);
                showAlert('Ошибка импорта', 'error');
            }
        }

        function init() {
            loadTab('dashboard');
            navItems.forEach(item => {
                item.addEventListener('click', (e) => {
                    e.preventDefault();
                    loadTab(item.dataset.tab);
                });
            });
            document.getElementById('btn-refresh').addEventListener('click', refreshAll);
            document.getElementById('btn-save').addEventListener('click', saveSettings);
            document.getElementById('btn-scan').addEventListener('click', () => {
                showAlert('Сканирование начато...', 'success');
            });
            document.getElementById('btn-export').addEventListener('click', exportSettings);
            document.getElementById('btn-import').addEventListener('click', () => importFile.click());
            importFile.addEventListener('change', () => {
                if (importFile.files.length) {
                    importSettings(importFile.files[0]);
                    importFile.value = '';
                }
            });
            document.getElementById('modalClose').addEventListener('click', closeEditModal);
            document.getElementById('infoModalClose').addEventListener('click', closeInfoModal);
            document.getElementById('btnCancel').addEventListener('click', closeEditModal);
            document.getElementById('btnSave').addEventListener('click', saveEditedValue);
            document.getElementById('newValue').addEventListener('keypress', (e) => {
                if (e.key === 'Enter') {
                    saveEditedValue();
                }
            });
            editModal.addEventListener('click', (e) => {
                if (e.target === editModal) closeEditModal();
            });
            infoModal.addEventListener('click', (e) => {
                if (e.target === infoModal) closeInfoModal();
            });
            refreshTimer = setInterval(refreshTabValues, CONFIG.refreshInterval);
            refreshTabValues();
        }

        document.addEventListener('DOMContentLoaded', init);
    </script>
</body>
</html>
)MTZP_HTML";

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
