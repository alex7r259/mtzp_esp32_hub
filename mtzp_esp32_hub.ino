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
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>МТЗП-2T | Панель управления</title>
    <style>
        :root {
            --primary: #2196f3;
            --primary-dark: #1976d2;
            --secondary: #03a9f4;
            --success: #4caf50;
            --warning: #ff9800;
            --danger: #f44336;
            --dark: #1a237e;
            --light: #e3f2fd;
            --gray: #f5f5f5;
            --dark-gray: #424242;
            --text: #212121;
            --text-light: #757575;
            --border: #e0e0e0;
            --shadow: 0 2px 10px rgba(0,0,0,0.1);
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: var(--text);
            min-height: 100vh;
            padding: 20px;
        }

        .container {
            max-width: 1400px;
            margin: 0 auto;
            background: white;
            border-radius: 20px;
            box-shadow: var(--shadow);
            overflow: hidden;
        }

        /* Header */
        .header {
            background: linear-gradient(to right, var(--primary), var(--primary-dark));
            color: white;
            padding: 25px 30px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            position: relative;
            overflow: hidden;
        }

        .header::before {
            content: '';
            position: absolute;
            top: -50%;
            right: -50%;
            width: 200%;
            height: 200%;
            background: radial-gradient(circle, rgba(255,255,255,0.1) 1px, transparent 1px);
            background-size: 30px 30px;
            opacity: 0.1;
        }

        .header-content h1 {
            font-size: 28px;
            font-weight: 300;
            margin-bottom: 5px;
        }

        .header-content .subtitle {
            font-size: 14px;
            opacity: 0.9;
        }

        .status-badge {
            display: inline-block;
            padding: 8px 16px;
            background: rgba(255,255,255,0.2);
            border-radius: 20px;
            font-size: 14px;
            margin-top: 10px;
        }

        .connection-status {
            display: flex;
            align-items: center;
            gap: 10px;
            background: rgba(255,255,255,0.1);
            padding: 10px 20px;
            border-radius: 10px;
            backdrop-filter: blur(10px);
        }

        .status-indicator {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: var(--success);
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0% { opacity: 0.5; }
            50% { opacity: 1; }
            100% { opacity: 0.5; }
        }

        /* Main Layout */
        .main-layout {
            display: flex;
            min-height: 800px;
        }

        /* Sidebar */
        .sidebar {
            width: 280px;
            background: var(--gray);
            border-right: 1px solid var(--border);
            padding: 25px 0;
        }

        .nav-section {
            padding: 0 20px 20px;
            border-bottom: 1px solid var(--border);
            margin-bottom: 20px;
        }

        .nav-section h3 {
            color: var(--primary-dark);
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 15px;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .nav-section h3 i {
            font-size: 16px;
        }

        .nav-item {
            display: flex;
            align-items: center;
            padding: 12px 15px;
            margin: 5px 0;
            color: var(--text);
            text-decoration: none;
            border-radius: 10px;
            transition: all 0.3s ease;
            cursor: pointer;
            gap: 12px;
        }

        .nav-item:hover {
            background: var(--primary);
            color: white;
            transform: translateX(5px);
        }

        .nav-item.active {
            background: linear-gradient(to right, var(--primary), var(--secondary));
            color: white;
            box-shadow: 0 4px 12px rgba(33, 150, 243, 0.3);
        }

        .nav-item i {
            font-size: 18px;
            width: 24px;
            text-align: center;
        }

        .quick-actions {
            padding: 0 20px;
        }

        .action-btn {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
            width: 100%;
            padding: 12px;
            margin: 8px 0;
            background: white;
            border: 2px solid var(--border);
            border-radius: 10px;
            color: var(--text);
            font-weight: 500;
            transition: all 0.3s ease;
            cursor: pointer;
        }

        .action-btn:hover {
            border-color: var(--primary);
            color: var(--primary);
            transform: translateY(-2px);
        }

        .action-btn.primary {
            background: var(--primary);
            border-color: var(--primary);
            color: white;
        }

        .action-btn.primary:hover {
            background: var(--primary-dark);
            transform: translateY(-2px);
        }

        /* Main Content */
        .main-content {
            flex: 1;
            padding: 30px;
            background: white;
        }

        .content-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 30px;
            padding-bottom: 20px;
            border-bottom: 2px solid var(--border);
        }

        .content-header h2 {
            color: var(--primary-dark);
            font-size: 24px;
            font-weight: 600;
        }

        .refresh-btn {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 10px 20px;
            background: var(--light);
            border: none;
            border-radius: 10px;
            color: var(--primary);
            font-weight: 500;
            cursor: pointer;
            transition: all 0.3s ease;
        }

        .refresh-btn:hover {
            background: var(--primary);
            color: white;
        }

        /* Register Cards */
        .register-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(350px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }

        .register-card {
            background: white;
            border-radius: 15px;
            padding: 20px;
            border: 1px solid var(--border);
            transition: all 0.3s ease;
            position: relative;
            overflow: hidden;
        }

        .register-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 10px 25px rgba(0,0,0,0.1);
            border-color: var(--primary);
        }

        .register-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            width: 5px;
            height: 100%;
            background: var(--primary);
        }

        .card-header {
            display: flex;
            justify-content: space-between;
            align-items: flex-start;
            margin-bottom: 15px;
        }

        .register-name {
            font-size: 16px;
            font-weight: 600;
            color: var(--text);
            line-height: 1.4;
        }

        .register-id {
            background: var(--light);
            color: var(--primary);
            padding: 4px 12px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: 600;
        }

        .value-display {
            font-size: 32px;
            font-weight: 700;
            color: var(--primary-dark);
            text-align: center;
            padding: 20px;
            margin: 10px 0;
            background: linear-gradient(to right, #f8f9fa, #e9ecef);
            border-radius: 10px;
            border: 2px solid var(--border);
        }

        .unit {
            font-size: 14px;
            color: var(--text-light);
            margin-left: 5px;
        }

        .card-footer {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-top: 20px;
            padding-top: 15px;
            border-top: 1px solid var(--border);
        }

        .format-badge {
            background: var(--gray);
            color: var(--text-light);
            padding: 4px 10px;
            border-radius: 15px;
            font-size: 12px;
        }

        .edit-form {
            display: flex;
            gap: 10px;
            margin-top: 15px;
        }

        .edit-input {
            flex: 1;
            padding: 10px 15px;
            border: 2px solid var(--border);
            border-radius: 8px;
            font-size: 14px;
            transition: all 0.3s ease;
        }

        .edit-input:focus {
            outline: none;
            border-color: var(--primary);
        }

        .edit-btn {
            padding: 10px 20px;
            background: var(--primary);
            color: white;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-weight: 500;
            transition: all 0.3s ease;
        }

        .edit-btn:hover {
            background: var(--primary-dark);
        }

        /* Alerts and Status */
        .alert {
            padding: 15px 20px;
            border-radius: 10px;
            margin: 15px 0;
            display: flex;
            align-items: center;
            gap: 10px;
            animation: slideIn 0.3s ease;
        }

        @keyframes slideIn {
            from { transform: translateX(-20px); opacity: 0; }
            to { transform: translateX(0); opacity: 1; }
        }

        .alert.success {
            background: #e8f5e9;
            color: #2e7d32;
            border-left: 4px solid #4caf50;
        }

        .alert.warning {
            background: #fff3e0;
            color: #f57c00;
            border-left: 4px solid #ff9800;
        }

        .alert.error {
            background: #ffebee;
            color: #c62828;
            border-left: 4px solid #f44336;
        }

        /* Loading Animation */
        .loader {
            display: inline-block;
            width: 20px;
            height: 20px;
            border: 3px solid var(--border);
            border-top-color: var(--primary);
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }

        @keyframes spin {
            to { transform: rotate(360deg); }
        }

        /* Responsive */
        @media (max-width: 1200px) {
            .register-grid {
                grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            }
        }

        @media (max-width: 768px) {
            .main-layout {
                flex-direction: column;
            }
            
            .sidebar {
                width: 100%;
                padding: 20px;
            }
            
            .register-grid {
                grid-template-columns: 1fr;
            }
            
            .header {
                flex-direction: column;
                text-align: center;
                gap: 15px;
            }
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
            opacity: 0;
            visibility: hidden;
            transition: all 0.3s ease;
        }

        .modal-overlay.active {
            opacity: 1;
            visibility: visible;
        }

        .modal {
            background: white;
            border-radius: 15px;
            width: 90%;
            max-width: 500px;
            transform: translateY(20px);
            transition: transform 0.3s ease;
        }

        .modal-overlay.active .modal {
            transform: translateY(0);
        }

        .modal-header {
            padding: 20px;
            border-bottom: 1px solid var(--border);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .modal-body {
            padding: 20px;
        }

        .close-btn {
            background: none;
            border: none;
            font-size: 24px;
            cursor: pointer;
            color: var(--text-light);
        }

        /* Value indicators */
        .value-indicator {
            display: inline-flex;
            align-items: center;
            gap: 5px;
            padding: 4px 12px;
            border-radius: 15px;
            font-size: 12px;
            font-weight: 600;
        }

        .value-normal { background: #e8f5e9; color: #2e7d32; }
        .value-warning { background: #fff3e0; color: #f57c00; }
        .value-critical { background: #ffebee; color: #c62828; }

        /* Toggle Switch */
        .toggle-switch {
            position: relative;
            display: inline-block;
            width: 50px;
            height: 24px;
        }

        .toggle-switch input {
            opacity: 0;
            width: 0;
            height: 0;
        }

        .slider {
            position: absolute;
            cursor: pointer;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: #ccc;
            transition: .4s;
            border-radius: 34px;
        }

        .slider:before {
            position: absolute;
            content: "";
            height: 16px;
            width: 16px;
            left: 4px;
            bottom: 4px;
            background-color: white;
            transition: .4s;
            border-radius: 50%;
        }

        input:checked + .slider {
            background-color: var(--success);
        }

        input:checked + .slider:before {
            transform: translateX(26px);
        }
    </style>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css">
</head>
<body>
    <div class="container">
        <!-- Header -->
        <div class="header">
            <div class="header-content">
                <h1><i class="fas fa-bolt"></i> МТЗП-2T Контроллер</h1>
                <div class="subtitle">Панель управления защитными устройствами</div>
                <div class="status-badge">
                    <i class="fas fa-microchip"></i> Адрес устройства: 0x01
                </div>
            </div>
            <div class="connection-status">
                <div class="status-indicator"></div>
                <span>Подключено к устройству</span>
            </div>
        </div>

        <!-- Main Layout -->
        <div class="main-layout">
            <!-- Sidebar Navigation -->
            <div class="sidebar">
                <div class="nav-section">
                    <h3><i class="fas fa-sliders-h"></i> Основные разделы</h3>
                    <div class="nav-item active" data-tab="dashboard">
                        <i class="fas fa-tachometer-alt"></i>
                        <span>Панель приборов</span>
                    </div>
                    <div class="nav-item" data-tab="protection">
                        <i class="fas fa-shield-alt"></i>
                        <span>Защиты</span>
                    </div>
                    <div class="nav-item" data-tab="measurements">
                        <i class="fas fa-chart-line"></i>
                        <span>Измерения</span>
                    </div>
                    <div class="nav-item" data-tab="settings">
                        <i class="fas fa-cog"></i>
                        <span>Настройки</span>
                    </div>
                    <div class="nav-item" data-tab="events">
                        <i class="fas fa-history"></i>
                        <span>Журнал событий</span>
                    </div>
                    <div class="nav-item" data-tab="info">
                        <i class="fas fa-info-circle"></i>
                        <span>Информация</span>
                    </div>
                    <div class="nav-item" data-tab="all">
                        <i class="fas fa-list"></i>
                        <span>Все параметры</span>
                    </div>
                </div>

                <div class="nav-section">
                    <h3><i class="fas fa-bolt"></i> Быстрые действия</h3>
                    <button class="action-btn" id="btn-refresh">
                        <i class="fas fa-sync-alt"></i>
                        <span>Обновить данные</span>
                    </button>
                    <button class="action-btn" id="btn-save">
                        <i class="fas fa-save"></i>
                        <span>Сохранить настройки</span>
                    </button>
                    <button class="action-btn" id="btn-reset">
                        <i class="fas fa-undo"></i>
                        <span>Сброс устройства</span>
                    </button>
                    <button class="action-btn" id="btn-import">
                        <i class="fas fa-upload"></i>
                        <span>Импорт настроек</span>
                    </button>
                    <button class="action-btn primary" id="btn-export">
                        <i class="fas fa-download"></i>
                        <span>Экспорт настроек</span>
                    </button>
                    <input type="file" id="import-file" accept="application/json" style="display:none" />
                </div>

                <div class="nav-section">
                    <h3><i class="fas fa-chart-bar"></i> Статистика</h3>
                    <div style="padding: 10px; background: white; border-radius: 10px; margin-top: 10px;">
                        <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                            <span style="color: var(--text-light);">Время работы:</span>
                            <span style="font-weight: 600;" id="uptime">--:--:--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
                            <span style="color: var(--text-light);">Аварии:</span>
                            <span class="value-critical" id="alarm-count">0</span>
                        </div>
                        <div style="display: flex; justify-content: space-between;">
                            <span style="color: var(--text-light);">Температура:</span>
                            <span style="font-weight: 600;" id="temperature">--°C</span>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Main Content -->
            <div class="main-content">
                <!-- Content Header -->
                <div class="content-header">
                    <h2 id="page-title">Панель приборов</h2>
                    <button class="refresh-btn" id="refresh-all">
                        <i class="fas fa-redo"></i>
                        <span>Обновить все</span>
                    </button>
                </div>

                <!-- Alerts Area -->
                <div id="alerts-container"></div>

                <!-- Content will be loaded here -->
                <div id="content-area">
                    <!-- Dashboard will be loaded here by JavaScript -->
                </div>
            </div>
        </div>
    </div>

    <!-- Modal for detailed view -->
    <div class="modal-overlay" id="modal-overlay">
        <div class="modal">
            <div class="modal-header">
                <h3 id="modal-title">Детальная информация</h3>
                <button class="close-btn" id="modal-close">&times;</button>
            </div>
            <div class="modal-body" id="modal-content">
                <!-- Modal content will be loaded here -->
            </div>
        </div>
    </div>

    <script>
        // Register data structure
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

        // Menu structure
        const menuStructure = {
            dashboard: {
                title: "Панель приборов",
                sections: [
                    { title: "Основные параметры", registers: [33, 34, 35, 41, 42, 43, 49, 50, 51, 32] },
                    { title: "Токовая информация", registers: [44, 45, 46, 47, 48] },
                    { title: "Мощность и энергия", registers: [54, 55, 56, 57, 58, 59, 80, 81, 82, 83] },
                    { title: "Состояние системы", registers: [12, 23, 24, 25, 26, 28, 29, 30, 31] }
                ]
            },
            protection: {
                title: "Настройки защит",
                sections: [
                    { title: "МТЗ-1", registers: [115, 117, 118, 119, 120, 121] },
                    { title: "МТЗ-2", registers: [122, 124, 125] },
                    { title: "МТЗ-3", registers: [126, 128, 129, 130] },
                    { title: "УМТЗ", registers: [131, 133, 134, 135] },
                    { title: "ЗМТ", registers: [136, 138, 139, 140] },
                    { title: "ЗММН", registers: [141, 143, 144, 145, 146, 147, 148] },
                    { title: "ЗНФ", registers: [149, 151, 152, 153] },
                    { title: "ЗОФ", registers: [154, 156, 157, 158] },
                    { title: "БКИ", registers: [159, 161, 162, 163] },
                    { title: "НЗЗ", registers: [164, 166, 167, 168, 169, 170] },
                    { title: "Внешние защиты", registers: [171, 172, 174, 175, 176, 177, 178] },
                    { title: "Местные защиты", registers: [179, 181, 182, 183, 184] },
                    { title: "УРОВ", registers: [195, 197, 198] }
                ]
            },
            measurements: {
                title: "Текущие измерения",
                sections: [
                    { title: "Фазные токи", registers: [41, 42, 43, 44, 45, 46, 47, 48] },
                    { title: "Фазные напряжения", registers: [33, 34, 35, 36, 37, 38, 39, 40] },
                    { title: "Сопротивление изоляции", registers: [49, 50, 51, 52, 53] },
                    { title: "Дополнительные параметры", registers: [60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74] }
                ]
            },
            settings: {
                title: "Настройки системы",
                sections: [
                    { title: "Конфигурация RS485", registers: [1, 2, 3, 5, 6] },
                    { title: "Настройки времени", registers: [17, 18, 19, 20, 21, 22] },
                    { title: "Параметры системы", registers: [252, 253, 254] },
                    { title: "Управление КА", registers: [188, 189, 190, 193, 194] },
                    { title: "АПВ/АВР", registers: [205, 207, 208, 209, 210, 212] }
                ]
            },
            events: {
                title: "Журнал событий",
                sections: [
                    { title: "Статистика", registers: [100, 102, 104, 106, 108, 110, 112] },
                    { title: "Протоколы", registers: [101, 103, 105, 107, 109, 111, 113] },
                    { title: "Запись мощности", registers: [114] }
                ]
            },
            info: {
                title: "Информация",
                sections: [
                    { title: "Изделие", registers: [10, 11] },
                    { title: "RTC", registers: [16, 17, 18, 19, 20, 21, 22] },
                    { title: "Диагностика", registers: [13, 14, 15, 199, 200, 201, 202, 203, 204] }
                ]
            },
            all: {
                title: "Все параметры",
                sections: [
                    { title: "Все регистры", registers: registerTable.map(item => item.id) }
                ]
            }
        };

        // State management
        let currentTab = 'dashboard';
        let refreshInterval = null;
        const loadingRegisters = new Set();

        // DOM Elements
        const navItems = document.querySelectorAll('.nav-item');
        const contentArea = document.getElementById('content-area');
        const pageTitle = document.getElementById('page-title');
        const refreshAllBtn = document.getElementById('refresh-all');
        const btnRefresh = document.getElementById('btn-refresh');
        const modalOverlay = document.getElementById('modal-overlay');
        const modalClose = document.getElementById('modal-close');
        const modalTitle = document.getElementById('modal-title');
        const modalContent = document.getElementById('modal-content');
        const btnImport = document.getElementById('btn-import');
        const btnExport = document.getElementById('btn-export');
        const importFile = document.getElementById('import-file');

        // Format value based on register format
        function formatValue(raw, format, scale) {
            if (raw === undefined || raw === null) return '--';
            const scaleFactor = Math.pow(10, scale);
            switch(format) {
                case 'HEX': return `0x${raw.toString(16).toUpperCase().padStart(4, '0')}`;
                case 'OCT': return `0o${raw.toString(8)}`;
                case 'BIN': return `0b${raw.toString(2).padStart(16, '0')}`;
                case 'SDC': return (raw / scaleFactor).toFixed(scale);
                case 'SWT': return raw ? 'ВКЛ' : 'ВЫКЛ';
                case 'REL': return (raw / scaleFactor).toFixed(scale);
                default: return (raw / scaleFactor).toFixed(scale);
            }
        }

        function parseValue(text, format, scale) {
            const trimmed = text.trim();
            const scaleFactor = Math.pow(10, scale);
            switch(format) {
                case 'HEX': return parseInt(trimmed.replace(/^0x/i, ''), 16);
                case 'OCT': return parseInt(trimmed.replace(/^0o/i, ''), 8);
                case 'BIN': return parseInt(trimmed.replace(/^0b/i, ''), 2);
                case 'SWT': return ['1', 'вкл', 'true', 'on', 'да'].includes(trimmed.toLowerCase()) ? 1 : 0;
                case 'SDC':
                case 'REL': return Math.round(parseFloat(trimmed) * scaleFactor);
                default: return Math.round(parseFloat(trimmed) * scaleFactor);
            }
        }

        // Get value indicator class
        function getValueIndicator(value, regId) {
            if (value === undefined) return '';
            if (regId >= 28 && regId <= 31 && value > 0) return 'value-critical';
            if (value > 1000) return 'value-warning';
            return 'value-normal';
        }

        // Create register card
        function createRegisterCard(regId) {
            const reg = registerIndex.get(regId);
            if (!reg) return '';

            return `
                <div class="register-card" data-reg-id="${regId}">
                    <div class="card-header">
                        <div class="register-name">${reg.name}</div>
                        <div class="register-id">#${reg.id}</div>
                    </div>
                    
                    <div class="value-display" id="value-${regId}">
                        <span class="loading-value">
                            <div class="loader"></div>
                        </span>
                    </div>
                    
                    <div class="card-footer">
                        <div class="format-badge">
                            ${reg.format} | масштаб: ${reg.scale}
                        </div>
                        <div class="value-indicator ${getValueIndicator(null, regId)}" id="indicator-${regId}">
                            ожидание...
                        </div>
                    </div>
                    
                    <div class="edit-form" style="display: none;" id="edit-form-${regId}">
                        <input type="text" class="edit-input" placeholder="Новое значение" id="edit-input-${regId}">
                        <button class="edit-btn" onclick="saveRegister(${regId})">
                            <i class="fas fa-check"></i> Применить
                        </button>
                        <button class="edit-btn" style="background: var(--danger);" onclick="cancelEdit(${regId})">
                            <i class="fas fa-times"></i> Отмена
                        </button>
                    </div>
                    
                    <div style="margin-top: 15px; display: flex; gap: 10px;">
                        <button class="action-btn" style="flex: 1;" onclick="toggleEdit(${regId})">
                            <i class="fas fa-edit"></i> Изменить
                        </button>
                        <button class="action-btn" style="flex: 1;" onclick="showRegisterDetails(${regId})">
                            <i class="fas fa-info-circle"></i> Подробнее
                        </button>
                    </div>
                </div>
            `;
        }

        // Load tab content
        function loadTabContent(tabId) {
            const tab = menuStructure[tabId];
            if (!tab) return;

            currentTab = tabId;
            pageTitle.textContent = tab.title;

            let html = '';
            tab.sections.forEach(section => {
                html += `
                    <div style="margin-bottom: 40px;">
                        <h3 style="margin-bottom: 20px; color: var(--primary-dark); display: flex; align-items: center; gap: 10px;">
                            <i class="fas fa-folder-open"></i>
                            ${section.title}
                        </h3>
                        <div class="register-grid">
                            ${section.registers.map(regId => createRegisterCard(regId)).join('')}
                        </div>
                    </div>
                `;
            });

            contentArea.innerHTML = html;
            refreshTabValues();
        }

        // Refresh values for current tab
        async function refreshTabValues() {
            const tab = menuStructure[currentTab];
            if (!tab) return;
            const regIds = tab.sections.flatMap(section => section.registers);
            for (const regId of regIds) {
                await updateRegisterValue(regId);
            }
        }

        // Update single register value
        async function updateRegisterValue(regId) {
            if (loadingRegisters.has(regId)) return;
            loadingRegisters.add(regId);
            const valueElement = document.getElementById(`value-${regId}`);
            const indicatorElement = document.getElementById(`indicator-${regId}`);
            if (valueElement) {
                valueElement.innerHTML = '<div class="loader"></div>';
            }

            try {
                const response = await fetch(`/api/read?reg=${regId}`);
                const data = await response.json();
                
                if (data.ok) {
                    const reg = registerIndex.get(regId);
                    const formattedValue = formatValue(data.value, reg?.format || 'DEC', reg?.scale || 0);
                    
                    if (valueElement) {
                        valueElement.innerHTML = `
                            <span style="font-size: 32px; font-weight: 700;">${formattedValue}</span>
                            ${reg?.format === 'SWT' ? '' : '<span class="unit">ед.</span>'}
                        `;
                    }
                    
                    if (indicatorElement) {
                        indicatorElement.className = `value-indicator ${getValueIndicator(data.value, regId)}`;
                        indicatorElement.textContent = 'актуально';
                    }
                } else {
                    if (valueElement) valueElement.innerHTML = '<span style="color: var(--danger);">Ошибка</span>';
                    if (indicatorElement) {
                        indicatorElement.className = 'value-indicator value-critical';
                        indicatorElement.textContent = 'ошибка';
                    }
                }
            } catch (error) {
                console.error('Error reading register:', error);
                if (valueElement) valueElement.innerHTML = '<span style="color: var(--danger);">Ошибка</span>';
                if (indicatorElement) {
                    indicatorElement.className = 'value-indicator value-critical';
                    indicatorElement.textContent = 'ошибка сети';
                }
            } finally {
                loadingRegisters.delete(regId);
            }
        }

        // Toggle edit mode
        function toggleEdit(regId) {
            const editForm = document.getElementById(`edit-form-${regId}`);
            const currentValue = document.getElementById(`value-${regId}`).textContent;
            const input = document.getElementById(`edit-input-${regId}`);
            editForm.style.display = editForm.style.display === 'none' ? 'flex' : 'none';
            if (input) {
                input.value = currentValue.replace('ед.', '').trim();
                input.focus();
            }
        }

        function cancelEdit(regId) {
            document.getElementById(`edit-form-${regId}`).style.display = 'none';
        }

        async function saveRegister(regId) {
            const input = document.getElementById(`edit-input-${regId}`);
            const value = input.value.trim();
            const reg = registerIndex.get(regId);
            if (!reg || !value) return;

            try {
                const rawValue = parseValue(value, reg.format, reg.scale);
                const response = await fetch(`/api/write?reg=${regId}&val=${rawValue}`, { method: 'POST' });
                const result = await response.json();
                if (result.ok) {
                    showAlert('Значение успешно обновлено', 'success');
                    updateRegisterValue(regId);
                    cancelEdit(regId);
                } else {
                    showAlert('Ошибка при записи значения', 'error');
                }
            } catch (error) {
                console.error('Error writing register:', error);
                showAlert('Ошибка сети', 'error');
            }
        }

        async function showRegisterDetails(regId) {
            const reg = registerIndex.get(regId);
            if (!reg) return;

            try {
                const response = await fetch(`/api/read?reg=${regId}`);
                const data = await response.json();
                
                modalTitle.textContent = reg.name;
                modalContent.innerHTML = `
                    <div style="padding: 20px;">
                        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-bottom: 20px;">
                            <div style="background: var(--gray); padding: 15px; border-radius: 10px;">
                                <div style="color: var(--text-light); font-size: 12px;">ID регистра</div>
                                <div style="font-size: 24px; font-weight: 700; color: var(--primary);">#${reg.id}</div>
                            </div>
                            <div style="background: var(--gray); padding: 15px; border-radius: 10px;">
                                <div style="color: var(--text-light); font-size: 12px;">Формат</div>
                                <div style="font-size: 18px; font-weight: 600;">${reg.format}</div>
                            </div>
                        </div>
                        
                        <div style="background: linear-gradient(135deg, var(--primary), var(--secondary)); color: white; padding: 30px; border-radius: 10px; text-align: center; margin-bottom: 20px;">
                            <div style="font-size: 12px; opacity: 0.9;">Текущее значение</div>
                            <div style="font-size: 48px; font-weight: 800; margin: 10px 0;">
                                ${formatValue(data.value, reg.format, reg.scale)}
                            </div>
                            <div style="font-size: 14px; opacity: 0.9;">масштаб: ${reg.scale}</div>
                        </div>
                        
                        <div style="background: var(--gray); padding: 15px; border-radius: 10px; margin-bottom: 20px;">
                            <div style="color: var(--text-light); font-size: 12px;">Описание</div>
                            <div style="margin-top: 5px;">${reg.name}</div>
                        </div>
                        
                        <div style="display: flex; gap: 10px;">
                            <button class="action-btn primary" style="flex: 1;" onclick="toggleEdit(${regId}); modalOverlay.classList.remove('active');">
                                <i class="fas fa-edit"></i> Изменить значение
                            </button>
                            <button class="action-btn" style="flex: 1;" onclick="modalOverlay.classList.remove('active');">
                                <i class="fas fa-times"></i> Закрыть
                            </button>
                        </div>
                    </div>
                `;
                
                modalOverlay.classList.add('active');
            } catch (error) {
                console.error('Error loading register details:', error);
                showAlert('Ошибка при загрузке данных', 'error');
            }
        }

        function showAlert(message, type = 'info') {
            const container = document.getElementById('alerts-container');
            const alertId = 'alert-' + Date.now();
            const alert = document.createElement('div');
            alert.className = `alert ${type}`;
            alert.id = alertId;
            alert.innerHTML = `
                <i class="fas fa-${type === 'success' ? 'check-circle' : type === 'warning' ? 'exclamation-triangle' : 'info-circle'}"></i>
                <span>${message}</span>
                <button onclick="document.getElementById('${alertId}').remove()" style="margin-left: auto; background: none; border: none; color: inherit; cursor: pointer;">
                    <i class="fas fa-times"></i>
                </button>
            `;
            container.appendChild(alert);
            setTimeout(() => {
                if (document.getElementById(alertId)) {
                    document.getElementById(alertId).remove();
                }
            }, 5000);
        }

        async function updateSystemStats() {
            try {
                const uptimeResponse = await fetch('/api/read?reg=96');
                const uptimeData = await uptimeResponse.json();
                if (uptimeData.ok) {
                    const hours = Math.floor(uptimeData.value / 3600);
                    const minutes = Math.floor((uptimeData.value % 3600) / 60);
                    const seconds = uptimeData.value % 60;
                    document.getElementById('uptime').textContent = 
                        `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
                }

                const tempResponse = await fetch('/api/read?reg=32');
                const tempData = await tempResponse.json();
                if (tempData.ok) {
                    document.getElementById('temperature').textContent = 
                        `${(tempData.value / 10).toFixed(1)}°C`;
                }

                const alarmResponse = await fetch('/api/read?reg=77');
                const alarmData = await alarmResponse.json();
                if (alarmData.ok) {
                    document.getElementById('alarm-count').textContent = alarmData.value;
                }
            } catch (error) {
                console.error('Error updating stats:', error);
            }
        }

        async function exportSettings() {
            showAlert('Экспорт настроек...', 'info');
            const data = {};
            for (const reg of registerTable) {
                try {
                    const response = await fetch(`/api/read?reg=${reg.id}`);
                    const payload = await response.json();
                    if (payload.ok) {
                        data[reg.id] = payload.value;
                    }
                } catch (error) {
                    console.error('Export error for reg', reg.id, error);
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
            showAlert('Экспорт настроек завершен', 'success');
        }

        async function importSettings(file) {
            try {
                const text = await file.text();
                const data = JSON.parse(text);
                const entries = Object.entries(data);
                for (const [regId, value] of entries) {
                    await fetch(`/api/write?reg=${regId}&val=${value}`, { method: 'POST' });
                }
                showAlert('Импорт настроек завершен', 'success');
                refreshTabValues();
            } catch (error) {
                console.error('Import error', error);
                showAlert('Ошибка импорта настроек', 'error');
            }
        }

        function init() {
            loadTabContent('dashboard');

            navItems.forEach(item => {
                item.addEventListener('click', () => {
                    navItems.forEach(nav => nav.classList.remove('active'));
                    item.classList.add('active');
                    loadTabContent(item.dataset.tab);
                });
            });

            refreshAllBtn.addEventListener('click', refreshTabValues);
            btnRefresh.addEventListener('click', refreshTabValues);

            modalClose.addEventListener('click', () => {
                modalOverlay.classList.remove('active');
            });

            modalOverlay.addEventListener('click', (e) => {
                if (e.target === modalOverlay) {
                    modalOverlay.classList.remove('active');
                }
            });

            document.getElementById('btn-save').addEventListener('click', () => {
                fetch('/api/write?reg=0&val=1')
                    .then(() => showAlert('Настройки сохранены', 'success'));
            });

            document.getElementById('btn-reset').addEventListener('click', () => {
                if (confirm('Вы уверены, что хотите выполнить сброс устройства?')) {
                    fetch('/api/write?reg=0&val=2')
                        .then(() => showAlert('Устройство сброшено', 'warning'));
                }
            });

            btnExport.addEventListener('click', exportSettings);
            btnImport.addEventListener('click', () => importFile.click());
            importFile.addEventListener('change', () => {
                if (importFile.files.length) {
                    importSettings(importFile.files[0]);
                    importFile.value = '';
                }
            });

            refreshInterval = setInterval(() => {
                refreshTabValues();
                updateSystemStats();
            }, 3000);

            updateSystemStats();
        }

        document.addEventListener('DOMContentLoaded', init);
    </script>
</body>
</html>
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
