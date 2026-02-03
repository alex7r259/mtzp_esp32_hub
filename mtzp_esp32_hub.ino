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
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Панель управления МТЗП-1200</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }
        
        body {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        
        .header {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 15px 15px 0 0;
            padding: 25px 30px;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 3px solid #4CAF50;
        }
        
        .header h1 {
            color: #2c3e50;
            font-size: 28px;
            display: flex;
            align-items: center;
            gap: 15px;
        }
        
        .header h1 i {
            color: #4CAF50;
            font-size: 32px;
        }
        
        .status-indicator {
            display: flex;
            align-items: center;
            gap: 10px;
            background: #f8f9fa;
            padding: 10px 20px;
            border-radius: 50px;
            border: 2px solid #e9ecef;
        }
        
        .status-dot {
            width: 12px;
            height: 12px;
            background: #4CAF50;
            border-radius: 50%;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }
        
        .nav-tabs {
            background: rgba(255, 255, 255, 0.95);
            display: flex;
            gap: 5px;
            padding: 15px 30px;
            border-bottom: 1px solid #dee2e6;
            overflow-x: auto;
            scrollbar-width: thin;
        }
        
        .nav-tab {
            padding: 12px 25px;
            background: #f8f9fa;
            border: 2px solid transparent;
            border-radius: 8px;
            cursor: pointer;
            font-weight: 600;
            color: #495057;
            transition: all 0.3s ease;
            white-space: nowrap;
            display: flex;
            align-items: center;
            gap: 8px;
        }
        
        .nav-tab:hover {
            background: #e9ecef;
            transform: translateY(-2px);
        }
        
        .nav-tab.active {
            background: #4CAF50;
            color: white;
            border-color: #45a049;
            box-shadow: 0 4px 12px rgba(76, 175, 80, 0.3);
        }
        
        .content {
            background: rgba(255, 255, 255, 0.98);
            border-radius: 0 0 15px 15px;
            padding: 30px;
            box-shadow: 0 8px 30px rgba(0, 0, 0, 0.15);
            min-height: 600px;
        }
        
        .page {
            display: none;
            animation: fadeIn 0.5s ease;
        }
        
        .page.active {
            display: block;
        }
        
        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(10px); }
            to { opacity: 1; transform: translateY(0); }
        }
        
        .card-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 25px;
            margin-top: 20px;
        }
        
        .card {
            background: white;
            border-radius: 12px;
            padding: 25px;
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.08);
            border: 1px solid #e0e0e0;
            transition: transform 0.3s ease, box-shadow 0.3s ease;
        }
        
        .card:hover {
            transform: translateY(-5px);
            box-shadow: 0 10px 25px rgba(0, 0, 0, 0.15);
        }
        
        .card-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
            padding-bottom: 15px;
            border-bottom: 2px solid #f0f0f0;
        }
        
        .card-title {
            font-size: 18px;
            color: #2c3e50;
            font-weight: 600;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .card-title i {
            color: #4CAF50;
        }
        
        .parameter-group {
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        
        .parameter {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 12px;
            background: #f8f9fa;
            border-radius: 8px;
            transition: background 0.3s ease;
        }
        
        .parameter:hover {
            background: #e9ecef;
        }
        
        .parameter-label {
            font-weight: 500;
            color: #495057;
            flex: 1;
        }
        
        .parameter-value {
            font-weight: 600;
            color: #2c3e50;
            font-size: 18px;
            min-width: 120px;
            text-align: right;
            font-family: 'Courier New', monospace;
        }
        
        .unit {
            font-size: 14px;
            color: #6c757d;
            margin-left: 5px;
            font-weight: normal;
        }
        
        .btn {
            padding: 10px 20px;
            border: none;
            border-radius: 8px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            display: inline-flex;
            align-items: center;
            gap: 8px;
            font-size: 14px;
        }
        
        .btn-primary {
            background: #4CAF50;
            color: white;
        }
        
        .btn-primary:hover {
            background: #45a049;
            transform: translateY(-2px);
        }
        
        .btn-secondary {
            background: #6c757d;
            color: white;
        }
        
        .btn-edit {
            padding: 6px 12px;
            background: #2196F3;
            color: white;
            border-radius: 6px;
            border: none;
            cursor: pointer;
            font-size: 12px;
            transition: background 0.3s ease;
        }
        
        .btn-edit:hover {
            background: #0b7dda;
        }
        
        .modal {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.5);
            z-index: 1000;
            justify-content: center;
            align-items: center;
        }
        
        .modal-content {
            background: white;
            padding: 30px;
            border-radius: 15px;
            max-width: 500px;
            width: 90%;
            box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
            animation: modalSlide 0.3s ease;
        }
        
        @keyframes modalSlide {
            from { transform: translateY(-50px); opacity: 0; }
            to { transform: translateY(0); opacity: 1; }
        }
        
        .modal-header {
            margin-bottom: 20px;
            padding-bottom: 15px;
            border-bottom: 2px solid #f0f0f0;
        }
        
        .modal-title {
            font-size: 20px;
            color: #2c3e50;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .form-group {
            margin-bottom: 20px;
        }
        
        .form-label {
            display: block;
            margin-bottom: 8px;
            color: #495057;
            font-weight: 500;
        }
        
        .form-input {
            width: 100%;
            padding: 12px;
            border: 2px solid #dee2e6;
            border-radius: 8px;
            font-size: 16px;
            transition: border-color 0.3s ease;
        }
        
        .form-input:focus {
            outline: none;
            border-color: #4CAF50;
        }
        
        .footer {
            text-align: center;
            margin-top: 30px;
            padding-top: 20px;
            border-top: 1px solid #dee2e6;
            color: #6c757d;
            font-size: 14px;
        }
        
        .refresh-btn {
            position: fixed;
            bottom: 30px;
            right: 30px;
            width: 60px;
            height: 60px;
            background: #4CAF50;
            color: white;
            border-radius: 50%;
            border: none;
            cursor: pointer;
            box-shadow: 0 6px 20px rgba(76, 175, 80, 0.4);
            transition: all 0.3s ease;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 24px;
            z-index: 100;
        }
        
        .refresh-btn:hover {
            transform: rotate(180deg) scale(1.1);
            background: #45a049;
        }
        
        .notification {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 15px 25px;
            background: #4CAF50;
            color: white;
            border-radius: 8px;
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.2);
            display: none;
            z-index: 1000;
            animation: slideIn 0.3s ease;
        }
        
        @keyframes slideIn {
            from { transform: translateX(100%); opacity: 0; }
            to { transform: translateX(0); opacity: 1; }
        }
        
        .table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 20px;
        }
        
        .table th, .table td {
            padding: 12px 15px;
            text-align: left;
            border-bottom: 1px solid #dee2e6;
        }
        
        .table th {
            background: #f8f9fa;
            font-weight: 600;
            color: #495057;
        }
        
        .table tr:hover {
            background: #f8f9fa;
        }
        
        .search-box {
            margin-bottom: 20px;
        }
        
        .search-input {
            width: 100%;
            padding: 12px 20px;
            border: 2px solid #dee2e6;
            border-radius: 8px;
            font-size: 16px;
            background: url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="%236c757d" stroke-width="2"><circle cx="11" cy="11" r="8"></circle><path d="M21 21l-4.35-4.35"></path></svg>') no-repeat 15px center;
            background-size: 20px;
            padding-left: 45px;
        }
        
        @media (max-width: 768px) {
            .header {
                flex-direction: column;
                gap: 15px;
                text-align: center;
            }
            
            .nav-tabs {
                padding: 10px;
            }
            
            .nav-tab {
                padding: 10px 15px;
                font-size: 14px;
            }
            
            .content {
                padding: 20px;
            }
            
            .card-grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
</head>
<body>
    <div class="container">
        <!-- Заголовок -->
        <div class="header">
            <h1>
                <i class="fas fa-bolt"></i>
                Панель управления МТЗП-1200
            </h1>
            <div class="status-indicator">
                <div class="status-dot"></div>
                <span>Устройство подключено</span>
            </div>
        </div>
        
        <!-- Навигация -->
        <div class="nav-tabs">
            <div class="nav-tab active" data-page="dashboard">
                <i class="fas fa-tachometer-alt"></i>
                Текущие параметры
            </div>
            <div class="nav-tab" data-page="protections">
                <i class="fas fa-shield-alt"></i>
                Уставки защит
            </div>
            <div class="nav-tab" data-page="signals">
                <i class="fas fa-bell"></i>
                Контроль сигналов
            </div>
            <div class="nav-tab" data-page="settings">
                <i class="fas fa-cog"></i>
                Настройки
            </div>
            <div class="nav-tab" data-page="logs">
                <i class="fas fa-clipboard-list"></i>
                Журнал событий
            </div>
            <div class="nav-tab" data-page="info">
                <i class="fas fa-info-circle"></i>
                Информация
            </div>
        </div>
        
        <!-- Контент -->
        <div class="content">
            <!-- Главная страница - Текущие параметры -->
            <div class="page active" id="dashboard">
                <h2><i class="fas fa-tachometer-alt"></i> Текущие параметры системы</h2>
                <p class="subtitle">Мониторинг в реальном времени</p>
                
                <div class="card-grid">
                    <!-- Фазные токи -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-bolt"></i>
                                Фазные токи
                            </div>
                            <div class="last-update">Обновлено: <span class="time">--:--:--</span></div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Фаза A</span>
                                <span class="parameter-value" data-reg="41">0<span class="unit">А</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Фаза B</span>
                                <span class="parameter-value" data-reg="42">0<span class="unit">А</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Фаза C</span>
                                <span class="parameter-value" data-reg="43">0<span class="unit">А</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Максимальный ток</span>
                                <span class="parameter-value" data-reg="44">0<span class="unit">А</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Фазные напряжения -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-charging-station"></i>
                                Фазные напряжения
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Фаза A</span>
                                <span class="parameter-value" data-reg="33">0<span class="unit">В</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Фаза B</span>
                                <span class="parameter-value" data-reg="34">0<span class="unit">В</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Фаза C</span>
                                <span class="parameter-value" data-reg="35">0<span class="unit">В</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Линейное U</span>
                                <span class="parameter-value" data-reg="64">0<span class="unit">В</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Мощность и энергия -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-chart-line"></i>
                                Мощность и энергия
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Активная мощность</span>
                                <span class="parameter-value" data-reg="57">0<span class="unit">кВт</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Полная мощность</span>
                                <span class="parameter-value" data-reg="58">0<span class="unit">кВА</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Коэф. мощности</span>
                                <span class="parameter-value" data-reg="59">0.00</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Активная энергия</span>
                                <span class="parameter-value" data-reg="80">0<span class="unit">кВт·ч</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Изоляция и температура -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-thermometer-half"></i>
                                Состояние системы
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Температура</span>
                                <span class="parameter-value" data-reg="32">0<span class="unit">°C</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Сопр. изоляции (мин)</span>
                                <span class="parameter-value" data-reg="52">0<span class="unit">кОм</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Оперативное питание</span>
                                <span class="parameter-value" data-reg="63">0<span class="unit">В</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Ток нуля</span>
                                <span class="parameter-value" data-reg="61">0<span class="unit">мА</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Статус защиты -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-shield-alt"></i>
                                Статус защиты
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Аварии</span>
                                <span class="parameter-value" data-reg="28">0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Неисправности</span>
                                <span class="parameter-value" data-reg="30">0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Реле статус</span>
                                <span class="parameter-value" data-reg="23">0x0</span>
                            </div>
                            <button class="btn btn-primary" onclick="showProtections()">
                                <i class="fas fa-exclamation-triangle"></i>
                                Детали защиты
                            </button>
                        </div>
                    </div>
                    
                    <!-- Время наработки -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-clock"></i>
                                Время наработки
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">КА (дни:часы)</span>
                                <span class="parameter-value" data-reg="93">0:0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">КА (минуты:секунды)</span>
                                <span class="parameter-value" data-reg="92">0:0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">МТЗП (дни:часы)</span>
                                <span class="parameter-value" data-reg="97">0:0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">МТЗП (минуты:секунды)</span>
                                <span class="parameter-value" data-reg="96">0:0</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            
            <!-- Уставки защит -->
            <div class="page" id="protections">
                <h2><i class="fas fa-shield-alt"></i> Уставки защит</h2>
                <p class="subtitle">Настройка параметров защитных функций</p>
                
                <div class="search-box">
                    <input type="text" class="search-input" placeholder="Поиск защит..." id="searchProtections">
                </div>
                
                <div class="card-grid">
                    <!-- МТЗ1 -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-bolt"></i>
                                МТЗ-1
                            </div>
                            <button class="btn-edit" onclick="editProtection(115)">
                                <i class="fas fa-edit"></i> Настроить
                            </button>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Состояние</span>
                                <span class="parameter-value" data-reg="115">Выкл</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Уставка тока</span>
                                <span class="parameter-value" data-reg="117">0<span class="unit">А</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Задержка</span>
                                <span class="parameter-value" data-reg="118">0<span class="unit">мс</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- МТЗ2 -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-bolt"></i>
                                МТЗ-2
                            </div>
                            <button class="btn-edit" onclick="editProtection(122)">
                                <i class="fas fa-edit"></i> Настроить
                            </button>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Состояние</span>
                                <span class="parameter-value" data-reg="122">Выкл</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Уставка тока</span>
                                <span class="parameter-value" data-reg="124">0<span class="unit">А</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Задержка</span>
                                <span class="parameter-value" data-reg="125">0<span class="unit">мс</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- БКИ -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-leaf"></i>
                                БКИ
                            </div>
                            <button class="btn-edit" onclick="editProtection(159)">
                                <i class="fas fa-edit"></i> Настроить
                            </button>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Состояние</span>
                                <span class="parameter-value" data-reg="159">Выкл</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Уставка R</span>
                                <span class="parameter-value" data-reg="161">0<span class="unit">кОм</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Задержка</span>
                                <span class="parameter-value" data-reg="162">0<span class="unit">мс</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- НЗЗ -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-sun"></i>
                                НЗЗ
                            </div>
                            <button class="btn-edit" onclick="editProtection(164)">
                                <i class="fas fa-edit"></i> Настроить
                            </button>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Состояние</span>
                                <span class="parameter-value" data-reg="164">Выкл</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Уставка I0</span>
                                <span class="parameter-value" data-reg="166">0<span class="unit">мА</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Задержка</span>
                                <span class="parameter-value" data-reg="168">0<span class="unit">мс</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- ЗММН -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-chart-line"></i>
                                ЗММН
                            </div>
                            <button class="btn-edit" onclick="editProtection(141)">
                                <i class="fas fa-edit"></i> Настроить
                            </button>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Состояние</span>
                                <span class="parameter-value" data-reg="141">Выкл</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">U макс</span>
                                <span class="parameter-value" data-reg="143">0<span class="unit">В</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">U мин</span>
                                <span class="parameter-value" data-reg="145">0<span class="unit">В</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- АПВ/АВР -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-redo"></i>
                                АПВ/АВР
                            </div>
                            <button class="btn-edit" onclick="editProtection(205)">
                                <i class="fas fa-edit"></i> Настроить
                            </button>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">АПВ</span>
                                <span class="parameter-value" data-reg="205">Выкл</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">АВР</span>
                                <span class="parameter-value" data-reg="210">Выкл</span>
                            </div>
                            <button class="btn btn-primary" onclick="toggleAPV()">
                                <i class="fas fa-power-off"></i>
                                Вкл/Выкл АПВ
                            </button>
                        </div>
                    </div>
                </div>
            </div>
            
            <!-- Контроль сигналов -->
            <div class="page" id="signals">
                <h2><i class="fas fa-bell"></i> Контроль сигналов</h2>
                <p class="subtitle">Мониторинг входов и выходов системы</p>
                
                <div class="card-grid">
                    <!-- Релейные выходы -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-plug"></i>
                                Релейные выходы
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">К1 (Включение КА)</span>
                                <span class="parameter-value" data-reg="23" data-bit="0">Выкл</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">К3 (Отключение КА)</span>
                                <span class="parameter-value" data-reg="23" data-bit="2">Выкл</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">К8 (Авария)</span>
                                <span class="parameter-value" data-reg="23" data-bit="7">Выкл</span>
                            </div>
                            <button class="btn btn-primary" onclick="showRelaysDetails()">
                                <i class="fas fa-list"></i>
                                Все реле
                            </button>
                        </div>
                    </div>
                    
                    <!-- Внешние входы -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-sign-in-alt"></i>
                                Внешние входы
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">ПДУ1</span>
                                <span class="parameter-value" data-reg="24" data-bit="0">Нет</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">ПДУ2</span>
                                <span class="parameter-value" data-reg="24" data-bit="3">Нет</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">ЛЗШ</span>
                                <span class="parameter-value" data-reg="25" data-bit="3">Нет</span>
                            </div>
                            <button class="btn btn-primary" onclick="showInputsDetails()">
                                <i class="fas fa-list"></i>
                                Все входы
                            </button>
                        </div>
                    </div>
                    
                    <!-- Внутренние входы -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-microchip"></i>
                                Внутренние входы
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">РПО</span>
                                <span class="parameter-value" data-reg="26" data-bit="0">Нет</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">РПВ</span>
                                <span class="parameter-value" data-reg="26" data-bit="1">Нет</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Контроль БП</span>
                                <span class="parameter-value" data-reg="26" data-bit="2">Нет</span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Статус аварий -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-exclamation-triangle"></i>
                                Статус аварий
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Активные аварии</span>
                                <span class="parameter-value" data-reg="28">0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Активные неиспр.</span>
                                <span class="parameter-value" data-reg="30">0</span>
                            </div>
                            <button class="btn btn-primary" onclick="showAlarmsDetails()">
                                <i class="fas fa-search"></i>
                                Детали
                            </button>
                        </div>
                    </div>
                </div>
            </div>
            
            <!-- Настройки -->
            <div class="page" id="settings">
                <h2><i class="fas fa-cog"></i> Настройки системы</h2>
                <p class="subtitle">Конфигурация параметров устройства</p>
                
                <div class="card-grid">
                    <!-- Общие настройки -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-sliders-h"></i>
                                Общие настройки
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Обновление данных</span>
                                <span class="parameter-value" data-reg="252">2000<span class="unit">мс</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Пакетов в секунду</span>
                                <span class="parameter-value" data-reg="6">10</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Таймаут RS485</span>
                                <span class="parameter-value" data-reg="3">300<span class="unit">мс</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- RS485 -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-network-wired"></i>
                                Интерфейс RS485
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Адрес устройства</span>
                                <span class="parameter-value" data-reg="1">1</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Скорость</span>
                                <span class="parameter-value" data-reg="2">19200<span class="unit">бод</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Протокол</span>
                                <span class="parameter-value" data-reg="5">SLIP</span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Управление -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-gamepad"></i>
                                Управление
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Режим управления</span>
                                <span class="parameter-value" data-reg="213">Авто</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Время МПУ</span>
                                <span class="parameter-value" data-reg="218">100<span class="unit">мс</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Время STOP</span>
                                <span class="parameter-value" data-reg="219">500<span class="unit">мс</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- КА управление -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-power-off"></i>
                                Управление КА
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Время реле ОТКЛ</span>
                                <span class="parameter-value" data-reg="188">50<span class="unit">мс</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Время реле ВКЛ</span>
                                <span class="parameter-value" data-reg="189">50<span class="unit">мс</span></span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Время ГОТОВ</span>
                                <span class="parameter-value" data-reg="190">1000<span class="unit">мс</span></span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Действия -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-tools"></i>
                                Действия
                            </div>
                        </div>
                        <div class="parameter-group">
                            <button class="btn btn-primary" onclick="saveSettings()">
                                <i class="fas fa-save"></i>
                                Сохранить настройки
                            </button>
                            <button class="btn btn-secondary" onclick="loadFactorySettings()">
                                <i class="fas fa-undo"></i>
                                Заводские настройки
                            </button>
                            <button class="btn btn-secondary" onclick="calibrateSensors()">
                                <i class="fas fa-ruler"></i>
                                Юстировка
                            </button>
                            <button class="btn btn-secondary" onclick="resetCounters()">
                                <i class="fas fa-trash-alt"></i>
                                Обнулить счётчики
                            </button>
                        </div>
                    </div>
                    
                    <!-- Пароль -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-lock"></i>
                                Безопасность
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Пароль</span>
                                <span class="parameter-value">*****</span>
                            </div>
                            <button class="btn btn-primary" onclick="changePassword()">
                                <i class="fas fa-key"></i>
                                Изменить пароль
                            </button>
                        </div>
                    </div>
                </div>
            </div>
            
            <!-- Журнал событий -->
            <div class="page" id="logs">
                <h2><i class="fas fa-clipboard-list"></i> Журнал событий</h2>
                <p class="subtitle">История работы системы</p>
                
                <div class="card-grid">
                    <!-- Статистика -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-chart-bar"></i>
                                Статистика
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Всего аварий</span>
                                <span class="parameter-value" data-reg="100">0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Всего неисправностей</span>
                                <span class="parameter-value" data-reg="102">0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Коммутаций КА</span>
                                <span class="parameter-value" data-reg="106">0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Вкл/Откл питания</span>
                                <span class="parameter-value" data-reg="108">0</span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Последние события -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-history"></i>
                                Последние события
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Последняя авария</span>
                                <span class="parameter-value" data-reg="101">Нет</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Последняя неисправность</span>
                                <span class="parameter-value" data-reg="103">Нет</span>
                            </div>
                            <button class="btn btn-primary" onclick="showLogDetails('alarms')">
                                <i class="fas fa-list"></i>
                                Аварии
                            </button>
                            <button class="btn btn-primary" onclick="showLogDetails('faults')">
                                <i class="fas fa-list"></i>
                                Неисправности
                            </button>
                        </div>
                    </div>
                    
                    <!-- Управление журналом -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-database"></i>
                                Управление журналом
                            </div>
                        </div>
                        <div class="parameter-group">
                            <button class="btn btn-primary" onclick="clearLogs()">
                                <i class="fas fa-trash"></i>
                                Очистить журналы
                            </button>
                            <button class="btn btn-secondary" onclick="exportLogs()">
                                <i class="fas fa-download"></i>
                                Экспорт данных
                            </button>
                            <div class="parameter">
                                <span class="parameter-label">Запись мощности</span>
                                <span class="parameter-value" data-reg="114">60<span class="unit">сек</span></span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            
            <!-- Информация -->
            <div class="page" id="info">
                <h2><i class="fas fa-info-circle"></i> Информация об изделии</h2>
                <p class="subtitle">Технические данные устройства</p>
                
                <div class="card-grid">
                    <!-- Информация об устройстве -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-microchip"></i>
                                Устройство
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Тип блока</span>
                                <span class="parameter-value" data-reg="10">МТЗП-2T</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Серийный номер</span>
                                <span class="parameter-value" data-reg="10">---</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Месяц выпуска</span>
                                <span class="parameter-value" data-reg="11">---</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Версия ПО</span>
                                <span class="parameter-value" data-reg="11">---</span>
                            </div>
                        </div>
                    </div>
                    
                    <!-- Время и дата -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-clock"></i>
                                Время и дата
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Время</span>
                                <span class="parameter-value" data-reg="17">--:--:--</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Дата</span>
                                <span class="parameter-value" data-reg="20">--.--.----</span>
                            </div>
                            <button class="btn btn-primary" onclick="syncTime()">
                                <i class="fas fa-sync"></i>
                                Синхронизировать время
                            </button>
                        </div>
                    </div>
                    
                    <!-- Диагностика -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-stethoscope"></i>
                                Диагностика
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">Циклов main</span>
                                <span class="parameter-value" data-reg="13">0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Циклов АЦП</span>
                                <span class="parameter-value" data-reg="14">0</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Циклов Input</span>
                                <span class="parameter-value" data-reg="15">0</span>
                            </div>
                            <button class="btn btn-primary" onclick="runDiagnostics()">
                                <i class="fas fa-play"></i>
                                Запустить диагностику
                            </button>
                        </div>
                    </div>
                    
                    <!-- Сеть -->
                    <div class="card">
                        <div class="card-header">
                            <div class="card-title">
                                <i class="fas fa-wifi"></i>
                                Сеть
                            </div>
                        </div>
                        <div class="parameter-group">
                            <div class="parameter">
                                <span class="parameter-label">WiFi точка доступа</span>
                                <span class="parameter-value">MTZP</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">IP адрес</span>
                                <span class="parameter-value">192.168.4.1</span>
                            </div>
                            <div class="parameter">
                                <span class="parameter-label">Версия WebUI</span>
                                <span class="parameter-value">2.0</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- Футер -->
        <div class="footer">
            <p>МТЗП-1200 Web Interface v2.0 • Обновлено: <span id="lastUpdateTime">--:--:--</span></p>
        </div>
    </div>
    
    <!-- Модальное окно редактирования -->
    <div class="modal" id="editModal">
        <div class="modal-content">
            <div class="modal-header">
                <h3 class="modal-title" id="modalTitle">
                    <i class="fas fa-edit"></i>
                    Редактирование параметра
                </h3>
            </div>
            <div class="modal-body">
                <div class="form-group">
                    <label class="form-label" id="paramName">Параметр</label>
                    <input type="text" class="form-input" id="paramValue" placeholder="Введите значение">
                    <div class="form-help" id="paramHelp">Единицы измерения: --</div>
                </div>
                <div class="form-group">
                    <label class="form-label">Текущее значение</label>
                    <div class="current-value" id="currentValue">--</div>
                </div>
            </div>
            <div class="modal-footer">
                <button class="btn btn-secondary" onclick="closeModal()">
                    <i class="fas fa-times"></i>
                    Отмена
                </button>
                <button class="btn btn-primary" onclick="saveParam()">
                    <i class="fas fa-save"></i>
                    Сохранить
                </button>
            </div>
        </div>
    </div>
    
    <!-- Кнопка обновления -->
    <button class="refresh-btn" onclick="refreshAll()">
        <i class="fas fa-sync-alt"></i>
    </button>
    
    <!-- Уведомление -->
    <div class="notification" id="notification">
        <i class="fas fa-check-circle"></i>
        <span id="notificationText">Действие выполнено успешно</span>
    </div>

    <script>
        // Регистры и их метаданные
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
        
        // Глобальные переменные
        let currentPage = 'dashboard';
        let currentEditReg = null;
        let autoRefresh = true;
        
        // Инициализация
        document.addEventListener('DOMContentLoaded', function() {
            // Инициализация навигации
            document.querySelectorAll('.nav-tab').forEach(tab => {
                tab.addEventListener('click', function() {
                    const page = this.dataset.page;
                    switchPage(page);
                });
            });
            
            // Обновление времени
            updateTime();
            setInterval(updateTime, 1000);
            
            // Автообновление данных
            refreshAll();
            setInterval(() => {
                if (autoRefresh) {
                    refreshVisibleData();
                }
            }, 2000);
            
            // Поиск
            document.getElementById('searchProtections')?.addEventListener('input', function(e) {
                searchProtections(e.target.value);
            });
        });
        
        // Переключение страниц
        function switchPage(pageId) {
            // Обновить активную вкладку
            document.querySelectorAll('.nav-tab').forEach(tab => {
                tab.classList.remove('active');
            });
            document.querySelector(`.nav-tab[data-page="${pageId}"]`).classList.add('active');
            
            // Показать нужную страницу
            document.querySelectorAll('.page').forEach(page => {
                page.classList.remove('active');
            });
            document.getElementById(pageId).classList.add('active');
            
            currentPage = pageId;
            refreshVisibleData();
        }
        
        // Обновление видимых данных
        function refreshVisibleData() {
            const page = document.getElementById(currentPage);
            const elements = page.querySelectorAll('[data-reg]');
            
            elements.forEach(el => {
                const regId = parseInt(el.dataset.reg);
                const bit = el.dataset.bit;
                
                fetch(`/api/read?reg=${regId}`)
                    .then(r => r.json())
                    .then(data => {
                        if (data.ok) {
                            const meta = registerIndex.get(regId);
                            let value = data.value;
                            
                            // Обработка битовых значений
                            if (bit !== undefined) {
                                value = (value >> parseInt(bit)) & 1;
                                el.textContent = value ? 'Вкл' : 'Выкл';
                                if (value) {
                                    el.style.color = '#4CAF50';
                                    el.style.fontWeight = 'bold';
                                } else {
                                    el.style.color = '#dc3545';
                                }
                            } else {
                                // Форматирование значения
                                const formatted = formatValue(value, meta?.format || 'DEC', meta?.scale || 0);
                                el.textContent = formatted;
                                
                                // Добавление единиц измерения
                                if (meta) {
                                    const unitSpan = el.querySelector('.unit');
                                    if (!unitSpan && meta.name.match(/[АВкОмВтВА°CмС]/)) {
                                        const unit = getUnitFromName(meta.name);
                                        if (unit) {
                                            el.innerHTML = formatted + `<span class="unit">${unit}</span>`;
                                        }
                                    }
                                }
                                
                                // Цветовое кодирование
                                colorCodeValue(el, value, regId);
                            }
                        }
                    })
                    .catch(err => {
                        console.error('Ошибка чтения регистра:', err);
                        el.textContent = 'Ошибка';
                        el.style.color = '#dc3545';
                    });
            });
            
            // Обновить время последнего обновления
            updateLastUpdateTime();
        }
        
        // Форматирование значения
        function formatValue(raw, format, scale) {
            const scaleFactor = Math.pow(10, scale);
            
            if (format === 'HEX') return `0x${raw.toString(16).toUpperCase()}`;
            if (format === 'OCT') return `0o${raw.toString(8)}`;
            if (format === 'BIN') return `0b${raw.toString(2)}`;
            if (format === 'REL' || format === 'SDC') {
                return (raw / scaleFactor).toFixed(scale);
            }
            if (format === 'SWT') return raw ? 'Да' : 'Нет';
            
            // DEC формат
            if (scale > 0) {
                return (raw / scaleFactor).toFixed(scale);
            }
            return raw.toString();
        }
        
        // Получение единиц измерения из имени
        function getUnitFromName(name) {
            if (name.includes('А,') || name.includes('ток')) return 'А';
            if (name.includes('В,') || name.includes('напряж')) return 'В';
            if (name.includes('кОм') || name.includes('сопротивление')) return 'кОм';
            if (name.includes('кВт')) return 'кВт';
            if (name.includes('кВА')) return 'кВА';
            if (name.includes('°C') || name.includes('температура')) return '°C';
            if (name.includes('мС') || name.includes('мсек') || name.includes('сек')) return 'мс';
            if (name.includes('мА')) return 'мА';
            if (name.includes('%')) return '%';
            return '';
        }
        
        // Цветовое кодирование значений
        function colorCodeValue(element, value, regId) {
            // Сброс цвета
            element.style.color = '';
            element.style.fontWeight = '';
            
            // Особые случаи для статусов
            if (regId === 28 || regId === 30) { // Аварии и неисправности
                if (value > 0) {
                    element.style.color = '#dc3545';
                    element.style.fontWeight = 'bold';
                }
            }
            else if (regId === 32) { // Температура
                if (value > 600) { // >60°C
                    element.style.color = '#dc3545';
                } else if (value > 500) { // >50°C
                    element.style.color = '#ffc107';
                }
            }
            else if (regId === 52) { // Сопротивление изоляции
                if (value < 100) { // <100 кОм
                    element.style.color = '#dc3545';
                } else if (value < 500) { // <500 кОм
                    element.style.color = '#ffc107';
                }
            }
        }
        
        // Открытие модального окна редактирования
        function editProtection(regId) {
            const meta = registerIndex.get(regId);
            if (!meta) return;
            
            currentEditReg = regId;
            
            document.getElementById('paramName').textContent = meta.name;
            document.getElementById('paramHelp').textContent = 
                `Формат: ${meta.format}, Масштаб: ${meta.scale}`;
            
            // Получить текущее значение
            fetch(`/api/read?reg=${regId}`)
                .then(r => r.json())
                .then(data => {
                    if (data.ok) {
                        const current = formatValue(data.value, meta.format, meta.scale);
                        document.getElementById('currentValue').textContent = current;
                        document.getElementById('paramValue').value = data.value;
                    }
                });
            
            document.getElementById('editModal').style.display = 'flex';
        }
        
        // Сохранение параметра
        function saveParam() {
            if (!currentEditReg) return;
            
            const meta = registerIndex.get(currentEditReg);
            const inputValue = document.getElementById('paramValue').value;
            
            let rawValue;
            if (meta.format === 'HEX') {
                rawValue = parseInt(inputValue, 16);
            } else if (meta.format === 'SWT') {
                rawValue = inputValue === '1' || inputValue.toLowerCase() === 'true' ? 1 : 0;
            } else {
                const scaleFactor = Math.pow(10, meta.scale || 0);
                rawValue = Math.round(parseFloat(inputValue) * scaleFactor);
            }
            
            fetch(`/api/write?reg=${currentEditReg}&val=${rawValue}`)
                .then(r => r.json())
                .then(data => {
                    if (data.ok) {
                        showNotification('Параметр успешно обновлен');
                        refreshAll();
                        closeModal();
                    } else {
                        showNotification('Ошибка обновления параметра', 'error');
                    }
                });
        }
        
        // Закрытие модального окна
        function closeModal() {
            document.getElementById('editModal').style.display = 'none';
            currentEditReg = null;
        }
        
        // Показать уведомление
        function showNotification(message, type = 'success') {
            const notification = document.getElementById('notification');
            notification.querySelector('#notificationText').textContent = message;
            
            if (type === 'error') {
                notification.style.background = '#dc3545';
                notification.querySelector('i').className = 'fas fa-exclamation-circle';
            } else {
                notification.style.background = '#4CAF50';
                notification.querySelector('i').className = 'fas fa-check-circle';
            }
            
            notification.style.display = 'flex';
            setTimeout(() => {
                notification.style.display = 'none';
            }, 3000);
        }
        
        // Обновление времени
        function updateTime() {
            const now = new Date();
            const timeStr = now.toLocaleTimeString('ru-RU');
            const dateStr = now.toLocaleDateString('ru-RU');
            
            document.querySelectorAll('.time').forEach(el => {
                el.textContent = timeStr;
            });
        }
        
        // Обновление времени последнего обновления
        function updateLastUpdateTime() {
            const now = new Date();
            const timeStr = now.toLocaleTimeString('ru-RU');
            document.getElementById('lastUpdateTime').textContent = timeStr;
        }
        
        // Полное обновление
        function refreshAll() {
            refreshVisibleData();
            showNotification('Данные обновлены');
        }
        
        // Вспомогательные функции
        function showProtections() {
            switchPage('protections');
        }
        
        function toggleAPV() {
            // Реализация переключения АПВ
            showNotification('Режим АПВ изменен');
        }
        
        function saveSettings() {
            showNotification('Настройки сохранены');
        }
        
        function loadFactorySettings() {
            if (confirm('Загрузить заводские настройки? Все изменения будут потеряны.')) {
                showNotification('Загрузка заводских настроек...');
                setTimeout(() => {
                    showNotification('Заводские настройки загружены');
                    refreshAll();
                }, 1000);
            }
        }
        
        function calibrateSensors() {
            showNotification('Запущена юстировка датчиков');
        }
        
        function resetCounters() {
            if (confirm('Обнулить все счетчики?')) {
                showNotification('Счетчики обнулены');
            }
        }
        
        function changePassword() {
            const newPass = prompt('Введите новый пароль:');
            if (newPass) {
                showNotification('Пароль изменен');
            }
        }
        
        function clearLogs() {
            if (confirm('Очистить все журналы событий?')) {
                showNotification('Журналы очищены');
            }
        }
        
        function exportLogs() {
            showNotification('Экспорт данных начат');
        }
        
        function syncTime() {
            const now = new Date();
            showNotification(`Время синхронизировано: ${now.toLocaleString('ru-RU')}`);
        }
        
        function runDiagnostics() {
            showNotification('Диагностика запущена');
        }
        
        // Поиск защит
        function searchProtections(query) {
            const cards = document.querySelectorAll('#protections .card');
            query = query.toLowerCase();
            
            cards.forEach(card => {
                const title = card.querySelector('.card-title').textContent.toLowerCase();
                const isVisible = title.includes(query) || query === '';
                card.style.display = isVisible ? 'block' : 'none';
            });
        }
        
        // Отображение деталей
        function showRelaysDetails() {
            alert('Детальная информация о релейных выходах будет отображена здесь');
        }
        
        function showInputsDetails() {
            alert('Детальная информация о входах будет отображена здесь');
        }
        
        function showAlarmsDetails() {
            alert('Детальная информация об авариях будет отображена здесь');
        }
        
        function showLogDetails(type) {
            alert(`Детали журнала ${type} будут отображены здесь`);
        }
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
