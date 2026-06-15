import re
import os

input_file_path = "/root/LTS-Respooler/Firmware/(.txt) Control Board Code"
output_dir = "/root/LTS-Respooler/Firmware/build/src"
output_file_path = os.path.join(output_dir, "main.cpp")

# Create output dir if not exist
os.makedirs(output_dir, exist_ok=True)

with open(input_file_path, "r", encoding="utf-8") as f:
    code = f.read()

# 1. Modify includes and add forward declarations
code = code.replace("#include <Arduino.h>", """#include <Arduino.h>
#include <WebServer.h>
#include <Update.h>

// --- Forward Declarations for PlatformIO C++ Compilation ---
void handleCommand(const std::string& cmd);
void sendStatus(bool forceSend = false);
void loadSettings();
void applySpeedTarget();
void setMotorCurrent(int strengthPercent);
int calculatePWM(int pct);
void setConnLed(uint8_t duty);
uint32_t stepFreqFromIntervalUs(unsigned long us);
void stepperSetFreq(uint32_t freqHz);
void stepperStart(uint32_t freqHz);
void stepperStop();
void reSanitizeDriver();
void playTone(unsigned int freqHz, unsigned long durationMs);
void playStepperJingle();
int fanPWMFromSpeed(int speedPercent);
void setServoAngle(int angle);
int getSpeedPercentFromInterval(unsigned long us);
void wifiScanTask(void* parameter);
void sendOTAUpdate();
void otaLedPulseTask(void* parameter);
""")

# 2. Modify Board info & Pins to be conditional
pins_old = """// ------------------------ Board Info ----------------------------
#define FIRMWARE_VERSION "1.2.1"
#define BOARD_NAME "LTS CB"

// --------------------- Hardware Pin Defines ---------------------
#define TMC_UART_RX 18
#define TMC_UART_TX 17
#define STEP_PIN 14
#define DIR_PIN 21
#define EN_PIN 40
#define LED_PIN 9
#define LED_CONN_PIN 47
#define FILAMENT_PIN 11
#define BUTTON_PIN 10
#define FAN_PIN 13
#define SERVO_PIN 12"""

pins_new = """// ------------------------ Board Info ----------------------------
#define FIRMWARE_VERSION "1.2.1-Web"

#ifdef BOARD_VARIANT_CB
  #define BOARD_NAME "LTS CB (Control)"
  #define TMC_UART_RX 18
  #define TMC_UART_TX 17
  #define STEP_PIN 14
  #define DIR_PIN 21
  #define EN_PIN 40
  #define LED_PIN 9
  #define LED_CONN_PIN 47
  #define FILAMENT_PIN 11
  #define BUTTON_PIN 10
  #define FAN_PIN 13
  #define SERVO_PIN 12
#elif defined(BOARD_VARIANT_DB)
  #define BOARD_NAME "LTS DB (Driver)"
  #define TMC_UART_RX 21
  #define TMC_UART_TX 22
  #define STEP_PIN 14
  #define DIR_PIN 27
  #define EN_PIN 13
  #define LED_PIN 19
  #define LED_CONN_PIN 2
  #define FILAMENT_PIN 33
  #define BUTTON_PIN 25
  #define FAN_PIN 32
  #define SERVO_PIN 26
#elif defined(BOARD_VARIANT_S3)
  #define BOARD_NAME "LTS Generic S3"
  #define TMC_UART_RX 4
  #define TMC_UART_TX 4
  #define STEP_PIN 1
  #define DIR_PIN 2
  #define EN_PIN 3
  #define LED_PIN 5
  #define LED_CONN_PIN 6
  #define FILAMENT_PIN 7
  #define BUTTON_PIN 8
  #define FAN_PIN 9
  #define SERVO_PIN 10
#elif defined(BOARD_VARIANT_C3)
  #define BOARD_NAME "LTS Generic C3"
  #define TMC_UART_RX 3
  #define TMC_UART_TX 3
  #define STEP_PIN 0
  #define DIR_PIN 1
  #define EN_PIN 2
  #define LED_PIN 4
  #define LED_CONN_PIN 5
  #define FILAMENT_PIN 6
  #define BUTTON_PIN 7
  #define FAN_PIN 8
  #define SERVO_PIN 9
#else
  #define BOARD_NAME "LTS DB (Driver)"
  #define TMC_UART_RX 21
  #define TMC_UART_TX 22
  #define STEP_PIN 14
  #define DIR_PIN 27
  #define EN_PIN 13
  #define LED_PIN 19
  #define LED_CONN_PIN 2
  #define FILAMENT_PIN 33
  #define BUTTON_PIN 25
  #define FAN_PIN 32
  #define SERVO_PIN 26
#endif"""

code = code.replace(pins_old, pins_new)

# 3. Increase Filament loss confirm timeout to 1500 ms to avoid glitches
code = code.replace('#define FILAMENT_LOSS_CONFIRM_MS 100', '#define FILAMENT_LOSS_CONFIRM_MS 1500')

# 3b. Add single-wire UART configuration for S3/C3
serial_old = "TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);"
serial_new = """TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
#if defined(BOARD_VARIANT_S3) || defined(BOARD_VARIANT_C3)
  // Single-wire UART configuration for TMC2209
  gpio_set_direction((gpio_num_t)TMC_UART_TX, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_pullup_en((gpio_num_t)TMC_UART_TX);
#endif"""
code = code.replace(serial_old, serial_new)

# Remove duplicate default parameter value in function definition to satisfy compiler
code = code.replace("void sendStatus(bool forceSend = false) {", "void sendStatus(bool forceSend) {")

# 4. Insert HTML page and WebServer logic, logging ring buffer variables, and reset reason detector
diagnostics_code = """
// ----------------- Diagnostics & WebServer -----------------
#define MAX_LOG_LINES 50
String logLines[MAX_LOG_LINES];
int logHead = 0;
int logCount = 0;
SemaphoreHandle_t logMutex = nullptr;

void addToLog(const String& msg) {
  if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
  Serial.println(msg);
  logLines[logHead] = msg;
  logHead = (logHead + 1) % MAX_LOG_LINES;
  if (logCount < MAX_LOG_LINES) logCount++;
  if (logMutex) xSemaphoreGive(logMutex);
}

String getResetReasonString() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "Unbekannter Reset";
    case ESP_RST_POWERON:   return "Power-on Reset (Netzteil eingeschaltet)";
    case ESP_RST_EXT:       return "Externer Pin Reset";
    case ESP_RST_SW:        return "Software Reset";
    case ESP_RST_PANIC:     return "Software Panic / Absturz (Crash)";
    case ESP_RST_INT_WDT:   return "Interrupt Watchdog Reset";
    case ESP_RST_TASK_WDT:  return "Task Watchdog Reset";
    case ESP_RST_WDT:       return "Anderer Watchdog Reset";
    case ESP_RST_DEEPSLEEP: return "Deep Sleep Wake Reset";
    case ESP_RST_BROWNOUT:  return "Brownout Reset (Spannungseinbruch! Netzteil zu schwach!)";
    case ESP_RST_SDIO:      return "SDIO Reset";
    default:                return "Undefinierter Reset-Grund";
  }
}

WebServer webServer(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>LTS Respooler Pro Console</title>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-base: #0a0e17;
            --bg-card: rgba(20, 27, 45, 0.65);
            --border-card: rgba(255, 255, 255, 0.08);
            --primary: #00f2fe;
            --secondary: #4facfe;
            --text-main: #f3f4f6;
            --text-muted: #9ca3af;
            --success: #10b981;
            --error: #ef4444;
            --warning: #f59e0b;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Outfit', sans-serif; }
        body {
            background-color: var(--bg-base);
            color: var(--text-main);
            min-height: 100vh;
            padding: 2rem 1rem;
            background-image: radial-gradient(circle at 10% 20%, rgba(0, 242, 254, 0.04) 0%, transparent 40%),
                              radial-gradient(circle at 90% 80%, rgba(79, 172, 254, 0.04) 0%, transparent 40%);
        }
        .container { max-width: 900px; margin: 0 auto; }
        header { text-align: center; margin-bottom: 2rem; }
        h1 {
            font-size: 2.25rem;
            font-weight: 700;
            background: linear-gradient(135deg, var(--primary), var(--secondary));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 0.25rem;
            letter-spacing: -0.03em;
        }
        .subtitle { color: var(--text-muted); font-size: 1rem; font-weight: 300; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 1.25rem; margin-bottom: 1.25rem; }
        .card {
            background: var(--bg-card);
            border: 1px solid var(--border-card);
            border-radius: 16px;
            padding: 1.25rem;
            backdrop-filter: blur(12px);
            box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.4);
            transition: transform 0.2s, box-shadow 0.2s;
        }
        .card:hover { transform: translateY(-2px); box-shadow: 0 12px 40px 0 rgba(0, 242, 254, 0.08); }
        .card-title {
            font-size: 1.1rem;
            font-weight: 600;
            margin-bottom: 1rem;
            border-bottom: 1px solid var(--border-card);
            padding-bottom: 0.5rem;
            display: flex;
            align-items: center;
            justify-content: space-between;
            text-transform: uppercase;
            letter-spacing: 0.05em;
            color: var(--secondary);
        }
        .metric { display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.75rem; }
        .metric-label { color: var(--text-muted); font-size: 0.9rem; }
        .metric-value { font-weight: 600; font-size: 1rem; }
        .status-dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; margin-right: 0.5rem; }
        .status-dot.active { background-color: var(--success); box-shadow: 0 0 8px var(--success); animation: pulse 1.5s infinite; }
        .status-dot.inactive { background-color: var(--text-muted); }
        .status-dot.alarm { background-color: var(--error); box-shadow: 0 0 8px var(--error); animation: pulse 1s infinite; }
        @keyframes pulse {
            0% { opacity: 0.4; }
            50% { opacity: 1; }
            100% { opacity: 0.4; }
        }
        .progress-bar-container { background: rgba(255, 255, 255, 0.04); border-radius: 10px; height: 12px; width: 100%; overflow: hidden; margin: 0.5rem 0; border: 1px solid var(--border-card); }
        .progress-bar { height: 100%; background: linear-gradient(90deg, var(--primary), var(--secondary)); width: 0%; transition: width 0.3s ease; }
        .console {
            background: rgba(5, 7, 12, 0.8);
            border: 1px solid var(--border-card);
            border-radius: 12px;
            height: 220px;
            overflow-y: auto;
            padding: 0.75rem;
            font-family: 'Courier New', Courier, monospace;
            font-size: 0.85rem;
            color: #34d399;
            margin-bottom: 0.75rem;
            box-shadow: inset 0 2px 8px rgba(0,0,0,0.8);
        }
        .console-line { margin-bottom: 0.2rem; line-height: 1.2; }
        .btn-group { display: grid; grid-template-columns: repeat(3, 1fr); gap: 0.5rem; }
        button {
            padding: 0.6rem 0.75rem;
            border-radius: 8px;
            border: none;
            cursor: pointer;
            font-weight: 600;
            font-size: 0.9rem;
            transition: all 0.2s;
            text-align: center;
        }
        .btn-primary { background: linear-gradient(135deg, var(--primary), var(--secondary)); color: #05070c; }
        .btn-primary:hover { opacity: 0.9; box-shadow: 0 0 12px rgba(0, 242, 254, 0.3); }
        .btn-danger { background: var(--error); color: white; }
        .btn-danger:hover { box-shadow: 0 0 12px rgba(239, 68, 68, 0.3); }
        .btn-warning { background: var(--warning); color: black; }
        .btn-warning:hover { box-shadow: 0 0 12px rgba(245, 158, 11, 0.3); }
        .file-upload-label {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            border: 2px dashed var(--border-card);
            border-radius: 12px;
            padding: 1.5rem;
            cursor: pointer;
            transition: border-color 0.2s;
            color: var(--text-muted);
            text-align: center;
        }
        .file-upload-label:hover { border-color: var(--primary); color: var(--text-main); }
        .file-upload input[type="file"] { display: none; }
        .upload-prog-container { display: none; margin-top: 1rem; }
        .upload-status { font-size: 0.85rem; margin-top: 0.5rem; text-align: center; }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>LTS Respooler Pro</h1>
            <div class="subtitle" id="fw-info">Management Console & Diagnostics</div>
        </header>
        <div class="grid">
            <div class="card">
                <div class="card-title">Live Status</div>
                <div class="metric">
                    <span class="metric-label">System-Zustand</span>
                    <span class="metric-value" id="sys-state" style="display:flex; align-items:center;">
                        <span class="status-dot inactive" id="state-dot"></span>
                        <span id="state-text">Lade...</span>
                    </span>
                </div>
                <div class="metric">
                    <span class="metric-label">Filament Sensor</span>
                    <span class="metric-value" id="filament-status">Lade...</span>
                </div>
                <div class="metric">
                    <span class="metric-label">ESP32 Temperatur</span>
                    <span class="metric-value" id="temp-status">Lade...</span>
                </div>
                <div class="metric">
                    <span class="metric-label">Boot Grund</span>
                    <span class="metric-value" id="boot-status" style="font-size:0.8rem; text-align:right;">Lade...</span>
                </div>
            </div>
            
            <div class="card">
                <div class="card-title">Wickel-Fortschritt</div>
                <div class="metric">
                    <span class="metric-label">Fortschritt</span>
                    <span class="metric-value" id="progress-percent">0%</span>
                </div>
                <div class="progress-bar-container">
                    <div class="progress-bar" id="progress-bar"></div>
                </div>
                <div class="metric">
                    <span class="metric-label">Restlaufzeit</span>
                    <span class="metric-value" id="rem-time">0s</span>
                </div>
            </div>

            <div class="card">
                <div class="card-title">Steuerung</div>
                <div class="btn-group">
                    <button class="btn-primary" onclick="sendCmd('START')">Start</button>
                    <button class="btn-warning" onclick="sendCmd('PAUSE')">Pause</button>
                    <button class="btn-danger" onclick="sendCmd('STOP')">Stop</button>
                </div>
            </div>
        </div>

        <div class="card" style="margin-bottom: 1.25rem;">
            <div class="card-title">
                <span>Echtzeit-Diagnose-Logs</span>
                <button onclick="clearLogs()" style="padding:0.25rem 0.5rem; font-size:0.75rem; background:rgba(255,255,255,0.05); color:var(--text-muted); border:1px solid var(--border-card);">Logs leeren</button>
            </div>
            <div class="console" id="console">
                <div class="console-line"><span class="console-msg">Warte auf Log-Daten...</span></div>
            </div>
        </div>

        <div class="card">
            <div class="card-title">Firmware Update (OTA)</div>
            <label class="file-upload-label" id="drop-zone">
                <span id="upload-label-text">Klicke oder ziehe die .bin Firmware-Datei hierher</span>
                <input type="file" id="firmware-file" accept=".bin">
            </label>
            <div class="upload-prog-container" id="upload-prog-container">
                <div class="progress-bar-container">
                    <div class="progress-bar" id="upload-bar" style="width: 0%"></div>
                </div>
                <div class="upload-status" id="upload-status">Lade hoch...</div>
            </div>
        </div>
    </div>

    <script>
        function sendCmd(cmdName) {
            fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ CMD: cmdName })
            })
            .then(res => res.json())
            .catch(err => console.error('Fehler beim Senden des Befehls:', err));
        }

        function updateStatus() {
            fetch('/status')
            .then(res => res.json())
            .then(data => {
                document.getElementById('fw-info').innerText = `Konsole & Diagnosedaten (FW: ${data.FW} | Board: ${data.VAR})`;
                const stateText = document.getElementById('state-text');
                const stateDot = document.getElementById('state-dot');
                stateText.innerText = stateMap(data.STAT);
                
                stateDot.className = 'status-dot';
                if (data.STAT === 'R') {
                    stateDot.classList.add('active');
                } else if (data.STAT === 'A') {
                    stateDot.classList.add('alarm');
                } else {
                    stateDot.classList.add('inactive');
                }

                const filStatus = document.getElementById('filament-status');
                if (data.HAS_FIL) {
                    filStatus.innerText = 'Filament eingelegt';
                    filStatus.style.color = 'var(--success)';
                } else {
                    filStatus.innerText = 'Fehlt / Runout';
                    filStatus.style.color = 'var(--error)';
                }

                document.getElementById('temp-status').innerText = data.TEMP + ' °C';
                document.getElementById('boot-status').innerText = data.BOOT_REASON || 'Unbekannt';

                const prog = parseFloat(data.PROG).toFixed(1);
                document.getElementById('progress-percent').innerText = prog + '%';
                document.getElementById('progress-bar').style.width = prog + '%';
                
                const rem = data.REM;
                if (rem > 0) {
                    const m = Math.floor(rem / 60);
                    const s = rem % 60;
                    document.getElementById('rem-time').innerText = `${m}m ${s}s`;
                } else {
                    document.getElementById('rem-time').innerText = '--';
                }
            })
            .catch(err => console.error('Fehler beim Abrufen des Status:', err));
        }

        function stateMap(stat) {
            switch(stat) {
                case 'I': return 'Bereit (Idle)';
                case 'P': return 'Pausiert';
                case 'A': return 'Auto-Stop (StallGuard)';
                case 'D': return 'Fertig (Done)';
                case 'R': return 'Spult (Running)';
                case 'U': return 'OTA Update...';
                default: return 'Unbekannt (' + stat + ')';
            }
        }

        function updateLogs() {
            fetch('/logs')
            .then(res => res.json())
            .then(data => {
                const con = document.getElementById('console');
                let html = '';
                
                if (!data.logs || data.logs.length === 0) {
                    con.innerHTML = '<div class="console-line"><span class="console-msg">Keine Log-Einträge vorhanden.</span></div>';
                    return;
                }

                data.logs.forEach(line => {
                    let cls = 'console-msg';
                    if (line.includes('stalled') || line.includes('runout') || line.includes('Brownout') || line.includes('Reset') || line.includes('Absturz')) {
                        cls += ' error';
                    } else if (line.includes('Connected') || line.includes('success') || line.includes('fertig') || line.includes('started')) {
                        cls += ' success';
                    } else if (line.includes('changed') || line.includes('Pausiert') || line.includes('State')) {
                        cls += ' warn';
                    }
                    html += `<div class="console-line"><span class="${cls}">${line}</span></div>`;
                });
                
                const shouldScroll = con.scrollHeight - con.clientHeight - con.scrollTop < 40;
                con.innerHTML = html;
                if (shouldScroll) {
                    con.scrollTop = con.scrollHeight;
                }
            })
            .catch(err => console.error('Fehler beim Abrufen der Logs:', err));
        }

        function clearLogs() {
            document.getElementById('console').innerHTML = '';
        }

        const fileInput = document.getElementById('firmware-file');
        const dropZone = document.getElementById('drop-zone');
        const labelText = document.getElementById('upload-label-text');
        const progContainer = document.getElementById('upload-prog-container');
        const uploadBar = document.getElementById('upload-bar');
        const uploadStatus = document.getElementById('upload-status');

        fileInput.addEventListener('change', (e) => {
            const file = e.target.files[0];
            if (file) uploadFile(file);
        });

        function uploadFile(file) {
            const xhr = new XMLHttpRequest();
            const formData = new FormData();
            formData.append('update', file);

            progContainer.style.display = 'block';
            labelText.innerText = file.name;
            uploadStatus.innerText = 'Lade hoch...';

            xhr.upload.addEventListener('progress', (e) => {
                if (e.lengthComputable) {
                    const percent = (e.loaded / e.total) * 100;
                    uploadBar.style.width = percent + '%';
                    uploadStatus.innerText = `Lade hoch: ${Math.round(percent)}%`;
                }
            });

            xhr.addEventListener('load', () => {
                if (xhr.status === 200) {
                    uploadStatus.innerText = 'Erfolgreich! Gerät startet neu...';
                    uploadStatus.style.color = 'var(--success)';
                } else {
                    uploadStatus.innerText = 'Fehler beim Hochladen!';
                    uploadStatus.style.color = 'var(--error)';
                }
            });

            xhr.addEventListener('error', () => {
                uploadStatus.innerText = 'Fehler beim Upload!';
                uploadStatus.style.color = 'var(--error)';
            });

            xhr.open('POST', '/update');
            xhr.send(formData);
        }

        dropZone.addEventListener('dragover', (e) => {
            e.preventDefault();
            dropZone.style.borderColor = 'var(--primary)';
        });
        dropZone.addEventListener('dragleave', () => {
            dropZone.style.borderColor = 'var(--border-card)';
        });
        dropZone.addEventListener('drop', (e) => {
            e.preventDefault();
            dropZone.style.borderColor = 'var(--border-card)';
            const file = e.dataTransfer.files[0];
            if (file && file.name.endsWith('.bin')) {
                uploadFile(file);
            } else {
                alert('Bitte nur .bin Dateien hochladen!');
            }
        });

        setInterval(updateStatus, 1000);
        setInterval(updateLogs, 1500);
        updateStatus();
        updateLogs();
    </script>
</body>
</html>
)rawliteral";

void handleStatus() {
  StaticJsonDocument<1024> doc;
  float progVal = progress;
  int remVal = 0;
  if (currentState == 'D') {
    progVal = 100.0f;
    remVal = 0;
  } else if (totalEstimatedTime > 0) {
    unsigned long elapsed = 0;
    if (spoolingStartTime > 0) elapsed = millis() - spoolingStartTime;
    else if (pausedElapsed > 0) elapsed = pausedElapsed;
    else if (currentState == 'A') elapsed = pausedElapsed;
    unsigned long effectiveTotal = totalEstimatedTime;
    if (targetWeight == 2) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_2);
    else if (targetWeight == 3) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_3);
    
    if (effectiveTotal == 0) {
      progVal = 0.0f;
      remVal = 0;
    } else if (elapsed >= effectiveTotal) {
      progVal = 100.0f;
      remVal = 0;
    } else {
      progVal = (100.0f * (float)elapsed) / (float)effectiveTotal;
      remVal = (int)((effectiveTotal - elapsed) / 1000UL);
    }
  }
  if (remHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < remHoldExpiry) remVal = lastRemainingTime;
    else remHoldActive = false;
  }
  if (progHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < progHoldExpiry) progVal = lastProgress;
    else progHoldActive = false;
  }
  if (currentState == 'R') {
    unsigned long nowMs = millis();
    if (nowMs < remStartHoldUntil) remVal = lastRemainingTime;
  }
  
  doc["STAT"] = String(currentState);
  doc["HAS_FIL"] = filamentDetected;
  doc["USE_FIL"] = useFilamentSensor;
  doc["PROG"] = progVal;
  doc["REM"] = remVal;
  doc["TEMP"] = (int)temperatureRead();
  doc["WIFI_SSID"] = wifiSSID.length() > 0 ? wifiSSID.c_str() : nullptr;
  doc["WIFI_OK"] = WiFi.status() == WL_CONNECTED;
  doc["FW"] = FIRMWARE_VERSION;
  doc["SPD"] = speedPercent;
  doc["JIN"] = jingleStyle;
  doc["LED"] = ledBrightness;
  doc["DIR"] = motorDirection;
  doc["POW"] = motorStrength;
  doc["TRQ"] = highSpeedMode ? 0 : torqueLimit;
  doc["WGT"] = targetWeight;
  doc["DUR"] = calibrationAt80Speed / 1000;
  doc["HS"] = highSpeedMode;
  doc["FAN_SPD"] = fanSpeed;
  doc["FAN_ON"] = (fanAlwaysOn || isMotorRunning || millis() < fanStopAfter);
  doc["FAN_ALW"] = fanAlwaysOn;
  doc["SV_R"] = servoAngleR;
  doc["SV_L"] = servoAngleL;
  doc["SV_STP"] = servoStepMm;
  doc["SV_HOME"] = servoHomeIsR ? "R" : "L";
  doc["VAR"] = (boardVariant == 2) ? "PRO" : (boardVariant == 1) ? "STD" : "UNK";
  doc["BOOT_REASON"] = getResetReasonString();
  uint32_t ioin = driver.IOIN();
  doc["TMC_OK"] = (ioin != 0 && ioin != 0xFFFFFFFF);
  doc["TMC_IOIN"] = "0x" + String(ioin, HEX);
  doc["HEAP"] = ESP.getFreeHeap();
  
  String jsonOut;
  serializeJson(doc, jsonOut);
  webServer.send(200, "application/json", jsonOut);
}

void handleLogs() {
  String json = "{\\"logs\\":[";
  if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
  int idx = (logHead - logCount + MAX_LOG_LINES) % MAX_LOG_LINES;
  for (int i = 0; i < logCount; i++) {
    String escaped = logLines[idx];
    escaped.replace("\\"", "\\\\\\"");
    escaped.replace("\\n", "\\\\n");
    escaped.replace("\\r", "\\\\r");
    json += "\\"" + escaped + "\\"";
    if (i < logCount - 1) json += ",";
    idx = (idx + 1) % MAX_LOG_LINES;
  }
  if (logMutex) xSemaphoreGive(logMutex);
  json += "]}";
  webServer.send(200, "application/json", json);
}

void handleWebCommand() {
  if (webServer.hasArg("plain")) {
    String body = webServer.arg("plain");
    handleCommand(body.c_str());
    webServer.send(200, "application/json", "{\\"status\\":\\"ok\\"}");
  } else {
    webServer.send(400, "application/json", "{\\"status\\":\\"error\\"}");
  }
}
"""

# Insert the diagnostic code blocks just before the handleCommand definition
code = code.replace("void handleCommand(const std::string& cmd) {", diagnostics_code + "\nvoid handleCommand(const std::string& cmd) {")

# 5. Insert logging for state changes and WiFi changes
# Locate setup() and loop() to hook our diagnostic additions

# Add state logging hook
state_hook = """  static char lastStateLogged = ' ';
  if (currentState != lastStateLogged) {
    String stateStr;
    switch (currentState) {
      case 'I': stateStr = "Bereit (Idle)"; break;
      case 'P': stateStr = "Pausiert"; break;
      case 'A': stateStr = "Auto-Stop (StallGuard)"; break;
      case 'D': stateStr = "Wickeln fertig (Done)"; break;
      case 'R': stateStr = "Rampe (Running)"; break;
      case 'U': stateStr = "OTA Update... Bitte warten"; break;
      default: stateStr = String(currentState); break;
    }
    addToLog("[System] Zustand gewechselt zu: " + stateStr);
    lastStateLogged = currentState;
  }"""

# Add WiFi status logging hook
wifi_hook = """  static bool lastWifiConnected = false;
  bool currentWifiConnected = WiFi.status() == WL_CONNECTED;
  if (currentWifiConnected != lastWifiConnected) {
    lastWifiConnected = currentWifiConnected;
    if (currentWifiConnected) {
      addToLog("[System] WiFi verbunden! IP: " + WiFi.localIP().toString() + " RSSI: " + String(WiFi.RSSI()) + " dBm");
    } else {
      addToLog("[System] WiFi Verbindung verloren.");
    }
  }"""

# Add Filament status logging hook
filament_hook = """  static bool lastFilamentLogState = false;
  if (filamentDetected != lastFilamentLogState) {
    lastFilamentLogState = filamentDetected;
    if (filamentDetected) {
      addToLog("[Sensor] Filament erkannt.");
    } else {
      addToLog("[Sensor] Filament nicht erkannt (fehlt oder Runout).");
    }
  }"""

# Insert hooks at the beginning of loop()
loop_start = "void loop() {\n  unsigned long now = millis();"
loop_start_modified = """void loop() {
  webServer.handleClient();
  unsigned long now = millis();
""" + state_hook + "\n" + wifi_hook + "\n" + filament_hook

code = code.replace(loop_start, loop_start_modified)

# 6. Add boot reason logging and webServer setup inside setup()
setup_init = """void setup() {
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);"""

setup_init_modified = """void setup() {
  logMutex = xSemaphoreCreateMutex();
  addToLog("[System] LTS-Respooler startet... Reset-Grund: " + getResetReasonString());
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);"""

code = code.replace(setup_init, setup_init_modified)

# Register WebServer callbacks in setup
# We will do this right at the end of setup(), just before or after loadSettings/WLAN connect
setup_end = """  pAdvertising->start();
  ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL);
  setConnLed(150);
  currentState = 'I';
}"""

setup_end_modified = """  pAdvertising->start();
  ledc_set_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, FAN_CHANNEL);
  setConnLed(150);
  currentState = 'I';

  // Setup Web Server
  webServer.on("/", HTTP_GET, []() {
    webServer.send(200, "text/html", index_html);
  });
  webServer.on("/status", HTTP_GET, handleStatus);
  webServer.on("/logs", HTTP_GET, handleLogs);
  webServer.on("/api/command", HTTP_POST, handleWebCommand);
  webServer.on("/update", HTTP_POST, []() {
    webServer.sendHeader("Connection", "close");
    webServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
      addToLog("[OTA] Firmware-Upload startet: " + upload.filename);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        addToLog("[OTA] Fehler: " + String(Update.errorString()));
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        addToLog("[OTA] Schreibfehler bei Chunk");
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        addToLog("[OTA] Upload fertig! Groesse: " + String(upload.totalSize) + " Bytes. Reboot...");
      } else {
        addToLog("[OTA] Fehler beim Beenden: " + String(Update.errorString()));
      }
    }
  });
  webServer.begin();
  addToLog("[System] WebServer gestartet auf Port 80.");
}"""

code = code.replace(setup_end, setup_end_modified)

# 7. Add detailed logging to motor start, stop, StallGuard and filament runout in loop
# Motor start log
motor_start = """      stepIntervalMicros = START_INTERVAL_US;
      lastAccelUpdate = now;
      isMotorRunning = true;"""

motor_start_modified = """      stepIntervalMicros = START_INTERVAL_US;
      lastAccelUpdate = now;
      isMotorRunning = true;
      addToLog("[Motor] Start gewaehlt. Ziel-Geschwindigkeit: " + String(speedPercent) + "%");"""

code = code.replace(motor_start, motor_start_modified)

# StallGuard trigger log
stallguard_trigger = """          else if (now - torqueBelowStartTime >= TORQUE_BELOW_MS) {
            isMotorRunning = false;"""

stallguard_trigger_modified = """          else if (now - torqueBelowStartTime >= TORQUE_BELOW_MS) {
            addToLog("[StallGuard] Blockade erkannt! SG-Wert: " + String(sg) + ", Limit: " + String(limit) + ". Motor stoppt.");
            isMotorRunning = false;"""

code = code.replace(stallguard_trigger, stallguard_trigger_modified)

# Filament Runout trigger log
runout_trigger = """  if (isMotorRunning && useFilamentSensor && !filamentDetected && filamentLostSince > 0 && now - filamentLostSince > FILAMENT_LOSS_CONFIRM_MS) {
    isMotorRunning = false;"""

runout_trigger_modified = """  if (isMotorRunning && useFilamentSensor && !filamentDetected && filamentLostSince > 0 && now - filamentLostSince > FILAMENT_LOSS_CONFIRM_MS) {
    addToLog("[Sensor] Filament-Runout waehrend des Betriebs erkannt! Spulvorgang beendet.");
    isMotorRunning = false;"""

code = code.replace(runout_trigger, runout_trigger_modified)

# Filament loss while running but NOT runout yet (brief loss/glitch)
glitch_log = """  if (useFilamentSensor && (currentState == 'P' || currentState == 'A') && !filamentDetected) {"""
glitch_log_modified = """  if (useFilamentSensor && (currentState == 'P' || currentState == 'A') && !filamentDetected) {
    addToLog("[Sensor] Filament im Pausen- oder Alarmzustand verloren.");"""
code = code.replace(glitch_log, glitch_log_modified)


with open(output_file_path, "w", encoding="utf-8") as f:
    f.write(code)

print("Firmware modification completed successfully! Output file is:", output_file_path)
