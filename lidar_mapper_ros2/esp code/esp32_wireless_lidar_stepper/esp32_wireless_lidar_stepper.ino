#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <HardwareSerial.h>

// ---------- WiFi config — EDIT THESE ----------
const char* WIFI_SSID = "Krish";
const char* WIFI_PASSWORD = "bhandara";
const char* LAPTOP_IP = "172.20.10.3";
const int UDP_PORT = 5005;

WiFiUDP udp;
WebServer server(80);

// ---------- Stepper (DRV8825) ----------
const int STEP_PIN = 25;   // <-- VERIFY against your actual wiring
const int DIR_PIN = 26;    // <-- VERIFY against your actual wiring

const int STEPS_PER_REV = 200;     // standard NEMA17 full-step count
const int MICROSTEP = 16;          // matches the 1/16 (MS1=0,MS2=0,MS3=1) config discussed earlier
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEP) / 360.0;  // ≈ 8.89 at 1/16

const unsigned long STEP_INTERVAL_US = 1600;  // time between individual step pulses — tune for speed vs torque

long currentStepPosition = 0;
long targetStepPosition = 0;
unsigned long lastStepMicros = 0;

int panStart = 0;
int panEnd = 360;      // stepper CAN go further, but stays at 180 by default — see note below
float panStep = 0.5;
unsigned long stepDelayMs = 500;

float currentAngleDeg = 0;
int panDirection = 1;
unsigned long lastPanUpdateTime = 0;

bool autoSweepMode = true;
int manualTargetAngle = 90;

// ---------- Lidar UART ----------
HardwareSerial LidarSerial(2);
const int LIDAR_RX_PIN = 16;
const int LIDAR_TX_PIN = 17;
const long LIDAR_BAUD = 460800;

struct SamplePacket {
  float angle_deg;
  uint16_t distance_mm;
  uint8_t quality;
  uint8_t servo_angle;   // name kept as-is for protocol compatibility with your existing ROS nodes
};
const int BATCH_SIZE = 20;
SamplePacket batchBuffer[BATCH_SIZE];
int batchCount = 0;

unsigned long totalPacketsSent = 0;
unsigned long totalSamplesRead = 0;
unsigned long totalSamplesRejectedMoving = 0;   // NEW: track how many got dropped for motion
unsigned long bootTime = 0;

// ---------- Stepper: non-blocking target-following ----------
void setTargetAngle(float angleDeg) {
  targetStepPosition = (long)round(angleDeg * STEPS_PER_DEGREE);
}

// NEW: true only when the stepper has actually reached its target and stopped
bool stepperSettled() {
  return currentStepPosition == targetStepPosition;
}

void updateStepper() {
  if (currentStepPosition == targetStepPosition) return;

  unsigned long now = micros();
  if (now - lastStepMicros < STEP_INTERVAL_US) return;

  bool movingForward = targetStepPosition > currentStepPosition;
  digitalWrite(DIR_PIN, movingForward ? HIGH : LOW);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(3);   // minimum pulse width for DRV8825 — negligible, not a real blocking concern
  digitalWrite(STEP_PIN, LOW);

  currentStepPosition += movingForward ? 1 : -1;
  currentAngleDeg = currentStepPosition / STEPS_PER_DEGREE;

  lastStepMicros = now;
}

// ---------- Lidar protocol (unchanged from before) ----------
void sendLidarCommand(uint8_t cmd) {
  LidarSerial.write(0xA5);
  LidarSerial.write(cmd);
}

bool readResponseDescriptor() {
  uint8_t buf[7];
  int idx = 0;
  unsigned long start = millis();
  while (idx < 7) {
    while (LidarSerial.available()) {
      buf[idx++] = LidarSerial.read();
      if (idx >= 7) break;
    }
    if (millis() - start > 2000) return false;
    yield();
  }
  return buf[0] == 0xA5 && buf[1] == 0x5A;
}

void startLidarScan() {
  while (LidarSerial.available()) LidarSerial.read();
  sendLidarCommand(0x20);
  readResponseDescriptor();
}

bool readScanSample(float &angle_deg, uint16_t &distance_mm, uint8_t &quality) {
  static uint8_t packet[5];
  static int index = 0;

  while (LidarSerial.available()) {
    uint8_t b = LidarSerial.read();

    if (index == 0) {
      uint8_t s = b & 0x01;
      uint8_t sn = (b >> 1) & 0x01;
      if (s == sn) continue;
      packet[0] = b;
      index = 1;
      continue;
    }

    packet[index++] = b;
    if (index < 5) continue;
    index = 0;

    uint8_t S = packet[0] & 0x01;
    uint8_t Sn = (packet[0] >> 1) & 0x01;
    uint8_t C = packet[1] & 0x01;
    if (S == Sn || C == 0) continue;

    quality = packet[0] >> 2;
    uint16_t angle_q6 = ((uint16_t)packet[2] << 7) | ((uint16_t)packet[1] >> 1);
    angle_deg = angle_q6 / 64.0f;
    uint16_t distance_q2 = ((uint16_t)packet[4] << 8) | packet[3];
    distance_mm = distance_q2 / 4;
    return true;
  }
  return false;
}

void sendBatch() {
  if (batchCount == 0) return;
  udp.beginPacket(LAPTOP_IP, UDP_PORT);
  udp.write((uint8_t*)batchBuffer, batchCount * sizeof(SamplePacket));
  udp.endPacket();
  totalPacketsSent++;
  batchCount = 0;
}

// ---------- Web page ----------
const char* PAGE_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>LiDAR Rig Control (Stepper)</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; background: #1a1a1a; color: #eee; text-align: center; padding: 20px; }
    h2 { color: #4CAF50; }
    .card { background: #2a2a2a; border-radius: 10px; padding: 20px; margin: 15px auto; max-width: 400px; }
    input[type=range] { width: 100%; }
    input[type=number] { width: 70px; background: #333; color: #eee; border: 1px solid #555; border-radius: 4px; padding: 4px; }
    button { background: #4CAF50; color: white; border: none; padding: 10px 20px; border-radius: 6px; margin: 5px; font-size: 16px; }
    .stat { display: flex; justify-content: space-between; padding: 4px 0; font-size: 14px; }
    .paramRow { display: flex; justify-content: space-between; align-items: center; padding: 6px 0; }
    #angleVal { font-size: 24px; color: #4CAF50; }
  </style>
</head>
<body>
  <h2>LiDAR Rig Control (Stepper)</h2>
  <div class="card">
    <div>Current angle: <span id="angleVal">--</span>&deg;</div>
    <input type="range" min="0" max="360" value="90" id="angleSlider" oninput="onSlide(this.value)">
    <button onclick="setMode('manual')">Manual Mode</button>
    <button onclick="setMode('auto')">Auto Sweep</button>
  </div>
  <div class="card">
    <h3 style="margin-top:0; color:#ddd;">Sweep Parameters</h3>
    <div class="paramRow"><span>Pan Start (deg)</span><input type="number" id="panStart" value="0"></div>
    <div class="paramRow"><span>Pan End (deg)</span><input type="number" id="panEnd" value="360"></div>
    <div class="paramRow"><span>Pan Step (deg)</span><input type="number" step="0.1" id="panStep" value="0.5"></div>
    <div class="paramRow"><span>Step Delay (ms)</span><input type="number" id="stepDelay" value="500"></div>
    <button onclick="applyParams()">Apply</button>
  </div>
  <div class="card">
    <div class="stat"><span>Mode:</span><span id="modeVal">--</span></div>
    <div class="stat"><span>Pan range:</span><span id="rangeVal">--</span></div>
    <div class="stat"><span>WiFi RSSI:</span><span id="rssiVal">--</span></div>
    <div class="stat"><span>Samples read:</span><span id="samplesVal">--</span></div>
    <div class="stat"><span>Rejected (moving):</span><span id="rejectedVal">--</span></div>
    <div class="stat"><span>Packets sent:</span><span id="packetsVal">--</span></div>
    <div class="stat"><span>Uptime:</span><span id="uptimeVal">--</span></div>
  </div>
  <script>
    function onSlide(val) {
      document.getElementById('angleVal').innerText = val;
      fetch('/setAngle?angle=' + val);
    }
    function setMode(m) { fetch('/setMode?mode=' + m); }
    function applyParams() {
      const ps = document.getElementById('panStart').value;
      const pe = document.getElementById('panEnd').value;
      const st = document.getElementById('panStep').value;
      const dl = document.getElementById('stepDelay').value;
      fetch(`/setSweepParams?panStart=${ps}&panEnd=${pe}&panStep=${st}&stepDelay=${dl}`);
    }
    function refreshStatus() {
      fetch('/status').then(r => r.json()).then(d => {
        document.getElementById('angleVal').innerText = d.angle.toFixed(1);
        document.getElementById('modeVal').innerText = d.mode;
        document.getElementById('rangeVal').innerText = d.panStart + '-' + d.panEnd + ' step ' + d.panStep;
        document.getElementById('rssiVal').innerText = d.rssi + ' dBm';
        document.getElementById('samplesVal').innerText = d.samples;
        document.getElementById('rejectedVal').innerText = d.rejectedMoving;
        document.getElementById('packetsVal').innerText = d.packets;
        document.getElementById('uptimeVal').innerText = d.uptime + 's';
        if (d.mode === 'auto') document.getElementById('angleSlider').value = d.angle;
        if (document.activeElement.id !== 'panStart') document.getElementById('panStart').value = d.panStart;
        if (document.activeElement.id !== 'panEnd') document.getElementById('panEnd').value = d.panEnd;
        if (document.activeElement.id !== 'panStep') document.getElementById('panStep').value = d.panStep;
        if (document.activeElement.id !== 'stepDelay') document.getElementById('stepDelay').value = d.stepDelay;
      });
    }
    setInterval(refreshStatus, 1000);
    refreshStatus();
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", PAGE_HTML); }

void handleSetAngle() {
  if (server.hasArg("angle")) {
    manualTargetAngle = server.arg("angle").toInt();
    autoSweepMode = false;
  }
  server.send(200, "text/plain", "OK");
}

void handleSetMode() {
  if (server.hasArg("mode")) {
    String mode = server.arg("mode");
    if (mode == "auto") autoSweepMode = true;
    else if (mode == "manual") autoSweepMode = false;
  }
  server.send(200, "text/plain", "OK");
}

void handleSetSweepParams() {
  if (server.hasArg("panStart")) panStart = server.arg("panStart").toInt();
  if (server.hasArg("panEnd")) panEnd = server.arg("panEnd").toInt();
  if (server.hasArg("panStep")) panStep = server.arg("panStep").toFloat();   // was toInt() — truncated 0.5 to 0
  if (server.hasArg("stepDelay")) stepDelayMs = server.arg("stepDelay").toInt();

  if (panStart < 0) panStart = 0;
  if (panEnd > 360) panEnd = 360;   // stepper can go further than a servo's 180 — see note below
  if (panStart >= panEnd) panEnd = panStart + 1;
  if (panStep < 0.1) panStep = 0.1;
  if (stepDelayMs < 20) stepDelayMs = 20;

  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  String json = "{";
  json += "\"angle\":" + String(currentAngleDeg) + ",";
  json += "\"mode\":\"" + String(autoSweepMode ? "auto" : "manual") + "\",";
  json += "\"panStart\":" + String(panStart) + ",";
  json += "\"panEnd\":" + String(panEnd) + ",";
  json += "\"panStep\":" + String(panStep) + ",";
  json += "\"stepDelay\":" + String(stepDelayMs) + ",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"samples\":" + String(totalSamplesRead) + ",";
  json += "\"rejectedMoving\":" + String(totalSamplesRejectedMoving) + ",";
  json += "\"packets\":" + String(totalPacketsSent) + ",";
  json += "\"uptime\":" + String((millis() - bootTime) / 1000);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(500);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. Control panel: http://");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);

  server.on("/", handleRoot);
  server.on("/setAngle", handleSetAngle);
  server.on("/setMode", handleSetMode);
  server.on("/setSweepParams", handleSetSweepParams);
  server.on("/status", handleStatus);
  server.begin();

  currentStepPosition = (long)(panStart * STEPS_PER_DEGREE);
  targetStepPosition = currentStepPosition;
  currentAngleDeg = panStart;

  startLidarScan();

  bootTime = millis();
  lastPanUpdateTime = millis();
}

void loop() {
  server.handleClient();
  updateStepper();   // non-blocking — call every loop iteration

  if (autoSweepMode) {
    if (millis() - lastPanUpdateTime > stepDelayMs) {
      float nextAngle = currentAngleDeg + panStep * panDirection;
      if (nextAngle >= panEnd) {
        nextAngle = panEnd;
        panDirection = -1;
      } else if (nextAngle <= panStart) {
        nextAngle = panStart;
        panDirection = 1;
      }
      setTargetAngle(nextAngle);
      lastPanUpdateTime = millis();
    }
  } else {
    setTargetAngle(manualTargetAngle);
  }

  float angle_deg;
  uint16_t distance_mm;
  uint8_t quality;

  while (readScanSample(angle_deg, distance_mm, quality)) {
    totalSamplesRead++;

    if (distance_mm >= 80 && distance_mm <= 12000 && quality >= 10) {
      if (!stepperSettled()) {
        // Stepper is still moving toward its target — this sample's tagged angle
        // wouldn't accurately reflect where the lidar physically is right now.
        totalSamplesRejectedMoving++;
        continue;
      }

      batchBuffer[batchCount].angle_deg = angle_deg;
      batchBuffer[batchCount].distance_mm = distance_mm;
      batchBuffer[batchCount].quality = quality;
      batchBuffer[batchCount].servo_angle = (uint8_t)currentAngleDeg;
      batchCount++;
      if (batchCount >= BATCH_SIZE) sendBatch();
    }
  }
}
