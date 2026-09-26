//in this mode, the lidar and the servo is conneted to the esp all wired and the esp trasmits the lidars data through wifi , it connects to an existing wifi and you have to enter your laptops ip too
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

const char* WIFI_SSID = "STAFF";
const char* WIFI_PASSWORD = "Indian@123";
const char* LAPTOP_IP = "172.22.59.148";
const int UDP_PORT = 5005;

WiFiUDP udp;
WebServer server(80);

Servo panServo;
const int SERVO_PIN = 27;

int currentAngle = 0;
int panDirection = 1;

int panStart = 0;
int panEnd = 180;
int panStep = 2;
unsigned long stepDelayMs = 300;

unsigned long lastStepTime = 0;

bool autoSweepMode = true;
int manualTargetAngle = 90;

HardwareSerial LidarSerial(2);

const int LIDAR_RX_PIN = 16;
const int LIDAR_TX_PIN = 17;
const long LIDAR_BAUD = 460800;

struct SamplePacket {
  float angle_deg;
  uint16_t distance_mm;
  uint8_t quality;
  uint8_t servo_angle;
};

const int BATCH_SIZE = 20;
SamplePacket batchBuffer[BATCH_SIZE];
int batchCount = 0;

unsigned long totalPacketsSent = 0;
unsigned long totalSamplesRead = 0;
unsigned long bootTime = 0;

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

    if (millis() - start > 2000) {
      return false;
    }

    yield();
  }

  return buf[0] == 0xA5 && buf[1] == 0x5A;
}

void startLidarScan() {
  while (LidarSerial.available()) {
    LidarSerial.read();
  }

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

      if (s == sn) {
        continue;
      }

      packet[0] = b;
      index = 1;
      continue;
    }

    packet[index++] = b;

    if (index < 5) {
      continue;
    }

    index = 0;

    uint8_t S = packet[0] & 0x01;
    uint8_t Sn = (packet[0] >> 1) & 0x01;
    uint8_t C = packet[1] & 0x01;

    if (S == Sn || C == 0) {
      continue;
    }

    quality = packet[0] >> 2;

    uint16_t angle_q6 =
      ((uint16_t)packet[2] << 7) |
      ((uint16_t)packet[1] >> 1);

    angle_deg = angle_q6 / 64.0f;

    uint16_t distance_q2 =
      ((uint16_t)packet[4] << 8) |
      packet[3];

    distance_mm = distance_q2 / 4;

    return true;
  }

  return false;
}

void sendBatch() {
  if (batchCount == 0) return;

  udp.beginPacket(LAPTOP_IP, UDP_PORT);
  udp.write(
    (uint8_t*)batchBuffer,
    batchCount * sizeof(SamplePacket)
  );
  udp.endPacket();

  totalPacketsSent++;

  batchCount = 0;
}

const char* PAGE_HTML = R"rawliteral(
<!DOCTYPE html>
<html>

<head>

<title>RPLIDAR C1 3D Scanner</title>

<meta name="viewport"
content="width=device-width, initial-scale=1">

<style>

body {
  font-family: Arial, sans-serif;
  background:#111;
  color:#eee;
  margin:0;
  padding:15px;
  text-align:center;
}

h2 {
  color:#4CAF50;
}

.card {
  background:#222;
  border-radius:10px;
  padding:15px;
  margin:12px auto;
  max-width:900px;
}

button {
  background:#4CAF50;
  color:white;
  border:none;
  border-radius:6px;
  padding:10px 16px;
  margin:4px;
  font-size:15px;
}

input[type=range] {
  width:100%;
}

input[type=number] {
  width:70px;
  background:#333;
  color:white;
  border:1px solid #555;
  padding:5px;
}

.stat {
  display:flex;
  justify-content:space-between;
  padding:4px;
}

canvas {
  width:100%;
  max-width:850px;
  background:#050505;
  border-radius:8px;
  display:block;
  margin:auto;
}

#angleVal {
  font-size:26px;
  color:#4CAF50;
}

</style>

</head>

<body>

<h2>RPLIDAR C1 3D Scanner</h2>

<div class="card">

<div>
Current Servo Angle:
<span id="angleVal">0</span>°
</div>

<input
type="range"
min="0"
max="180"
value="0"
id="angleSlider"
oninput="onSlide(this.value)"
>

<button onclick="setMode('manual')">
Manual Mode
</button>

<button onclick="setMode('auto')">
Auto Sweep
</button>

<button onclick="clearMap()">
Clear Map
</button>

</div>

<div class="card">

<h3>Sweep Parameters</h3>

<div class="stat">
<span>Start</span>
<input type="number" id="panStart" value="0">
</div>

<div class="stat">
<span>End</span>
<input type="number" id="panEnd" value="180">
</div>

<div class="stat">
<span>Step</span>
<input type="number" id="panStep" value="2">
</div>

<div class="stat">
<span>Delay</span>
<input type="number" id="stepDelay" value="300">
</div>

<button onclick="applyParams()">
Apply
</button>

</div>

<div class="card">

<canvas
id="map"
width="900"
height="650">
</canvas>

</div>

<div class="card">

<div class="stat">
<span>Mode</span>
<span id="modeVal">--</span>
</div>

<div class="stat">
<span>Servo Angle</span>
<span id="servoVal">--</span>
</div>

<div class="stat">
<span>WiFi RSSI</span>
<span id="rssiVal">--</span>
</div>

<div class="stat">
<span>Samples Read</span>
<span id="samplesVal">--</span>
</div>

<div class="stat">
<span>Packets Sent</span>
<span id="packetsVal">--</span>
</div>

<div class="stat">
<span>Accumulated Points</span>
<span id="pointsVal">0</span>
</div>

<div class="stat">
<span>Uptime</span>
<span id="uptimeVal">--</span>
</div>

</div>

<script>

const canvas =
document.getElementById("map");

const ctx =
canvas.getContext("2d");

let points = [];

const MAX_POINTS = 80000;

const SCALE = 0.22;

function onSlide(val) {

  document.getElementById("angleVal").innerText = val;

  fetch("/setAngle?angle=" + val);

}

function setMode(mode) {

  fetch("/setMode?mode=" + mode);

}

function applyParams() {

  const ps =
    document.getElementById("panStart").value;

  const pe =
    document.getElementById("panEnd").value;

  const st =
    document.getElementById("panStep").value;

  const dl =
    document.getElementById("stepDelay").value;

  fetch(
    `/setSweepParams?panStart=${ps}&panEnd=${pe}&panStep=${st}&stepDelay=${dl}`
  );

}

function clearMap() {

  points = [];

  drawMap();

}

function addPoint(angle, distance, servo) {

  if (distance < 80 ||
      distance > 12000) {
    return;
  }

  if (points.length >= MAX_POINTS) {
    points.splice(0, 1000);
  }

  const theta =
    angle * Math.PI / 180;

  const phi =
    servo * Math.PI / 180;

  const r = distance;

  const x =
    r * Math.cos(theta);

  const y =
    r * Math.sin(theta) *
    Math.cos(phi);

  const z =
    r * Math.sin(theta) *
    Math.sin(phi);

  points.push({
    x:x,
    y:y,
    z:z
  });

}

function drawMap() {

  ctx.fillStyle = "#050505";

  ctx.fillRect(
    0,
    0,
    canvas.width,
    canvas.height
  );

  const cx =
    canvas.width / 2;

  const cy =
    canvas.height / 2;

  ctx.strokeStyle =
    "#333";

  ctx.beginPath();

  ctx.moveTo(0,cy);
  ctx.lineTo(canvas.width,cy);

  ctx.moveTo(cx,0);
  ctx.lineTo(cx,canvas.height);

  ctx.stroke();

  for (let i=0;
       i<points.length;
       i++) {

    const p = points[i];

    const px =
      cx + p.x * SCALE;

    const py =
      cy - p.z * SCALE;

    if (
      px < 0 ||
      px >= canvas.width ||
      py < 0 ||
      py >= canvas.height
    ) {
      continue;
    }

    ctx.fillStyle =
      "#4CAF50";

    ctx.fillRect(
      px,
      py,
      2,
      2
    );

  }

  document.getElementById(
    "pointsVal"
  ).innerText = points.length;

}

function processUDP(data) {

  const view =
    new DataView(data);

  const packetSize = 9;

  for (
    let offset=0;
    offset + packetSize <= view.byteLength;
    offset += packetSize
  ) {

    const angle =
      view.getFloat32(
        offset,
        true
      );

    const distance =
      view.getUint16(
        offset + 4,
        true
      );

    const quality =
      view.getUint8(
        offset + 6
      );

    const servo =
      view.getUint8(
        offset + 7
      );

    if (quality < 10) {
      continue;
    }

    addPoint(
      angle,
      distance,
      servo
    );

  }

  drawMap();

}

async function startUDP() {

  try {

    const response =
      await fetch("/status");

  } catch(e) {

  }

}

function refreshStatus() {

  fetch("/status")
  .then(r => r.json())
  .then(d => {

    document.getElementById(
      "angleVal"
    ).innerText = d.angle;

    document.getElementById(
      "servoVal"
    ).innerText = d.angle;

    document.getElementById(
      "modeVal"
    ).innerText = d.mode;

    document.getElementById(
      "rssiVal"
    ).innerText =
      d.rssi + " dBm";

    document.getElementById(
      "samplesVal"
    ).innerText =
      d.samples;

    document.getElementById(
      "packetsVal"
    ).innerText =
      d.packets;

    document.getElementById(
      "uptimeVal"
    ).innerText =
      d.uptime + "s";

    document.getElementById(
      "angleSlider"
    ).value =
      d.angle;

    if (
      document.activeElement.id !==
      "panStart"
    )
      document.getElementById(
        "panStart"
      ).value = d.panStart;

    if (
      document.activeElement.id !==
      "panEnd"
    )
      document.getElementById(
        "panEnd"
      ).value = d.panEnd;

    if (
      document.activeElement.id !==
      "panStep"
    )
      document.getElementById(
        "panStep"
      ).value = d.panStep;

    if (
      document.activeElement.id !==
      "stepDelay"
    )
      document.getElementById(
        "stepDelay"
      ).value = d.stepDelay;

  });

}

setInterval(
  refreshStatus,
  1000
);

refreshStatus();

drawMap();

</script>

</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(
    200,
    "text/html",
    PAGE_HTML
  );
}

void handleSetAngle() {

  if (server.hasArg("angle")) {

    manualTargetAngle =
      constrain(
        server.arg("angle").toInt(),
        0,
        180
      );

    autoSweepMode = false;

  }

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

void handleSetMode() {

  if (server.hasArg("mode")) {

    String mode =
      server.arg("mode");

    if (mode == "auto") {
      autoSweepMode = true;
    }

    else if (mode == "manual") {
      autoSweepMode = false;
    }

  }

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

void handleSetSweepParams() {

  if (server.hasArg("panStart"))
    panStart =
      server.arg("panStart").toInt();

  if (server.hasArg("panEnd"))
    panEnd =
      server.arg("panEnd").toInt();

  if (server.hasArg("panStep"))
    panStep =
      server.arg("panStep").toInt();

  if (server.hasArg("stepDelay"))
    stepDelayMs =
      server.arg("stepDelay").toInt();

  panStart =
    constrain(panStart,0,179);

  panEnd =
    constrain(panEnd,panStart+1,180);

  panStep =
    constrain(panStep,1,30);

  stepDelayMs =
    max(stepDelayMs,(unsigned long)20);

  if (currentAngle < panStart)
    currentAngle = panStart;

  if (currentAngle > panEnd)
    currentAngle = panEnd;

  panServo.write(currentAngle);

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

void handleStatus() {

  String json = "{";

  json +=
    "\"angle\":" +
    String(currentAngle) + ",";

  json +=
    "\"mode\":\"" +
    String(
      autoSweepMode ?
      "auto" :
      "manual"
    ) +
    "\",";

  json +=
    "\"panStart\":" +
    String(panStart) + ",";

  json +=
    "\"panEnd\":" +
    String(panEnd) + ",";

  json +=
    "\"panStep\":" +
    String(panStep) + ",";

  json +=
    "\"stepDelay\":" +
    String(stepDelayMs) + ",";

  json +=
    "\"rssi\":" +
    String(WiFi.RSSI()) + ",";

  json +=
    "\"samples\":" +
    String(totalSamplesRead) + ",";

  json +=
    "\"packets\":" +
    String(totalPacketsSent) + ",";

  json +=
    "\"uptime\":" +
    String(
      (millis()-bootTime)/1000
    );

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}

void setup() {

  Serial.begin(115200);

  delay(500);

  LidarSerial.begin(
    LIDAR_BAUD,
    SERIAL_8N1,
    LIDAR_RX_PIN,
    LIDAR_TX_PIN
  );

  delay(500);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  Serial.print(
    "Connecting to WiFi"
  );

  while (
    WiFi.status() !=
    WL_CONNECTED
  ) {

    delay(300);

    Serial.print(".");

  }

  Serial.println();

  Serial.print(
    "Connected. Control panel: http://"
  );

  Serial.println(
    WiFi.localIP()
  );

  udp.begin(UDP_PORT);

  server.on(
    "/",
    handleRoot
  );

  server.on(
    "/setAngle",
    handleSetAngle
  );

  server.on(
    "/setMode",
    handleSetMode
  );

  server.on(
    "/setSweepParams",
    handleSetSweepParams
  );

  server.on(
    "/status",
    handleStatus
  );

  server.begin();

  panServo.setPeriodHertz(50);

  panServo.attach(
    SERVO_PIN,
    500,
    2500
  );

  panServo.write(
    panStart
  );

  currentAngle =
    panStart;

  delay(500);

  startLidarScan();

  bootTime =
    millis();

  lastStepTime =
    millis();

}

void loop() {

  server.handleClient();

  if (autoSweepMode) {

    if (
      millis() -
      lastStepTime >=
      stepDelayMs
    ) {

      currentAngle +=
        panStep *
        panDirection;

      if (
        currentAngle >= panEnd
      ) {

        currentAngle =
          panEnd;

        panDirection =
          -1;

      }

      else if (
        currentAngle <= panStart
      ) {

        currentAngle =
          panStart;

        panDirection =
          1;

      }

      panServo.write(
        currentAngle
      );

      lastStepTime =
        millis();

    }

  }

  else {

    if (
      currentAngle !=
      manualTargetAngle
    ) {

      currentAngle =
        manualTargetAngle;

      panServo.write(
        currentAngle
      );

    }

  }

  float angle_deg;

  uint16_t distance_mm;

  uint8_t quality;

  while (
    readScanSample(
      angle_deg,
      distance_mm,
      quality
    )
  ) {

    totalSamplesRead++;

    if (
      distance_mm >= 80 &&
      distance_mm <= 12000 &&
      quality >= 10
    ) {

      batchBuffer[batchCount]
        .angle_deg =
        angle_deg;

      batchBuffer[batchCount]
        .distance_mm =
        distance_mm;

      batchBuffer[batchCount]
        .quality =
        quality;

      batchBuffer[batchCount]
        .servo_angle =
        (uint8_t)currentAngle;

      batchCount++;

      if (
        batchCount >=
        BATCH_SIZE
      ) {

        sendBatch();

      }

    }

  }

}
