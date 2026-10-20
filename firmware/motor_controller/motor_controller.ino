// MotorControl - ESP32-S3 + DRV8912 + 4x N20 encoder motors
//   M0 / M1 : drive motors (left / right)  -> differential drive from joystick
//   M2 / M3 : claw motors                  -> one button: turn N degrees, stop; next press turns N degrees back
// Wi-Fi SoftAP + WebSocket (ws://192.168.4.1/ws)
// Libraries: ESPAsyncWebServer (+AsyncTCP), ArduinoJson (6.21+ or 7.x). Board package: esp32 3.x (analogWrite)
//
// ---- Protocol (JSON text frames) ----
// App -> ESP32 : {"t":"drive","l":-255..255,"r":-255..255}          (send ~20 Hz, 300 ms failsafe)
//                {"t":"claw"}                                       toggle-turn button press
//                {"t":"cfg","cpr":1400,"deg":10,"inv2":false,"inv3":false,"pmin":70,"pmax":200}
//                {"t":"enc0"}                                       zero all encoders
// ESP32 -> App : {"t":"tel","enc":[a,b,c,d],"spd":[..counts/s..],"claw":{"busy":false,"next":1},"cpr":1400,"deg":10}
//
// ---- How to add a feature ----
// 1) write  void cmdXxx(JsonDocument& d)   2) add it to COMMANDS   3) App.send({t:'xxx'}) in www/index.html

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ---------------- Wi-Fi SoftAP ----------------
const char* AP_SSID = "ESP32_Robot_AP";
const char* AP_PASS = "Password123";
IPAddress local_ip(192, 168, 4, 1), gateway(192, 168, 4, 1), subnet(255, 255, 255, 0);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------- Pins (same as before) ----------------
const int PWM_PINS[4] = {10, 12, 14, 2};
const int DIR_PINS[4] = {11, 13, 21, 3};
// NOTE: GPIO19/20 are the S3's native USB pins, GPIO3 is a strapping pin.
const int ENC_PINS[4][2] = {{4, 5}, {6, 7}, {15, 16}, {19, 20}};

// ---------------- Encoders (full 4x quadrature) ----------------
volatile long encTicks[4] = {0, 0, 0, 0};
volatile uint8_t encState[4] = {0, 0, 0, 0};
const int8_t QDEC[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR encISR(void* arg) {
  int i = (int)(intptr_t)arg;
  uint8_t s = (digitalRead(ENC_PINS[i][0]) << 1) | digitalRead(ENC_PINS[i][1]);
  encState[i] = ((encState[i] << 2) | s) & 0x0F;
  encTicks[i] += QDEC[encState[i]];
}

long getTicks(int i) { noInterrupts(); long v = encTicks[i]; interrupts(); return v; }

// ---------------- Settings (changeable from the app) ----------------
int   cprClaw = 1400;       // encoder counts per OUTPUT-shaft revolution (4x) of the claw motors
float clawDeg = 10.0f;      // angle per button press
bool  invert[4] = {false, false, false, false};   // flip direction of claw motors (index 2,3)
int   pwmMin = 70, pwmMax = 200;                  // claw PWM range (min overcomes stiction)

// ---------------- Motors ----------------
void setMotor(int i, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(DIR_PINS[i], speed >= 0 ? HIGH : LOW);
  analogWrite(PWM_PINS[i], abs(speed));
}

// Drive failsafe
unsigned long lastDrive = 0;
bool driveActive = false;
void stopDrive() { setMotor(0, 0); setMotor(1, 0); driveActive = false; }

// ---------------- Claw: turn N degrees then stop ----------------
struct ClawMotor { long target; int dir; bool done; };
ClawMotor claw[2];                 // index 0 -> M2, 1 -> M3
bool clawBusy = false;
int  clawNext = 1;                 // direction of the NEXT press (+1 / -1)
unsigned long clawStart = 0;
const unsigned long CLAW_TIMEOUT_MS = 3000;

void startClaw() {
  if (clawBusy) return;
  long delta = lroundf(clawDeg / 360.0f * cprClaw);
  if (delta < 1) delta = 1;
  for (int k = 0; k < 2; k++) {
    int m = 2 + k;
    claw[k].dir = clawNext * (invert[m] ? -1 : 1);
    claw[k].target = getTicks(m) + claw[k].dir * delta;
    claw[k].done = false;
  }
  clawNext = -clawNext;
  clawStart = millis();
  clawBusy = true;
}

void updateClaw() {
  if (!clawBusy) return;
  long delta = max(1L, lroundf(clawDeg / 360.0f * cprClaw));
  bool all = true;
  for (int k = 0; k < 2; k++) {
    int m = 2 + k;
    if (claw[k].done) continue;
    long pos = getTicks(m);
    long err = claw[k].target - pos;
    bool reached = (claw[k].dir > 0) ? (pos >= claw[k].target) : (pos <= claw[k].target);
    if (reached) { setMotor(m, 0); claw[k].done = true; continue; }
    float frac = min(1.0f, (float)labs(err) / max(1.0f, delta * 0.6f));   // slow down near target
    int pwm = pwmMin + (int)((pwmMax - pwmMin) * frac);
    setMotor(m, claw[k].dir * pwm);
    all = false;
  }
  if (all || millis() - clawStart > CLAW_TIMEOUT_MS) {
    setMotor(2, 0); setMotor(3, 0);
    clawBusy = false;
  }
}

// ---------------- Command handlers ----------------
void cmdDrive(JsonDocument& d) {
  setMotor(0, d["l"] | 0);
  setMotor(1, d["r"] | 0);
  lastDrive = millis();
  driveActive = true;
}
void cmdClaw(JsonDocument&) { startClaw(); }
void cmdCfg(JsonDocument& d) {
  if (d.containsKey("cpr"))  cprClaw = constrain((int)d["cpr"], 20, 20000);
  if (d.containsKey("deg"))  clawDeg = constrain((float)d["deg"], 1.0f, 180.0f);
  if (d.containsKey("inv2")) invert[2] = d["inv2"];
  if (d.containsKey("inv3")) invert[3] = d["inv3"];
  if (d.containsKey("pmin")) pwmMin = constrain((int)d["pmin"], 0, 255);
  if (d.containsKey("pmax")) pwmMax = constrain((int)d["pmax"], pwmMin, 255);
}
void cmdEnc0(JsonDocument&) {
  noInterrupts(); for (int i = 0; i < 4; i++) encTicks[i] = 0; interrupts();
}

// >>> ADD NEW COMMANDS HERE <<<
struct Command { const char* name; void (*fn)(JsonDocument&); };
const Command COMMANDS[] = {
  {"drive", cmdDrive},
  {"claw",  cmdClaw},
  {"cfg",   cmdCfg},
  {"enc0",  cmdEnc0},
};

void handleMessage(const char* json) {
  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, json)) return;
  const char* t = doc["t"] | "";
  for (const Command& c : COMMANDS) if (strcmp(c.name, t) == 0) { c.fn(doc); return; }
}

void onEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_DISCONNECT) { stopDrive(); return; }
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT && len < 384) {
    char buf[384];
    memcpy(buf, data, len);
    buf[len] = 0;
    handleMessage(buf);
  }
}

// ---------------- Telemetry (20 Hz) ----------------
unsigned long lastTel = 0;
long prevTicks[4] = {0, 0, 0, 0};

void sendTelemetry() {
  unsigned long now = millis();
  float dt = (now - lastTel) / 1000.0f;
  StaticJsonDocument<512> doc;
  doc["t"] = "tel";
  JsonArray enc = doc["enc"].to<JsonArray>();
  JsonArray spd = doc["spd"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    long v = getTicks(i);
    enc.add(v);
    spd.add(dt > 0 ? (int)((v - prevTicks[i]) / dt) : 0);
    prevTicks[i] = v;
  }
  JsonObject c = doc["claw"].to<JsonObject>();
  c["busy"] = clawBusy;
  c["next"] = clawNext;
  doc["cpr"] = cprClaw;
  doc["deg"] = clawDeg;
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < 4; i++) { pinMode(PWM_PINS[i], OUTPUT); pinMode(DIR_PINS[i], OUTPUT); setMotor(i, 0); }

  for (int i = 0; i < 4; i++) {
    pinMode(ENC_PINS[i][0], INPUT_PULLUP);
    pinMode(ENC_PINS[i][1], INPUT_PULLUP);
    encState[i] = (digitalRead(ENC_PINS[i][0]) << 1) | digitalRead(ENC_PINS[i][1]);
    attachInterruptArg(digitalPinToInterrupt(ENC_PINS[i][0]), encISR, (void*)(intptr_t)i, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENC_PINS[i][1]), encISR, (void*)(intptr_t)i, CHANGE);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  if (WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4)) {
    Serial.print("Wi-Fi AP ready: ");
    Serial.println(WiFi.softAPIP());
  }
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  ws.cleanupClients();

  if (driveActive && millis() - lastDrive > 300) stopDrive();   // failsafe
  updateClaw();

  if (millis() - lastTel >= 50) {
    if (ws.count() > 0) sendTelemetry();
    lastTel = millis();
  }
  // >>> ADD NEW PERIODIC TASKS HERE <<<
}
