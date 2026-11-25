// RoverC Pro firmware for M5StickC / Plus / Plus2: a thin UDP relay for ROS 2.
//
// Uses M5's own libraries: M5Unified (stick: screen, buttons, IMU, battery) and
// M5-RoverC (rover: motors + gripper over I2C on the hat pins G0/G26).
//
// The ROS 2 driver (ros2_ws/src/rover_driver) sends drive commands over UDP at
// 20 Hz; the rover replies with IMU and battery telemetry at 50 Hz. The packet
// layout is in include/protocol.h.
//
// WiFi: joins WIFI_SSID from secrets.h if set; otherwise (or if that fails) it
// starts its own hotspot. Reachable as roverc.local either way.
//
// Safety: the rover stops if no command arrives for 0.5 s. Button A toggles a
// lock that stops it and ignores commands until pressed again (ROS can also
// lock/unlock it).

#include <M5Unified.h>
#include <M5_RoverC.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <esp_wifi_ap_get_sta_list.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy include/secrets.example.h to include/secrets.h"
#endif

#include "protocol.h"

static const uint32_t COMMAND_TIMEOUT_MS = 500;   // stop if commands stop for this long
static const uint32_t PEER_TIMEOUT_MS = 2000;     // stop sending telemetry after this
static const uint32_t TELEMETRY_PERIOD_MS = 20;   // 50 Hz
static const uint32_t SLOW_PERIOD_MS = 1000;      // battery, temperature, RSSI, rates
static const uint32_t SCREEN_REFRESH_MS = 250;

M5_RoverC rover;
WiFiUDP udp;

bool roverFound = false;
bool locked = false;
bool moving = false;
bool apMode = false;

// The PC running the ROS driver: whoever sent the last valid command.
IPAddress peerIp;
uint16_t peerPort = 0;
uint32_t lastCommandMs = 0;
uint16_t lastCommandSeq = 0;
uint16_t telemetrySeq = 0;
uint32_t commandsThisSecond = 0;
uint32_t commandRate = 0;  // commands per second, for the screen

uint8_t gripAngle = 0;  // 0 = never set

// Slow readings, refreshed once a second.
int16_t battMv = 0;
int16_t battMa = 0;
int16_t vbusMv = -1;
int8_t battPct = -1;
int8_t chargeState = 0;
float imuTemp = NAN;
int8_t rssi = 0;

uint32_t lastTelemetryMs = 0;
uint32_t lastSlowMs = 0;
uint32_t lastScreenMs = 0;
String lastScreen;

bool havePeer() {
  return peerPort != 0 && millis() - lastCommandMs < PEER_TIMEOUT_MS;
}

void stopRover() {
  rover.setSpeed(0, 0, 0);
  moving = false;
}

void setGrip(uint8_t angle) {
  // The gripper is on servo port 0; port 1 gets the same angle, as in M5's example.
  gripAngle = constrain(angle, 10, 90);
  rover.setServoAngle(0, gripAngle);
  rover.setServoAngle(1, gripAngle);
}

// ---------- Commands from ROS ----------

void applyCommand(const CommandPacket &cmd) {
  if (cmd.flags & CMD_FLAG_LOCK) locked = true;
  if (cmd.flags & CMD_FLAG_UNLOCK) locked = false;

  if (locked) {
    stopRover();
    return;
  }

  if (cmd.grip != 0 && cmd.grip != gripAngle) setGrip(cmd.grip);

  int8_t x = constrain(cmd.x, -100, 100);
  int8_t y = constrain(cmd.y, -100, 100);
  int8_t z = constrain(cmd.z, -100, 100);
  rover.setSpeed(x, y, z);
  moving = (x != 0 || y != 0 || z != 0);
}

void receiveCommands() {
  while (udp.parsePacket() > 0) {
    uint8_t buf[64];
    int len = udp.read(buf, sizeof(buf));
    if (len != sizeof(CommandPacket)) continue;

    CommandPacket cmd;
    memcpy(&cmd, buf, sizeof(cmd));
    if (cmd.magic[0] != 'R' || cmd.magic[1] != 'C' || cmd.version != PROTOCOL_VERSION) continue;

    peerIp = udp.remoteIP();
    peerPort = udp.remotePort();
    lastCommandMs = millis();
    lastCommandSeq = cmd.seq;
    commandsThisSecond++;
    applyCommand(cmd);
  }
}

// ---------- Telemetry to ROS ----------

void sendTelemetry() {
  TelemetryPacket t = {};
  t.magic[0] = 'R';
  t.magic[1] = 'T';
  t.version = PROTOCOL_VERSION;

  bool imuOk = M5.Imu.isEnabled() && M5.Imu.update();
  const auto &imu = M5.Imu.getImuData();
  for (int i = 0; i < 3; i++) {
    t.accel[i] = imu.accel.value[i];
    t.gyro[i] = imu.gyro.value[i];
  }

  t.flags = (locked ? TLM_FLAG_LOCKED : 0) | (roverFound ? TLM_FLAG_ROVER_FOUND : 0) |
            (moving ? TLM_FLAG_MOVING : 0) | (imuOk ? TLM_FLAG_IMU_OK : 0);
  t.seq = telemetrySeq++;
  t.cmd_seq = lastCommandSeq;
  t.uptime_ms = millis();
  t.imu_temp = imuTemp;
  t.batt_mv = battMv;
  t.batt_ma = battMa;
  t.vbus_mv = vbusMv;
  t.batt_pct = battPct;
  t.charge_state = chargeState;
  t.rssi = rssi;
  t.grip = gripAngle;

  udp.beginPacket(peerIp, peerPort);
  udp.write((const uint8_t *)&t, sizeof(t));
  udp.endPacket();
}

// In hotspot mode, find the signal strength of the station that is the ROS PC.
int8_t apRssiForPeer() {
  wifi_sta_list_t stations = {};
  wifi_sta_mac_ip_list_t withIp = {};
  if (esp_wifi_ap_get_sta_list(&stations) != ESP_OK) return 0;
  esp_wifi_ap_get_sta_list_with_ip(&stations, &withIp);
  for (int i = 0; i < withIp.num && i < stations.num; i++) {
    if (withIp.sta[i].ip.addr == (uint32_t)peerIp) return stations.sta[i].rssi;
  }
  return 0;
}

void readSlowSensors() {
  battMv = M5.Power.getBatteryVoltage();
  battMa = (int16_t)constrain(M5.Power.getBatteryCurrent(), -32768, 32767);
  vbusMv = M5.Power.getVBUSVoltage();
  battPct = (int8_t)constrain(M5.Power.getBatteryLevel(), -1, 100);
  chargeState = (int8_t)M5.Power.getChargeState();

  float t;
  imuTemp = M5.Imu.getTemp(&t) ? t : NAN;

  if (apMode) {
    rssi = havePeer() ? apRssiForPeer() : 0;
  } else {
    rssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  }

  commandRate = commandsThisSecond;
  commandsThisSecond = 0;
}

// ---------- Screen ----------

String batteryText() {
  String s = "Batt ";
  if (battPct >= 0) s += String(battPct) + "% ";
  if (battMv > 0) s += String(battMv / 1000.0f, 2) + "V";
  if (chargeState == (int8_t)m5::charge_state_t::charging) s += " chg";
  return s + "\n";
}

String rosText() {
  if (!havePeer()) return "ROS: waiting\n";
  return "ROS " + peerIp.toString() + "\n " + String(commandRate) + " cmd/s\n";
}

String networkText() {
  String s;
  if (apMode) {
    s += "AP " AP_SSID "\n";
    s += WiFi.softAPIP().toString() + "\n";
    s += batteryText();

    wifi_sta_list_t stations = {};
    wifi_sta_mac_ip_list_t withIp = {};
    esp_wifi_ap_get_sta_list(&stations);
    esp_wifi_ap_get_sta_list_with_ip(&stations, &withIp);

    s += "Joined (" + String(withIp.num) + "):\n";
    for (int i = 0; i < withIp.num; i++) {
      uint32_t ip = withIp.sta[i].ip.addr;
      if (!ip) {
        s += " (getting IP)\n";
        continue;
      }
      s += " " + IPAddress(ip).toString();
      if (havePeer() && ip == (uint32_t)peerIp) s += " ros";
      s += "\n";
    }
  } else {
    if (WiFi.status() == WL_CONNECTED) {
      s += "WiFi " + WiFi.SSID() + "\n";
      s += WiFi.localIP().toString() + "\n";
    } else {
      s += "WiFi lost, retrying\n";
    }
    s += batteryText();
    s += rosText();
  }
  return s;
}

// Redraws only when the text changed, so the screen doesn't flicker.
void drawScreen() {
  const char *head = "READY";
  uint16_t color = TFT_GREEN;
  if (!roverFound) {
    head = "RoverC NOT FOUND";
    color = TFT_RED;
  } else if (locked) {
    head = "LOCKED (btn A)";
    color = TFT_RED;
  } else if (moving) {
    head = "DRIVING";
    color = TFT_YELLOW;
  }

  String body = networkText();
  String all = String(head) + "\n" + body;
  if (all == lastScreen) return;
  lastScreen = all;

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextColor(color, TFT_BLACK);
  M5.Display.println(head);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.print(body);
}

// ---------- Setup ----------

void startWifi() {
  if (strlen(WIFI_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // lower latency for driving
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    M5.Display.println("Joining WiFi...");
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) delay(500);
    if (WiFi.status() == WL_CONNECTED) {
      apMode = false;
      return;
    }
    WiFi.disconnect(true);
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);  // rover is 192.168.4.1
  apMode = true;
}

void setup() {
  M5.begin();
  M5.Display.setRotation(3);
  M5.Display.setTextSize(M5.Display.width() >= 240 ? 2 : 1);  // Plus/Plus2 vs StickC
  M5.Display.setTextWrap(false);  // cut long lines instead of wrapping
  M5.Display.fillScreen(TFT_BLACK);

  roverFound = rover.begin();  // Wire on G0 (SDA) / G26 (SCL), address 0x38
  stopRover();

  startWifi();
  MDNS.begin("roverc");  // roverc.local
  udp.begin(ROVER_UDP_PORT);

  readSlowSensors();
  Serial.printf("RoverC %s, listening on %s:%d\n", roverFound ? "found" : "NOT found",
                (apMode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str(), ROVER_UDP_PORT);
  drawScreen();
}

void loop() {
  M5.update();
  receiveCommands();

  if (M5.BtnA.wasPressed()) {
    locked = !locked;
    stopRover();
    drawScreen();
  }

  uint32_t now = millis();

  if (moving && now - lastCommandMs > COMMAND_TIMEOUT_MS) stopRover();

  if (havePeer() && now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }

  if (now - lastSlowMs >= SLOW_PERIOD_MS) {
    lastSlowMs = now;
    readSlowSensors();
  }

  if (now - lastScreenMs >= SCREEN_REFRESH_MS) {
    lastScreenMs = now;
    drawScreen();
  }
}
