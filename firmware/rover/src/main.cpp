// RoverC Pro browser control for M5StickC / Plus / Plus2.
//
// Uses M5's own libraries: M5Unified (stick: screen, buttons) and M5-RoverC
// (rover: motors + gripper over I2C on the hat pins G0/G26).
//
// WiFi: joins WIFI_SSID from secrets.h if set; otherwise (or if that fails)
// it starts its own hotspot.
//
// Screen: READY / LOCKED on top, then the network. On home WiFi it shows the
// network name, the rover's IP and which browsers have the page open. In
// hotspot mode it lists every device joined to the hotspot, tagged "web" if it
// has the page open and "drive" if it is driving.
//
// Button A on the stick toggles a lock that stops the rover and ignores the
// browser until pressed again.

#include <M5Unified.h>
#include <M5_RoverC.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <esp_wifi_ap_get_sta_list.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy include/secrets.example.h to include/secrets.h"
#endif

#include "page.h"

// Stop if no drive command arrives for this long. The page resends every 150 ms.
static const uint32_t COMMAND_TIMEOUT_MS = 500;
// A browser counts as "page open" if it checked in this recently. The page pings every 1 s.
static const uint32_t PAGE_OPEN_MS = 3000;
static const uint32_t SCREEN_REFRESH_MS = 250;
static const int MAX_WEB_CLIENTS = 4;

M5_RoverC rover;
WebServer server(80);

bool roverFound = false;
bool locked = false;
bool moving = false;
bool apMode = false;
uint32_t lastCommandMs = 0;
uint32_t driverIp = 0;  // who sent the last non-zero drive command

struct WebClient {
  uint32_t ip;
  uint32_t lastSeenMs;
};
WebClient webClients[MAX_WEB_CLIENTS] = {};

String lastScreen;
uint32_t lastScreenMs = 0;

void stopRover() {
  rover.setSpeed(0, 0, 0);
  moving = false;
}

// ---------- Who is using the page ----------

// Record that the current request's sender is alive. Reuses its slot, or
// takes the stalest one when the table is full.
void noteClient() {
  uint32_t ip = (uint32_t)server.client().remoteIP();
  WebClient *slot = &webClients[0];
  for (WebClient &c : webClients) {
    if (c.ip == ip) {
      slot = &c;
      break;
    }
    if (c.lastSeenMs < slot->lastSeenMs) slot = &c;
  }
  slot->ip = ip;
  slot->lastSeenMs = millis();
}

bool pageOpen(uint32_t ip) {
  for (const WebClient &c : webClients) {
    if (c.ip == ip && millis() - c.lastSeenMs < PAGE_OPEN_MS) return true;
  }
  return false;
}

String tags(uint32_t ip) {
  String s;
  if (pageOpen(ip)) s += " web";
  if (moving && ip == driverIp) s += " drive";
  return s;
}

// ---------- Screen ----------

String networkText() {
  String s;
  if (apMode) {
    s += "AP " AP_SSID "\n";
    s += WiFi.softAPIP().toString() + "\n";

    wifi_sta_list_t stations = {};
    wifi_sta_mac_ip_list_t withIp = {};
    esp_wifi_ap_get_sta_list(&stations);
    esp_wifi_ap_get_sta_list_with_ip(&stations, &withIp);

    s += "Joined (" + String(withIp.num) + "):\n";
    for (int i = 0; i < withIp.num; i++) {
      uint32_t ip = withIp.sta[i].ip.addr;
      if (ip) {
        s += " " + IPAddress(ip).toString() + tags(ip) + "\n";
      } else {
        s += " (getting IP)\n";
      }
    }
  } else {
    if (WiFi.status() == WL_CONNECTED) {
      s += "WiFi " + WiFi.SSID() + "\n";
      s += WiFi.localIP().toString() + "\n";
    } else {
      s += "WiFi lost, retrying\n";
    }

    String list;
    int count = 0;
    for (const WebClient &c : webClients) {
      if (c.ip && pageOpen(c.ip)) {
        list += " " + IPAddress(c.ip).toString() + tags(c.ip) + "\n";
        count++;
      }
    }
    s += "Browsers (" + String(count) + "):\n" + list;
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

// ---------- Web handlers ----------

String statusText() {
  if (!roverFound) return "rover not found - check the stick is seated";
  if (locked) return "LOCKED - press button A on the rover";
  return "ok";
}

int8_t speedArg(const char *name) {
  return (int8_t)constrain(server.arg(name).toInt(), -100, 100);
}

void handleDrive() {
  noteClient();
  if (locked) {
    stopRover();
  } else {
    int8_t x = speedArg("x");
    int8_t y = speedArg("y");
    int8_t z = speedArg("z");
    rover.setSpeed(x, y, z);
    moving = (x != 0 || y != 0 || z != 0);
    if (moving) driverIp = (uint32_t)server.client().remoteIP();
    lastCommandMs = millis();
  }
  server.send(200, "text/plain", statusText());
}

void handleGrip() {
  noteClient();
  // Same 10..90 range M5's example uses. The gripper is on servo port 0;
  // port 1 gets the same angle, as in M5's example.
  uint8_t angle = constrain(server.arg("angle").toInt(), 10, 90);
  rover.setServoAngle(0, angle);
  rover.setServoAngle(1, angle);
  server.send(200, "text/plain", "ok");
}

void handlePing() {
  noteClient();
  server.send(200, "text/plain", statusText());
}

void handlePage() {
  noteClient();
  server.send(200, "text/html", PAGE);
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
      MDNS.begin("roverc");  // http://roverc.local
      apMode = false;
      return;
    }
    WiFi.disconnect(true);
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);  // http://192.168.4.1
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

  server.on("/", handlePage);
  server.on("/drive", handleDrive);
  server.on("/grip", handleGrip);
  server.on("/ping", handlePing);
  server.begin();

  Serial.printf("RoverC %s, open http://%s\n", roverFound ? "found" : "NOT found",
                (apMode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str());
  drawScreen();
}

void loop() {
  M5.update();
  server.handleClient();

  if (M5.BtnA.wasPressed()) {
    locked = !locked;
    stopRover();
    drawScreen();
  }

  if (moving && millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    stopRover();
  }

  if (millis() - lastScreenMs >= SCREEN_REFRESH_MS) {
    lastScreenMs = millis();
    drawScreen();
  }
}
