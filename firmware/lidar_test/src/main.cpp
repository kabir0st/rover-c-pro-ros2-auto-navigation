// LD06 lidar test for the M5StickC / Plus / Plus2. Stands in for LDROBOT's USB
// debug board.
//
// Wiring, to the stick's own Grove port (not the RoverC's I2C Grove ports):
//   LD06 pin 1 Tx  -> G33 (white)
//   LD06 pin 2 PWM -> not connected: the lidar uses its own ~10 Hz speed control
//   LD06 pin 3 GND -> GND (black)
//   LD06 pin 4 P5V -> 5V  (red)
// The yellow wire (G32) stays unused.
//
// It does two jobs:
// - Screen: parses the lidar packets and shows whether the lidar works.
// - USB bridge: copies every lidar byte to the USB serial port at 230400, so the
//   PC sees the stick as if it were an LD06 on a USB adapter (ld06_view.py reads it).
//   Nothing else is printed on USB serial, so that stream stays clean.

#include <M5Unified.h>

static const int LIDAR_RX_PIN = 33;
static const uint32_t LIDAR_BAUD = 230400;
static const uint32_t NO_DATA_MS = 1000;       // status turns red after this long without data
static const uint32_t RATE_PERIOD_MS = 1000;
static const uint32_t SCREEN_REFRESH_MS = 250;
static const uint16_t FRONT_WINDOW = 150;      // 0.01 deg; points within 1.5 deg of 0 count as "front"

// LD06 packet, 47 bytes, little-endian:
//   0x54, 0x2C, speed u16 (deg/s), start angle u16 (0.01 deg),
//   12 x {distance u16 (mm), intensity u8}, end angle u16 (0.01 deg),
//   timestamp u16 (ms), CRC-8 of the first 46 bytes.
static const uint8_t HEADER = 0x54;
static const uint8_t VER_LEN = 0x2C;  // packet type 1, 12 points
static const int POINTS = 12;
static const int PACKET_SIZE = 47;
static const int END_ANGLE_OFFSET = 6 + 3 * POINTS;

uint8_t crcTable[256];

uint8_t packet[PACKET_SIZE];
int packetLen = 0;

// Updated per packet.
uint32_t lastByteMs = 0;
uint32_t lastPacketMs = 0;
uint32_t lastFrontMs = 0;
uint16_t speedDegS = 0;
uint16_t frontMm = 0;
uint32_t crcErrors = 0;
uint32_t bytesThisPeriod = 0;
uint32_t packetsThisPeriod = 0;

// Updated once a second.
uint32_t bytesPerSec = 0;
uint32_t packetsPerSec = 0;
uint32_t lastRateMs = 0;

uint32_t lastScreenMs = 0;
String lastScreen;

// CRC-8, polynomial 0x4D, initial value 0, MSB first: the LD06's checksum.
void buildCrcTable() {
  for (int i = 0; i < 256; i++) {
    uint8_t crc = i;
    for (int bit = 0; bit < 8; bit++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
    crcTable[i] = crc;
  }
}

uint8_t crc8(const uint8_t *data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) crc = crcTable[crc ^ data[i]];
  return crc;
}

uint16_t u16(const uint8_t *p) {
  return p[0] | (p[1] << 8);
}

// After a bad packet, keep any later header already in the buffer instead of
// throwing it away, so the next good packet isn't lost.
void resync() {
  for (int i = 1; i < PACKET_SIZE; i++) {
    if (packet[i] == HEADER && (i == PACKET_SIZE - 1 || packet[i + 1] == VER_LEN)) {
      packetLen = PACKET_SIZE - i;
      memmove(packet, packet + i, packetLen);
      return;
    }
  }
  packetLen = 0;
}

void handlePacket() {
  if (crc8(packet, PACKET_SIZE - 1) != packet[PACKET_SIZE - 1]) {
    crcErrors++;
    resync();
    return;
  }
  packetLen = 0;

  uint32_t now = millis();
  lastPacketMs = now;
  packetsThisPeriod++;
  speedDegS = u16(packet + 2);

  uint16_t start = u16(packet + 4);
  uint16_t end = u16(packet + END_ANGLE_OFFSET);
  uint32_t span = (end + 36000 - start) % 36000;
  for (int i = 0; i < POINTS; i++) {
    uint32_t angle = (start + span * i / (POINTS - 1)) % 36000;
    uint32_t offFront = angle > 18000 ? 36000 - angle : angle;
    uint16_t dist = u16(packet + 6 + 3 * i);
    if (dist > 0 && offFront <= FRONT_WINDOW) {
      frontMm = dist;
      lastFrontMs = now;
    }
  }
}

void feed(uint8_t b) {
  if (packetLen == 0) {
    if (b == HEADER) packet[packetLen++] = b;
    return;
  }
  if (packetLen == 1) {
    if (b == VER_LEN) {
      packet[packetLen++] = b;
    } else {
      packetLen = (b == HEADER) ? 1 : 0;
    }
    return;
  }
  packet[packetLen++] = b;
  if (packetLen == PACKET_SIZE) handlePacket();
}

void readLidar() {
  uint8_t buf[256];
  int n;
  while ((n = Serial2.available()) > 0) {
    n = Serial2.read(buf, min(n, (int)sizeof(buf)));
    if (n <= 0) break;
    Serial.write(buf, n);  // USB bridge
    lastByteMs = millis();
    bytesThisPeriod += n;
    for (int i = 0; i < n; i++) feed(buf[i]);
  }
}

// Redraws only when the text changed, so the screen doesn't flicker.
void drawScreen() {
  uint32_t now = millis();
  const char *head;
  const char *hint = "";
  uint16_t color;
  if (lastByteMs == 0 || now - lastByteMs > NO_DATA_MS) {
    head = "NO DATA";
    hint = "Check 5V, GND\nand Tx -> G33";
    color = TFT_RED;
  } else if (lastPacketMs == 0 || now - lastPacketMs > NO_DATA_MS) {
    head = "BAD DATA";
    hint = "Bytes arrive but\nno valid packets";
    color = TFT_RED;
  } else {
    head = "LD06 OK";
    color = TFT_GREEN;
  }

  String body = "Scan " + String(speedDegS / 360.0f, 1) + " Hz\n";
  body += String(packetsPerSec) + " pkt/s\n";
  if (lastFrontMs != 0 && now - lastFrontMs < NO_DATA_MS) {
    body += "Front " + String(frontMm) + " mm\n";
  } else {
    body += "Front --\n";
  }
  body += "CRC err " + String(crcErrors) + "\n";
  body += String(bytesPerSec) + " B/s\n";
  body += hint;

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

void setup() {
  auto cfg = M5.config();
  cfg.output_power = true;  // keep 5V on the Grove port: it powers the lidar
  cfg.serial_baudrate = 0;  // USB serial is started below, with a bigger buffer
  M5.begin(cfg);
  M5.Display.setRotation(3);
  M5.Display.setTextSize(M5.Display.width() >= 240 ? 2 : 1);  // Plus/Plus2 vs StickC
  M5.Display.setTextWrap(false);
  M5.Display.fillScreen(TFT_BLACK);

  buildCrcTable();

  // The lidar sends ~18 kB/s. The default 256-byte buffers would overflow while
  // the screen redraws.
  Serial.setTxBufferSize(4096);
  Serial.begin(LIDAR_BAUD);
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, -1);  // receive only

  drawScreen();
}

void loop() {
  M5.update();
  readLidar();

  uint32_t now = millis();
  if (now - lastRateMs >= RATE_PERIOD_MS) {
    lastRateMs = now;
    bytesPerSec = bytesThisPeriod;
    packetsPerSec = packetsThisPeriod;
    bytesThisPeriod = packetsThisPeriod = 0;
  }

  if (now - lastScreenMs >= SCREEN_REFRESH_MS) {
    lastScreenMs = now;
    drawScreen();
  }
}
