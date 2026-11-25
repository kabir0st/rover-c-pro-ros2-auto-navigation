// UDP protocol between the rover and the ROS 2 driver.
// ros2_ws/src/rover_driver/rover_driver/protocol.py mirrors this file; change both together.
//
// Little-endian, packed. The rover listens on ROVER_UDP_PORT and sends telemetry
// back to whoever sent the last valid command.
#pragma once
#include <stdint.h>

#define ROVER_UDP_PORT 4210
#define PROTOCOL_VERSION 1

// ---- PC -> rover: sent at a steady 20 Hz, so it also works as the heartbeat ----

#define CMD_FLAG_LOCK 0x01    // lock the rover (same as pressing button A)
#define CMD_FLAG_UNLOCK 0x02  // unlock it

struct __attribute__((packed)) CommandPacket {
  char magic[2];     // 'R','C'
  uint8_t version;   // PROTOCOL_VERSION
  uint8_t flags;     // CMD_FLAG_*
  uint16_t seq;
  int8_t x;          // strafe  -100..100, + = right    (M5_RoverC::setSpeed axes)
  int8_t y;          // forward -100..100, + = forward
  int8_t z;          // turn    -100..100, + = left (counter-clockwise)
  uint8_t grip;      // gripper servo angle 10..90, 0 = leave as is
};
static_assert(sizeof(CommandPacket) == 10, "CommandPacket must be 10 bytes");

// ---- rover -> PC: sent at 50 Hz to the last command sender ----

#define TLM_FLAG_LOCKED 0x01
#define TLM_FLAG_ROVER_FOUND 0x02  // RoverC base answered on I2C at boot
#define TLM_FLAG_MOVING 0x04
#define TLM_FLAG_IMU_OK 0x08

struct __attribute__((packed)) TelemetryPacket {
  char magic[2];        // 'R','T'
  uint8_t version;      // PROTOCOL_VERSION
  uint8_t flags;        // TLM_FLAG_*
  uint16_t seq;         // telemetry counter
  uint16_t cmd_seq;     // seq of the last command received
  uint32_t uptime_ms;
  float accel[3];       // g, stick's IMU axes
  float gyro[3];        // deg/s
  float imu_temp;       // deg C, NAN if unknown
  int16_t batt_mv;      // stick battery voltage, <= 0 if unknown
  int16_t batt_ma;      // + charging / - discharging, 0 if the board can't measure it
  int16_t vbus_mv;      // 5 V input voltage, -1 if the board can't measure it
  int8_t batt_pct;      // 0..100, -1 if unknown
  int8_t charge_state;  // M5Unified m5::charge_state_t
  int8_t rssi;          // WiFi signal of the link to the PC in dBm, 0 if unknown
  uint8_t grip;         // current gripper angle, 0 = never set
};
static_assert(sizeof(TelemetryPacket) == 50, "TelemetryPacket must be 50 bytes");
