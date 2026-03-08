#include <actuator.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define USB_SERIAL Serial

Dynamixel2Arduino dxl(DXL_SERIAL);

const uint32_t DXL_BAUD = 1000000;
const uint32_t USB_BAUD = 1000000;
const float    DXL_PROTOCOL_VERSION = 1.0;

static const uint8_t N = 6;
static const uint8_t IDS[N] = {1,2,3,4,5,6};

using namespace ControlTableItem;

static inline void writeU16LE(uint16_t v) {
  USB_SERIAL.write((uint8_t)(v & 0xFF));
  USB_SERIAL.write((uint8_t)((v >> 8) & 0xFF));
}

static bool readLine(char* buf, size_t maxLen, uint32_t timeout_ms = 120) {
  uint32_t t0 = millis();
  size_t i = 0;

  while ((millis() - t0) < timeout_ms) {
    while (USB_SERIAL.available()) {
      char c = (char)USB_SERIAL.read();

      if (c == '\n') {
        while (i > 0 && buf[i - 1] == '\r') i--;
        buf[i] = '\0';
        return true;
      }

      if (i + 1 < maxLen) buf[i++] = c;
    }
  }

  if (i > 0) {
    while (i > 0 && buf[i - 1] == '\r') i--;
    buf[i] = '\0';
  } else {
    buf[0] = '\0';
  }
  return false;
}

static void torqueAll(bool on) {
  for (uint8_t i = 0; i < N; i++) {
    if (on) dxl.torqueOn(IDS[i]);
    else    dxl.torqueOff(IDS[i]);
  }
}

void setup() {
  USB_SERIAL.begin(USB_BAUD);
  while (!USB) {}

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.begin(DXL_BAUD);

  delay(100);

  for (uint8_t i = 0; i < N; i++) {
    dxl.ping(IDS[i]);
  }

  torqueAll(true);
}

void loop() {
  if (!USB_SERIAL.available()) return;

  int c = USB_SERIAL.read();
  if (c < 0) return;

  // Q -> 6 x PRESENT_POSITION as uint16 LE
  if ((char)c == 'Q') {
    for (uint8_t i = 0; i < N; i++) {
      uint16_t pos = (uint16_t)dxl.readControlTableItem(PRESENT_POSITION, IDS[i]);
      writeU16LE(pos);
    }
    return;
  }

  // G -> 6 x GOAL_POSITION as uint16 LE
  if ((char)c == 'G') {
    for (uint8_t i = 0; i < N; i++) {
      uint16_t gp = (uint16_t)dxl.readControlTableItem(GOAL_POSITION, IDS[i]);
      writeU16LE(gp);
    }
    return;
  }

  // R -> extended register dump per motor, 21 bytes each:
  // [id u8]
  // [torque_enable u8]
  // [cw_angle_limit u16]
  // [ccw_angle_limit u16]
  // [moving_speed u16]
  // [moving u8]
  // [torque_limit u16]
  // [torque_ctrl_mode_enable u8]
  // [max_torque u16]
  // [alarm_led u8]
  // [shutdown u8]
  // [present_voltage u8]
  // [present_temperature u8]
  if ((char)c == 'R') {
    for (uint8_t i = 0; i < N; i++) {
      uint8_t id = IDS[i];

      uint8_t  torque_enable   = (uint8_t)dxl.readControlTableItem(TORQUE_ENABLE, id);
      uint16_t cw_limit        = (uint16_t)dxl.readControlTableItem(CW_ANGLE_LIMIT, id);
      uint16_t ccw_limit       = (uint16_t)dxl.readControlTableItem(CCW_ANGLE_LIMIT, id);
      uint16_t moving_speed    = (uint16_t)dxl.readControlTableItem(MOVING_SPEED, id);
      uint8_t  moving          = (uint8_t)dxl.readControlTableItem(MOVING, id);
      uint16_t torque_limit    = (uint16_t)dxl.readControlTableItem(TORQUE_LIMIT, id);
      uint8_t  torque_ctrl_en  = (uint8_t)dxl.readControlTableItem(TORQUE_CTRL_MODE_ENABLE, id);
      uint16_t max_torque      = (uint16_t)dxl.readControlTableItem(MAX_TORQUE, id);
      uint8_t  alarm_led       = (uint8_t)dxl.readControlTableItem(ALARM_LED, id);
      uint8_t  shutdown        = (uint8_t)dxl.readControlTableItem(SHUTDOWN, id);
      uint8_t  present_voltage = (uint8_t)dxl.readControlTableItem(PRESENT_VOLTAGE, id);
      uint8_t  present_temp    = (uint8_t)dxl.readControlTableItem(PRESENT_TEMPERATURE, id);

      USB_SERIAL.write(id);
      USB_SERIAL.write(torque_enable);
      writeU16LE(cw_limit);
      writeU16LE(ccw_limit);
      writeU16LE(moving_speed);
      USB_SERIAL.write(moving);
      writeU16LE(torque_limit);
      USB_SERIAL.write(torque_ctrl_en);
      writeU16LE(max_torque);
      USB_SERIAL.write(alarm_led);
      USB_SERIAL.write(shutdown);
      USB_SERIAL.write(present_voltage);
      USB_SERIAL.write(present_temp);
    }
    return;
  }

  // J a b c d e f\n -> set raw GOAL_POSITION values
  if ((char)c == 'J') {
    char line[128];
    if (!readLine(line, sizeof(line), 150)) return;

    long g1, g2, g3, g4, g5, g6;
    int n = sscanf(line, " %ld %ld %ld %ld %ld %ld", &g1, &g2, &g3, &g4, &g5, &g6);
    if (n != 6) return;

    uint16_t goals[6];
    goals[0] = (uint16_t)g1;
    goals[1] = (uint16_t)g2;
    goals[2] = (uint16_t)g3;
    goals[3] = (uint16_t)g4;
    goals[4] = (uint16_t)g5;
    goals[5] = (uint16_t)g6;

    for (uint8_t i = 0; i < N; i++) {
      dxl.writeControlTableItem(GOAL_POSITION, IDS[i], goals[i]);
    }
    return;
  }

  // L a b c d e f\n -> set raw TORQUE_LIMIT values
  if ((char)c == 'L') {
    char line[128];
    if (!readLine(line, sizeof(line), 150)) return;

    long t1, t2, t3, t4, t5, t6;
    int n = sscanf(line, " %ld %ld %ld %ld %ld %ld", &t1, &t2, &t3, &t4, &t5, &t6);
    if (n != 6) return;

    uint16_t tl[6];
    tl[0] = (uint16_t)t1;
    tl[1] = (uint16_t)t2;
    tl[2] = (uint16_t)t3;
    tl[3] = (uint16_t)t4;
    tl[4] = (uint16_t)t5;
    tl[5] = (uint16_t)t6;

    for (uint8_t i = 0; i < N; i++) {
      dxl.writeControlTableItem(TORQUE_LIMIT, IDS[i], tl[i]);
    }
    return;
  }

  // T 1\n / T 0\n
  if ((char)c == 'T') {
    char line[32];
    if (!readLine(line, sizeof(line), 150)) return;

    int on = 0;
    if (sscanf(line, " %d", &on) == 1) {
      torqueAll(on != 0);
      USB_SERIAL.print("OK T ");
      USB_SERIAL.println(on ? 1 : 0);
    }
    return;
  }
}