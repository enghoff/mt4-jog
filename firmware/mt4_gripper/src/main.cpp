/*
 * MT4 gripper exercise firmware
 *
 * Stock MT4 M3 PWM path (flash @ 0x5A8E): Timer4 OC4B on D7 / PH4, duty in OCR4B,
 * TOP in ICR4 = 0x447A. WLKATA S values are 0–1000 (40 ≈ open, 60 ≈ closed).
 *
 * On boot, alternates S40 and S60 every 2 s. Serial @ 115200:
 *   ?       status
 *   go      resume auto cycle
 *   stop    hold current PWM, pause cycle
 *   s <n>   set S value 0–1000 (e.g. s 40)
 *   40 / 60 shorthand for s 40 / s 60
 *   0       PWM off (same as stock M3 S0)
 *
 * Hardware checklist:
 *   - Arm 12 V supply ON (USB alone is not enough for gripper power)
 *   - IDC cable: arm base → extender
 *   - Gripper on extender GRIPPER / PWM port
 *   - Extender is powered from the IDC (stock ESP32 firmware should be running)
 */

#include <Arduino.h>
#include <avr/io.h>

static constexpr uint16_t S_OPEN = 40;
static constexpr uint16_t S_CLOSED = 60;
static constexpr uint16_t S_MAX = 1000;
static constexpr uint16_t PWM_TOP = 0x447A;  // stock scaling constant (17530)
static constexpr unsigned long HOLD_MS = 2000;

static uint16_t currentS = S_OPEN;
static bool autoCycle = true;
static bool atOpen = true;
static unsigned long lastToggleMs = 0;

static uint16_t sToOcr(uint16_t s) {
  if (s == 0) {
    return 0;
  }
  if (s > S_MAX) {
    s = S_MAX;
  }
  return (uint16_t)((uint32_t)s * PWM_TOP / S_MAX);
}

static void spindleEnablePins() {
  // Stock paths @ 0x115C and @ 0x588A.
  DDRH |= (1 << PH4);   // D7 = Timer4 OC4B
  DDRE |= (1 << PE3);   // D3
  PORTE |= (1 << PE5);  // D5 tool enable
}

static void gripperPwmInit() {
  spindleEnablePins();

  // Match stock MT4 Timer4 setup (flash @ 0x5A8E).
  TCCR4A = 0x23;  // COM4B non-inverting PWM enabled
  TCCR4B = (TCCR4B & 0xE0) | 0x1A;
  ICR4 = PWM_TOP;
  OCR4A = 0xFFFF;  // stock init; output pin is OC4B
  OCR4B = 0;
}

static void gripperPwmOff() {
  OCR4B = 0;
  TCCR4A &= (uint8_t)~0x20;  // clear COM4B (stock stop @ 0x1150)
}

static void gripperPwmOn() {
  TCCR4A |= 0x20;  // enable COM4B PWM output
}

static void setGripperS(uint16_t s) {
  currentS = s;
  if (s == 0) {
    gripperPwmOff();
    Serial.println(F("M3 S0 (off)"));
    return;
  }

  spindleEnablePins();
  gripperPwmOn();
  OCR4B = sToOcr(s);
  Serial.print(F("M3 S"));
  Serial.print(s);
  Serial.print(F(" -> OCR4B="));
  Serial.print(OCR4B);
  Serial.print(F(" ICR4="));
  Serial.println(ICR4);
}

static void printStatus() {
  Serial.print(F("auto="));
  Serial.print(autoCycle ? F("on") : F("off"));
  Serial.print(F(" S="));
  Serial.print(currentS);
  Serial.print(F(" OCR4B="));
  Serial.print(OCR4B);
  Serial.print(F(" TCCR4A=0x"));
  Serial.println(TCCR4A, HEX);
}

static void handleLine(char* line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }
  if (line[0] == '\0') {
    return;
  }

  if (strcmp(line, "?") == 0) {
    printStatus();
    return;
  }
  if (strcmp(line, "go") == 0) {
    autoCycle = true;
    lastToggleMs = millis();
    Serial.println(F("auto cycle on"));
    return;
  }
  if (strcmp(line, "stop") == 0) {
    autoCycle = false;
    Serial.println(F("auto cycle off"));
    return;
  }
  if (strcmp(line, "0") == 0) {
    autoCycle = false;
    setGripperS(0);
    return;
  }
  if (strcmp(line, "40") == 0) {
    autoCycle = false;
    setGripperS(S_OPEN);
    return;
  }
  if (strcmp(line, "60") == 0) {
    autoCycle = false;
    setGripperS(S_CLOSED);
    return;
  }
  if (strncmp(line, "s ", 2) == 0) {
    long v = atol(line + 2);
    if (v < 0) {
      v = 0;
    }
    if (v > S_MAX) {
      v = S_MAX;
    }
    autoCycle = false;
    setGripperS((uint16_t)v);
    return;
  }

  Serial.println(F("commands: ? go stop 0 40 60 s <0-1000>"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }

  gripperPwmInit();
  lastToggleMs = millis();

  Serial.println(F("MT4 gripper test — cycling S40/S60 every 2s"));
  Serial.println(F("PWM: D7 (Timer4 OC4B) | need 12V + IDC + extender + gripper"));
  setGripperS(S_OPEN);
}

void loop() {
  if (autoCycle) {
    unsigned long now = millis();
    if (now - lastToggleMs >= HOLD_MS) {
      lastToggleMs = now;
      atOpen = !atOpen;
      setGripperS(atOpen ? S_OPEN : S_CLOSED);
    }
  }

  if (Serial.available()) {
    static char buf[32];
    static uint8_t len = 0;
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        buf[len] = '\0';
        handleLine(buf);
        len = 0;
      } else if ((size_t)len + 1 < sizeof(buf)) {
        buf[len++] = c;
      }
    }
  }
}
