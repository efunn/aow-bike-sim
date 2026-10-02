// Four-channel force sensor, Teensy 3.6 (Serial + Keyboard + Mouse + Joystick USB type).
// Sensors: TE FS2050, 1500 gf (14.71 N) range (datasheet: ../sheet/tefs20.pdf).
// FS2050: 1 V zero, 3 V span at 5 V, ratiometric; at a 3.3 V supply the span is 1.98 V.
//
// Derived from rewire-keyboard/firmware/multiForceKeyboard/multiForceKeyboard.ino
// (what this board was running; forceKeyboard.ino has the wrong pins)
// joystick output (X/Y/Z/Zrotate, 0-1023 = 0-4 N); that sketch's span ran zero -> 3.3 V (~25% low)
// raw 13-bit counts streamed over USB serial whenever a host has the port open
// (Convert to Newtons in host, e.g. ../force_sensor.py)

const int NUM_CH = 4;
const int PINS[NUM_CH] = {A20, A18, A8, A6};   // sensors from left to right -> (X, Y, Z, Zrotate)
const int ADC_BITS = 13;
const int ADC_AVERAGES = 4;
const float ADC_MAX = 8192.0;
const float ADC_VREF = 3.3;
const float MAX_FORCE_N = 14.709975;           // 1500 gf
const float SPAN_V = 3.0 * 3.3 / 5.0;          // FS2050 span at a 3.3 V supply (nominal)
const float ZERO_V[NUM_CH] = {0.815, 0.660, 0.683, 0.669};  // unloaded, measured 2026-10-02
const float JOY_FULL_N = 4.0;                  // joystick 1023 = this
const uint32_t JOY_PERIOD_US = 1000;

int raw[NUM_CH];
uint32_t lastJoy = 0;

int joyCounts(int i, int counts) {
  float v = ADC_VREF * counts / ADC_MAX;
  float n = MAX_FORCE_N * (v - ZERO_V[i]) / SPAN_V;
  int out = (int)(1023.0 * n / JOY_FULL_N);
  return max(0, min(out, 1023));
}

void setup() {
  analogReadResolution(ADC_BITS);
  analogReadAveraging(ADC_AVERAGES);
  Joystick.useManualSend(true);
  Serial.begin(115200);   // (baud ignored)
}

void loop() {
  uint32_t t = micros();
  for (int i = 0; i < NUM_CH; i++) raw[i] = analogRead(PINS[i]);

  if (Serial.dtr()) {     // while a host is listening
    char line[48];
    int n = snprintf(line, sizeof line, "%lu,%d,%d,%d,%d\n",
                     (unsigned long)t, raw[0], raw[1], raw[2], raw[3]);
    Serial.write(line, n);
  }

  if (t - lastJoy >= JOY_PERIOD_US) {
    lastJoy = t;
    Joystick.X(joyCounts(0, raw[0]));
    Joystick.Y(joyCounts(1, raw[1]));
    Joystick.Z(joyCounts(2, raw[2]));
    Joystick.Zrotate(joyCounts(3, raw[3]));
    Joystick.send_now();
  }
}
