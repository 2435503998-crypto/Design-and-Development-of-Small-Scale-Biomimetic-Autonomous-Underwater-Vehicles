#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>
#include <string.h>

/*********** ====== Hardware & Channels ====== ***********/
static const int ESC_Pin     = 7;     // PWM output pin
static const int ESC_Channel = 0;     // LEDC channel
static const int PWM_freq    = 50;    // 50 Hz (20 ms period)
static const int PWM_bits    = 10;    // 10-bit (0..1023)

static const int anglePin = 0;        // ADC pin (ESP32-C3 ADC0)
static const int ADC_MAX  = 4095;     // 12-bit ADC

/*********** ====== Reference Signal ====== ***********/
static const float A_deg   = 20.0f;   // amplitude (deg)
static const float f_ref   = 2.0f;    // reference frequency (Hz)

/*********** ====== Control Period ====== ***********/
static const float Ts = 0.02f;                 // 20 ms control period
static const unsigned long Ts_ms = (unsigned long)(Ts * 1000.0f);

/*********** ====== PD Controller (with filtered D) ====== ***********/
// PD form: u = Kp * e - Kd * theta_dot_filt
static const float Kp = 2.6f;                 // proportional gain (start from 0.8~1.5)
static const float Kd = 0.21f;                 // derivative gain (start small)
static const float fd_d = 8.0f;                // D-term low-pass cutoff (Hz)
static const float beta_d = 1.0f - expf(-2.0f * PI * fd_d * Ts);

/*********** ====== Measurement Low-pass Filter ====== ***********/
static const float fc_meas = 5.0f;             // position LPF cutoff (Hz)
static const float alpha   = 1.0f - expf(-2.0f * PI * fc_meas * Ts);

/*********** ====== PWM Map & Limits ====== ***********/
static const int PWM_CENTER_US  = 1500;
static const int PWM_RANGE_US   = 500;
static const int PWM_MIN_US     = 1000;
static const int PWM_MAX_US     = 2000;
static const int DEAD_BAND_US   = 20;

/*********** ====== Soft-start ====== ***********/
static const float ramp_time_s = 3.0f;

/*********** ====== Output Scale ====== ***********/
static const float OUTPUT_SCALE = 1.0f;

/*********** ====== Turn-cycle Params ====== ***********/
static const float TURN_OFFSET_DEG         = 15.0f;        // left-turn bias (deg)
static const unsigned long TURN_DELAY_MS   = 5000UL;       // go straight 5 s, then prepare turn
static const unsigned long TURN_HOLD_MS    = 2000UL;       // hold turn 2 s
static const float CENTER_TOL_DEG          = 2.0f;         // re-center threshold (±2°)

/*********** ====== Internal States ====== ***********/
static float angle_filt_rad = 0.0f;
static float last_angle_filt_rad = 0.0f;
static float theta_dot_filt = 0.0f;

static unsigned long t0_ms   = 0;
static unsigned long last_ms = 0;

static int mode = 0;                 // 0=idle (hold center), 1=run (with turn cycle)
static float turn_offset_deg = 0.0f;
static unsigned long cycle_anchor_ms = 0;
static unsigned long turn_start_ms   = 0;

enum TurnState { TURN_IDLE = 0, TURN_WAIT_CENTER, TURN_LEFT };
static TurnState turn_state = TURN_IDLE;

/*********** ====== BLE (UART-like) ====== ***********/
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;  // notify (device -> phone)
BLECharacteristic* pRxCharacteristic = nullptr;  // write  (phone  -> device)
bool deviceConnected = false;

// Nordic UART Service (NUS)-like UUIDs
#define SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // notify
#define CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // write

/*********** ====== Helpers ====== ***********/
static inline float deg2rad(float d){ return d * PI / 180.0f; }
static inline float rad2deg(float r){ return r * 180.0f / PI; }

void writePWM_us(int pulse_us){
  pulse_us = constrain(pulse_us, PWM_MIN_US, PWM_MAX_US);
  int duty = (int)lround((pulse_us / 20000.0f) * ((1 << PWM_bits) - 1));
  ledcWrite(ESC_Channel, duty);
}

void ESC_Init(){
  ledcSetup(ESC_Channel, PWM_freq, PWM_bits);
  ledcAttachPin(ESC_Pin, ESC_Channel);
  writePWM_us(PWM_CENTER_US);
}

float read_angle_centered_rad(){
  int adc = analogRead(anglePin);
  float angle_abs_rad = (adc / (float)ADC_MAX) * 2.0f * PI; // [0..2π)
  float centered = angle_abs_rad - PI;                      // [-π..π)
  while (centered >  PI) centered -= 2.0f * PI;
  while (centered <= -PI) centered += 2.0f * PI;
  return centered;
}

static inline void reset_cycle_logic(unsigned long now_ms){
  turn_offset_deg = 0.0f;
  turn_state = TURN_IDLE;
  cycle_anchor_ms = now_ms;
  theta_dot_filt = 0.0f;
}

/*********** ====== BLE Callbacks ====== ***********/
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) override { deviceConnected = false; }
};

class RxCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string rx = pChar->getValue();
    if (rx.empty()) return;

    char c = rx[0];
    unsigned long now = millis();
    if (c == '1'){
      mode = 1;
      reset_cycle_logic(now);
      Serial.println("BLE: Start loop");
    } else if (c == '2'){
      mode = 0;
      turn_offset_deg = 0.0f;
      turn_state = TURN_IDLE;
      writePWM_us(PWM_CENTER_US);
      Serial.println("BLE: Stop & hold center");
    }
  }
};

void BLE_Init(){
  BLEDevice::init("AUV-ESP32C3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // TX: notify (device -> phone)
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY);

  // RX: write (phone -> device)
  pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
}

/*********** ====== Arduino Main ====== ***********/
void setup(){
  Serial.begin(115200);
  ESC_Init();
  BLE_Init();

  angle_filt_rad = read_angle_centered_rad();
  last_angle_filt_rad = angle_filt_rad;
  t0_ms = last_ms = millis();
  cycle_anchor_ms = t0_ms;

  const char* header = "ref_deg,meas_deg,u_cmd\n";
  if (pTxCharacteristic){
    pTxCharacteristic->setValue((uint8_t*)header, strlen(header));
    pTxCharacteristic->notify();
  }
  Serial.print(header);

  Serial.println("Controls: Serial or BLE RX -> send '1' to RUN, '2' to IDLE(center).");
}

void loop(){
  unsigned long now = millis();

  /******** Serial one-key control: '1' run, '2' idle(center) ********/
  if (Serial.available()){
    char c = (char)Serial.read();
    if (c == '1'){
      mode = 1;
      reset_cycle_logic(now);
      Serial.println("Start loop");
    } else if (c == '2'){
      mode = 0;
      turn_offset_deg = 0.0f;
      turn_state = TURN_IDLE;
      writePWM_us(PWM_CENTER_US);
      Serial.println("Stop & hold center");
    }
  }

  // Fixed-rate loop
  if (now - last_ms < Ts_ms) return;
  last_ms += Ts_ms;

  float t = (now - t0_ms) / 1000.0f;

  /******** Reference signal ********/
  float theta_ref;
  if (mode == 1){
    // Turning state machine
    switch (turn_state){
      case TURN_IDLE:
        if ((now - cycle_anchor_ms) >= TURN_DELAY_MS){
          turn_state = TURN_WAIT_CENTER;
          turn_offset_deg = 0.0f;
        }
        break;
      case TURN_WAIT_CENTER:
        if (fabsf(rad2deg(angle_filt_rad)) <= CENTER_TOL_DEG){
          turn_state = TURN_LEFT;
          turn_offset_deg = TURN_OFFSET_DEG;  // left turn
          turn_start_ms = now;
        }
        break;
      case TURN_LEFT:
        if ((now - turn_start_ms) >= TURN_HOLD_MS){
          turn_offset_deg = 0.0f;
          turn_state = TURN_IDLE;
          cycle_anchor_ms = now;
        }
        break;
    }
    theta_ref = deg2rad(A_deg) * sinf(2.0f * PI * f_ref * t) + deg2rad(turn_offset_deg);
  } else {
    theta_ref = 0.0f;  // idle: hold center
  }

  /******** Measurement & LPF ********/
  float angle_meas = read_angle_centered_rad();
  angle_filt_rad += alpha * (angle_meas - angle_filt_rad);

  // Estimate angular velocity with LPF (for D-term)
  float theta_dot = (angle_filt_rad - last_angle_filt_rad) / Ts;
  theta_dot_filt += beta_d * (theta_dot - theta_dot_filt);
  last_angle_filt_rad = angle_filt_rad;

  /******** PD Control (idle -> no control) ********/
  float u_cmd = 0.0f;
  if (mode == 1){
    float e = theta_ref - angle_filt_rad;
    u_cmd = Kp * e - Kd * theta_dot_filt;

    // Soft-start and output limit in normalized [-1, 1]
    float ramp = (t < ramp_time_s) ? (t / ramp_time_s) : 1.0f;
    if (u_cmd >  ramp) u_cmd =  ramp;
    if (u_cmd < -ramp) u_cmd = -ramp;
    if (u_cmd >  1.0f) u_cmd =  1.0f;
    if (u_cmd < -1.0f) u_cmd = -1.0f;

    u_cmd *= OUTPUT_SCALE;
  } else {
    writePWM_us(PWM_CENTER_US);  // force center when idle
  }

  /******** PWM output (only in RUN mode) ********/
  if (mode == 1){
    int pwm_us = PWM_CENTER_US + (int)lround(u_cmd * PWM_RANGE_US);
    if (abs(pwm_us - PWM_CENTER_US) < DEAD_BAND_US) pwm_us = PWM_CENTER_US;  // dead-band
    writePWM_us(pwm_us);
  }

  /******** Telemetry (BLE + Serial) ********/
  float value1 = 180.0f + rad2deg(theta_ref);       // ref_deg, shifted for plotting
  float value2 = 180.0f + rad2deg(angle_filt_rad);  // meas_deg, shifted
  float value3 = u_cmd * 50.0f;                     // control signal scaled

  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f\n", value1, value2, value3);
  if (deviceConnected && pTxCharacteristic && n > 0) {
    pTxCharacteristic->setValue((uint8_t*)buf, n);
    pTxCharacteristic->notify();
  }
  Serial.write((uint8_t*)buf, n);
}
