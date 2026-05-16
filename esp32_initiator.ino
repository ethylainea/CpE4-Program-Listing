// =========================================================================
// VETIVERTRACK - INITIATOR (SENDER)
// Collects soil + tilt data, sends via ESP-NOW to responder
// =========================================================================
#include <WiFi.h>
#include <esp_now.h>
#include <ModbusMaster.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Must be defined before any function that references it
struct KalmanState {
  float estimate;
  float errorCovariance;
  float processNoise;
  float measurementNoise;
  float kalmanGain;
};

// =========================================================================
// ESP-NOW CONFIGURATION
// =========================================================================
uint8_t receiverMACAddress[] = {0x88, 0x57, 0x21, 0xAD, 0x11, 0x24};

const char* AP_SSID     = "VetiverTrack-F01";
const char* AP_PASSWORD = "vetivert";

typedef struct struct_message {
  float moisture;
  float ec;
  float ph;
  float tilt;
  float pitch;
  float roll;
  unsigned long timestamp;
  float batteryPct;
} struct_message;

typedef struct struct_command {
  uint8_t cmd;
} struct_command;

struct_message outgoingData;
bool espNowReady = false;

// =========================================================================
// MODBUS SETUP
// =========================================================================
#define RX2           16
#define TX2           17
#define SENSOR_ID     1
#define BAUD_RATE     4800
#define READ_INTERVAL 5000

ModbusMaster node;
Adafruit_MPU6050 mpu;
bool mpuFound = false;

// =========================================================================
// MPU6050 CALIBRATION OFFSETS
// =========================================================================
const int16_t MPU_ACCEL_X_OFFSET = -4095;
const int16_t MPU_ACCEL_Y_OFFSET = -476;
const int16_t MPU_ACCEL_Z_OFFSET =  4149;
const int16_t MPU_GYRO_X_OFFSET  =   45;
const int16_t MPU_GYRO_Y_OFFSET  =   58;
const int16_t MPU_GYRO_Z_OFFSET  =    0;

// =========================================================================
// BATTERY MONITOR (GPIO34, voltage divider 10k + 1k)
// 3-cell LiPo: full=12.6V, cutoff=9.0V
// =========================================================================
#define BAT_PIN        34
#define BAT_R1         10000.0f
#define BAT_R2          1000.0f
#define BAT_VREF           3.3f
#define BAT_ADC_MAX     4095.0f
#define BAT_V_FULL        12.6f
#define BAT_V_EMPTY        9.0f
#define BAT_SAMPLES          8
#define BAT_UPDATE_INTERVAL  10000

float smoothedBatPct      = -1.0f;
const float BAT_EMA_ALPHA =  0.05f;

float rawBatteryPercent() {
  long sum = 0;
  for (int i = 0; i < BAT_SAMPLES; i++) {
    sum += analogRead(BAT_PIN);
    delay(2);
  }
  float adcVal = sum / (float)BAT_SAMPLES;
  float vPin   = (adcVal / BAT_ADC_MAX) * BAT_VREF;
  float vBat   = vPin * (BAT_R1 + BAT_R2) / BAT_R2;
  float pct    = (vBat - BAT_V_EMPTY) / (BAT_V_FULL - BAT_V_EMPTY) * 100.0f;
  Serial.printf("[BAT-INIT] ADC=%.0f  Vpin=%.3fV  Vbat=%.3fV  pct=%.1f%%\n",
                adcVal, vPin, vBat, pct);
  return constrain(pct, 0.0f, 100.0f);
}

void updateBatteryEMA() {
  float raw = rawBatteryPercent();
  if (smoothedBatPct < 0.0f) smoothedBatPct = raw;
  else smoothedBatPct = BAT_EMA_ALPHA * raw + (1.0f - BAT_EMA_ALPHA) * smoothedBatPct;
}

// =========================================================================
// TILT CONFIGURATION
// =========================================================================
const float TILT_CHANGE_THRESHOLD = 0.5;
const int   CALIBRATION_SAMPLES   = 100;

float baselinePitch      = 0.0;
float baselineRoll       = 0.0;
bool  baselineCalibrated = false;

float lastReportedPitch = 0.0;
float lastReportedRoll  = 0.0;
float lastReportedTilt  = 0.0;

float previousTilt  = 0.0;
bool  firstTiltRead = true;

// =========================================================================
// FILTER CONFIGURATION
// =========================================================================
#define USE_KALMAN_FILTER false
const float COMP_FILTER_ALPHA = 0.70;

KalmanState kalmanPitch = {0.0, 1.0, 0.01, 0.05, 0.0};
KalmanState kalmanRoll  = {0.0, 1.0, 0.01, 0.05, 0.0};

float compFilterPitch = 0.0;
float compFilterRoll  = 0.0;

// =========================================================================
// SOIL DATA
// =========================================================================
struct SoilData {
  float moisture;
  float ec;
  float ph;
  float tilt;
  float pitch;
  float roll;
  float deltaTilt;
};
SoilData soilData;

unsigned long lastReadTime  = 0;
unsigned long lastBatUpdate = 0;

// =========================================================================
// ESP-NOW CALLBACKS
// =========================================================================
void OnDataSent(const wifi_tx_info_t *wifi_tx_info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ESP-NOW: Sent OK" : "ESP-NOW: Send FAILED");
}

void OnCmdRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  struct_command cmd;
  memcpy(&cmd, data, sizeof(cmd));
  if (cmd.cmd == 1) {
    Serial.println("Calibrate command received from responder");
    calibrateBaseline();
  }
}

// =========================================================================
// ESP-NOW INIT
// =========================================================================
bool initESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASSWORD);

  Serial.print("Connecting to AP for channel sync");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" connected");
    Serial.printf("Channel: %d\n", WiFi.channel());
  } else {
    Serial.println(" FAILED (proceeding anyway)");
  }

  Serial.print("Initiator MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return false;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnCmdRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMACAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return false;
  }

  Serial.println("ESP-NOW ready");
  return true;
}

// =========================================================================
// SEND DATA VIA ESP-NOW
// =========================================================================
void sendDataESPNow(float pitch, float roll) {
  if (!espNowReady) return;

  outgoingData.moisture   = soilData.moisture;
  outgoingData.ec         = soilData.ec;
  outgoingData.ph         = soilData.ph;
  outgoingData.tilt       = soilData.tilt;
  outgoingData.pitch      = pitch;
  outgoingData.roll       = roll;
  outgoingData.timestamp  = millis();
  outgoingData.batteryPct = smoothedBatPct < 0.0f ? 0.0f : smoothedBatPct;

  esp_now_send(receiverMACAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
}

// =========================================================================
// KALMAN FILTER
// =========================================================================
float updateKalman(KalmanState &k, float measurement) {
  k.errorCovariance += k.processNoise;
  k.kalmanGain       = k.errorCovariance / (k.errorCovariance + k.measurementNoise);
  k.estimate         = k.estimate + k.kalmanGain * (measurement - k.estimate);
  k.errorCovariance  = (1.0 - k.kalmanGain) * k.errorCovariance;
  return k.estimate;
}

// =========================================================================
// COMPLEMENTARY FILTER
// =========================================================================
float updateComplementaryFilter(float &filtered, float measurement) {
  filtered = COMP_FILTER_ALPHA * filtered + (1.0 - COMP_FILTER_ALPHA) * measurement;
  return filtered;
}

// =========================================================================
// COMPUTE TILT ANGLES
// =========================================================================
void computeTiltAngles(float ax, float ay, float az, float &pitch, float &roll) {
  pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  roll  = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

// =========================================================================
// BASELINE CALIBRATION
// =========================================================================
void calibrateBaseline() {
  if (!mpuFound) {
    Serial.println("MPU6050 not available");
    return;
  }

  Serial.printf("Calibrating (%d samples)...\n", CALIBRATION_SAMPLES);

  float sumPitch = 0.0, sumRoll = 0.0;
  int validSamples = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      float pitch, roll;
      computeTiltAngles(a.acceleration.x, a.acceleration.y, a.acceleration.z, pitch, roll);
      sumPitch += pitch;
      sumRoll  += roll;
      validSamples++;
    }
    delay(10);
  }

  if (validSamples > 0) {
    baselinePitch      = sumPitch / validSamples;
    baselineRoll       = sumRoll  / validSamples;
    baselineCalibrated = true;
    firstTiltRead      = true;

    #if USE_KALMAN_FILTER
      kalmanPitch.estimate = 0.0;
      kalmanRoll.estimate  = 0.0;
    #else
      compFilterPitch = 0.0;
      compFilterRoll  = 0.0;
    #endif

    Serial.printf("Calibration done. Baseline: Pitch=%.2f° Roll=%.2f°\n\n",
                  baselinePitch, baselineRoll);
  } else {
    Serial.println("Calibration failed");
  }
}

// =========================================================================
// MPU6050 OFFSET WRITER
// =========================================================================
#define MPU6050_I2C_ADDR 0x68

void writeMPUOffset(uint8_t regHigh, int16_t value) {
  Wire.beginTransmission(MPU6050_I2C_ADDR);
  Wire.write(regHigh);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n--- VetiverTrack | Initiator ---\n");

  analogReadResolution(12);
  pinMode(BAT_PIN, INPUT);

  updateBatteryEMA();
  Serial.printf("Initial battery: %.1f%%\n", smoothedBatPct);

  espNowReady = initESPNow();
  if (!espNowReady) Serial.println("WARNING: Running without ESP-NOW");

  Wire.begin(22, 21);
  if (!mpu.begin()) {
    mpuFound = false;
    Serial.println("WARNING: MPU6050 not found");
  } else {
    mpuFound = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    writeMPUOffset(0x06, MPU_ACCEL_X_OFFSET);
    writeMPUOffset(0x08, MPU_ACCEL_Y_OFFSET);
    writeMPUOffset(0x0A, MPU_ACCEL_Z_OFFSET);
    writeMPUOffset(0x13, MPU_GYRO_X_OFFSET);
    writeMPUOffset(0x15, MPU_GYRO_Y_OFFSET);
    writeMPUOffset(0x17, MPU_GYRO_Z_OFFSET);

    Serial.println("MPU6050 ready (+-2g, 21Hz LPF, offsets applied)");
    Serial.printf("  Accel offsets: X=%d  Y=%d  Z=%d\n",
                  MPU_ACCEL_X_OFFSET, MPU_ACCEL_Y_OFFSET, MPU_ACCEL_Z_OFFSET);
    Serial.printf("  Gyro  offsets: X=%d  Y=%d  Z=%d\n",
                  MPU_GYRO_X_OFFSET,  MPU_GYRO_Y_OFFSET,  MPU_GYRO_Z_OFFSET);
    delay(500);
    calibrateBaseline();
  }

  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX2, TX2);
  node.begin(SENSOR_ID, Serial2);

  Serial.println("System ready. Send any key to recalibrate.\n");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();
    Serial.println("Recalibrating...");
    calibrateBaseline();
  }

  if (millis() - lastBatUpdate >= BAT_UPDATE_INTERVAL) {
    lastBatUpdate = millis();
    updateBatteryEMA();
  }

  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();

    float filteredPitch = 0.0, filteredRoll = 0.0;

    // --- TILT ---
    if (mpuFound && baselineCalibrated) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float rawPitch, rawRoll;
      computeTiltAngles(a.acceleration.x, a.acceleration.y, a.acceleration.z,
                        rawPitch, rawRoll);

      float relativePitch = rawPitch - baselinePitch;
      float relativeRoll  = rawRoll  - baselineRoll;

      #if USE_KALMAN_FILTER
        filteredPitch = updateKalman(kalmanPitch, relativePitch);
        filteredRoll  = updateKalman(kalmanRoll,  relativeRoll);
      #else
        filteredPitch = updateComplementaryFilter(compFilterPitch, relativePitch);
        filteredRoll  = updateComplementaryFilter(compFilterRoll,  relativeRoll);
      #endif

      float totalTilt = sqrt(filteredPitch * filteredPitch + filteredRoll * filteredRoll);

      float deltaTilt = firstTiltRead ? 0.0f : fabs(totalTilt - previousTilt);
      previousTilt    = totalTilt;
      firstTiltRead   = false;

      soilData.tilt      = totalTilt;
      soilData.pitch     = filteredPitch;
      soilData.roll      = filteredRoll;
      soilData.deltaTilt = deltaTilt;

      float pitchChange = abs(filteredPitch - lastReportedPitch);
      float rollChange  = abs(filteredRoll  - lastReportedRoll);
      float tiltChange  = abs(totalTilt     - lastReportedTilt);

      if (pitchChange > TILT_CHANGE_THRESHOLD ||
          rollChange  > TILT_CHANGE_THRESHOLD ||
          tiltChange  > TILT_CHANGE_THRESHOLD) {
        Serial.printf("[TILT] Pitch=%+.2f° Roll=%+.2f° Total=%.2f° Δ=%.3f°\n",
                      filteredPitch, filteredRoll, totalTilt, deltaTilt);
        lastReportedPitch = filteredPitch;
        lastReportedRoll  = filteredRoll;
        lastReportedTilt  = totalTilt;
      }
    }

    // --- MODBUS ---
    uint8_t result = node.readHoldingRegisters(0x0000, 4);
    if (result == node.ku8MBSuccess) {
      soilData.moisture = node.getResponseBuffer(0) / 10.0;
      soilData.ec       = node.getResponseBuffer(2);
      soilData.ph       = node.getResponseBuffer(3) / 10.0;

      Serial.printf("[DATA] M=%.1f%% | EC=%.0f | pH=%.1f | Tilt=%.2f° | Δ=%.3f° | Bat=%.1f%%\n",
                    soilData.moisture, soilData.ec, soilData.ph,
                    soilData.tilt, soilData.deltaTilt, smoothedBatPct);
    } else {
      Serial.println("[MODBUS] Read failed");
    }

    // --- SEND ---
    if (espNowReady) sendDataESPNow(filteredPitch, filteredRoll);
  }
}