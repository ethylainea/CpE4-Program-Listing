// =============================================================================
// VETIVERTRACK - RESPONDER (RECEIVER)
// Receives ESP-NOW data, hosts AP + web dashboard
// MAC ADDRESS: 88:57:21:AD:11:24
// RECEIVES FROM: 88:57:21:AD:36:74 (Initiator)
// =============================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <esp_now.h>
#include <HardwareSerial.h>
// =============================================================================
// DATE/TIME CONFIGURATION
// =============================================================================
const unsigned long BOOT_EPOCH = 1772719265UL;
String epochToDateTimeString(unsigned long epoch) {
  unsigned long t   = epoch + 28800UL;
  unsigned long sec = t % 60; t /= 60;
  unsigned long min = t % 60; t /= 60;
  unsigned long hr  = t % 24; t /= 24;
  unsigned long days = t;
  unsigned long year = 1970;
  while (true) {
    bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    unsigned long daysInYear = leap ? 366 : 365;
    if (days < daysInYear) break;
    days -= daysInYear;
    year++;
  }
  bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  const uint8_t monthDays[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  unsigned long month = 1;
  for (month = 1; month <= 12; month++) {
    uint8_t dim = monthDays[month - 1];
    if (leap && month == 2) dim = 29;
    if (days < dim) break;
    days -= dim;
  }
  unsigned long day = days + 1;
  char buf[20];
  snprintf(buf, sizeof(buf), "%04lu-%02lu-%02lu %02lu:%02lu:%02lu",
           year, month, day, hr, min, sec);
  return String(buf);
}
String getDateTime() {
  unsigned long elapsed = millis() / 1000UL;
  return epochToDateTimeString(BOOT_EPOCH + elapsed);
}
// =============================================================================
// ACCESS POINT CONFIGURATION
// =============================================================================
const char* AP_SSID     = "VetiverTrack-F01";
const char* AP_PASSWORD = "vetivert";
const IPAddress AP_IP     (192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET (255, 255, 255, 0);
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;
// =============================================================================
// ESP-NOW DATA STRUCTURE (must match initiator)
// =============================================================================
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
struct_message incomingData;
struct_command outgoingCmd;
uint8_t initiatorMAC[] = {0x88, 0x57, 0x21, 0xAD, 0x36, 0x74};
// =============================================================================
// STATS
// =============================================================================
unsigned long packetsReceived = 0;
unsigned long lastPacketTime  = 0;
bool firstPacket  = true;
bool modbusOk     = false;
bool dataReceived = false;
// =============================================================================
// PAUSE FLAG — when true, new packets are NOT logged or forwarded to Pico W
// =============================================================================
bool recordingPaused = false;
// =============================================================================
// BATTERY MONITOR — RESPONDER (GPIO34, voltage divider 10k + 4.7k)
// 2-cell LiPo: full=8.4V, cutoff=6.0V
// Divider ratio = 4.7k/(10k+4.7k) = 0.3197 -> GPIO34 max = 2.685V (safe)
// =============================================================================
#define BAT_PIN        34
#define BAT_R1         10000.0f
#define BAT_R2          4700.0f
#define BAT_VREF           3.3f
#define BAT_ADC_MAX     4095.0f
#define BAT_V_FULL         8.4f
#define BAT_V_CUTOFF       6.0f
#define BAT_SAMPLES          8
float responderBatPct = 0.0;
float readResponderBattery() {
  analogSetPinAttenuation(BAT_PIN, (adc_attenuation_t)3);
  long sum = 0;
  for (int i = 0; i < BAT_SAMPLES; i++) { sum += analogRead(BAT_PIN); delay(2); }
  float adcVal = sum / (float)BAT_SAMPLES;
  float vPin   = (adcVal / BAT_ADC_MAX) * BAT_VREF;
  float vBat   = vPin * (BAT_R1 + BAT_R2) / BAT_R2;
  float pct    = (vBat - BAT_V_CUTOFF) / (BAT_V_FULL - BAT_V_CUTOFF) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}
// =============================================================================
// INITIATOR (PICO W) BATTERY FORMULA — For reference in Pico W main.py:
// 4-cell LiPo: full=16.8V, cutoff=12.0V
// Divider ratio = 1k/(10k+1k) = 0.0909 -> GPIO34 max = 1.527V
// Formula: pct = (vBat - 12.0) / (16.8 - 12.0) * 100, clamped [0,100]
// vBat = vPin * (10000 + 1000) / 1000 = vPin * 11.0
// =============================================================================
// =============================================================================
// UART2 TO PICO W
// =============================================================================
HardwareSerial PicoSerial(2);
String mlSoilResult  = "waiting";
String mlSlopeResult = "waiting";
int    mlSlips       = -1;
float  mlDeltaS      = -1.0;
#define PRED_LOCK_COUNT 100
int    predReadingCount   = 0;
bool   predLocked         = false;
String lockedSoil         = "";
String lockedSlope        = "";
int    lockedSlips        = -1;
float  lockedDeltaS       = -1.0;
float  lockedAvgMoisture  = 0.0;
float  lockedAvgEC        = 0.0;
float  lockedAvgPH        = 0.0;
float  lockedAvgTilt      = 0.0;
float  predSumMoisture    = 0.0;
float  predSumEC          = 0.0;
float  predSumPH          = 0.0;
float  predSumTilt        = 0.0;
float  previousTilt     = -1;
float  currentDeltaTilt = 0.0;
void updateDeltaTilt(float tilt) {
  if (previousTilt < 0) {
    currentDeltaTilt = 0.0;  // First reading, no previous
  } else {
    currentDeltaTilt = fabs(tilt - previousTilt);
  }
  previousTilt = tilt;
}
void sendToPico(float moisture, float ec, float ph, float tilt);
// =============================================================================
// ECO-ENGINEERING RECOMMENDATION TYPES (defined early for forward-decl safety)
// =============================================================================
#define TR_ROOT_TENSILE_MPa   75.0f
#define TR_ROOT_TENSILE_Pa    75000000.0f
#define A_ROOT                0.005f
#define RAR_COEFF             1.2f
#define DS_TERRACING          37500.0f
#define DS_RHA                18500.0f
#define DS_COCO_COIR          15000.0f
struct Recommendation {
  const char* label;
  float dsAdded;
  float dsNet;
  float rar;
  float density;
};
// =============================================================================
// NODE SAMPLING (5 nodes × 10 readings each)
// =============================================================================
#define NUM_NODES      5
#define READS_PER_NODE 100
struct NodeReading {
  float moisture;
  float ec;
  float ph;
  float tilt;
  float deltaTilt;
};
struct NodeData {
  float sumMoisture;
  float sumEC;
  float sumPH;
  float sumTilt;
  int   count;
  bool  active;
  bool  done;
  NodeReading readings[READS_PER_NODE];
};
NodeData nodes[NUM_NODES];
bool nodeSamplingMode = false;
void initNodes() {
  for (int i = 0; i < NUM_NODES; i++) {
    nodes[i] = {0, 0, 0, 0, 0, false, false, {}};
  }
  nodeSamplingMode = false;
}
void accumulateNodes() {
  for (int i = 0; i < NUM_NODES; i++) {
    if (nodes[i].active && !nodes[i].done) {
      int idx = nodes[i].count;
      if (idx < READS_PER_NODE) {
        nodes[i].readings[idx].moisture  = incomingData.moisture;
        nodes[i].readings[idx].ec        = incomingData.ec;
        nodes[i].readings[idx].ph        = incomingData.ph;
        nodes[i].readings[idx].tilt      = incomingData.tilt;
        nodes[i].readings[idx].deltaTilt = currentDeltaTilt;
      }
      nodes[i].sumMoisture += incomingData.moisture;
      nodes[i].sumEC       += incomingData.ec;
      nodes[i].sumPH       += incomingData.ph;
      nodes[i].sumTilt     += incomingData.tilt;
      nodes[i].count++;
      if (nodes[i].count >= READS_PER_NODE) {
        nodes[i].active = false;
        nodes[i].done   = true;
        Serial.printf("[NODE %d] Completed %d readings\n", i + 1, READS_PER_NODE);
      }
    }
  }
}
// =============================================================================
// LOG BUFFER (ring buffer, 500 readings)
// =============================================================================
#define MAX_LOG 500
struct LogEntry {
  float moisture;
  float ec;
  float ph;
  float pitch;
  float roll;
  float tilt;
  float deltaTilt;
  float mlDeltaS;
  unsigned long timestamp;
  char datetime[20];
  char mlSoil[20];
  char mlSlope[24];
  int mlSlips;
};
LogEntry logBuffer[MAX_LOG];
int  logCount = 0;
bool logFull  = false;
int  pendingLogIdx = -1;  // index of log entry waiting for ML prediction
void appendLog() {
  int idx = logCount < MAX_LOG ? logCount : MAX_LOG - 1;
  if (logCount >= MAX_LOG) {
    memmove(&logBuffer[0], &logBuffer[1], sizeof(LogEntry) * (MAX_LOG - 1));
    logFull = true;
    // Adjust pending index since we shifted
    if (pendingLogIdx > 0) pendingLogIdx--;
    else pendingLogIdx = -1;
  }
  logBuffer[idx].moisture  = incomingData.moisture;
  logBuffer[idx].ec        = incomingData.ec;
  logBuffer[idx].ph        = incomingData.ph;
  logBuffer[idx].pitch     = incomingData.pitch;
  logBuffer[idx].roll      = incomingData.roll;
  logBuffer[idx].tilt      = incomingData.tilt;
  logBuffer[idx].deltaTilt = currentDeltaTilt;
  logBuffer[idx].mlDeltaS  = -1.0;
  logBuffer[idx].timestamp = incomingData.timestamp;
  // Store "pending" until Pico responds with the prediction for THIS reading
  strncpy(logBuffer[idx].mlSoil,  "pending",  sizeof(logBuffer[idx].mlSoil)  - 1);
  strncpy(logBuffer[idx].mlSlope, "pending", sizeof(logBuffer[idx].mlSlope) - 1);
  logBuffer[idx].mlSoil[sizeof(logBuffer[idx].mlSoil)   - 1] = '\0';
  logBuffer[idx].mlSlope[sizeof(logBuffer[idx].mlSlope) - 1] = '\0';
  String dt = getDateTime();
  strncpy(logBuffer[idx].datetime, dt.c_str(), sizeof(logBuffer[idx].datetime) - 1);
  logBuffer[idx].datetime[sizeof(logBuffer[idx].datetime) - 1] = '\0';
  // Only track this entry if the previous one already got its prediction back.
  // If pendingLogIdx is still set, the Pico reply hasn't arrived yet — leave
  // it so readFromPico() can still fill it in.
  if (pendingLogIdx < 0) pendingLogIdx = idx;
  if (logCount < MAX_LOG) logCount++;
}
// =============================================================================
// ESP-NOW CALLBACKS
// =============================================================================
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataPtr, int len) {
  if (len != sizeof(struct_message)) {
    Serial.printf("[WARN] Unexpected packet size — got %d, expected %d\n", len, sizeof(struct_message));
    return;
  }
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  updateDeltaTilt(incomingData.tilt);
  packetsReceived++;
  unsigned long currentTime = millis();
  unsigned long timeSinceLastPacket = firstPacket ? 0 : (currentTime - lastPacketTime);
  firstPacket    = false;
  lastPacketTime = currentTime;
  dataReceived   = true;
  modbusOk       = (incomingData.moisture > 0 || incomingData.ec > 0);
  // Always accumulate nodes regardless of pause
  accumulateNodes();
  // Only log + forward to Pico W when NOT paused
  if (!recordingPaused) {
    sendToPico(incomingData.moisture, incomingData.ec, incomingData.ph, incomingData.tilt);
    appendLog();
    if (mlSoilResult != "waiting" && mlSlopeResult != "waiting") {
      predSumMoisture += incomingData.moisture;
      predSumEC       += incomingData.ec;
      predSumPH       += incomingData.ph;
      predSumTilt     += incomingData.tilt;
      predReadingCount++;
      if (predReadingCount >= PRED_LOCK_COUNT) {
        predLocked        = true;
        lockedSoil        = mlSoilResult;
        lockedSlope       = mlSlopeResult;
        lockedSlips       = mlSlips;
        lockedDeltaS      = mlDeltaS;
        lockedAvgMoisture = predSumMoisture / predReadingCount;
        lockedAvgEC       = predSumEC       / predReadingCount;
        lockedAvgPH       = predSumPH       / predReadingCount;
        lockedAvgTilt     = predSumTilt     / predReadingCount;
        Serial.printf("[PRED] Updated after %d readings\n", predReadingCount);
        // Reset for next window
        predReadingCount = 0;
        predSumMoisture  = 0.0;
        predSumEC        = 0.0;
        predSumPH        = 0.0;
        predSumTilt      = 0.0;
      }
    }
  } else {
    Serial.println("[PAUSED] Packet received but not logged");
  }
  Serial.printf("\n[PKT #%lu] From: ", packetsReceived);
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", recv_info->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  if (timeSinceLastPacket > 0) Serial.printf(" | +%lu ms", timeSinceLastPacket);
  Serial.println();
  Serial.printf("  Moisture: %.1f%%  EC: %.0f uS/cm  pH: %.1f  Paused: %s\n",
                incomingData.moisture, incomingData.ec, incomingData.ph,
                recordingPaused ? "YES" : "no");
  Serial.printf("  Pitch: %+.2f°  Roll: %+.2f°  Total Tilt: %.2f°  ΔTilt: %.3f°\n",
                incomingData.pitch, incomingData.roll, incomingData.tilt, currentDeltaTilt);
}
void OnCmdSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Calibrate cmd: Sent OK" : "Calibrate cmd: Send FAILED");
}
// =============================================================================
// PICO W UART
// =============================================================================
void sendToPico(float moisture, float ec, float ph, float tilt) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%.1f,%.0f,%.1f,%.2f\n", moisture, ec, ph, tilt);
  PicoSerial.print(buf);
  Serial.printf("[PICO TX] %s", buf);
}
void readFromPico() {
  static String picoLine = "";
  while (PicoSerial.available()) {
    char c = PicoSerial.read();
    if (c == '\n') {
      picoLine.trim();
      if (picoLine.length() > 0) {
        int comma1 = picoLine.indexOf(',');
        if (comma1 > 0) {
          int comma2 = picoLine.indexOf(',', comma1 + 1);
          mlSoilResult = picoLine.substring(0, comma1);
          mlSoilResult.trim();
          if (comma2 > 0) {
            int comma3 = picoLine.indexOf(',', comma2 + 1);
            mlSlopeResult = picoLine.substring(comma1 + 1, comma2);
            mlSlopeResult.trim();
            if (comma3 > 0) {
              mlSlips  = picoLine.substring(comma2 + 1, comma3).toInt();
              mlDeltaS = picoLine.substring(comma3 + 1).toFloat();
            } else {
              mlSlips  = picoLine.substring(comma2 + 1).toInt();
              mlDeltaS = -1.0;
            }
          } else {
            mlSlopeResult = picoLine.substring(comma1 + 1);
            mlSlopeResult.trim();
            mlSlips  = -1;
            mlDeltaS = -1.0;
          }
          Serial.printf("[PICO RX] soil=%s, slope=%s, slips=%d, deltaS=%.4f Pa\n",
                        mlSoilResult.c_str(), mlSlopeResult.c_str(), mlSlips, mlDeltaS);
          // Fix late prediction: update the pending log entry with the prediction
          // that was computed for THAT specific reading
          if (pendingLogIdx >= 0 && pendingLogIdx < logCount) {
            strncpy(logBuffer[pendingLogIdx].mlSoil,  mlSoilResult.c_str(),  sizeof(logBuffer[pendingLogIdx].mlSoil)  - 1);
            strncpy(logBuffer[pendingLogIdx].mlSlope, mlSlopeResult.c_str(), sizeof(logBuffer[pendingLogIdx].mlSlope) - 1);
            logBuffer[pendingLogIdx].mlSoil[sizeof(logBuffer[pendingLogIdx].mlSoil)   - 1] = '\0';
            logBuffer[pendingLogIdx].mlSlope[sizeof(logBuffer[pendingLogIdx].mlSlope) - 1] = '\0';
            logBuffer[pendingLogIdx].mlSlips  = mlSlips;
            logBuffer[pendingLogIdx].mlDeltaS = mlDeltaS;
            pendingLogIdx = -1;  // Prediction synced
          }
        }
      }
      picoLine = "";
    } else {
      picoLine += c;
    }
  }
}
// =============================================================================
// ECO-ENGINEERING RECOMMENDATIONS (Wu-Waldron Model)
// =============================================================================
// (constants and Recommendation struct defined earlier for forward-decl safety)

// Compute RAR and planting density from a net ΔS requirement
// If dsNet <= 0, no vetiver planting needed
void computeRarDensity(float dsNet, float &rar, float &density) {
  if (dsNet <= 0.0f) {
    rar     = 0.0f;
    density = 0.0f;
    return;
  }
  // RAR = ΔS_target / (1.2 * Tr)   [unitless ratio]
  rar     = dsNet / (RAR_COEFF * TR_ROOT_TENSILE_Pa);
  // D = RAR * 1000 / A_root   [slips/m²]
  density = (rar * 1000.0f) / A_ROOT;
}

// Global recommendation result array — avoids Arduino forward-decl issue
// with structs in function signatures
static Recommendation g_recs[7];

// Fill g_recs[8] with all 8 intervention combinations
void computeRecommendations(float deltaS) {
  const char* labels[7] = {
    "Vetiver Only",
    "Terracing + Vetiver",
    "RHA + Vetiver",
    "Coco-Coir + Vetiver",
    "Terracing + RHA + Vetiver",
    "Terracing + Coco + Vetiver",
    "RHA + Coco + Vetiver"
  };
  float dsVals[7] = {
    0.0f,
    DS_TERRACING,
    DS_RHA,
    DS_COCO_COIR,
    DS_TERRACING + DS_RHA,
    DS_TERRACING + DS_COCO_COIR,
    DS_RHA + DS_COCO_COIR
  };
  for (int i = 0; i < 7; i++) {
    g_recs[i].label   = labels[i];
    g_recs[i].dsAdded = dsVals[i];
    g_recs[i].dsNet   = deltaS - dsVals[i];
    computeRarDensity(g_recs[i].dsNet, g_recs[i].rar, g_recs[i].density);
  }
}

// =============================================================================
// HTTP HANDLERS
// =============================================================================
void handleApiData() {
  String json = "{";
  json += "\"datetime\":\""  + getDateTime()                           + "\",";
  json += "\"received\":"    + String(dataReceived ? "true" : "false") + ",";
  json += "\"paused\":"      + String(recordingPaused ? "true" : "false") + ",";
  json += "\"moisture\":"    + String(incomingData.moisture, 1)        + ",";
  json += "\"ec\":"          + String(incomingData.ec, 0)              + ",";
  json += "\"ph\":"          + String(incomingData.ph, 1)              + ",";
  json += "\"tilt\":"        + String(incomingData.tilt, 2)            + ",";
  json += "\"deltaTilt\":"   + String(currentDeltaTilt, 2)             + ",";
  json += "\"pitch\":"       + String(incomingData.pitch, 2)           + ",";
  json += "\"roll\":"        + String(incomingData.roll, 2)            + ",";
  json += "\"timestamp\":"   + String(incomingData.timestamp)          + ",";
  json += "\"uptime\":"      + String(millis())                        + ",";
  json += "\"packets\":"     + String(packetsReceived)                 + ",";
  json += "\"modbus\":"      + String(modbusOk ? "true" : "false")    + ",";
  json += "\"logCount\":"    + String(logCount)                        + ",";
  json += "\"logFull\":"     + String(logFull ? "true" : "false")      + ",";
  json += "\"predLocked\":"  + String(predLocked ? "true" : "false")  + ",";
  json += "\"predCount\":"   + String(predReadingCount)                + ",";
  json += "\"predTarget\":"  + String(PRED_LOCK_COUNT)                 + ",";
  json += "\"mlSoil\":\""    + mlSoilResult                           + "\",";
  json += "\"mlSlope\":\""   + mlSlopeResult                          + "\",";
  json += "\"mlSlips\":"     + String(mlSlips)                        + ",";
  json += "\"mlDeltaS\":"   + String(mlDeltaS, 4)                    + ",";
  json += "\"initBatPct\":" + String(incomingData.batteryPct, 1)     + ",";
  json += "\"respBatPct\":"  + String(responderBatPct, 1)             + "";
  json += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}
// POST /api/pause — toggle pause/resume
void handleApiPause() {
  recordingPaused = !recordingPaused;
  Serial.printf("[PAUSE] Recording %s\n", recordingPaused ? "PAUSED" : "RESUMED");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json",
    String("{\"paused\":") + (recordingPaused ? "true" : "false") + "}");
}
void handleHistory() {
  int count = logFull ? MAX_LOG : logCount;
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("[");
  char row[240];
  for (int i = 0; i < count; i++) {
    snprintf(row, sizeof(row),
      "%s{\"t\":\"%s\",\"m\":%.1f,\"e\":%.0f,\"p\":%.1f,\"pi\":%.2f,\"r\":%.2f,\"ti\":%.2f,\"dt\":%.2f,\"ds\":%.4f,\"ms\":\"%s\",\"sl\":\"%s\",\"mlSlips\":%d}",
      i > 0 ? "," : "",
      logBuffer[i].datetime,
      logBuffer[i].moisture,
      logBuffer[i].ec,
      logBuffer[i].ph,
      logBuffer[i].pitch,
      logBuffer[i].roll,
      logBuffer[i].tilt,
      logBuffer[i].deltaTilt,
      logBuffer[i].mlDeltaS,
      logBuffer[i].mlSoil,
      logBuffer[i].mlSlope,
      logBuffer[i].mlSlips);
    // Include mlSlips in JSON response
    server.sendContent(row);
  }
  server.sendContent("]");
  server.sendContent("");
}
void handleApiCalibrate() {
  outgoingCmd.cmd = 1;
  esp_err_t result = esp_now_send(initiatorMAC, (uint8_t *)&outgoingCmd, sizeof(outgoingCmd));
  if (result == ESP_OK) {
    Serial.println("Calibrate command sent to initiator");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(500, "application/json", "{\"ok\":false}");
  }
}
void handleApiPrediction() {
  String json = "{";
  json += "\"locked\":"  + String(predLocked ? "true" : "false") + ",";
  json += "\"count\":"   + String(predReadingCount)               + ",";
  json += "\"target\":"  + String(PRED_LOCK_COUNT)                + ",";
  if (predLocked) {
    json += "\"soil\":\""     + lockedSoil                    + "\",";
    json += "\"slope\":\""    + lockedSlope                   + "\",";
    json += "\"slips\":"      + String(lockedSlips)           + ",";
    json += "\"deltaS\":"     + String(lockedDeltaS, 4)       + ",";
    json += "\"moisture\":"   + String(lockedAvgMoisture, 1)  + ",";
    json += "\"ec\":"         + String(lockedAvgEC, 1)        + ",";
    json += "\"ph\":"         + String(lockedAvgPH, 2)        + ",";
    json += "\"tilt\":"       + String(lockedAvgTilt, 2)      + "";
  } else {
    json += "\"soil\":\"waiting\",\"slope\":\"waiting\",\"slips\":-1";
    json += ",\"deltaS\":-1";
    json += ",\"moisture\":0,\"ec\":0,\"ph\":0,\"tilt\":0";
  }
  json += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}
void handleApiRecommendations() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!predLocked || lockedDeltaS < 0) {
    server.send(200, "application/json", "{\"ready\":false}");
    return;
  }
  computeRecommendations(lockedDeltaS);
  String json = "{\"ready\":true,\"deltaS\":" + String(lockedDeltaS, 4) + ",\"combos\":[";
  for (int i = 0; i < 7; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"label\":\"" + String(g_recs[i].label) + "\",";
    json += "\"dsAdded\":"  + String(g_recs[i].dsAdded, 1) + ",";
    json += "\"dsNet\":"    + String(g_recs[i].dsNet,   4) + ",";
    json += "\"rar\":"      + String(g_recs[i].rar,     6) + ",";
    json += "\"density\":"  + String(g_recs[i].density, 2);
    json += "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}
void handleApiReset() {
  logCount         = 0;
  logFull          = false;
  predReadingCount = 0;
  predLocked       = false;
  lockedSoil       = "";
  lockedSlope      = "";
  lockedSlips      = -1;
  lockedDeltaS     = -1.0;
  predSumMoisture  = 0.0;
  predSumEC        = 0.0;
  predSumPH        = 0.0;
  predSumTilt      = 0.0;
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
  Serial.println("[RESET] Log and prediction state cleared");
}
// GET /download — CSV only (with delta tilt column restored)
void handleDownload() {
  int count = logFull ? MAX_LOG : logCount;
  server.sendHeader("Content-Disposition", "attachment; filename=\"vetivertrack_log.csv\"");
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  // Delta tilt column restored
  server.sendContent("Datetime,Moisture_%,EC_uS_cm,pH,Pitch_deg,Roll_deg,Tilt_deg,DeltaTilt_deg,DeltaS_Pa,Soil_Suitability,Slope_Risk\n");
  char row[180];
  for (int i = 0; i < count; i++) {
    snprintf(row, sizeof(row), "%s,%.1f,%.0f,%.1f,%.2f,%.2f,%.2f,%.2f,%.4f,%s,%s\n",
             logBuffer[i].datetime,
             logBuffer[i].moisture,
             logBuffer[i].ec,
             logBuffer[i].ph,
             logBuffer[i].pitch,
             logBuffer[i].roll,
             logBuffer[i].tilt,
             logBuffer[i].deltaTilt,
             logBuffer[i].mlDeltaS,
             logBuffer[i].mlSoil,
             logBuffer[i].mlSlope);
    server.sendContent(row);
  }
  server.sendContent("");
}
// GET /download/prediction — prediction results CSV
void handleDownloadPrediction() {
  server.sendHeader("Content-Disposition", "attachment; filename=\"vetivertrack_prediction.csv\"");
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("VetiverTrack Stable Prediction Report\n");
  server.sendContent("Based on," + String(PRED_LOCK_COUNT) + " readings\n\n");
  server.sendContent("Field,Value\n");
  if (predLocked) {
    server.sendContent("Soil Suitability," + lockedSoil + "\n");
    server.sendContent("Slope Risk,"       + lockedSlope + "\n");
    char buf[80];
    snprintf(buf, sizeof(buf), "Vetiver Slips,%d\n",          lockedSlips);       server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Required Shear Strength,%.4f Pa\n", lockedDeltaS); server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Avg Moisture,%.1f%%\n",       lockedAvgMoisture); server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Avg EC,%.1f uS/cm\n",         lockedAvgEC);       server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Avg pH,%.2f\n",                lockedAvgPH);       server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Avg Delta Tilt,%.2f deg\n",   lockedAvgTilt);     server.sendContent(buf);
  } else {
    server.sendContent("Not enough readings yet," + String(predReadingCount) + "/" + String(PRED_LOCK_COUNT) + "\n");
  }
  server.sendContent("");
}
// GET /download/both — CSV + prediction combined in one file
void handleDownloadBoth() {
  int count = logFull ? MAX_LOG : logCount;
  server.sendHeader("Content-Disposition", "attachment; filename=\"vetivertrack_full.csv\"");
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  // Section 1: Prediction
  server.sendContent("=== PREDICTION RESULTS ===\n");
  if (predLocked) {
    server.sendContent("Soil Suitability," + lockedSoil  + "\n");
    server.sendContent("Slope Risk,"       + lockedSlope + "\n");
    char buf[80];
    snprintf(buf, sizeof(buf), "Vetiver Slips,%d\n",         lockedSlips);       server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Required Shear Strength,%.4f Pa\n", lockedDeltaS); server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Avg Moisture,%.1f%%\n",      lockedAvgMoisture); server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Avg EC,%.1f uS/cm\n",        lockedAvgEC);       server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Avg pH,%.2f\n",               lockedAvgPH);       server.sendContent(buf);
    snprintf(buf, sizeof(buf), "Avg Delta Tilt,%.2f deg\n",  lockedAvgTilt);     server.sendContent(buf);
  } else {
    server.sendContent("Status,Not enough readings (" + String(predReadingCount) + "/" + String(PRED_LOCK_COUNT) + ")\n");
  }
  // Section 2: Raw log
  server.sendContent("\n=== RAW DATA LOG ===\n");
  server.sendContent("Datetime,Moisture_%,EC_uS_cm,pH,Pitch_deg,Roll_deg,Tilt_deg,DeltaTilt_deg,DeltaS_Pa,Soil_Suitability,Slope_Risk\n");
  char row[180];
  for (int i = 0; i < count; i++) {
    snprintf(row, sizeof(row), "%s,%.1f,%.0f,%.1f,%.2f,%.2f,%.2f,%.2f,%.4f,%s,%s\n",
             logBuffer[i].datetime,
             logBuffer[i].moisture,
             logBuffer[i].ec,
             logBuffer[i].ph,
             logBuffer[i].pitch,
             logBuffer[i].roll,
             logBuffer[i].tilt,
             logBuffer[i].deltaTilt,
             logBuffer[i].mlDeltaS,
             logBuffer[i].mlSoil,
             logBuffer[i].mlSlope);
    server.sendContent(row);
  }
  server.sendContent("");
}
// GET /api/nodes
void handleApiNodes() {
  String json = "{\"nodes\":[";
  for (int i = 0; i < NUM_NODES; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"id\":"     + String(i + 1)                              + ",";
    json += "\"active\":" + String(nodes[i].active ? "true" : "false") + ",";
    json += "\"done\":"   + String(nodes[i].done   ? "true" : "false") + ",";
    json += "\"count\":"  + String(nodes[i].count)                     + ",";
    if (nodes[i].count > 0) {
      json += "\"moisture\":" + String(nodes[i].sumMoisture / nodes[i].count, 1) + ",";
      json += "\"ec\":"       + String(nodes[i].sumEC       / nodes[i].count, 0) + ",";
      json += "\"ph\":"       + String(nodes[i].sumPH       / nodes[i].count, 1) + ",";
      json += "\"tilt\":"     + String(nodes[i].sumTilt     / nodes[i].count, 2);
    } else {
      json += "\"moisture\":null,\"ec\":null,\"ph\":null,\"tilt\":null";
    }
    json += "}";
  }
  json += "]";
  // Only compute overall avg when ALL 5 nodes are done
  int doneCount = 0;
  float totM = 0, totE = 0, totP = 0, totT = 0;
  for (int i = 0; i < NUM_NODES; i++) {
    if (nodes[i].done && nodes[i].count > 0) {
      totM += nodes[i].sumMoisture / nodes[i].count;
      totE += nodes[i].sumEC       / nodes[i].count;
      totP += nodes[i].sumPH       / nodes[i].count;
      totT += nodes[i].sumTilt     / nodes[i].count;
      doneCount++;
    }
  }
  if (doneCount == NUM_NODES) {
    json += ",\"avg\":{";
    json += "\"moisture\":" + String(totM / doneCount, 1) + ",";
    json += "\"ec\":"       + String(totE / doneCount, 0) + ",";
    json += "\"ph\":"       + String(totP / doneCount, 1) + ",";
    json += "\"tilt\":"     + String(totT / doneCount, 2) + ",";
    json += "\"nodesDone\":" + String(doneCount);
    json += "}";
  } else {
    json += ",\"avg\":null";
    json += ",\"doneCount\":" + String(doneCount);
  }
  json += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}
// POST /api/node/start?id=1
void handleNodeStart() {
  if (!server.hasArg("id")) { server.send(400, "application/json", "{\"ok\":false,\"msg\":\"missing id\"}"); return; }
  int id = server.arg("id").toInt() - 1;
  if (id < 0 || id >= NUM_NODES) { server.send(400, "application/json", "{\"ok\":false,\"msg\":\"invalid id\"}"); return; }
  // Check if any other node is currently active
  for (int i = 0; i < NUM_NODES; i++) {
    if (i != id && nodes[i].active) {
      Serial.printf("[NODE %d] Blocked — Node %d is still recording\n", id + 1, i + 1);
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(409, "application/json",
        String("{\"ok\":false,\"msg\":\"Node ") + String(i + 1) + " is still recording\"}");
      return;
    }
  }
  nodes[id] = {0, 0, 0, 0, 0, true, false, {}};
  nodeSamplingMode = true;
  Serial.printf("[NODE %d] Started sampling\n", id + 1);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
}
// POST /api/node/reset
void handleNodeReset() {
  initNodes();
  Serial.println("[NODES] All nodes reset");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
}
// GET /download/node?id=1 — download single node data as CSV
void handleDownloadNode() {
  if (!server.hasArg("id")) { server.send(400, "text/plain", "missing id"); return; }
  int id = server.arg("id").toInt() - 1;
  if (id < 0 || id >= NUM_NODES) { server.send(400, "text/plain", "invalid id"); return; }
  if (!nodes[id].done && nodes[id].count == 0) { server.send(400, "text/plain", "node has no data"); return; }
  // Compute prediction for this node
  float avgM = nodes[id].count > 0 ? nodes[id].sumMoisture / nodes[id].count : 0;
  float avgE = nodes[id].count > 0 ? nodes[id].sumEC       / nodes[id].count : 0;
  float avgP = nodes[id].count > 0 ? nodes[id].sumPH       / nodes[id].count : 0;
  float avgT = nodes[id].count > 0 ? nodes[id].sumTilt     / nodes[id].count : 0;
  // Soil suitability
  String soil = (avgE <= 10000 && avgP >= 6.0 && avgP <= 8.0) ? "suitable" : "not_suitable";
  // Slope risk using live delta tilt
  String slope;
  float dt = currentDeltaTilt;
  if (dt < 0.5 || avgM < 50)                          slope = "stable";
  else if (dt <= 2.0 && avgM >= 50 && avgM < 80)      slope = "pre_failure";
  else if (dt > 2.0 && avgM >= 80)                    slope = "failure_imminent";
  else if (avgM >= 80)                                 slope = "failure_imminent";
  else                                                  slope = "pre_failure";
  // Slip count
  int slips = 0;
  String slipsText;
  if (soil != "suitable") {
    slipsText = "N/A (soil not suitable)";
  } else if (slope == "stable") {
    slips = 0;
    slipsText = "0";
  } else {
    float density = (avgM - 50) * 40.0 / 27.0;
    slips = slope == "pre_failure"
      ? (int)round(fabs(density) / 0.85)
      : (int)round(fabs(density) * 3.0 / 0.85);
    slipsText = String(slips);
  }
  char fname[40];
  snprintf(fname, sizeof(fname), "attachment; filename=\"node_%d.csv\"", id + 1);
  server.sendHeader("Content-Disposition", fname);
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  // Header block — matches image format
  char buf[80];
  snprintf(buf, sizeof(buf), "VetiverTrack Node %d Prediction Report\n", id + 1);
  server.sendContent(buf);
  snprintf(buf, sizeof(buf), "Based on,%d readings\n", nodes[id].count);
  server.sendContent(buf);
  server.sendContent("\n");
  // Prediction results
  server.sendContent("Soil Suitability," + soil + "\n");
  server.sendContent("Slope Risk,"       + slope + "\n");
  server.sendContent("Vetiver Slips,"    + slipsText + "\n");
  snprintf(buf, sizeof(buf), "Avg Moisture,%.2f%%\n",      avgM); server.sendContent(buf);
  snprintf(buf, sizeof(buf), "Avg EC,%.1f uS/cm\n",        avgE); server.sendContent(buf);
  snprintf(buf, sizeof(buf), "Avg pH,%.1f\n",               avgP); server.sendContent(buf);
  snprintf(buf, sizeof(buf), "Avg Delta Tilt,%.2f deg\n",  avgT); server.sendContent(buf);
  // Individual readings section
  server.sendContent("\nReading,pH,EC (uS/cm),Moisture (%),Delta Tilt (deg)\n");
  int readCount = nodes[id].count < READS_PER_NODE ? nodes[id].count : READS_PER_NODE;
  char row[100];
  for (int i = 0; i < readCount; i++) {
    snprintf(row, sizeof(row), "%d,%.1f,%.0f,%.1f,%.2f\n",
      i + 1,
      nodes[id].readings[i].ph,
      nodes[id].readings[i].ec,
      nodes[id].readings[i].moisture,
      nodes[id].readings[i].deltaTilt);
    server.sendContent(row);
  }
  server.sendContent("");
}
// GET /download/node/csv?id=1 — download ALL node readings as detailed CSV (without prediction header)
void handleDownloadNodeCSV() {
  if (!server.hasArg("id")) { server.send(400, "text/plain", "missing id"); return; }
  int id = server.arg("id").toInt() - 1;
  if (id < 0 || id >= NUM_NODES) { server.send(400, "text/plain", "invalid id"); return; }
  if (nodes[id].count == 0) { server.send(400, "text/plain", "node has no data"); return; }
  char fname[50];
  snprintf(fname, sizeof(fname), "attachment; filename=\"node_%d_all_readings.csv\"", id + 1);
  server.sendHeader("Content-Disposition", fname);
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  // Header
  char buf[80];
  snprintf(buf, sizeof(buf), "VetiverTrack Node %d — All %d Readings\n", id + 1, nodes[id].count);
  server.sendContent(buf);
  server.sendContent("Generated," + getDateTime() + "\n\n");
  // Column headers
  server.sendContent("Reading #,Datetime,Moisture (%),EC (uS/cm),pH,Tilt (deg),Delta Tilt (deg)\n");
  // All readings
  int readCount = nodes[id].count < READS_PER_NODE ? nodes[id].count : READS_PER_NODE;
  char row[150];
  unsigned long baseTime = BOOT_EPOCH;
  unsigned long elapsed = millis() / 1000UL;
  
  for (int i = 0; i < readCount; i++) {
    // Approximate timestamp for each reading (rough estimate)
    unsigned long readingTime = baseTime + elapsed - (readCount - i - 1) * 2; // Assume ~2 sec between readings
    String dt = epochToDateTimeString(readingTime);
    
    snprintf(row, sizeof(row), "%d,%s,%.1f,%.0f,%.1f,%.2f,%.2f\n",
      i + 1,
      dt.c_str(),
      nodes[id].readings[i].moisture,
      nodes[id].readings[i].ec,
      nodes[id].readings[i].ph,
      nodes[id].readings[i].tilt,
      nodes[id].readings[i].deltaTilt);
    server.sendContent(row);
  }
  server.sendContent("");
}
// =============================================================================
// DASHBOARD HTML
// =============================================================================
const char DASHBOARD[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>VetiverTrack</title>
<style>
:root{
  --bg:#0f1117;--card:#1a1d27;--border:#252836;--sidebar:#13161f;
  --text:#e8eaf0;--muted:#6b7080;--accent:#6366f1;
  --green:#22c55e;--red:#ef4444;
  --ph:#a78bfa;--ec:#38bdf8;--mo:#34d399;
  --pitch:#f97316;--roll:#facc15;--total:#ef4444;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;
  background:var(--bg);color:var(--text);display:flex;flex-direction:column;height:100vh;overflow:hidden}
.topbar{display:flex;align-items:center;justify-content:space-between;
  padding:0 20px;height:52px;border-bottom:1px solid var(--border);
  background:var(--sidebar);flex-shrink:0;z-index:10}
.topbar-left{display:flex;align-items:center;gap:10px}
.topbar h1{font-size:.95rem;font-weight:700;letter-spacing:.06em}
#dot{width:8px;height:8px;border-radius:50%;background:var(--green);animation:pulse 2s infinite;flex-shrink:0}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
#clock{font-size:.74rem;color:var(--muted)}
.layout{display:flex;flex:1;overflow:hidden}
.sidebar{width:200px;background:var(--sidebar);border-right:1px solid var(--border);
  display:flex;flex-direction:column;padding:16px 0;flex-shrink:0}
.nav-label{font-size:.58rem;text-transform:uppercase;letter-spacing:.12em;color:var(--muted);padding:0 16px 8px}
.nav-item{display:flex;align-items:center;gap:10px;padding:10px 16px;
  font-size:.82rem;color:var(--muted);cursor:pointer;border-left:3px solid transparent;transition:all .15s;user-select:none}
.nav-item:hover{color:var(--text);background:rgba(255,255,255,.03)}
.nav-item.active{color:var(--text);border-left-color:var(--accent);background:rgba(99,102,241,.08)}
.nav-icon{font-size:1rem;width:18px;text-align:center}
.main{flex:1;overflow-y:auto;padding:20px}
.tab{display:none}.tab.active{display:block}
.card{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:16px;margin-bottom:14px}
.clabel{font-size:.63rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:12px}
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-bottom:10px}
.metric{background:var(--bg);border-radius:8px;padding:11px;text-align:center}
.metric .v{font-size:1.5rem;font-weight:700;line-height:1}
.metric .l{font-size:.6rem;color:var(--muted);text-transform:uppercase;margin-top:5px}
.ph-c{color:var(--ph)}.ec-c{color:var(--ec)}.mo-c{color:var(--mo)}.ti-c{color:var(--total)}
.section-lbl{font-size:.62rem;text-transform:uppercase;letter-spacing:.09em;color:var(--muted);margin:12px 0 8px}
#no-data{color:var(--muted);text-align:center;padding:40px;font-size:.85rem}
.sys-row{display:flex;justify-content:space-between;padding:7px 0;
  border-bottom:1px solid rgba(255,255,255,.05);font-size:.82rem}
.sys-row:last-child{border-bottom:none}
.sys-key{color:var(--muted)}.sys-val{color:var(--text)}
/* Pause button */
.pause-bar{display:flex;align-items:center;justify-content:space-between;
  margin-bottom:12px;padding:10px 14px;border-radius:8px;background:var(--bg);border:1px solid var(--border)}
.pause-bar span{font-size:.75rem;color:var(--muted)}
#btn-pause{padding:8px 18px;border:none;border-radius:7px;font-size:.78rem;font-weight:700;cursor:pointer;
  background:#1e3a5f;color:#93c5fd;transition:background .15s}
#btn-pause.paused{background:#3b1a1a;color:#fca5a5}
/* Data log */
.tbl-controls{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;flex-wrap:wrap;gap:8px}
.tbl-controls span{font-size:.75rem;color:var(--muted)}
.btn-csv{padding:8px 16px;border-radius:8px;border:none;background:#166534;
  color:#bbf7d0;font-size:.78rem;font-weight:700;cursor:pointer;text-decoration:none;display:inline-block}
.tbl-wrap{overflow-x:auto;border-radius:8px;border:1px solid var(--border)}
table{width:100%;border-collapse:collapse;font-size:.78rem}
thead th{background:var(--bg);color:var(--muted);font-size:.65rem;text-transform:uppercase;
  letter-spacing:.08em;padding:10px 12px;text-align:left;border-bottom:1px solid var(--border);white-space:nowrap}
tbody tr{border-bottom:1px solid rgba(255,255,255,.04);transition:background .1s}
tbody tr:last-child{border-bottom:none}
tbody tr:hover{background:rgba(255,255,255,.03)}
tbody td{padding:9px 12px;color:var(--text);white-space:nowrap}
tbody td:first-child{color:var(--muted);font-size:.72rem}
/* Charts */
.chart-wrap{position:relative;width:100%;height:200px;margin-bottom:4px}
.chart-wrap canvas{width:100%;height:100%;border-radius:8px;background:var(--bg)}
.legend{display:flex;gap:14px;flex-wrap:wrap;margin-bottom:8px}
.legend span{font-size:.62rem;display:flex;align-items:center;gap:4px;color:var(--muted)}
.legend i{width:10px;height:3px;border-radius:2px;display:inline-block}
/* Node sampling */
.node-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:12px;margin-bottom:14px}
.node-card{background:var(--bg);border:1px solid var(--border);border-radius:10px;padding:14px;position:relative}
.node-card.active{border-color:var(--accent);box-shadow:0 0 0 1px var(--accent)}
.node-card.done{border-color:var(--green)}
.node-title{font-size:.75rem;font-weight:700;margin-bottom:4px}
.node-status{font-size:.65rem;color:var(--muted);margin-bottom:10px}
.node-progress{height:6px;border-radius:3px;background:var(--border);overflow:hidden;margin-bottom:10px}
.node-bar{height:100%;border-radius:3px;background:var(--accent);transition:width .4s}
.node-bar.done{background:var(--green)}
.node-readings{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-bottom:10px}
.node-reading{background:var(--card);border-radius:6px;padding:7px;text-align:center}
.node-reading .val{font-size:.95rem;font-weight:700}
.node-reading .lbl{font-size:.58rem;color:var(--muted);text-transform:uppercase}
.node-btns{display:flex;gap:6px;flex-wrap:wrap}
.btn-start{flex:1;padding:8px;border:none;border-radius:7px;background:var(--accent);color:#fff;font-size:.78rem;font-weight:700;cursor:pointer}
.btn-dl-node{flex:1;padding:8px;border:none;border-radius:7px;background:#166534;color:#bbf7d0;font-size:.78rem;font-weight:700;cursor:pointer}
.btn-reset-all{padding:8px 18px;border:1px solid var(--border);border-radius:7px;background:#3b1a1a;color:#fca5a5;font-size:.78rem;cursor:pointer}
.avg-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
.avg-card{background:var(--bg);border-radius:8px;padding:11px;text-align:center}
.avg-card .v{font-size:1.3rem;font-weight:700;line-height:1}
.avg-card .l{font-size:.58rem;color:var(--muted);text-transform:uppercase;margin-top:4px}
.status-bar{height:6px;border-radius:3px;margin-top:8px;background:var(--border);overflow:hidden}
.status-fill{height:100%;border-radius:3px;transition:width .5s}
.alert-box{border-radius:10px;padding:12px 14px;margin-bottom:10px;font-size:.82rem;border:1px solid}
.alert-ok{background:rgba(34,197,94,.08);border-color:rgba(34,197,94,.25);color:#86efac}
.alert-warn{background:rgba(251,191,36,.08);border-color:rgba(251,191,36,.25);color:#fde68a}
.alert-danger{background:rgba(239,68,68,.08);border-color:rgba(239,68,68,.25);color:#fca5a5}
.alert-icon{font-size:1rem;margin-right:6px}
/* Download modal */
#dl-modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.6);z-index:200;align-items:center;justify-content:center}
#dl-modal.open{display:flex}
.dl-box{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:24px;width:320px;max-width:90vw}
.dl-box h3{font-size:.9rem;font-weight:700;margin-bottom:16px}
.dl-option{display:block;width:100%;padding:11px 14px;margin-bottom:8px;border-radius:8px;border:1px solid var(--border);
  background:var(--bg);color:var(--text);font-size:.82rem;cursor:pointer;text-align:left;transition:background .15s}
.dl-option:hover{background:rgba(99,102,241,.12);border-color:var(--accent)}
.dl-cancel{display:block;width:100%;padding:9px;border:none;background:transparent;color:var(--muted);font-size:.78rem;cursor:pointer;margin-top:4px}
/* Recommendation table */
.rec-table-wrap{overflow-x:auto;border-radius:8px;border:1px solid var(--border);margin-top:10px}
.rec-table{width:100%;border-collapse:collapse;font-size:.78rem}
.rec-table thead th{background:var(--bg);color:var(--muted);font-size:.62rem;text-transform:uppercase;
  letter-spacing:.08em;padding:9px 12px;text-align:left;border-bottom:1px solid var(--border);white-space:nowrap}
.rec-table tbody tr{border-bottom:1px solid rgba(255,255,255,.04);transition:background .1s}
.rec-table tbody tr:last-child{border-bottom:none}
.rec-table tbody tr:hover{background:rgba(255,255,255,.03)}
.rec-table tbody tr.rec-best{background:rgba(34,197,94,.07);border-left:3px solid var(--green)}
.rec-table tbody td{padding:9px 12px;color:var(--text);white-space:nowrap}
.rec-best-badge{display:inline-block;font-size:.58rem;padding:2px 7px;border-radius:10px;
  background:rgba(34,197,94,.2);color:#86efac;margin-left:6px;vertical-align:middle;font-weight:700}
.delta-s-info{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:10px}
.delta-s-box{flex:1;min-width:120px;background:var(--bg);border-radius:8px;padding:11px;text-align:center}
.delta-s-box .v{font-size:1.2rem;font-weight:700;color:var(--accent)}
.delta-s-box .l{font-size:.58rem;color:var(--muted);text-transform:uppercase;margin-top:4px}
.rec-no-action{text-align:center;padding:20px;color:var(--green);font-size:.85rem}
/* Generate prediction button */
.btn-gen-pred{width:100%;padding:13px;border:none;border-radius:10px;
  background:var(--accent);color:#fff;font-size:.88rem;font-weight:700;
  cursor:pointer;transition:opacity .15s;margin-top:6px}
.btn-gen-pred:hover{opacity:.88}
.btn-gen-pred:disabled{background:#2a2d3a;color:var(--muted);cursor:not-allowed;opacity:1}
/* Intervention steps modal */
#intervention-modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.7);
  z-index:500;align-items:center;justify-content:center;padding:16px}
#intervention-modal.open{display:flex}
.intervention-box{background:var(--card);border:1px solid var(--border);border-radius:16px;
  padding:24px;width:100%;max-width:480px;max-height:88vh;overflow-y:auto}
.intervention-box h2{font-size:1rem;font-weight:700;margin-bottom:4px}
.intervention-box .int-ref{font-size:.65rem;color:var(--muted);margin-bottom:16px;line-height:1.5}
.int-step{display:flex;gap:12px;margin-bottom:14px;align-items:flex-start}
.int-step-num{min-width:26px;height:26px;border-radius:50%;background:var(--accent);
  color:#fff;font-size:.72rem;font-weight:700;display:flex;align-items:center;
  justify-content:center;flex-shrink:0;margin-top:1px}
.int-step-text{font-size:.8rem;color:var(--text);line-height:1.55}
.int-step-text strong{color:var(--accent)}
.int-close{display:block;width:100%;padding:11px;border:none;border-radius:8px;
  background:#252836;color:var(--muted);font-size:.82rem;font-weight:700;
  cursor:pointer;margin-top:10px;transition:background .15s}
.int-close:hover{background:#2f3347}
/* clickable rec rows */
.rec-table tbody tr.rec-clickable{cursor:pointer}
.rec-table tbody tr.rec-clickable:hover{background:rgba(99,102,241,.1)!important}

#toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%);
  background:#1e293b;color:#e2e8f0;padding:10px 20px;border-radius:8px;
  font-size:.82rem;opacity:0;transition:opacity .3s;pointer-events:none;
  border:1px solid var(--border);white-space:nowrap;z-index:999}
#toast.show{opacity:1}
/* Hamburger menu styles */
#hamburger{display:none;flex-direction:column;gap:4px;background:none;border:none;
  color:var(--muted);cursor:pointer;font-size:1.2rem;padding:8px;margin-right:8px;
  align-items:center;justify-content:center}
#hamburger span{display:block;width:24px;height:2px;background:currentColor;
  border-radius:1px}
body.open-menu #hamburger{color:var(--muted);font-size:1.4rem;padding:8px}
body.open-menu #hamburger span{display:none}
body.open-menu #hamburger::after{content:'✕';font-size:1.4rem}
@media(max-width:600px){
  #hamburger{display:flex}
  .sidebar{display:none;width:100%;height:auto;flex-direction:column;padding:16px 0;
    border-right:none;border-top:1px solid var(--border);order:2;
    position:fixed;top:52px;left:0;right:0;background:var(--sidebar);z-index:100;
    max-height:calc(100vh - 52px);box-shadow:0 2px 8px rgba(0,0,0,.3);overflow-y:auto}
  body.open-menu .sidebar{display:flex}
  body.open-menu::before{content:'';position:fixed;inset:0;top:52px;
    background:rgba(0,0,0,.5);z-index:99}
  .layout{flex-direction:column}
  .nav-label{display:block;padding:8px 16px;font-size:.65rem}
  .nav-item{flex-direction:row;gap:10px;padding:12px 16px;font-size:.82rem;
    border-left:none;border-top:none;width:100%;text-align:left}
  .nav-item:hover{background:rgba(255,255,255,.05)}
  .nav-item.active{border-left:3px solid var(--accent);background:rgba(99,102,241,.08);
    padding-left:13px}
  .nav-icon{font-size:1rem;width:18px;text-align:center}
  .main{order:1;padding:12px;padding-bottom:80px}
  .card{padding:12px;margin-bottom:10px}
  .grid3{grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:8px}
  .metric{background:var(--bg);border-radius:8px;padding:8px;text-align:center}
  .metric .v{font-size:1.2rem;font-weight:700;line-height:1}
  .metric .l{font-size:.55rem;color:var(--muted);text-transform:uppercase;margin-top:3px}
  .section-lbl{font-size:.6rem;margin:10px 0 6px}
  .avg-grid{grid-template-columns:repeat(2,1fr)}
  .node-grid{grid-template-columns:1fr}
}
</style>
</head>
<body>
<div class="topbar">
  <div class="topbar-left">
    <button id="hamburger" onclick="toggleMenu()" title="Toggle menu">
      <span></span>
      <span></span>
      <span></span>
    </button>
    <div id="dot"></div>
    <h1>VetiverTrack</h1>
  </div>
  <span id="clock"></span>
</div>
<div class="layout">
  <nav class="sidebar">
    <div class="nav-label">Navigation</div>
    <div class="nav-item active" onclick="showTab('dashboard',this)">
      <span class="nav-icon">📊</span><span>Dashboard</span>
    </div>
    <div class="nav-item" onclick="showTab('logs',this)">
      <span class="nav-icon">📋</span><span>Data Log</span>
    </div>
    <div class="nav-item" onclick="showTab('nodes',this)">
      <span class="nav-icon">📍</span><span>Node Sampling</span>
    </div>
  </nav>
  <div class="main">
    <!-- ══ TAB 1: DASHBOARD ══ -->
    <div class="tab active" id="tab-dashboard">
      <div class="card">
        <div class="clabel">Latest Reading</div>
        <div id="content"><div id="no-data">Waiting for sensor data...</div></div>
      </div>
      <div class="card" id="prediction-card">
        <div class="clabel">Prediction per 100 Readings</div>
        <div id="pred-status" style="text-align: center; padding: 20px; font-size: 14px;">
          <span style="color: var(--warn); font-weight: bold;">Awaiting 100 readings...</span>
          <div id="reading-progress" style="margin-top: 10px; font-size: 13px; color: var(--muted);">0 / 100</div>
        </div>
        <div id="pred-result" style="display: none;">
          <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:4px">
            <div style="background:var(--bg);border-radius:10px;padding:14px;text-align:center">
              <div style="font-size:.6rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:8px">Soil Suitability</div>
              <div id="pred-soil-badge" style="display:inline-block;padding:6px 18px;border-radius:20px;font-size:.85rem;font-weight:700;background:var(--border);color:var(--muted)">—</div>
              <div style="font-size:.65rem;color:var(--muted);margin-top:6px">Avg of 100 readings</div>
            </div>
            <div style="background:var(--bg);border-radius:10px;padding:14px;text-align:center">
              <div style="font-size:.6rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:8px">Slope Risk</div>
              <div id="pred-slope-badge" style="display:inline-block;padding:6px 18px;border-radius:20px;font-size:.85rem;font-weight:700;background:var(--border);color:var(--muted)">—</div>
              <div style="font-size:.65rem;color:var(--muted);margin-top:6px">Avg of 100 readings</div>
            </div>
          </div>
          <div style="margin-top:16px;text-align:center;padding:12px;background:var(--bg);border-radius:8px">
            <div style="font-size:.7rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:6px">Vetiver Slips to Plant</div>
            <div id="pred-slips" style="font-size:1.4rem;font-weight:700;color:var(--text)">—</div>
          </div>
        </div>
      </div>
      <!-- ══ RECOMMENDATIONS CARD ══ -->
      <div class="card" id="rec-card">
        <div class="clabel">Eco-Engineering Recommendations</div>
        <div id="rec-waiting" style="text-align:center;padding:20px;font-size:.82rem;color:var(--muted)">
          Awaiting prediction lock (100 readings)...
        </div>
        <div id="rec-no-action" class="rec-no-action" style="display:none">
         no shear strength intervention required.
        </div>
        <div id="rec-content" style="display:none">
          <div class="delta-s-info">
            <div class="delta-s-box">
              <div class="v" id="rec-delta-s">—</div>
              <div class="l">Required ΔS (Pa)</div>
            </div>
            <div class="delta-s-box">
              <div class="v" style="color:var(--mo)" id="rec-moisture">—</div>
              <div class="l">Locked Moisture %</div>
            </div>
          </div>
          <div style="font-size:.68rem;color:var(--muted);margin-bottom:8px;line-height:1.6">
            All 8 intervention combinations are shown below. Interventions reduce the shear strength
            deficit that vetiver roots must supply, lowering the required planting density (slips/m²).
            <br>Tr = 75 MPa · correction factor = 1.2 · A<sub>root</sub> = 0.005 m²
          </div>
          <div class="rec-table-wrap">
            <table class="rec-table">
              <thead>
                <tr>
                  <th>Intervention Combination</th>
                  <th>Added ΔS (Pa)</th>
                  <th>Net ΔS (Pa)</th>
                  <th>RAR</th>
                  <th>Slips / m²</th>
                </tr>
              </thead>
              <tbody id="rec-tbody"></tbody>
            </table>
          </div>
          <div style="font-size:.63rem;color:var(--muted);margin-top:8px">
            🟢 Highlighted row = combination requiring fewest vetiver slips.<br>
            Density = 0 means the interventions alone satisfy the shear strength requirement.
          </div>
        </div>
      </div>
      <div class="card">
        <div class="clabel">Soil Parameters — History</div>
        <div class="legend">
          <span><i style="background:var(--ph)"></i> pH</span>
          <span><i style="background:var(--ec)"></i> EC (scaled /100)</span>
          <span><i style="background:var(--mo)"></i> Moisture %</span>
        </div>
        <div class="chart-wrap"><canvas id="soilChart"></canvas></div>
      </div>
      <div class="card">
        <div class="clabel">MPU6050 Delta Tilt — History</div>
        <div class="legend">
          <span><i style="background:var(--total)"></i> Delta Tilt</span>
        </div>
        <div class="chart-wrap"><canvas id="tiltChart"></canvas></div>
      </div>
      <div class="card">
        <div class="clabel">System Status</div>
        <div class="sys-row"><span class="sys-key">Uptime</span><span class="sys-val" id="up">—</span></div>
        <div class="sys-row"><span class="sys-key">Packets Received</span><span class="sys-val" id="pkt-count">—</span></div>
        <div class="sys-row"><span class="sys-key">Modbus</span><span class="sys-val" id="mb">—</span></div>
        <div class="sys-row"><span class="sys-key">Link Status</span><span class="sys-val" id="link">—</span></div>
        <div class="sys-row"><span class="sys-key">Last Update</span><span class="sys-val" id="lastUpdate">—</span></div>
        <div class="sys-row"><span class="sys-key">Log Buffer</span><span class="sys-val" id="log-badge">—</span></div>
        <div class="sys-row"><span class="sys-key">Initiator Battery</span><span class="sys-val" id="bat-init-pct">—</span></div>
        <div class="sys-row"><span class="sys-key">Responder Battery</span><span class="sys-val" id="bat-resp-pct">—</span></div>
        <div style="display:flex;gap:10px;margin-top:14px">
          <button onclick="doCalibrate()" style="flex:1;padding:10px;border-radius:8px;border:none;background:#1e3a5f;color:#93c5fd;font-size:.8rem;font-weight:700;cursor:pointer">Recalibrate Sensor</button>
          <button onclick="doResetLog()" style="flex:1;padding:10px;border-radius:8px;border:none;background:#3b1a1a;color:#fca5a5;font-size:.8rem;font-weight:700;cursor:pointer">Reset Log</button>
        </div>
      </div>
    </div>
<!-- ══ TAB 2: DATA LOG ══ -->
    <div class="tab" id="tab-logs">
      <!-- PAUSE BAR -->
      <div class="pause-bar" style="margin-bottom:14px">
        <span id="pause-status-text">Recording: <strong style="color:var(--green)">Active</strong></span>
        <button id="btn-pause" onclick="togglePause()">Pause Recording</button>
      </div>
      <div class="card">
        <div class="tbl-controls">
          <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
            <label style="font-size:.72rem;color:var(--muted)">From</label>
            <input type="date" id="filter-from-date" onchange="applyFilter()" oninput="applyFilter()"
              style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:5px 8px;color:var(--text);font-size:.75rem;cursor:pointer">
            <input type="time" id="filter-from-time" onchange="applyFilter()" oninput="applyFilter()" disabled
              style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:5px 8px;color:var(--text);font-size:.75rem;cursor:pointer;opacity:.4">
            <label style="font-size:.72rem;color:var(--muted)">To</label>
            <input type="date" id="filter-to-date" onchange="applyFilter()" oninput="applyFilter()"
              style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:5px 8px;color:var(--text);font-size:.75rem;cursor:pointer">
            <input type="time" id="filter-to-time" onchange="applyFilter()" oninput="applyFilter()" disabled
              style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:5px 8px;color:var(--text);font-size:.75rem;cursor:pointer;opacity:.4">
            <button onclick="clearFilter()"
              style="background:transparent;border:1px solid var(--border);border-radius:6px;padding:5px 10px;color:var(--muted);font-size:.72rem;cursor:pointer">Clear</button>
          </div>
          <div style="display:flex;align-items:center;gap:10px">
            <span id="tbl-count" style="font-size:.72rem;color:var(--muted)">— readings</span>
            <button class="btn-csv" onclick="openDownloadModal()">Download</button>
          </div>
        </div>
        <div class="tbl-wrap">
          <table>
            <thead>
              <tr>
                <th>Datetime</th>
                <th>Moisture %</th>
                <th>EC uS/cm</th>
                <th>pH</th>
                <th>Pitch °</th>
                <th>Roll °</th>
                <th>Tilt °</th>
                <th>Δ Tilt °</th>
                <th>ΔS (Pa)</th>
                <th>Soil</th>
                <th>Slope</th>
              </tr>
            </thead>
            <tbody id="log-tbody">
              <tr><td colspan="11" style="text-align:center;color:var(--muted);padding:30px">No data yet</td></tr>
            </tbody>
          </table>
        </div>
        <div id="pagination" style="display:flex;align-items:center;justify-content:space-between;margin-top:12px;flex-wrap:wrap;gap:8px">
          <span id="page-info" style="font-size:.72rem;color:var(--muted)"></span>
          <div style="display:flex;gap:6px;align-items:center">
            <button id="btn-first" onclick="goPage(1)" style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:5px 10px;color:var(--muted);font-size:.72rem;cursor:pointer">«</button>
            <button id="btn-prev"  onclick="goPage(currentPage-1)" style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:5px 10px;color:var(--muted);font-size:.72rem;cursor:pointer">‹ Prev</button>
            <span id="page-btns" style="display:flex;gap:4px"></span>
            <button id="btn-next"  onclick="goPage(currentPage+1)" style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:5px 10px;color:var(--muted);font-size:.72rem;cursor:pointer">Next ›</button>
            <button id="btn-last"  onclick="goPage(totalPages)" style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:5px 10px;color:var(--muted);font-size:.72rem;cursor:pointer">»</button>
          </div>
        </div>
      </div>
    </div>
    <!-- ══ TAB 3: NODE SAMPLING ══ -->
    <div class="tab" id="tab-nodes">
      <div class="card" style="margin-bottom:14px">
        <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:8px">
          <div class="clabel" style="margin:0">Node Sampling</div>
          <button class="btn-reset-all" onclick="resetAllNodes()">Reset All</button>
        </div>
        <div style="font-size:.72rem;color:var(--muted);line-height:1.6">
          Take <strong style="color:var(--text)">100 readings per node</strong> across 5 locations on the slope.
          Press <strong style="color:var(--accent)">Start</strong> to begin. Recording stops automatically at 100 readings.
          Download per-node data after each node completes.
          Upload CSV files for any number of nodes, then press <strong style="color:var(--accent)">Generate Overall Prediction</strong> to compute the combined result.
        </div>
        <div id="nodes-progress-bar-wrap" style="margin-top:10px">
          <div style="display:flex;justify-content:space-between;font-size:.68rem;color:var(--muted);margin-bottom:4px">
            <span>Nodes completed</span><span id="nodes-done-count">0 / 5</span>
          </div>
          <div style="height:5px;border-radius:3px;background:var(--border);overflow:hidden">
            <div id="nodes-done-bar" style="height:100%;border-radius:3px;background:var(--green);width:0%;transition:width .4s"></div>
          </div>
        </div>
      </div>
      <!-- Node cards -->
      <div class="node-grid" id="node-grid"></div>
      <!-- Success banner when CSV uploads are present -->
      <div id="upload-success-banner" style="display:none;background:rgba(34,197,94,.12);border:1px solid rgba(34,197,94,.3);border-radius:10px;padding:14px;margin-bottom:14px;text-align:center">
        <div style="font-size:.9rem;font-weight:600;color:var(--green);margin-bottom:6px" id="upload-banner-title">CSV Data Loaded</div>
        <div style="font-size:.75rem;color:var(--muted)" id="upload-banner-sub">Press Generate Overall Prediction below to compute results</div>
      </div>
      <!-- Generate Overall Prediction button — shown when ≥1 node has data -->
      <div id="gen-pred-wrap" style="display:none;margin-bottom:14px">
        <button class="btn-gen-pred" id="btn-gen-pred" onclick="runGeneratePrediction()">
          Generate Overall Prediction
        </button>
        <div id="gen-pred-hint" style="font-size:.65rem;color:var(--muted);text-align:center;margin-top:6px"></div>
      </div>
      <!-- Overall averages — shown after prediction generated -->
      <div class="card" id="avg-card" style="display:none">
        <div class="clabel" id="avg-card-label">Overall Average</div>
        <div class="avg-grid">
          <div class="avg-card"><div class="v mo-c" id="avg-moisture">—</div><div class="l">Moisture %</div></div>
          <div class="avg-card"><div class="v ec-c" id="avg-ec">—</div><div class="l">EC uS/cm</div></div>
          <div class="avg-card"><div class="v ph-c" id="avg-ph">—</div><div class="l">pH</div></div>
          <div class="avg-card"><div class="v ti-c" id="avg-tilt">—</div><div class="l">Δ Tilt °</div></div>
        </div>
        <div id="avg-note" style="font-size:.68rem;color:var(--muted);text-align:center;margin-top:10px"></div>
      </div>
      <!-- Prediction — shown only when all 5 nodes done -->
      <div class="card" id="node-pred-card" style="display:none">
        <div class="clabel" id="node-pred-card-label">Overall Prediction</div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:4px">
          <div style="background:var(--bg);border-radius:10px;padding:14px;text-align:center">
            <div style="font-size:.6rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:8px">Soil Suitability</div>
            <div id="node-soil-badge" style="display:inline-block;padding:6px 18px;border-radius:20px;font-size:.85rem;font-weight:700;background:var(--border);color:var(--muted)">—</div>
            <div style="font-size:.65rem;color:var(--muted);margin-top:6px">Based on avg EC + pH</div>
          </div>
          <div style="background:var(--bg);border-radius:10px;padding:14px;text-align:center">
            <div style="font-size:.6rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:8px">Slope Risk</div>
            <div id="node-slope-badge" style="display:inline-block;padding:6px 18px;border-radius:20px;font-size:.85rem;font-weight:700;background:var(--border);color:var(--muted)">—</div>
            <div style="font-size:.65rem;color:var(--muted);margin-top:6px">Based on avg delta tilt + moisture</div>
          </div>
        </div>
        <div style="margin-top:12px;background:var(--bg);border-radius:10px;padding:16px;text-align:center">
          <div style="font-size:.6rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:8px">Vetiver Slips to Plant</div>
          <div id="node-slips-value" style="font-size:2.2rem;font-weight:700;color:var(--muted)">—</div>
          <div id="node-slips-sub" style="font-size:.68rem;color:var(--muted);margin-top:5px">Complete all 5 nodes to see recommendation</div>
        </div>
        <div id="node-pred-alert" style="margin-top:10px"></div>
        <div style="text-align:center;margin-top:12px">
          <button onclick="downloadNodePrediction()"
            style="padding:10px 24px;border-radius:8px;border:none;background:#166534;color:#bbf7d0;font-size:.8rem;font-weight:700;cursor:pointer">
            Download Overall Prediction CSV
          </button>
        </div>
      </div>
      <!-- Node eco-engineering recommendations card -->
      <div class="card" id="node-rec-card" style="display:none">
        <div class="clabel">Eco-Engineering Recommendations</div>
        <div id="node-rec-no-action" style="display:none;font-size:.82rem;color:var(--muted);text-align:center;padding:16px">
          no shear strength intervention required.
        </div>
        <div id="node-rec-content" style="display:none">
          <div class="delta-s-info">
            <div class="delta-s-box">
              <div class="v" id="node-rec-delta-s">&mdash;</div>
              <div class="l">Required &Delta;S (Pa)</div>
            </div>
            <div class="delta-s-box">
              <div class="v" style="color:var(--mo)" id="node-rec-moisture">&mdash;</div>
              <div class="l">Avg Moisture %</div>
            </div>
          </div>
          <div style="font-size:.68rem;color:var(--muted);margin-bottom:8px;line-height:1.6">
            All 8 intervention combinations shown below. Interventions reduce the shear strength
            deficit that vetiver roots must supply, lowering the required planting density (slips/m&sup2;).
            <br>Tr = 75 MPa &middot; correction factor = 1.2 &middot; A<sub>root</sub> = 0.005 m&sup2;
          </div>
          <div class="rec-table-wrap">
            <table class="rec-table">
              <thead>
                <tr>
                  <th>Intervention Combination</th>
                  <th>Added &Delta;S (Pa)</th>
                  <th>Net &Delta;S (Pa)</th>
                  <th>RAR</th>
                  <th>Slips / m&sup2;</th>
                </tr>
              </thead>
              <tbody id="node-rec-tbody"></tbody>
            </table>
          </div>
          <div style="font-size:.63rem;color:var(--muted);margin-top:8px">
            Highlighted row = combination requiring fewest vetiver slips.<br>
            Density = 0 means the interventions alone satisfy the shear strength requirement.
          </div>
        </div>
      </div>
    </div>
  </div><!-- .main -->
</div><!-- .layout -->
<!-- DOWNLOAD MODAL -->
<div id="dl-modal">
  <div class="dl-box">
    <h3>Download Options</h3>
    <button id="dl-csv-btn" class="dl-option" onclick="doDownload('csv')">Download CSV Log only</button>
    <button id="dl-pred-btn" class="dl-option" onclick="doDownload('pred')">Download Prediction Results only</button>
    <button id="dl-both-btn" class="dl-option" onclick="doDownload('both')">Download Both (CSV + Prediction)</button>
    <button class="dl-cancel" onclick="closeDownloadModal()">Cancel</button>
  </div>
</div>
<!-- NODE DOWNLOAD MODAL -->
<div id="node-dl-modal" style="display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.5);z-index:1000;align-items:center;justify-content:center">
  <div style="background:var(--bg);border:1px solid var(--border);border-radius:12px;padding:20px;max-width:320px;box-shadow:0 8px 24px rgba(0,0,0,0.15)">
    <h3 style="margin:0 0 16px 0;font-size:1rem">Download Node Data</h3>
    <button onclick="downloadNodeAverage(window._nodeDownloadId)" style="width:100%;padding:12px;margin-bottom:8px;background:var(--accent);color:var(--bg);border:none;border-radius:8px;font-weight:700;cursor:pointer;font-size:.85rem">
      Download Node Excel Report
    </button>
    <button onclick="closeNodeDownloadModal()" style="width:100%;padding:12px;background:var(--border);color:var(--muted);border:none;border-radius:8px;font-weight:700;cursor:pointer;font-size:.85rem">
      Cancel
    </button>
  </div>
</div>
<div id="toast"></div>
<!-- INTERVENTION STEPS MODAL -->
<div id="intervention-modal">
  <div class="intervention-box">
    <h2 id="int-modal-title">Intervention Steps</h2>
    <div class="int-ref" id="int-modal-ref"></div>
    <div id="int-modal-steps"></div>
    <button class="int-close" onclick="closeInterventionModal()">Close</button>
  </div>
</div>
<script>
// =========================================================================
// CLOCK
// =========================================================================
setInterval(function(){ document.getElementById('clock').textContent = new Date().toLocaleTimeString(); }, 1000);
// =========================================================================
// TAB SWITCHING
// =========================================================================
function showTab(name, el) {
  document.querySelectorAll('.tab').forEach(function(t){ t.classList.remove('active'); });
  document.querySelectorAll('.nav-item').forEach(function(n){ n.classList.remove('active'); });
  document.getElementById('tab-' + name).classList.add('active');
  el.classList.add('active');
  if (name === 'dashboard') setTimeout(renderCharts, 50);
  if (name === 'logs') renderTable();
  if (name === 'nodes') { loadNodes(); startNodePolling(); }
  closeMenu();
}
// =========================================================================
// TOAST
// =========================================================================
function showToast(msg, duration) {
  duration = duration || 2500;
  var t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(function(){ t.classList.remove('show'); }, duration);
}
// =========================================================================
// MOBILE HAMBURGER MENU
// =========================================================================
function toggleMenu() {
  var body = document.body;
  body.classList.toggle('open-menu');
}
function closeMenu() {
  document.body.classList.remove('open-menu');
}
// Close menu when nav item is clicked
document.querySelectorAll('.nav-item').forEach(function(item) {
  item.addEventListener('click', function() {
    closeMenu();
  });
});
// Close menu when Escape key is pressed
document.addEventListener('keydown', function(e) {
  if (e.key === 'Escape' || e.key === 'Esc') {
    closeMenu();
  }
});
// Close menu when clicking on overlay (outside sidebar)
document.addEventListener('click', function(e) {
  var sidebar = document.querySelector('.sidebar');
  var hamburger = document.getElementById('hamburger');
  var body = document.body;
  
  if (body.classList.contains('open-menu')) {
    if (sidebar && !sidebar.contains(e.target) && 
        hamburger && !hamburger.contains(e.target)) {
      closeMenu();
    }
  }
});
// =========================================================================
// DOWNLOAD MODAL
// =========================================================================
function openDownloadModal() { document.getElementById('dl-modal').classList.add('open'); }
function closeDownloadModal() { document.getElementById('dl-modal').classList.remove('open'); }
function doDownload(type) {
  // CSV can be downloaded anytime, but prediction requires predTarget+ readings
  if ((type === 'pred' || type === 'both') && window._currentLogCount < (window._predTarget || 10)) {
    showToast('Need ' + (window._predTarget || 10) + '+ readings for prediction (currently ' + window._currentLogCount + ')');
    return;
  }
  closeDownloadModal();
  if (type === 'csv')  { window.location.href = '/download'; }
  if (type === 'pred') { window.location.href = '/download/prediction'; }
  if (type === 'both') { window.location.href = '/download/both'; }
}
// =========================================================================
// PAUSE / RESUME
// =========================================================================
var isPaused = false;
async function togglePause() {
  try {
    var r = await fetch('/api/pause', { method: 'POST' });
    var d = await r.json();
    isPaused = d.paused;
    updatePauseUI();
    showToast(isPaused ? 'Recording paused' : 'Recording resumed');
  } catch(e) { showToast('Request failed'); }
}
function updatePauseUI() {
  var btn = document.getElementById('btn-pause');
  var txt = document.getElementById('pause-status-text');
  if (isPaused) {
    btn.textContent = 'Resume Recording';
    btn.classList.add('paused');
    txt.innerHTML = 'Recording: <strong style="color:var(--red)">Paused</strong>';
  } else {
    btn.textContent = 'Pause Recording';
    btn.classList.remove('paused');
    txt.innerHTML = 'Recording: <strong style="color:var(--green)">Active</strong>';
  }
}
// =========================================================================
// CHART DRAWING
// =========================================================================
function drawChart(canvasId, datasets, yMin, yMax) {
  var c = document.getElementById(canvasId);
  if (!c) return;
  var dpr  = window.devicePixelRatio || 1;
  var rect = c.parentElement.getBoundingClientRect();
  c.width  = rect.width  * dpr;
  c.height = rect.height * dpr;
  var ctx  = c.getContext('2d');
  ctx.scale(dpr, dpr);
  var W = rect.width, H = rect.height;
  var pad = {t:10, r:10, b:22, l:42};
  var gW = W - pad.l - pad.r;
  var gH = H - pad.t - pad.b;
  ctx.clearRect(0, 0, W, H);
  var maxLen = 0;
  for (var i = 0; i < datasets.length; i++) if (datasets[i].data.length > maxLen) maxLen = datasets[i].data.length;
  if (maxLen < 2) {
    ctx.fillStyle = '#6b7080'; ctx.font = '11px -apple-system,sans-serif';
    ctx.textAlign = 'center'; ctx.fillText('Waiting for data...', W / 2, H / 2); return;
  }
  if (yMin === undefined || yMax === undefined) {
    yMin = Infinity; yMax = -Infinity;
    for (var i = 0; i < datasets.length; i++)
      for (var j = 0; j < datasets[i].data.length; j++) {
        if (datasets[i].data[j] < yMin) yMin = datasets[i].data[j];
        if (datasets[i].data[j] > yMax) yMax = datasets[i].data[j];
      }
    var margin = (yMax - yMin) * 0.12 || 1;
    yMin -= margin; yMax += margin;
  }
  var yRange = yMax - yMin || 1;
  ctx.lineWidth = 1;
  for (var i = 0; i <= 4; i++) {
    var y = pad.t + (gH / 4) * i;
    ctx.strokeStyle = '#252836'; ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke();
    ctx.fillStyle = '#6b7080'; ctx.font = '9px -apple-system,sans-serif'; ctx.textAlign = 'right';
    var labelVal = yMax - (yRange / 4) * i;
    if (Math.abs(labelVal) < 0.01) {
      ctx.fillText(labelVal.toFixed(4), pad.l - 4, y + 3);
    } else {
      ctx.fillText(labelVal.toFixed(1), pad.l - 4, y + 3);
    }
  }
  ctx.fillStyle = '#6b7080'; ctx.font = '9px -apple-system,sans-serif'; ctx.textAlign = 'center';
  ctx.fillText('oldest', pad.l + 20, H - 4);
  ctx.fillText('latest', W - pad.r - 20, H - 4);
  for (var d = 0; d < datasets.length; d++) {
    var ds = datasets[d];
    if (ds.data.length < 2) continue;
    ctx.strokeStyle = ds.color; ctx.lineWidth = ds.dashed ? 1.5 : 2;
    ctx.lineJoin = 'round';
    if (ds.dashed) ctx.setLineDash([5, 4]); else ctx.setLineDash([]);
    ctx.beginPath();
    for (var j = 0; j < ds.data.length; j++) {
      var x = pad.l + (j / (maxLen - 1)) * gW;
      var y = pad.t + gH - ((ds.data[j] - yMin) / yRange) * gH;
      if (j === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke(); ctx.setLineDash([]);
    var lastX = pad.l + ((ds.data.length - 1) / (maxLen - 1)) * gW;
    var lastY = pad.t + gH - ((ds.data[ds.data.length - 1] - yMin) / yRange) * gH;
    ctx.beginPath(); ctx.arc(lastX, lastY, 3.5, 0, Math.PI * 2);
    ctx.fillStyle = ds.color; ctx.fill();
  }
}
// =========================================================================
// DATA STATE
// =========================================================================
var historyData = [];
var latestData  = null;
var autoDownloadDone = false;
// =========================================================================
// BATTERY
// =========================================================================
function updateBattery(elId, pct) {
  var el = document.getElementById(elId);
  if (!el) return;
  if (pct === undefined || pct === null || pct < 0) { el.textContent = '—'; return; }
  var p = parseFloat(pct);
  el.textContent = p.toFixed(0) + '%';
  el.style.color = p > 50 ? 'var(--green)' : p > 20 ? 'var(--roll)' : 'var(--red)';
}
// =========================================================================
// ML BADGE
// =========================================================================
function updateMlBadge(elId, label) {
  var el = document.getElementById(elId);
  if (!el) return;
  el.textContent = label.replace(/_/g, ' ');
  el.style.background = 'var(--border)'; el.style.color = 'var(--muted)';
  if (label === 'suitable')         { el.style.background = 'rgba(34,197,94,.15)';  el.style.color = '#86efac'; }
  else if (label === 'not_suitable'){ el.style.background = 'rgba(239,68,68,.15)';  el.style.color = '#fca5a5'; }
  else if (label === 'stable')      { el.style.background = 'rgba(34,197,94,.15)';  el.style.color = '#86efac'; }
  else if (label === 'pre_failure') { el.style.background = 'rgba(251,191,36,.15)'; el.style.color = '#fde68a'; }
  else if (label === 'failure_imminent'){ el.style.background = 'rgba(239,68,68,.15)'; el.style.color = '#fca5a5'; }
}
var globalActualReadingCount = 0;
// =========================================================================
// FETCH + UPDATE
// =========================================================================
async function loadData() {
  try {
    var r = await fetch('/api/data');
    latestData = await r.json();
    var d = latestData;
    if (d.paused !== undefined && isPaused !== d.paused) {
      isPaused = d.paused;
      updatePauseUI();
    }
    window._currentLogCount = d.logCount;
    window._predTarget = d.predTarget || 10;
    var predCard = document.getElementById('prediction-card');
    var csvBtn = document.getElementById('dl-csv-btn');
    var predBtn = document.getElementById('dl-pred-btn');
    var bothBtn = document.getElementById('dl-both-btn');
    
    if (csvBtn) {
      csvBtn.disabled = false;
      csvBtn.style.opacity = '1';
      csvBtn.style.cursor = 'pointer';
      csvBtn.title = '';
    }
    
    var predStatus = document.getElementById('pred-status');
    var predResult = document.getElementById('pred-result');
    var target = d.predTarget || 10;
    if (d.predLocked) {
      if (predStatus) predStatus.style.display = 'none';
      if (predResult) predResult.style.display = 'block';
      if (predBtn) { predBtn.disabled = false; predBtn.style.opacity = '1'; predBtn.style.cursor = 'pointer'; predBtn.title = ''; }
      if (bothBtn) { bothBtn.disabled = false; bothBtn.style.opacity = '1'; bothBtn.style.cursor = 'pointer'; bothBtn.title = ''; }
    } else {
      if (predStatus) predStatus.style.display = 'block';
      if (predResult) predResult.style.display = 'none';
      var progressEl = document.getElementById('reading-progress');
      if (progressEl) progressEl.textContent = d.predCount + ' / ' + target;
      if (predBtn) { predBtn.disabled = true; predBtn.style.opacity = '0.5'; predBtn.style.cursor = 'not-allowed'; predBtn.title = 'Requires ' + target + '+ readings'; }
      if (bothBtn) { bothBtn.disabled = true; bothBtn.style.opacity = '0.5'; bothBtn.style.cursor = 'not-allowed'; bothBtn.title = 'Requires ' + target + '+ readings'; }
    }
    document.getElementById('log-badge').textContent =
      d.logCount + ' readings' + (d.logFull ? ' (full — downloading CSV...)' : '');
    if (d.logFull && !autoDownloadDone) {
      autoDownloadDone = true;
      showToast('Log full (500 readings) — downloading CSV automatically...');
      setTimeout(function(){ window.location.href = '/download'; }, 1500);
    }
    if (!d.received) {
      document.getElementById('content').innerHTML = '<div id="no-data">Waiting for sensor data...</div>';
    } else {
      var phValue = parseFloat(d.ph);
      var ecValue = parseFloat(d.ec);
      var moistureValue = parseFloat(d.moisture);
      var deltaTiltValue = d.deltaTilt !== undefined ? parseFloat(d.deltaTilt) : 0;
      
      // Color coding
      var phColor = (phValue >= 6.0 && phValue <= 8.0) ? '#86efac' : '#fca5a5';
      var ecColor = (ecValue <= 10000.0) ? '#86efac' : '#fca5a5';  // 10,000 uS/cm threshold
      var moistureColor = '#34d399';  // Always green for moisture display
      var tiltColor = getTiltColor(deltaTiltValue, moistureValue);
      
      document.getElementById('content').innerHTML =
        '<div class="section-lbl">Soil</div>' +
        '<div class="grid3">' +
          '<div class="metric"><div class="v" style="color:' + phColor + '">' + phValue.toFixed(1) + '</div><div class="l">pH</div><div style="font-size:.6rem;color:var(--muted);margin-top:2px">suitable: 6–8</div></div>' +
          '<div class="metric"><div class="v" style="color:' + ecColor + '">' + ecValue.toFixed(0) + '</div><div class="l">EC uS/cm</div><div style="font-size:.6rem;color:var(--muted);margin-top:2px">suitable: ≤ 10,000</div></div>' +
          '<div class="metric"><div class="v" style="color:' + moistureColor + '">' + moistureValue.toFixed(1) + '</div><div class="l">Moisture %</div><div style="font-size:.6rem;color:var(--muted);margin-top:2px;line-height:1.3">stable: &lt; 50%<br>pre-failure: 50–79%<br>failure: &gt; 80%</div></div>' +
        '</div>' +
        '<div class="section-lbl">Tilt</div>' +
        '<div class="grid3">' +
          '<div class="metric"><div class="v ti-c">' + (d.pitch >= 0 ? '+' : '') + parseFloat(d.pitch).toFixed(2) + '</div><div class="l">Pitch °</div></div>' +
          '<div class="metric"><div class="v ti-c">' + (d.roll  >= 0 ? '+' : '') + parseFloat(d.roll).toFixed(2)  + '</div><div class="l">Roll °</div></div>' +
          '<div class="metric"><div class="v ti-c">' + parseFloat(d.tilt).toFixed(2) + '</div><div class="l">Total °</div></div>' +
        '</div>' +
        '<div class="section-lbl">Delta Tilt</div>' +
        '<div class="grid3">' +
        '<div class="metric" style="grid-column:1/-1"><div class="v" style="color:' + tiltColor + '">' +
          (deltaTiltValue !== undefined ? deltaTiltValue.toFixed(2) : '—') +
        '</div><div class="l">Δ Tilt ° (vs previous reading)</div><div style="font-size:.6rem;color:var(--muted);margin-top:2px;line-height:1.3">stable: &lt; 0.5°<br>pre-failure: 0.5–2°<br>failure: &gt; 2°</div></div>' +
        '</div>';
    }
    var s = Math.floor(d.uptime / 1000);
    document.getElementById('up').textContent         = Math.floor(s/3600)+'h '+Math.floor((s%3600)/60)+'m '+(s%60)+'s';
    document.getElementById('pkt-count').textContent  = d.packets;
    document.getElementById('mb').textContent         = d.modbus   ? 'OK' : 'FAILED';
    document.getElementById('link').textContent       = d.received ? 'Receiving' : 'No data yet';
    document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
    updateBattery('bat-init-pct', d.initBatPct);
    updateBattery('bat-resp-pct', d.respBatPct);
  } catch(e) {
    document.getElementById('lastUpdate').textContent = 'update failed';
  }
}
function getTiltColor(deltaTilt, moisture) {
  if (deltaTilt === undefined) deltaTilt = 0;
  if (moisture === undefined) moisture = 0;
  deltaTilt = parseFloat(deltaTilt);
  moisture = parseFloat(moisture);
  
  if (deltaTilt < 0.5 || moisture < 50) {
    return '#86efac';  // Green - Stable
  } else if (deltaTilt <= 2.0 && moisture >= 50 && moisture < 80) {
    return '#fde68a';  // Yellow - Pre-Failure
  } else if (deltaTilt > 2.0 && moisture >= 80) {
    return '#fca5a5';  // Red - Failure Imminent
  }
  return '#6b7080';  // Default gray
}
async function loadHistory() {
  try {
    var r = await fetch('/history');
    historyData = await r.json();
    globalActualReadingCount = historyData ? historyData.length : 0;
    console.log('[loadHistory] Reading count: ' + globalActualReadingCount);
    document.getElementById('reading-progress').textContent = globalActualReadingCount + ' / ' + (window._predTarget || 10);
    loadPrediction();
    var active = document.querySelector('.tab.active');
    if (active && active.id === 'tab-dashboard') renderCharts();
    if (active && active.id === 'tab-logs')      renderTable();
  } catch(e) { 
    console.error('[loadHistory] Error:', e);
  }
}
// =========================================================================
// TABLE RENDER
// =========================================================================
var currentPage   = 1;
var totalPages    = 1;
var filteredRows  = [];
var ROWS_PER_PAGE = 25;
function applyFilter() {
  var fromDate = document.getElementById('filter-from-date').value;
  var fromTime = document.getElementById('filter-from-time').value;
  var toDate   = document.getElementById('filter-to-date').value;
  var toTime   = document.getElementById('filter-to-time').value;
  var fromTimeEl = document.getElementById('filter-from-time');
  var toTimeEl   = document.getElementById('filter-to-time');
  fromTimeEl.disabled = !fromDate; fromTimeEl.style.opacity = fromDate ? '1' : '0.4';
  toTimeEl.disabled   = !toDate;   toTimeEl.style.opacity   = toDate   ? '1' : '0.4';
  if (!fromDate) { fromTimeEl.value = ''; fromTime = ''; }
  if (!toDate)   { toTimeEl.value   = ''; toTime   = ''; }
  var all = [];
  for (var i = historyData.length - 1; i >= 0; i--) all.push(historyData[i]);
  if (!fromDate && !toDate) { filteredRows = all; currentPage = 1; renderTablePage(); return; }
  var fromDT = fromDate ? (fromDate + ' ' + (fromTime ? fromTime + ':00' : '00:00:00')) : '';
  var toDT   = toDate   ? (toDate   + ' ' + (toTime   ? toTime   + ':59' : '23:59:59')) : '';
  filteredRows = all.filter(function(row) {
    var dt = row.t;
    if (fromDT && dt < fromDT) return false;
    if (toDT   && dt > toDT)   return false;
    return true;
  });
  currentPage = 1;
  renderTablePage();
}
function clearFilter() {
  ['filter-from-date','filter-from-time','filter-to-date','filter-to-time'].forEach(function(id){
    document.getElementById(id).value = '';
  });
  document.getElementById('filter-from-time').disabled = true;
  document.getElementById('filter-to-time').disabled   = true;
  document.getElementById('filter-from-time').style.opacity = '0.4';
  document.getElementById('filter-to-time').style.opacity   = '0.4';
  applyFilter();
}
function goPage(p) {
  if (p < 1 || p > totalPages) return;
  currentPage = p;
  renderTablePage();
}
function renderTablePage() {
  var tbody = document.getElementById('log-tbody');
  var count = document.getElementById('tbl-count');
  totalPages = Math.max(1, Math.ceil(filteredRows.length / ROWS_PER_PAGE));
  if (currentPage > totalPages) currentPage = totalPages;
  var start = (currentPage - 1) * ROWS_PER_PAGE;
  var end   = Math.min(start + ROWS_PER_PAGE, filteredRows.length);
  var pageRows = filteredRows.slice(start, end);
  var isFiltered = filteredRows.length !== historyData.length;
  count.textContent = filteredRows.length + ' reading' + (filteredRows.length !== 1 ? 's' : '') + (isFiltered ? ' (filtered)' : ' stored');
  var rows = '';
  for (var i = 0; i < pageRows.length; i++) {
    var row = pageRows[i];
    rows += '<tr>' +
      '<td>' + row.t + '</td>' +
      '<td>' + parseFloat(row.m).toFixed(1)  + '</td>' +
      '<td>' + parseFloat(row.e).toFixed(0)  + '</td>' +
      '<td>' + parseFloat(row.p).toFixed(1)  + '</td>' +
      '<td>' + (row.pi >= 0 ? '+' : '') + parseFloat(row.pi).toFixed(2) + '</td>' +
      '<td>' + (row.r  >= 0 ? '+' : '') + parseFloat(row.r).toFixed(2)  + '</td>' +
      '<td>' + parseFloat(row.ti).toFixed(2) + '</td>' +
      '<td>' + (row.dt !== undefined ? parseFloat(row.dt).toFixed(2) : '—') + '</td>' +
      '<td>' + (row.ds !== undefined && row.ds >= 0 ? parseFloat(row.ds).toFixed(2) : '—') + '</td>' +
      '<td style="font-size:.7rem;' + (row.ms === 'pending' ? 'opacity:.4;color:var(--muted)' : 'color:' + (row.ms === 'suitable' ? '#86efac' : '#fca5a5')) + '">' + (row.ms || '—') + '</td>' +
      '<td style="font-size:.7rem;' + (row.sl === 'pending' ? 'opacity:.4;color:var(--muted)' : '') + '">' + (row.sl || '—') + '</td>' +
      '</tr>';
  }
  tbody.innerHTML = rows || '<tr><td colspan="11" style="text-align:center;color:var(--muted);padding:30px">No data</td></tr>';
  document.getElementById('page-info').textContent = 'Showing ' + (start + 1) + '–' + end + ' of ' + filteredRows.length;
  var btnHtml = '';
  var startP = Math.max(1, currentPage - 2);
  var endP   = Math.min(totalPages, currentPage + 2);
  for (var p = startP; p <= endP; p++) {
    var isActive = p === currentPage;
    btnHtml += '<button onclick="goPage(' + p + ')" style="' +
      (isActive ? 'background:var(--accent);color:#fff;border-color:var(--accent);' : 'background:var(--bg);color:var(--muted);') +
      'border:1px solid var(--border);border-radius:6px;padding:5px 9px;font-size:.72rem;cursor:pointer;">' + p + '</button>';
  }
  document.getElementById('page-btns').innerHTML = btnHtml;
  document.getElementById('btn-first').disabled = currentPage === 1;
  document.getElementById('btn-prev').disabled  = currentPage === 1;
  document.getElementById('btn-next').disabled  = currentPage === totalPages;
  document.getElementById('btn-last').disabled  = currentPage === totalPages;
}
function renderTable() { applyFilter(); }
// =========================================================================
// CHARTS
// =========================================================================
function renderCharts() {
  if (!historyData || historyData.length < 1) return;
  var phArr=[], ecArr=[], moArr=[], tiArr=[];
  for (var i = 0; i < historyData.length; i++) {
    phArr.push(historyData[i].p);
    ecArr.push(historyData[i].e / 100);
    moArr.push(historyData[i].m);
    tiArr.push(historyData[i].dt !== undefined ? historyData[i].dt : historyData[i].ti);
  }
  drawChart('soilChart', [
    {data: phArr, color: '#a78bfa'},
    {data: ecArr, color: '#38bdf8'},
    {data: moArr, color: '#34d399'}
  ]);
  drawChart('tiltChart', [
    {data: tiArr, color: '#ef4444'}
  ]);
}
// =========================================================================
// NODE SAMPLING
// =========================================================================
var uploadedNodeData = {1: null, 2: null, 3: null, 4: null, 5: null};
var nodeUploadFiles = {1: null, 2: null, 3: null, 4: null, 5: null};
function parseNodeCSV(csvText) {
  console.log('[DEBUG] parseNodeCSV called with text length:', csvText.length);
  var lines = csvText.trim().split('\n');
  console.log('[DEBUG] Total lines:', lines.length);
  
  var hasHeader = false;
  var startIdx = 0;
  var moistureCol = 0, ecCol = 1, phCol = 2, tiltCol = 3;
  
  if (lines.length > 0) {
    var firstLine = lines[0].toLowerCase();
    console.log('[DEBUG] First line:', lines[0]);
    if (firstLine.includes('moisture') || firstLine.includes('ec') || 
        firstLine.includes('ph') || firstLine.includes('tilt') ||
        firstLine.includes('delta tilt')) {
      hasHeader = true;
      startIdx = 1;
      console.log('[DEBUG] Header detected!');
      
      var headerParts = lines[0].split(',');
      console.log('[DEBUG] Header parts:', headerParts);
      for (var h = 0; h < headerParts.length; h++) {
        var col = headerParts[h].toLowerCase().trim();
        if (col.includes('moisture')) moistureCol = h;
        else if (col.includes('ec')) ecCol = h;
        else if (col.includes('ph')) phCol = h;
        else if (col.includes('tilt') || col.includes('delta tilt')) tiltCol = h;
      }
      console.log('[DEBUG] Column mapping - moisture:' + moistureCol + ', ec:' + ecCol + ', ph:' + phCol + ', tilt:' + tiltCol);
    }
  }
  
  var data = {readings: [], moisture: 0, ec: 0, ph: 0, tilt: 0, count: 0};
  
  for (var i = startIdx; i < lines.length; i++) {
    var line = lines[i].trim();
    if (line.length === 0) continue;
    
    var parts = line.split(',');
    if (parts.length >= 4) {
      var m = parseFloat(parts[moistureCol]);
      var e = parseFloat(parts[ecCol]);
      var p = parseFloat(parts[phCol]);
      var t = parseFloat(parts[tiltCol]);
      
      if (i <= startIdx + 2) {
        console.log('[DEBUG] Row ' + i + ': m=' + m + ', e=' + e + ', p=' + p + ', t=' + t);
      }
      
      if (!isNaN(m) && !isNaN(e) && !isNaN(p) && !isNaN(t) &&
          m >= 0 && m <= 100 && e >= 0 && p >= 0 && p <= 14 && t >= -180 && t <= 180) {
        
        data.readings.push({moisture: m, ec: e, ph: p, tilt: t});
        data.moisture += m;
        data.ec += e;
        data.ph += p;
        data.tilt += t;
        data.count++;
      }
    }
  }
  
  if (data.count > 0) {
    data.moisture /= data.count;
    data.ec /= data.count;
    data.ph /= data.count;
    data.tilt /= data.count;
  }
  
  console.log('[DEBUG] parseNodeCSV result: count=' + data.count);
  return data;
}
function onNodeFileSelected(nodeId, event) {
  var files = event.target.files;
  if (files.length === 0) return;
  
  var file = files[0];
  var reader = new FileReader();
  
  reader.onload = function(e) {
    try {
      var csvText = e.target.result;
      var parsedData = parseNodeCSV(csvText);
      
      if (parsedData.count === 0) {
        showToast('Node ' + nodeId + ': No valid data rows found in CSV', 4000);
        return;
      }
      
      uploadedNodeData[nodeId] = parsedData;
      nodeUploadFiles[nodeId] = file.name;
      
      showToast('Node ' + nodeId + ': ' + parsedData.count + ' readings loaded', 2500);
      loadNodes();
      checkAndGenerateUploadPrediction();
      
    } catch (error) {
      showToast('Node ' + nodeId + ': Error parsing CSV - ' + error.message, 4000);
    }
  };
  
  reader.readAsText(file);
}
function clearNodeUpload(nodeId) {
  uploadedNodeData[nodeId] = null;
  nodeUploadFiles[nodeId] = null;
  
  var input = document.getElementById('node-file-input-' + nodeId);
  if (input) input.value = '';
  
  // If a prediction was generated, hide it since data changed
  if (window._uploadPredResult) {
    window._uploadPredResult = null;
    window._predGenerated    = false;
    var avgCard      = document.getElementById('avg-card');
    var nodePredCard = document.getElementById('node-pred-card');
    var nodeRecCard  = document.getElementById('node-rec-card');
    if (avgCard)      avgCard.style.display      = 'none';
    if (nodePredCard) nodePredCard.style.display = 'none';
    if (nodeRecCard)  nodeRecCard.style.display  = 'none';
  }
  showToast('Node ' + nodeId + ' upload cleared', 2000);
  loadNodes();
  checkAndGenerateUploadPrediction();
}
function getNodeUploadStatus(nodeId) {
  if (uploadedNodeData[nodeId] === null) {
    return {label: 'No file', color: 'var(--muted)'};
  }
  return {label: 'Loaded (' + uploadedNodeData[nodeId].count + ' readings)', color: 'var(--green)'};
}
function checkAndGenerateUploadPrediction() {
  // No longer auto-generates — just updates the button state
  updateGenPredButton();
}
function updateGenPredButton() {
  var uploadedCount = 0;
  for (var i = 1; i <= 5; i++) {
    if (uploadedNodeData[i] !== null) uploadedCount++;
  }
  var wrap = document.getElementById('gen-pred-wrap');
  var btn  = document.getElementById('btn-gen-pred');
  var hint = document.getElementById('gen-pred-hint');
  var banner = document.getElementById('upload-success-banner');
  var bannerTitle = document.getElementById('upload-banner-title');
  var bannerSub   = document.getElementById('upload-banner-sub');
  if (uploadedCount > 0) {
    wrap.style.display = 'block';
    banner.style.display = 'block';
    if (uploadedCount === 5) {
      bannerTitle.textContent = 'All 5 Nodes CSV Data Loaded!';
      bannerSub.textContent   = 'Press Generate Overall Prediction to compute results';
    } else {
      bannerTitle.textContent = uploadedCount + ' of 5 Node CSVs Loaded';
      bannerSub.textContent   = 'You can generate a partial prediction now, or upload more nodes first';
    }
    hint.textContent = uploadedCount + ' node' + (uploadedCount > 1 ? 's' : '') + ' loaded — prediction will average across loaded nodes only';
  } else {
    wrap.style.display   = 'none';
    banner.style.display = 'none';
  }
}
function runGeneratePrediction() {
  var uploadedCount = 0;
  for (var i = 1; i <= 5; i++) {
    if (uploadedNodeData[i] !== null) uploadedCount++;
  }
  // If there are uploaded CSVs, use those
  if (uploadedCount > 0) {
    window._predGenerated = true;
    generateUploadPrediction(uploadedCount);
    return;
  }
  // Otherwise use live node averages (if all 5 done)
  if (window._liveNodeAvg) {
    window._predGenerated = true;
    generateLiveNodePrediction(window._liveNodeAvg);
    return;
  }
  showToast('No node data available — upload CSV files or complete node sampling first', 3500);
}
function generateLiveNodePrediction(avg) {
  var avgMoisture = avg.moisture;
  var avgEC       = avg.ec;
  var avgPH       = avg.ph;
  var avgTilt     = avg.tilt;
  var liveDT      = (latestData && latestData.deltaTilt !== undefined) ? parseFloat(latestData.deltaTilt) : 0.0;
  var soil = (avgEC <= 10000.0 && avgPH >= 6.0 && avgPH <= 8.0) ? 'suitable' : 'not_suitable';
  var slope;
  if (liveDT < 0.5 || avgMoisture < 50)                          slope = 'stable';
  else if (liveDT <= 2.0 && avgMoisture >= 50 && avgMoisture < 80) slope = 'pre_failure';
  else if (liveDT > 2.0 && avgMoisture >= 80)                     slope = 'failure_imminent';
  else if (avgMoisture >= 80)                                      slope = 'failure_imminent';
  else                                                              slope = 'pre_failure';
  var slips = 0;
  if (soil === 'suitable' && slope !== 'stable') {
    var nodeDS  = computeNodeDeltaS(avgMoisture, avgTilt);
    var nodeRAR = nodeDS > 0 ? nodeDS / (1.2 * 75000000) : 0;
    slips = nodeDS > 0 ? Math.round((nodeRAR * 1000) / 0.005) : 0;
  }
  window._nodePredResult = { soil: soil, slope: slope, slips: slips,
    moisture: avgMoisture, ec: avgEC, ph: avgPH, tilt: avgTilt, liveDT: liveDT, nodeCount: 5 };
  // Display using the same layout as upload prediction
  var nc = 5;
  var avgCard      = document.getElementById('avg-card');
  var nodePredCard = document.getElementById('node-pred-card');
  avgCard.style.display      = 'block';
  nodePredCard.style.display = 'block';
  var avgLabelEl = document.getElementById('avg-card-label');
  if (avgLabelEl) avgLabelEl.textContent = 'Overall Average — All 5 Live Nodes';
  var labelEl = document.getElementById('node-pred-card-label');
  if (labelEl) labelEl.textContent = 'Overall Prediction — All 5 Live Nodes';
  document.getElementById('avg-moisture').textContent = avgMoisture.toFixed(1) + ' %';
  document.getElementById('avg-ec').textContent       = parseFloat(avgEC).toFixed(0);
  document.getElementById('avg-ph').textContent       = parseFloat(avgPH).toFixed(1);
  document.getElementById('avg-tilt').textContent     = avgTilt.toFixed(2) + '°';
  document.getElementById('avg-note').textContent     = 'Average across all 5 completed live nodes';
  updateMlBadge('node-soil-badge',  soil);
  updateMlBadge('node-slope-badge', slope);
  var slipValEl = document.getElementById('node-slips-value');
  var slipSubEl = document.getElementById('node-slips-sub');
  if (soil !== 'suitable') {
    slipValEl.textContent = '—'; slipValEl.style.color = 'var(--muted)';
    slipSubEl.textContent = 'No computation — soil not suitable for Vetiver';
  } else if (slope === 'stable') {
    slipValEl.textContent = '0'; slipValEl.style.color = 'var(--green)';
    slipSubEl.textContent = 'Slope is stable — no vetiver planting required';
  } else {
    slipValEl.textContent = slips; slipValEl.style.color = slope === 'failure_imminent' ? '#fca5a5' : '#86efac';
    slipSubEl.textContent = 'slips recommended across all 5 nodes';
  }
  var alertHtml = '';
  if (slope === 'failure_imminent')
    alertHtml = '<div class="alert-box alert-danger">Failure Imminent — immediate vetiver deployment needed</div>';
  else if (slope === 'pre_failure')
    alertHtml = '<div class="alert-box alert-warn">Pre-Failure — slope creep detected, plan vetiver planting</div>';
  else
    alertHtml = '<div class="alert-box alert-ok">Slope stable — monitor periodically</div>';
  document.getElementById('node-pred-alert').innerHTML = alertHtml;
  renderNodeEcoRecs(avgMoisture);
  showToast('Prediction generated for 5 live nodes', 2500);
}
function generateUploadPrediction(nodeCount) {
  nodeCount = nodeCount || 5;
  var totalMoisture = 0;
  var totalEC = 0;
  var totalPH = 0;
  var totalTilt = 0;
  var actualCount = 0;
  
  for (var i = 1; i <= 5; i++) {
    var data = uploadedNodeData[i];
    if (data) {
      totalMoisture += data.moisture;
      totalEC += data.ec;
      totalPH += data.ph;
      totalTilt += data.tilt;
      actualCount++;
    }
  }
  
  if (actualCount === 0) return;
  var avgMoisture = totalMoisture / actualCount;
  var avgEC = totalEC / actualCount;
  var avgPH = totalPH / actualCount;
  var avgTilt = totalTilt / actualCount;
  
  var soilSuitable = (avgEC <= 10000.0 && avgPH >= 6.0 && avgPH <= 8.0);
  var uploadSoil = soilSuitable ? 'suitable' : 'not_suitable';
  
  var uploadSlope;
  if (avgTilt < 0.5 || avgMoisture < 50) {
    uploadSlope = 'stable';
  } else if (avgTilt <= 2.0 && avgMoisture >= 50 && avgMoisture < 80) {
    uploadSlope = 'pre_failure';
  } else if (avgTilt > 2.0 && avgMoisture >= 80) {
    uploadSlope = 'failure_imminent';
  } else if (avgMoisture >= 80) {
    uploadSlope = 'failure_imminent';
  } else {
    uploadSlope = 'pre_failure';
  }
  
  var uploadSlips = 0;
  if (soilSuitable && uploadSlope !== 'stable') {
    var uploadDS  = computeNodeDeltaS(avgMoisture, avgTilt);
    var uploadRAR = uploadDS > 0 ? uploadDS / (1.2 * 75000000) : 0;
    uploadSlips = uploadDS > 0 ? Math.round((uploadRAR * 1000) / 0.005) : 0;
  }
  
  window._uploadPredResult = {
    soil: uploadSoil,
    slope: uploadSlope,
    slips: uploadSlips,
    moisture: avgMoisture,
    ec: avgEC,
    ph: avgPH,
    tilt: avgTilt,
    fromUpload: true,
    nodeCount: actualCount
  };
  
  displayUploadPrediction();
  showToast('Prediction generated for ' + actualCount + ' node' + (actualCount > 1 ? 's' : ''), 2500);
}
function displayUploadPrediction() {
  var p = window._uploadPredResult;
  if (!p) return;
  var nc = p.nodeCount || 5;
  var ncLabel = nc === 5 ? 'all 5 nodes' : nc + ' node' + (nc > 1 ? 's' : '');
  
  var nodePredCard = document.getElementById('node-pred-card');
  nodePredCard.style.display = 'block';
  var labelEl = document.getElementById('node-pred-card-label');
  if (labelEl) labelEl.textContent = 'Overall Prediction — ' + (nc === 5 ? 'All 5 Nodes' : nc + ' Node' + (nc > 1 ? 's' : ''));
  
  var avgCard = document.getElementById('avg-card');
  avgCard.style.display = 'block';
  var avgLabelEl = document.getElementById('avg-card-label');
  if (avgLabelEl) avgLabelEl.textContent = 'Overall Average — ' + (nc === 5 ? 'All 5 Nodes' : nc + ' Node' + (nc > 1 ? 's' : ''));
  document.getElementById('avg-moisture').textContent = p.moisture.toFixed(1) + ' %';
  document.getElementById('avg-ec').textContent = parseFloat(p.ec).toFixed(0);
  document.getElementById('avg-ph').textContent = p.ph.toFixed(1);
  document.getElementById('avg-tilt').textContent = p.tilt.toFixed(2) + '°';
  document.getElementById('avg-note').textContent = 'Average across ' + ncLabel + ' (uploaded CSV data)';
  
  updateMlBadge('node-soil-badge', p.soil);
  updateMlBadge('node-slope-badge', p.slope);
  
  var slipValEl = document.getElementById('node-slips-value');
  var slipSubEl = document.getElementById('node-slips-sub');
  
  if (p.soil !== 'suitable') {
    slipValEl.textContent = '—';
    slipValEl.style.color = 'var(--muted)';
    slipSubEl.textContent = 'No computation — soil not suitable for Vetiver';
  } else if (p.slope === 'stable') {
    slipValEl.textContent = '0';
    slipValEl.style.color = 'var(--green)';
    slipSubEl.textContent = 'Slope is stable — no vetiver planting required';
  } else {
    slipValEl.textContent = p.slips;
    slipValEl.style.color = p.slope === 'failure_imminent' ? '#fca5a5' : '#86efac';
    slipSubEl.textContent = 'slips recommended across ' + ncLabel;
  }
  
  var alertHtml = '';
  if (p.slope === 'failure_imminent')
    alertHtml = '<div class="alert-box alert-danger">Failure Imminent — immediate vetiver deployment needed</div>';
  else if (p.slope === 'pre_failure')
    alertHtml = '<div class="alert-box alert-warn">Pre-Failure — slope creep detected, plan vetiver planting</div>';
  else
    alertHtml = '<div class="alert-box alert-ok">Slope stable — monitor periodically</div>';
  
  document.getElementById('node-pred-alert').innerHTML = alertHtml;
  renderNodeEcoRecs(p.moisture);
}
var nodePolling = null;
function renderNodeGrid(data) {
  var grid = document.getElementById('node-grid');
  var html = '';
  var doneCount = 0;
  for (var i = 0; i < data.nodes.length; i++) {
    var n = data.nodes[i];
    if (n.done) doneCount++;
    var pct = Math.round((n.count / 100) * 100);
    var stateClass = n.active ? 'active' : (n.done ? 'done' : '');
    var stateLabel = n.active
      ? 'Recording... (' + n.count + ' / 100)'
      : (n.done ? 'Completed (' + n.count + ' readings)' : 'Not started');
    var readingHtml = '';
    if (n.count > 0) {
      readingHtml =
        '<div class="node-readings">' +
          '<div class="node-reading"><div class="val mo-c">' + n.moisture.toFixed(1) + '%</div><div class="lbl">Moisture</div></div>' +
          '<div class="node-reading"><div class="val ec-c">' + parseFloat(n.ec).toFixed(0) + '</div><div class="lbl">EC uS/cm</div></div>' +
          '<div class="node-reading"><div class="val ph-c">' + n.ph.toFixed(1) + '</div><div class="lbl">pH</div></div>' +
          '<div class="node-reading"><div class="val ti-c">' + n.tilt.toFixed(2) + '°</div><div class="lbl">Tilt Avg</div></div>' +
        '</div>';
    } else {
      readingHtml = '<div style="font-size:.7rem;color:var(--muted);margin-bottom:10px;text-align:center;padding:8px">No readings yet</div>';
    }
    var startBtn = (!n.active)
      ? '<button class="btn-start" onclick="startNode(' + n.id + ')">' + (n.done ? 'Redo' : 'Start') + '</button>'
      : '<button class="btn-start" style="background:#374151;cursor:default" disabled>Recording...</button>';
    var dlBtn = n.done
      ? '<button class="btn-dl-node" onclick="downloadNodeAverage(' + n.id + ')">Download</button>'
      : '';
    var uploadStatus = getNodeUploadStatus(n.id);
    var uploadSection = '';
    
    if (uploadedNodeData[n.id] === null) {
      uploadSection = 
        '<div style="margin-top:8px;border-top:1px solid var(--border);padding-top:8px">' +
          '<div style="font-size:.7rem;color:var(--muted);margin-bottom:4px;font-weight:500">Upload CSV Data</div>' +
          '<input type="file" id="node-file-input-' + n.id + '" accept=".csv,.txt" ' +
            'onchange="onNodeFileSelected(' + n.id + ', event)" ' +
            'style="font-size:.7rem;padding:4px;border:1px solid var(--border);border-radius:4px;width:100%;cursor:pointer;box-sizing:border-box">' +
        '</div>';
    } else {
      var data = uploadedNodeData[n.id];
      uploadSection = 
        '<div style="margin-top:8px;border-top:1px solid var(--border);padding-top:8px">' +
          '<div style="font-size:.7rem;color:var(--green);margin-bottom:6px;font-weight:500">CSV Loaded</div>' +
          '<div style="font-size:.65rem;color:var(--muted);margin-bottom:4px;word-break:break-all">' + nodeUploadFiles[n.id] + '</div>' +
          '<div style="font-size:.65rem;color:var(--muted);margin-bottom:6px">' + data.count + ' readings parsed</div>' +
          '<button onclick="clearNodeUpload(' + n.id + ')" ' +
            'style="width:100%;padding:6px;font-size:.7rem;background:var(--border);border:none;border-radius:4px;cursor:pointer;color:var(--muted)">' +
            'Clear Upload' +
          '</button>' +
        '</div>';
    }
    html +=
      '<div class="node-card ' + stateClass + '">' +
        '<div class="node-title">Node ' + n.id + '</div>' +
        '<div class="node-status">' + stateLabel + '</div>' +
        '<div class="node-progress"><div class="node-bar ' + (n.done ? 'done' : '') + '" style="width:' + pct + '%"></div></div>' +
        readingHtml +
        '<div class="node-btns">' + startBtn + dlBtn + '</div>' +
        uploadSection +
      '</div>';
  }
  grid.innerHTML = html;
  var uploadedCount = 0;
  for (var u = 1; u <= 5; u++) {
    if (uploadedNodeData[u] !== null) uploadedCount++;
  }
  // Button state is managed separately (don't auto-hide banner if prediction was already generated)
  updateGenPredButton();
  document.getElementById('nodes-done-count').textContent = doneCount + ' / 5';
  document.getElementById('nodes-done-bar').style.width = (doneCount / 5 * 100) + '%';
  var avgCard      = document.getElementById('avg-card');
  var nodePredCard = document.getElementById('node-pred-card');
  // Only auto-show live-node prediction when ALL 5 are done AND no upload prediction is active
  if (data.avg && doneCount === 5 && !window._uploadPredResult) {
    // Show the generate button for live nodes too (will use _liveNodeAvg)
    window._liveNodeAvg = {
      moisture: data.avg.moisture,
      ec: data.avg.ec,
      ph: data.avg.ph,
      tilt: data.avg.tilt,
      nodeCount: 5
    };
    // Don't auto-show prediction cards — user must press Generate
    var genWrap = document.getElementById('gen-pred-wrap');
    if (genWrap && uploadedCount === 0) {
      genWrap.style.display = 'block';
      var hint = document.getElementById('gen-pred-hint');
      if (hint) hint.textContent = '5 live nodes completed — press Generate to compute prediction';
    }
    // If prediction was previously generated this session, keep showing it
    if (!window._predGenerated) {
      avgCard.style.display      = 'none';
      nodePredCard.style.display = 'none';
    }
  } else if (!window._uploadPredResult) {
    avgCard.style.display      = 'none';
    nodePredCard.style.display = 'none';
    var nodeRecCard = document.getElementById('node-rec-card');
    if (nodeRecCard) nodeRecCard.style.display = 'none';
    window._nodePredResult = null;
  }
  var anyActive = data.nodes.some(function(n){ return n.active; });
  if (!anyActive && doneCount === 5) stopNodePolling();
}

function computeNodeDeltaS(moisture, tilt) {
  if (moisture <= 50.0) return 0.0;
  var sinTiltMax = Math.sin(2.0 * Math.PI / 180.0);
  var sinTilt    = Math.sin(Math.abs(tilt || 0) * Math.PI / 180.0);
  return ((moisture - 50.0) / 30.0) * (sinTilt / sinTiltMax) * 20000.0;
}

function renderNodeEcoRecs(avgMoisture) {
  var card        = document.getElementById('node-rec-card');
  var noAction    = document.getElementById('node-rec-no-action');
  var recContent  = document.getElementById('node-rec-content');
  if (!card) return;
  card.style.display = 'block';

  var deltaS = computeNodeDeltaS(avgMoisture, (window._nodePredResult || window._uploadPredResult || {tilt:0}).tilt || 0);
  if (deltaS <= 0) {
    noAction.style.display   = 'block';
    recContent.style.display = 'none';
    return;
  }
  noAction.style.display   = 'none';
  recContent.style.display = 'block';

  document.getElementById('node-rec-delta-s').textContent  = deltaS.toFixed(4);
  document.getElementById('node-rec-moisture').textContent = avgMoisture.toFixed(1) + '%';

  var TR = 75000000.0, COEFF = 1.2, A_ROOT = 0.005;
  var DS_TERR = 37500.0, DS_RHA = 18500.0, DS_COCO = 15000.0;
  var combos = [
    { label: 'Vetiver Only',               ds: 0,                    key: 'vetiver' },
    { label: 'Bench Terracing + Vetiver',  ds: DS_TERR,              key: 'terracing' },
    { label: 'RHA + Vetiver',              ds: DS_RHA,               key: 'rha' },
    { label: 'Coco-Coir + Vetiver',        ds: DS_COCO,              key: 'coco' },
    { label: 'Terracing + RHA + Vetiver',  ds: DS_TERR + DS_RHA,     key: 'terracing_rha' },
    { label: 'Terracing + Coco + Vetiver', ds: DS_TERR + DS_COCO,    key: 'terracing_coco' },
    { label: 'RHA + Coco + Vetiver',       ds: DS_RHA  + DS_COCO,    key: 'rha_coco' }
  ];

  var rows = combos.map(function(c) {
    var dsNet   = deltaS - c.ds;
    var rar     = dsNet > 0 ? dsNet / (COEFF * TR)   : 0;
    var density = dsNet > 0 ? (rar * 1000) / A_ROOT  : 0;
    return { label: c.label, key: c.key, dsAdded: c.ds, dsNet: dsNet, rar: rar, density: Math.round(density) };
  });

  var bestDensity = Infinity, bestIdxNode = -1;
  rows.forEach(function(r, i){ if (r.density > 0 && r.density < bestDensity){ bestDensity = r.density; bestIdxNode = i; } });
  var tbody = document.getElementById('node-rec-tbody');
  tbody.innerHTML = rows.map(function(r, i) {
    var isBest   = (i === bestIdxNode);
    var isActive = (r.dsAdded > 0);  // has engineering technique
    var rowClass = isActive ? ' class="rec-clickable"' : '';
    var clickAttr = isActive ? ' onclick="openInterventionModal(\'' + r.key + '\')"' : '';
    var style    = isBest ? ' style="background:rgba(34,197,94,.08);font-weight:500"' : '';
    var dsNetStr = r.dsNet > 0 ? r.dsNet.toFixed(4) : '0 (covered)';
    var labelCell = r.label + (isBest ? ' &#9650;' : '') + (isActive ? ' <span style="font-size:.58rem;color:var(--accent);margin-left:4px">▶ Steps</span>' : '');
    return '<tr' + rowClass + style + clickAttr + '>' +
      '<td>' + labelCell + '</td>' +
      '<td>' + r.dsAdded.toFixed(1) + '</td>' +
      '<td>' + dsNetStr + '</td>' +
      '<td>' + r.rar.toFixed(6) + '</td>' +
      '<td>' + r.density + '</td>' +
    '</tr>';
  }).join('');
}

// =========================================================================
// INTERVENTION STEPS DATA + MODAL
// =========================================================================
var INTERVENTION_STEPS = {
  'terracing': {
    title: 'Bench Terracing',
    ref: 'Wei et al. (2018) — Effects of terracing on soil moisture retention in dry hilly catchments.\nWang et al. (2023) — Effect of terracing on soil moisture of slope farmland.\nAdded Shear Strength: +37,500 Pa',
    steps: [
      { num: 1, text: '<strong>Site survey & layout</strong> — Mark contour lines across the slope using a level or laser instrument. Spacing between terrace benches depends on slope gradient (typically 2–5 m vertical interval).' },
      { num: 2, text: '<strong>Excavate the bench cut</strong> — Using hand tools or a small excavator, cut horizontally into the slope along the marked contour. The bench width should be at least 1 m to allow planting and water collection.' },
      { num: 3, text: '<strong>Build the riser (back wall)</strong> — Compact the excavated material to form a firm vertical or slightly backward-sloping riser at the outer edge of each bench. Slope the riser at 70–80° to prevent collapse.' },
      { num: 4, text: '<strong>Shape the bench surface</strong> — Level the bench surface with a slight inward slope (2–3%) toward the hillside to retain rainfall and reduce runoff velocity.' },
      { num: 5, text: '<strong>Stabilise the riser face</strong> — Plant vetiver grass slips along the top and front of each riser immediately after construction to bind the soil and prevent erosion of the newly formed face.' },
      { num: 6, text: '<strong>Install drainage outlets</strong> — Place grassed waterways or small drains at the ends of each bench to safely discharge any excess water and prevent saturation behind the terrace.' },
      { num: 7, text: '<strong>Monitor and maintain</strong> — Inspect after every rainfall event for the first season. Repair any slumping, fill cracks, and re-plant any gaps in the vetiver cover immediately.' }
    ]
  },
  'rha': {
    title: 'Rice Husk Ash (RHA) Amendment',
    ref: 'Panchal & Shrivastava (2024) — Investigating the impact of vetiver grass composite with rice husk ash on soil shear strength.\nAdded Shear Strength: +18,500 Pa',
    steps: [
      { num: 1, text: '<strong>Source and process RHA</strong> — Use rice husk ash that has been burnt at 600–700 °C and cooled. Sieve to remove coarse particles (use material passing a 4.75 mm sieve). Store dry until use.' },
      { num: 2, text: '<strong>Calculate application rate</strong> — For slope stabilisation, an RHA content of 10–15% by dry weight of soil is the effective range identified in literature. For a 1 m² area to 15 cm depth, estimate the soil mass and compute RHA quantity accordingly.' },
      { num: 3, text: '<strong>Loosen the topsoil</strong> — Using a hand rake or rotary cultivator, loosen the slope topsoil to 15 cm depth without disturbing the subsoil structure. Remove stones and debris.' },
      { num: 4, text: '<strong>Mix RHA into loosened soil</strong> — Spread the calculated RHA evenly over the loosened surface. Blend thoroughly by turning and mixing to achieve uniform distribution through the 15 cm depth.' },
      { num: 5, text: '<strong>Compact lightly and irrigate</strong> — Firm the amended soil to remove large air voids. Apply water to initiate the pozzolanic reaction between silica in the ash and soil calcium hydroxide. Keep moist for 3–5 days.' },
      { num: 6, text: '<strong>Plant vetiver slips</strong> — Plant vetiver grass immediately after RHA incorporation to utilise the improved soil strength and provide additional root reinforcement as the grass establishes.' },
      { num: 7, text: '<strong>Curing period</strong> — The pozzolanic bond develops over 7–28 days. Restrict foot traffic and avoid overwatering during this period. Monitor for cracking or shrinkage.' }
    ]
  },
  'coco': {
    title: 'Coco-Coir Geotextile',
    ref: 'Bawadi et al. (2026) — Analysis performance of coconut coir as natural geotextiles.\nAdded Shear Strength: +15,000 Pa',
    steps: [
      { num: 1, text: '<strong>Prepare the slope surface</strong> — Remove loose stones, roots, and debris from the slope. Rake the surface smooth. Fill any rills or gullies with compacted soil and tamp firmly.' },
      { num: 2, text: '<strong>Cut coir mat to size</strong> — Unroll the coco-coir geotextile mat and cut panels to suit the slope dimensions. Overlap adjacent panels by at least 15 cm along both horizontal and vertical joints.' },
      { num: 3, text: '<strong>Anchor at the top</strong> — Dig a 15–20 cm deep anchor trench at the top of the slope. Fold the top edge of the mat into the trench and backfill firmly to lock the mat in place.' },
      { num: 4, text: '<strong>Unroll downslope and pin</strong> — Unroll the mat down the slope, keeping it taut and in contact with the soil surface. Pin with biodegradable wooden stakes or U-shaped wire pegs at 1 m intervals across and every 2 m downslope.' },
      { num: 5, text: '<strong>Secure at the base</strong> — Fold the bottom edge under and peg or bury to prevent the mat from lifting during heavy rainfall or surface flow.' },
      { num: 6, text: '<strong>Plant vetiver through the mat</strong> — Make planting holes through the mat at the specified spacing. Insert vetiver slips, ensuring roots make firm contact with the underlying soil. Water in immediately.' },
      { num: 7, text: '<strong>Monitor establishment</strong> — Check that the mat remains in contact with the soil after the first rainfall. Re-pin any lifted sections. The coir mat biodegrades over 2–5 years, by which time vetiver roots will have fully reinforced the slope.' }
    ]
  },
  'terracing_rha': {
    title: 'Bench Terracing + Rice Husk Ash + Vetiver',
    ref: 'Combined intervention: Terracing (+37,500 Pa) + RHA (+18,500 Pa).\nFollow both individual protocols in sequence.',
    steps: [
      { num: 1, text: '<strong>Complete Bench Terracing first</strong> — Carry out all terracing steps (site survey, bench excavation, riser construction, drainage installation) before applying RHA. The flat bench surface is needed for uniform RHA mixing.' },
      { num: 2, text: '<strong>Allow terrace to settle (3–7 days)</strong> — Let the newly formed bench surfaces compact under their own weight and any rainfall. Inspect and repair any settling before proceeding.' },
      { num: 3, text: '<strong>Apply RHA amendment to bench surfaces</strong> — Loosen the bench topsoil to 15 cm, calculate and apply RHA at 10–15% by dry weight, mix thoroughly, and compact lightly as per the RHA protocol.' },
      { num: 4, text: '<strong>Irrigate to start pozzolanic reaction</strong> — Water the amended bench surfaces and allow 3–5 days of curing. Avoid overwatering to prevent RHA wash-off.' },
      { num: 5, text: '<strong>Plant vetiver along risers and benches</strong> — Plant vetiver slips in rows along the top of each riser and across the bench surface at the recommended spacing.' },
      { num: 6, text: '<strong>Install drainage</strong> — Ensure grassed waterways are in place before the first rain event after construction.' },
      { num: 7, text: '<strong>Monitor both interventions</strong> — Check terrace riser integrity and RHA amendment performance after each significant rainfall for the first two seasons.' }
    ]
  },
  'terracing_coco': {
    title: 'Bench Terracing + Coco-Coir Geotextile + Vetiver',
    ref: 'Combined intervention: Terracing (+37,500 Pa) + Coco-Coir (+15,000 Pa).\nFollow both individual protocols in sequence.',
    steps: [
      { num: 1, text: '<strong>Complete Bench Terracing</strong> — Excavate, form, and compact all bench risers and surfaces. Install drainage outlets before applying the geotextile.' },
      { num: 2, text: '<strong>Allow surface to firm up (2–3 days)</strong> — Let the bench surfaces settle. Remove any loose material that might prevent the geotextile from lying flat.' },
      { num: 3, text: '<strong>Apply coco-coir mats to riser faces</strong> — Cut coir mat panels to fit each riser face. Anchor at the top of each riser, unroll down the face, and pin securely at 1 m intervals.' },
      { num: 4, text: '<strong>Cover bench surfaces with coir mat</strong> — Lay an additional mat strip across each bench surface, overlapping 15 cm with the riser mat. Anchor and pin.' },
      { num: 5, text: '<strong>Plant vetiver through the mat</strong> — Make planting holes at the specified spacing and insert vetiver slips through both the riser and bench mats, ensuring firm root contact with soil.' },
      { num: 6, text: '<strong>Water and establish</strong> — Irrigate after planting. Inspect pins and anchorage after the first rain event.' },
      { num: 7, text: '<strong>Long-term monitoring</strong> — Coir mat provides immediate erosion control while vetiver roots develop. Inspect quarterly and replace any damaged mat sections during the first year.' }
    ]
  },
  'rha_coco': {
    title: 'Rice Husk Ash + Coco-Coir Geotextile + Vetiver',
    ref: 'Combined intervention: RHA (+18,500 Pa) + Coco-Coir (+15,000 Pa).\nFollow both individual protocols in sequence.',
    steps: [
      { num: 1, text: '<strong>Prepare slope surface</strong> — Remove debris and rake smooth. Fill rills and gullies with compacted soil.' },
      { num: 2, text: '<strong>Apply RHA to topsoil</strong> — Loosen topsoil to 15 cm, apply RHA at 10–15% by dry weight, mix thoroughly, compact lightly, and irrigate to start curing (3–5 days).' },
      { num: 3, text: '<strong>Allow RHA curing before matting</strong> — Wait at least 3 days after RHA application before placing the geotextile, so the amended surface firms up slightly.' },
      { num: 4, text: '<strong>Anchor coir mat at top of slope</strong> — Dig anchor trench, fold top edge into it, and backfill firmly.' },
      { num: 5, text: '<strong>Unroll and pin coir mat over amended soil</strong> — Roll the mat down the slope over the RHA-amended surface. Pin at 1 m horizontal intervals and 2 m downslope intervals. Overlap adjacent panels 15 cm.' },
      { num: 6, text: '<strong>Plant vetiver through the mat</strong> — Create planting holes at specified spacing, insert vetiver slips through the mat into the amended soil below, and water in.' },
      { num: 7, text: '<strong>Monitor curing and establishment</strong> — RHA continues developing strength over 7–28 days. Coir mat protects the surface during this curing period. Inspect after each rainfall and re-pin lifted mat sections.' }
    ]
  }
};

function openInterventionModal(key) {
  var data = INTERVENTION_STEPS[key];
  if (!data) return;
  document.getElementById('int-modal-title').textContent = data.title;
  document.getElementById('int-modal-ref').textContent   = data.ref;
  var stepsHtml = data.steps.map(function(s) {
    return '<div class="int-step">' +
      '<div class="int-step-num">' + s.num + '</div>' +
      '<div class="int-step-text">' + s.text + '</div>' +
    '</div>';
  }).join('');
  document.getElementById('int-modal-steps').innerHTML = stepsHtml;
  document.getElementById('intervention-modal').classList.add('open');
}
function closeInterventionModal() {
  document.getElementById('intervention-modal').classList.remove('open');
}
// Close modal when clicking backdrop
document.getElementById('intervention-modal').addEventListener('click', function(e) {
  if (e.target === this) closeInterventionModal();
});

async function loadNodes() {
  try {
    var r = await fetch('/api/nodes');
    var d = await r.json();
    renderNodeGrid(d);
  } catch(e) {}
}
async function startNode(id) {
  try {
    var r = await fetch('/api/node/start?id=' + id, { method: 'POST' });
    var d = await r.json();
    if (d.ok) {
      showToast('Node ' + id + ' started — collecting 100 readings');
      loadNodes();
      startNodePolling();
    } else {
      showToast('Error: ' + d.msg, 3500);
    }
  } catch(e) { showToast('Request failed'); }
}
async function resetAllNodes() {
  try {
    await fetch('/api/node/reset', { method: 'POST' });
    showToast('All nodes reset — refreshing...');
    loadNodes();
    stopNodePolling();
    setTimeout(function() {
      location.reload();
    }, 1500);
  } catch(e) { showToast('Request failed'); }
}
function downloadNodeAverage(id) {
  window.location.href = '/download/node?id=' + id;
}
function downloadNodePrediction() {
  var p = window._uploadPredResult || window._nodePredResult;
  if (!p) { showToast('No prediction available yet'); return; }
  var lines = [];
  lines.push('VetiverTrack Node Prediction');
  lines.push('Generated,' + new Date().toLocaleString());
  lines.push('');
  lines.push('Field,Value');
  lines.push('Soil Suitability,' + p.soil.replace(/_/g,' '));
  lines.push('Slope Risk,'       + p.slope.replace(/_/g,' '));
  lines.push('Vetiver Slips,'    + (p.soil !== 'suitable' ? 'N/A (not suitable)' : p.slope === 'stable' ? '0' : p.slips + ' slips'));
  lines.push('Avg Moisture,' + p.moisture.toFixed(1) + '%');
  lines.push('Avg EC,'       + parseFloat(p.ec).toFixed(0) + ' uS/cm');
  lines.push('Avg pH,'       + p.ph.toFixed(1));
  lines.push('Avg Delta Tilt,' + p.tilt.toFixed(2) + ' deg');
  if (p.liveDT !== undefined) {
    lines.push('Live Delta Tilt,' + p.liveDT.toFixed(2) + ' deg');
  }
  var csv  = lines.join('\r\n');
  var blob = new Blob([csv], {type: 'text/csv'});
  var url  = URL.createObjectURL(blob);
  var a    = document.createElement('a');
  a.href = url; a.download = 'vetivertrack_node_prediction.csv';
  document.body.appendChild(a); a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
  showToast('Node prediction downloaded');
}
function startNodePolling() { if (!nodePolling) nodePolling = setInterval(loadNodes, 2000); }
function stopNodePolling()  { if (nodePolling) { clearInterval(nodePolling); nodePolling = null; } }
loadNodes();
// =========================================================================
// CALIBRATE + RESET
// =========================================================================
async function doCalibrate() {
  showToast('Sending recalibrate command...', 3000);
  try {
    var r = await fetch('/api/calibrate', { method: 'POST' });
    var d = await r.json();
    showToast(d.ok ? 'Recalibrate command sent!' : 'Command failed — is initiator online?');
  } catch(e) { showToast('Request failed'); }
}
async function doResetLog() {
  if (!confirm('Reset all stored log data? This cannot be undone.')) return;
  await fetch('/api/reset', {method:'POST'}).catch(function(){});
  historyData = []; latestData = null; autoDownloadDone = false;
  document.getElementById('log-badge').textContent = '0 readings';
  document.getElementById('content').innerHTML = '<div id="no-data">Log reset. Waiting for new data...</div>';
  showToast('Log cleared. New data will appear shortly.');
}
// =========================================================================
// INIT + POLLING
// =========================================================================
async function loadPrediction() {
  try {
    var r = await fetch('/api/prediction');
    var p = await r.json();
    var predStatus = document.getElementById('pred-status');
    var predResult = document.getElementById('pred-result');
    var progress   = document.getElementById('reading-progress');
    if (!p.locked) {
      if (predStatus) predStatus.style.display = 'block';
      if (predResult) predResult.style.display = 'none';
      if (progress)   progress.textContent = p.count + ' / ' + p.target;
    } else {
      if (predStatus) predStatus.style.display = 'none';
      if (predResult) predResult.style.display = 'block';
      updateMlBadge('pred-soil-badge',  p.soil);
      updateMlBadge('pred-slope-badge', p.slope);
      document.getElementById('pred-slips').textContent = p.slips >= 0 ? p.slips : '—';
      if (progress) progress.textContent = 'Updated every ' + p.target + ' readings';
    }
  } catch(e) {}
}
// =========================================================================
// RECOMMENDATIONS
// =========================================================================
async function loadRecommendations() {
  try {
    var r = await fetch('/api/recommendations');
    var d = await r.json();
    var waiting   = document.getElementById('rec-waiting');
    var noAction  = document.getElementById('rec-no-action');
    var content   = document.getElementById('rec-content');
    if (!d.ready) {
      if (waiting)  waiting.style.display  = 'block';
      if (noAction) noAction.style.display = 'none';
      if (content)  content.style.display  = 'none';
      return;
    }
    if (waiting) waiting.style.display = 'none';
    // deltaS == 0 means moisture <= 50%, no action needed
    if (d.deltaS <= 0) {
      if (noAction) noAction.style.display = 'block';
      if (content)  content.style.display  = 'none';
      return;
    }
    if (noAction) noAction.style.display = 'none';
    if (content)  content.style.display  = 'block';
    document.getElementById('rec-delta-s').textContent  = parseFloat(d.deltaS).toFixed(4);
    // Find locked moisture from prediction endpoint (already loaded)
    try {
      var pr = await fetch('/api/prediction');
      var pp = await pr.json();
      var mEl = document.getElementById('rec-moisture');
      if (mEl) mEl.textContent = pp.locked ? parseFloat(pp.moisture).toFixed(1) + '%' : '—';
    } catch(e) {}
    // Find the combo with lowest density > 0 (best = fewest slips while still needing vetiver)
    var bestIdx = -1;
    var bestDensity = Infinity;
    for (var i = 0; i < d.combos.length; i++) {
      if (d.combos[i].density > 0 && d.combos[i].density < bestDensity) {
        bestDensity = d.combos[i].density;
        bestIdx = i;
      }
    }
    var tbody = document.getElementById('rec-tbody');
    if (!tbody) return;
    tbody.innerHTML = '';
    for (var i = 0; i < d.combos.length; i++) {
      var c = d.combos[i];
      var isBest   = (i === bestIdx);
      var isActive = (c.dsAdded > 0);
      var tr = document.createElement('tr');
      if (isBest) tr.classList.add('rec-best');
      if (isActive) {
        tr.classList.add('rec-clickable');
        tr.setAttribute('onclick', 'openInterventionModal("' + getRechKey(c.label) + '")');
      }
      var densityText = c.density <= 0
        ? '<span style="color:var(--green);font-weight:700">0 ✓ Sufficient</span>'
        : parseFloat(c.density).toFixed(1);
      var netDsText = c.dsNet <= 0
        ? '<span style="color:var(--green)">' + parseFloat(c.dsNet).toFixed(4) + '</span>'
        : parseFloat(c.dsNet).toFixed(4);
      var labelExtra = isActive ? ' <span style="font-size:.58rem;color:var(--accent);margin-left:4px">▶ Steps</span>' : '';
      tr.innerHTML =
        '<td>' + c.label + (isBest ? '<span class="rec-best-badge">BEST</span>' : '') + labelExtra + '</td>' +
        '<td>' + parseFloat(c.dsAdded).toFixed(1) + '</td>' +
        '<td>' + netDsText + '</td>' +
        '<td>' + parseFloat(c.rar).toFixed(6) + '</td>' +
        '<td>' + densityText + '</td>';
      tbody.appendChild(tr);
    }
  } catch(e) {}
}
// =========================================================================
// LABEL → INTERVENTION KEY MAP (for dashboard rec table)
// =========================================================================
function getRechKey(label) {
  var map = {
    'Vetiver Only':                'vetiver',
    'Terracing + Vetiver':         'terracing',
    'RHA + Vetiver':               'rha',
    'Coco-Coir + Vetiver':         'coco',
    'Terracing + RHA + Vetiver':   'terracing_rha',
    'Terracing + Coco + Vetiver':  'terracing_coco',
    'RHA + Coco + Vetiver':        'rha_coco'
  };
  return map[label] || 'vetiver';
}
loadData();
loadHistory().then(function(){ renderCharts(); });
loadPrediction();
loadRecommendations();
setInterval(function(){ loadData(); loadHistory(); }, 5000);
setInterval(loadPrediction, 5000);
setInterval(loadRecommendations, 10000);
</script>
</body>
</html>
)rawhtml";
// =============================================================================
// CAPTIVE PORTAL HANDLERS
// =============================================================================
void sendJsRedirectPage() {
  String html =
    "<!DOCTYPE html><html><head>"
    "<meta charset='UTF-8'>"
    "<meta http-equiv='refresh' content='0;url=http://192.168.4.1/'>"
    "<title>VetiverTrack</title>"
    "</head><body>"
    "<script>window.location.replace('http://192.168.4.1/');<\/script>"
    "</body></html>";
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(200, "text/html", html);
}
void handleWindowsConnectTest() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(302, "text/plain", "");
}
void handleWindowsNCSI()     { server.sendHeader("Location","http://192.168.4.1/",true); server.send(302,"text/plain",""); }
void handleWindowsRedirect() { server.sendHeader("Location","http://192.168.4.1/",true); server.send(302,"text/plain",""); }
void handleAndroidPortal()   { server.sendHeader("Location","http://192.168.4.1/",true); server.send(302,"text/plain",""); }
void handleApplePortal()     { sendJsRedirectPage(); }
void handleCaptivePortal() {
  String ua = server.header("User-Agent");
  if (ua.indexOf("iPhone") >= 0 || ua.indexOf("iPad") >= 0 || ua.indexOf("Darwin") >= 0)
    sendJsRedirectPage();
  else {
    server.sendHeader("Location","http://192.168.4.1/",true);
    server.send(302,"text/plain","");
  }
}
void handleRoot() { server.send_P(200, "text/html", DASHBOARD); }
// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- VetiverTrack | Responder ---\n");
  PicoSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("UART2 to Pico W started");
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP started: %s\n", AP_SSID);
  Serial.printf("AP MAC:  %s\n", WiFi.softAPmacAddress().c_str());
  Serial.printf("STA MAC: %s\n", WiFi.macAddress().c_str());
  Serial.printf("Channel: %d\n", WiFi.channel());
  Serial.printf("IP: %s\n", AP_IP.toString().c_str());
  if (esp_now_init() != ESP_OK) {
    Serial.println("CRITICAL: ESP-NOW init failed. Halting.");
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnCmdSent);
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, initiatorMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    Serial.println("WARNING: Failed to register initiator as peer");
  else
    Serial.println("Initiator registered as peer");
  Serial.println("ESP-NOW ready. Waiting for data...");
  dnsServer.start(DNS_PORT, "*", AP_IP);
  Serial.println("DNS server started (captive portal)");
  server.on("/",                       HTTP_GET,  handleRoot);
  server.on("/api/data",               HTTP_GET,  handleApiData);
  server.on("/api/pause",              HTTP_POST, handleApiPause);
  server.on("/history",                HTTP_GET,  handleHistory);
  server.on("/download",               HTTP_GET,  handleDownload);
  server.on("/download/prediction",    HTTP_GET,  handleDownloadPrediction);
  server.on("/download/both",          HTTP_GET,  handleDownloadBoth);
  server.on("/download/node",          HTTP_GET,  handleDownloadNode);
  server.on("/download/node/csv",      HTTP_GET,  handleDownloadNodeCSV);
  server.on("/api/reset",              HTTP_POST, handleApiReset);
  server.on("/api/prediction",         HTTP_GET,  handleApiPrediction);
  server.on("/api/recommendations",    HTTP_GET,  handleApiRecommendations);
  server.on("/api/calibrate",          HTTP_POST, handleApiCalibrate);
  server.on("/api/nodes",              HTTP_GET,  handleApiNodes);
  server.on("/api/node/start",         HTTP_POST, handleNodeStart);
  server.on("/api/node/reset",         HTTP_POST, handleNodeReset);
  server.on("/connecttest.txt",        HTTP_GET, handleWindowsConnectTest);
  server.on("/ncsi.txt",               HTTP_GET, handleWindowsNCSI);
  server.on("/redirect",               HTTP_GET, handleWindowsRedirect);
  server.on("/success.txt",            HTTP_GET, handleWindowsRedirect);
  server.on("/generate_204",           HTTP_GET, handleAndroidPortal);
  server.on("/gen_204",                HTTP_GET, handleAndroidPortal);
  server.on("/portal",                 HTTP_GET, handleAndroidPortal);
  server.on("/mobile/status.php",      HTTP_GET, handleAndroidPortal);
  server.on("/hotspot-detect.html",    HTTP_GET, handleApplePortal);
  server.on("/library/test/success.html", HTTP_GET, handleApplePortal);
  server.on("/bag/api.php",            HTTP_GET, handleApplePortal);
  server.on("/captive.apple.com",      HTTP_GET, handleApplePortal);
  server.onNotFound(handleCaptivePortal);
  const char* headerKeys[] = {"User-Agent"};
  server.collectHeaders(headerKeys, 1);
  analogReadResolution(12);
  pinMode(BAT_PIN, INPUT);
  responderBatPct = readResponderBattery();
  Serial.printf("Responder battery: %.1f%%\n", responderBatPct);
  server.begin();
  Serial.println("Web server started on port 80");
  Serial.println("Dashboard: http://192.168.4.1");
}
// =============================================================================
// MAIN LOOP
// =============================================================================
unsigned long lastBatRead = 0;
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  readFromPico();
  if (millis() - lastBatRead >= 30000 || lastBatRead == 0) {
    responderBatPct = readResponderBattery();
    lastBatRead = millis();
  }
  if (!firstPacket && (millis() - lastPacketTime > 60000)) {
    Serial.println("[WARN] No data received for 60 seconds");
    lastPacketTime = millis();
  }
  delay(10);
}
