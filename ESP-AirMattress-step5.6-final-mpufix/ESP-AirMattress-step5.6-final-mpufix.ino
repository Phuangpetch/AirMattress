#define BLYNK_TEMPLATE_ID "TMPL6KBKEpKD2"
#define BLYNK_TEMPLATE_NAME "AirMattress"
#define BLYNK_AUTH_TOKEN "mj-Onv21agggBCiZelysK80-oVJ_AUz3"

#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define WIFI_SSID     "internet"
#define WIFI_PASSWORD "123456789"

#define UNO_RX_PIN D6
#define UNO_TX_PIN D5

#define VPIN_PUMP1  V1
#define VPIN_PUMP2  V2
#define VPIN_PAUSE  V3
#define VPIN_AUTO   V4
#define VPIN_MANUAL V5
#define VPIN_STOP   V6

#define DEVICE_ID "AIR-MATTRESS-01"
#define GAS_URL "https://script.google.com/macros/s/AKfycbwUInzOaTEHlkcxSWpXKOnEvazzZnGF33YL2pFs5dFeqWz1_IZ-Rr61HDfI_JJGnOsgKw/exec"

#define SDA_PIN D2
#define SCL_PIN D1

#define SERIAL_BAUD           115200
#define UNO_BAUD               9600
#define STATUS_REQ_MS         10000UL
#define PRINT_MS               1000UL
#define CMD_GAP_MS              150UL
#define BLYNK_CMD_COOLDOWN_MS   700UL
#define STATUS_HOLD_MS         1500UL
#define UNO_OFFLINE_MS        20000UL
#define UNO_RX_GRACE_MS        8000UL
#define UNO_MISS_LIMIT            3
#define NTP_RESYNC_MS         60000UL

#define BOOT_HTTP_HOLD_MS     10000UL
#define USER_IDLE_UPLOAD_MS   20000UL

#define PUMP_LOG_QUEUE_SIZE         30
#define LOG_SEND_MIN_GAP_MS       250UL
#define LOG_RETRY_MS             2000UL
#define CMD_CONFIRM_SUPPRESS_MS  4000UL

#define MPU_SAMPLE_MS             200UL
#define MPU_POSTURE_HOLD_MS      2000UL
#define MPU_UPLOAD_MS           10000UL
#define MPU_RETRY_MS             2000UL
#define MPU_LOG_MIN_GAP_MS        250UL
#define MPU_MOTION_GYRO_SUM_THRESHOLD 0.60f
#define MPU_ACC_MAG_MIN           7.5f
#define MPU_ACC_MAG_MAX          11.8f
#define MPU_SUPINE_ROLL_MIN      -5.0f
#define MPU_SUPINE_ROLL_MAX       5.0f
#define MPU_SUPINE_PITCH_MIN    -30.0f
#define MPU_SUPINE_PITCH_MAX     20.0f
#define MPU_LEFT_PITCH_MIN       40.0f
#define MPU_LEFT_PITCH_MAX       89.0f
#define MPU_LEFT_ROLL_MIN      -178.0f
#define MPU_LEFT_ROLL_MAX        -3.0f
#define MPU_RIGHT_PITCH_MIN     -89.0f
#define MPU_RIGHT_PITCH_MAX     -35.0f
#define MPU_RIGHT_ROLL_MIN      120.0f
#define MPU_RIGHT_ROLL_MAX      179.0f

#define SRC_BLYNK "BLYNK"
#define SRC_ESP   "ESP"
#define SRC_BLYNK_CMD "BLYNK_CMD"

#define ACT_SINGLE_P1   "SINGLE_P1"
#define ACT_SINGLE_P2   "SINGLE_P2"
#define ACT_SINGLE_OFF  "SINGLE_OFF"
#define ACT_MANUAL_ON   "MANUAL_ON"
#define ACT_MANUAL_OFF  "MANUAL_OFF"
#define ACT_AUTO_ON     "AUTO_ON"
#define ACT_AUTO_OFF    "AUTO_OFF"
#define ACT_PAUSE_ON    "PAUSE_ON"
#define ACT_PAUSE_OFF   "PAUSE_OFF"
#define ACT_STOP_ON     "STOP_ON"
#define ACT_STOP_OFF    "STOP_OFF"
#define ACT_STATUS_REQ  "STATUS_REQ"

SoftwareSerial unoSer(UNO_RX_PIN, UNO_TX_PIN);
Adafruit_MPU6050 mpu;

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASSWORD;

// -----------------------------------------------------------------------------
// Real state from UNO
// -----------------------------------------------------------------------------
String stateSource = "ESP";
String stateEvent  = "BOOT";
String stateMode   = "HOME";
String lcdLine1    = "ESP READY";
String lcdLine2    = "WAIT UNO";

bool stateP1 = false;
bool stateP2 = false;
bool statePause = false;
bool stateStop = false;
bool unoOnline = false;

// Last command sent from ESP/Blynk
String lastCmdSource = "ESP";
String lastCmdAction = "BOOT";

// Pending command echo suppression
String pendingCmdEvent = "";
unsigned long pendingCmdMs = 0;

// Serial receive
String rxLine = "";

// Timing
unsigned long lastStatusReqMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastCmdMs = 0;
unsigned long lastUnoStateMs = 0;
unsigned long lastUnoRxMs = 0;
unsigned long lastNtpSyncMs = 0;
unsigned long bootMs = 0;
unsigned long lastUserActionMs = 0;
int unoMissCount = 0;
unsigned long lastBlynkCmdMs = 0;
unsigned long suppressBlynkUntilMs = 0;
int lastSentSecond = -1;

// Shared HTTP timing
unsigned long lastHttpSendMs = 0;


// Cached UI values to avoid redundant Blynk writes
bool uiSentInit = false;
bool uiStopSent = false;
bool uiPauseSent = false;
bool uiAutoSent = false;
bool uiManualSent = false;
bool uiPump1Sent = false;
bool uiPump2Sent = false;

void virtualWriteIfChanged(int pin, bool value, bool &cacheRef)
{
  if (!uiSentInit || cacheRef != value)
  {
    Blynk.virtualWrite(pin, value ? 1 : 0);
    cacheRef = value;
  }
}

// Blynk duplicate guard
String lastBlynkAct = "";

// Compact serial state guards
String lastPrintedEvent = "";
String lastPrintedMode = "";
bool lastPrintedUnoOnline = false;
bool lastPrintedP1 = false;
bool lastPrintedP2 = false;
bool lastPrintedPause = false;
bool lastPrintedStop = false;
String lastPrintedL1 = "";
String lastPrintedL2 = "";

bool mpuReady = false;
String mpuStablePosture = "UNKNOWN";
String mpuCandidatePosture = "UNKNOWN";
String mpuLastSentPosture = "";
unsigned long lastMpuSampleMs = 0;
unsigned long mpuCandidateStartMs = 0;
unsigned long lastMpuSendMs = 0;
unsigned long nextMpuRetryMs = 0;
float mpuAx = 0.0f;
float mpuAy = 0.0f;
float mpuAz = 0.0f;
float mpuGx = 0.0f;
float mpuGy = 0.0f;
float mpuGz = 0.0f;
float mpuRollDeg = 0.0f;
float mpuPitchDeg = 0.0f;
float mpuAccMag = 0.0f;
float mpuGyroSum = 0.0f;
float mpuTempC = 0.0f;

// Google Sheets pump log queue
struct PumpLogItem
{
  String timestamp;
  String source;
  String event;
  String mode;
};

PumpLogItem pumpLogQueue[PUMP_LOG_QUEUE_SIZE];
int pumpLogHead = 0;
int pumpLogTail = 0;
int pumpLogCount = 0;
unsigned long nextLogRetryMs = 0;

// Last logged state
String lastLoggedSource = "";
String lastLoggedEvent  = "";
String lastLoggedMode   = "";

// -----------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------
String escapeJson(const String &text)
{
  String out = "";

  for (unsigned int i = 0; i < text.length(); i++)
  {
    char c = text[i];

    if (c == '\"') out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else out += c;
  }

  return out;
}

String getFieldValue(const String &text, const String &key)
{
  String token = key + "=";
  int startIndex = text.indexOf(token);

  if (startIndex < 0) return "";

  startIndex += token.length();

  int endIndex = text.indexOf(';', startIndex);
  if (endIndex < 0) endIndex = text.length();

  return text.substring(startIndex, endIndex);
}

bool textToBool(const String &text)
{
  return (text == "1" || text == "ON" || text == "on" || text == "true" || text == "TRUE");
}

String normalizeMode(const String &rawMode, const String &eventText)
{
  String mode = rawMode;
  String evt  = eventText;

  mode.trim();
  evt.trim();

  if (evt == "UNO_OFFLINE") return "ERROR";
  if (mode == "NONE" || mode.length() == 0) return "HOME";

  return mode;
}

bool shouldSuppressBlynkCallback()
{
  return millis() < suppressBlynkUntilMs;
}

bool isDuplicateBlynkCommand(const String &act)
{
  unsigned long now = millis();

  if (act == lastBlynkAct && (now - lastBlynkCmdMs) < BLYNK_CMD_COOLDOWN_MS)
  {
    return true;
  }

  lastBlynkAct = act;
  lastBlynkCmdMs = now;
  return false;
}

String getTimestampString()
{
  time_t now = time(nullptr);
  if (now < 100000)
  {
    return "";
  }

  struct tm timeInfo;
  localtime_r(&now, &timeInfo);

  char buf[25];
  snprintf(buf, sizeof(buf),
           "%04d-%02d-%02d %02d:%02d:%02d",
           timeInfo.tm_year + 1900,
           timeInfo.tm_mon + 1,
           timeInfo.tm_mday,
           timeInfo.tm_hour,
           timeInfo.tm_min,
           timeInfo.tm_sec);

  return String(buf);
}

void sendCommandToUno(const String &src, const String &act);

bool inFloatRange(float value, float minValue, float maxValue)
{
  return (value >= minValue && value <= maxValue);
}

String normalizePostureLabel(const String &posture)
{
  if (posture == "SUPINE") return "SUPINE";
  if (posture == "LEFT_SIDE") return "LEFT_SIDE";
  if (posture == "RIGHT_SIDE") return "RIGHT_SIDE";
  if (posture == "MOVING") return "MOVING";
  return "UNKNOWN";
}

void markUserActivity()
{
  lastUserActionMs = millis();
}

bool isUserIdleForUpload()
{
  return (millis() - lastUserActionMs) >= USER_IDLE_UPLOAD_MS;
}

bool isRecentBlynkEcho(const String &src, const String &evt)
{
  return (src == SRC_BLYNK &&
          evt == pendingCmdEvent &&
          (millis() - pendingCmdMs) <= CMD_CONFIRM_SUPPRESS_MS);
}

bool didStateActuallyChange(
  const String &oldSource,
  const String &oldEvent,
  const String &oldMode,
  bool oldP1,
  bool oldP2,
  bool oldPause,
  bool oldStop,
  bool oldUnoOnline,
  const String &newSource,
  const String &newEvent,
  const String &newMode,
  bool newP1,
  bool newP2,
  bool newPause,
  bool newStop,
  bool newUnoOnline)
{
  return oldSource    != newSource ||
         oldEvent     != newEvent  ||
         oldMode      != newMode   ||
         oldP1        != newP1     ||
         oldP2        != newP2     ||
         oldPause     != newPause  ||
         oldStop      != newStop   ||
         oldUnoOnline != newUnoOnline;
}

void printImportantStateIfChanged()
{
  bool changed =
      (stateEvent != lastPrintedEvent) ||
      (stateMode != lastPrintedMode) ||
      (unoOnline != lastPrintedUnoOnline) ||
      (stateP1 != lastPrintedP1) ||
      (stateP2 != lastPrintedP2) ||
      (statePause != lastPrintedPause) ||
      (stateStop != lastPrintedStop) ||
      (lcdLine1 != lastPrintedL1) ||
      (lcdLine2 != lastPrintedL2);

  if (!changed) return;

  Serial.println("=== STATE CHANGED ===");
  Serial.print("UNO    : "); Serial.println(unoOnline ? "ONLINE" : "OFFLINE");
  Serial.print("EVENT  : "); Serial.println(stateEvent);
  Serial.print("MODE   : "); Serial.println(stateMode);
  Serial.print("P1/P2  : "); Serial.print(stateP1 ? "ON" : "OFF"); Serial.print(" / "); Serial.println(stateP2 ? "ON" : "OFF");
  Serial.print("PAUSE  : "); Serial.println(statePause ? "ON" : "OFF");
  Serial.print("STOP   : "); Serial.println(stateStop ? "ON" : "OFF");
  Serial.print("LCD1   : "); Serial.println(lcdLine1);
  Serial.print("LCD2   : "); Serial.println(lcdLine2);

  lastPrintedEvent = stateEvent;
  lastPrintedMode = stateMode;
  lastPrintedUnoOnline = unoOnline;
  lastPrintedP1 = stateP1;
  lastPrintedP2 = stateP2;
  lastPrintedPause = statePause;
  lastPrintedStop = stateStop;
  lastPrintedL1 = lcdLine1;
  lastPrintedL2 = lcdLine2;
}

// -----------------------------------------------------------------------------
// Shared HTTP POST
// -----------------------------------------------------------------------------
bool postJsonToGoogleSheets(const String &json, const char *tag, int &httpCodeOut, String &responseOut)
{
  httpCodeOut = -1;
  responseOut = "";

  if (WiFi.status() != WL_CONNECTED)
  {
    return false;
  }

  std::unique_ptr<BearSSL::WiFiClientSecure> client(new BearSSL::WiFiClientSecure);
  client->setInsecure();

  HTTPClient https;
  https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  https.setTimeout(5000);

  if (!https.begin(*client, GAS_URL))
  {
    Serial.print(tag);
    Serial.println(" SEND : BEGIN FAIL");
    return false;
  }

  https.addHeader("Content-Type", "application/json");

  int httpCode = https.POST(json);
  String response = https.getString();

  https.end();

  httpCodeOut = httpCode;
  responseOut = response;
  lastHttpSendMs = millis();

  bool ok = (httpCode == 200 && response.indexOf("\"ok\":true") >= 0);

  Serial.print(tag);
  Serial.print(" SEND : ");
  Serial.print(ok ? "OK" : "FAIL");
  Serial.print(" | CODE=");
  Serial.println(httpCode);

  return ok;
}

// -----------------------------------------------------------------------------
// MPU6050
// -----------------------------------------------------------------------------
void initMpu()
{
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin())
  {
    mpuReady = false;
    Serial.println("MPU6050 : NOT FOUND");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpuReady = true;
  mpuStablePosture = "UNKNOWN";
  mpuCandidatePosture = "UNKNOWN";
  mpuCandidateStartMs = millis();

  Serial.println("MPU6050 : READY");
}

void readMpuSample()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(&accel, &gyro, &temp);

  mpuAx = accel.acceleration.x;
  mpuAy = accel.acceleration.y;
  mpuAz = accel.acceleration.z;

  mpuGx = gyro.gyro.x;
  mpuGy = gyro.gyro.y;
  mpuGz = gyro.gyro.z;

  mpuTempC = temp.temperature;

  mpuAccMag = sqrt((mpuAx * mpuAx) + (mpuAy * mpuAy) + (mpuAz * mpuAz));
  mpuGyroSum = fabs(mpuGx) + fabs(mpuGy) + fabs(mpuGz);

  mpuRollDeg = atan2(mpuAy, mpuAz) * 180.0f / PI;
  mpuPitchDeg = atan2(-mpuAx, sqrt((mpuAy * mpuAy) + (mpuAz * mpuAz))) * 180.0f / PI;
}

String classifyInstantPosture()
{
  if (mpuGyroSum > MPU_MOTION_GYRO_SUM_THRESHOLD) return "MOVING";
  if (mpuAccMag < MPU_ACC_MAG_MIN || mpuAccMag > MPU_ACC_MAG_MAX) return "UNKNOWN";

  if (inFloatRange(mpuRollDeg, MPU_SUPINE_ROLL_MIN, MPU_SUPINE_ROLL_MAX) &&
      inFloatRange(mpuPitchDeg, MPU_SUPINE_PITCH_MIN, MPU_SUPINE_PITCH_MAX))
  {
    return "SUPINE";
  }

  if (inFloatRange(mpuPitchDeg, MPU_LEFT_PITCH_MIN, MPU_LEFT_PITCH_MAX) &&
      inFloatRange(mpuRollDeg, MPU_LEFT_ROLL_MIN, MPU_LEFT_ROLL_MAX))
  {
    return "LEFT_SIDE";
  }

  if (inFloatRange(mpuPitchDeg, MPU_RIGHT_PITCH_MIN, MPU_RIGHT_PITCH_MAX) &&
      inFloatRange(mpuRollDeg, MPU_RIGHT_ROLL_MIN, MPU_RIGHT_ROLL_MAX))
  {
    return "RIGHT_SIDE";
  }

  return "UNKNOWN";
}

void updateMpuPosture()
{
  if (!mpuReady) return;
  if (millis() - lastMpuSampleMs < MPU_SAMPLE_MS) return;

  lastMpuSampleMs = millis();
  readMpuSample();

  String instantPosture = classifyInstantPosture();

  if (instantPosture == mpuStablePosture)
  {
    mpuCandidatePosture = instantPosture;
    mpuCandidateStartMs = millis();
    return;
  }

  if (instantPosture != mpuCandidatePosture)
  {
    mpuCandidatePosture = instantPosture;
    mpuCandidateStartMs = millis();
    return;
  }

  if (millis() - mpuCandidateStartMs >= MPU_POSTURE_HOLD_MS)
  {
    if (mpuStablePosture != mpuCandidatePosture)
    {
      mpuStablePosture = normalizePostureLabel(mpuCandidatePosture);
      Serial.print("MPU POSTURE : ");
      Serial.println(mpuStablePosture);
    }
  }
}

String buildMpuLogJson()
{
  String json = "{";
  json += "\"action\":\"mpu_log\",";
  json += "\"timestamp\":\"" + escapeJson(getTimestampString()) + "\",";
  json += "\"device_id\":\"" + escapeJson(String(DEVICE_ID)) + "\",";
  json += "\"temp\":\"" + String(mpuTempC, 2) + "\",";
  json += "\"posture\":\"" + escapeJson(normalizePostureLabel(mpuStablePosture)) + "\"";
  json += "}";
  return json;
}

bool isMpuDueNow()
{
  if (!mpuReady) return false;
  if (WiFi.status() != WL_CONNECTED) return false;

  unsigned long now = millis();

  if ((now - lastMpuSendMs) < MPU_UPLOAD_MS) return false;
  if (now < nextMpuRetryMs) return false;

  return true;
}

void trySendMpuLog()
{
  unsigned long now = millis();
  if ((now - lastHttpSendMs) < MPU_LOG_MIN_GAP_MS) return;

  String json = buildMpuLogJson();

  int httpCode = -1;
  String response = "";

  bool ok = postJsonToGoogleSheets(json, "MPU", httpCode, response);

  if (ok)
  {
    lastMpuSendMs = millis();
    mpuLastSentPosture = mpuStablePosture;
    nextMpuRetryMs = 0;
  }
  else
  {
    nextMpuRetryMs = millis() + MPU_RETRY_MS;
  }
}

// -----------------------------------------------------------------------------
// Google Sheets pump log
// -----------------------------------------------------------------------------
String getModeForCommandEvent(const String &src, const String &evt)
{
  if (src != SRC_BLYNK_CMD) return stateMode;

  if (evt == ACT_MANUAL_ON)  return "MANUAL";
  if (evt == ACT_MANUAL_OFF) return "HOME";

  if (evt == ACT_AUTO_ON)    return "AUTO";
  if (evt == ACT_AUTO_OFF)   return "HOME";

  if (evt == ACT_STOP_ON)    return "STOP";
  if (evt == ACT_STOP_OFF)   return "HOME";

  if (evt == ACT_SINGLE_P1)  return "HOME";
  if (evt == ACT_SINGLE_P2)  return "HOME";
  if (evt == ACT_SINGLE_OFF) return "HOME";

  if (evt == ACT_PAUSE_ON)   return "PAUSE";
  if (evt == ACT_PAUSE_OFF)  return "HOME";

  return stateMode;
}

bool samePumpLogFields(const String &srcA, const String &evtA, const String &modeA,
                       const String &srcB, const String &evtB, const String &modeB)
{
  return srcA == srcB &&
         evtA == evtB &&
         modeA == modeB;
}

void rememberLoggedStateFields(const String &src, const String &evt, const String &mode)
{
  lastLoggedSource = src;
  lastLoggedEvent  = evt;
  lastLoggedMode   = mode;
}

bool isSameAsLastLoggedFields(const String &src, const String &evt, const String &mode)
{
  return src == lastLoggedSource &&
         evt == lastLoggedEvent &&
         mode == lastLoggedMode;
}

bool shouldLogSource(const String &src, const String &evt)
{
  if (src == SRC_BLYNK) return true;
  if (src == SRC_ESP && evt != "UNO_OFFLINE") return true;
  if (src == SRC_BLYNK_CMD) return true;
  return false;
}

void enqueuePumpLog(const String &src, const String &evt)
{
  String mode = getModeForCommandEvent(src, evt);

  if (isSameAsLastLoggedFields(src, evt, mode))
  {
    return;
  }

  if (pumpLogCount > 0)
  {
    int lastIndex = (pumpLogTail - 1 + PUMP_LOG_QUEUE_SIZE) % PUMP_LOG_QUEUE_SIZE;
    if (samePumpLogFields(src, evt, mode,
                          pumpLogQueue[lastIndex].source,
                          pumpLogQueue[lastIndex].event,
                          pumpLogQueue[lastIndex].mode))
    {
      return;
    }
  }

  if (pumpLogCount >= PUMP_LOG_QUEUE_SIZE)
  {
    pumpLogHead = (pumpLogHead + 1) % PUMP_LOG_QUEUE_SIZE;
    pumpLogCount--;
  }

  pumpLogQueue[pumpLogTail].timestamp = getTimestampString();
  pumpLogQueue[pumpLogTail].source    = src;
  pumpLogQueue[pumpLogTail].event     = evt;
  pumpLogQueue[pumpLogTail].mode      = mode;
  pumpLogTail = (pumpLogTail + 1) % PUMP_LOG_QUEUE_SIZE;
  pumpLogCount++;

  Serial.print("QUEUE +1 : ");
  Serial.print(src);
  Serial.print(" / ");
  Serial.print(evt);
  Serial.print(" / ");
  Serial.print(mode);
  Serial.print(" | COUNT=");
  Serial.println(pumpLogCount);
}

String buildPumpLogJsonFromFields(const String &timestamp, const String &src,
                                  const String &evt, const String &mode)
{
  String json = "{";
  json += "\"action\":\"pump_log\",";
  json += "\"timestamp\":\"" + escapeJson(timestamp) + "\",";
  json += "\"device_id\":\"" + escapeJson(String(DEVICE_ID)) + "\",";
  json += "\"source\":\"" + escapeJson(src) + "\",";
  json += "\"event\":\"" + escapeJson(evt) + "\",";
  json += "\"mode\":\"" + escapeJson(mode) + "\"";
  json += "}";
  return json;
}

bool hasPendingPumpLog()
{
  return (pumpLogCount > 0);
}

void trySendNextPumpLog()
{
  if (pumpLogCount <= 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long now = millis();
  if ((now - lastHttpSendMs) < LOG_SEND_MIN_GAP_MS) return;
  if (now < nextLogRetryMs) return;

  String json = buildPumpLogJsonFromFields(pumpLogQueue[pumpLogHead].timestamp,
                                       pumpLogQueue[pumpLogHead].source,
                                       pumpLogQueue[pumpLogHead].event,
                                       pumpLogQueue[pumpLogHead].mode);

  int httpCode = -1;
  String response = "";

  bool ok = postJsonToGoogleSheets(json, "PUMP", httpCode, response);

  if (ok)
  {
    rememberLoggedStateFields(pumpLogQueue[pumpLogHead].source,
                             pumpLogQueue[pumpLogHead].event,
                             pumpLogQueue[pumpLogHead].mode);
    pumpLogHead = (pumpLogHead + 1) % PUMP_LOG_QUEUE_SIZE;
    pumpLogCount--;
    nextLogRetryMs = 0;
  }
  else
  {
    nextLogRetryMs = millis() + LOG_RETRY_MS;
  }
}

// -----------------------------------------------------------------------------
// Scheduler: Control first, upload later
// -----------------------------------------------------------------------------
void serviceHttpScheduler()
{
  if ((millis() - bootMs) < BOOT_HTTP_HOLD_MS)
  {
    return;
  }

  if (!isUserIdleForUpload())
  {
    return;
  }

  // FIX: Do not let pending pump logs starve MPU uploads forever.
  // If MPU is due, send it first. Pump log queue will continue on later cycles.
  if (isMpuDueNow())
  {
    trySendMpuLog();
    return;
  }

  if (hasPendingPumpLog())
  {
    trySendNextPumpLog();
    return;
  }
}

// -----------------------------------------------------------------------------
// Blynk UI sync from UNO real state
// -----------------------------------------------------------------------------
void updateBlynkUI()
{
  if (!Blynk.connected()) return;

  suppressBlynkUntilMs = millis() + BLYNK_CMD_COOLDOWN_MS;

  bool wantStop   = stateStop;
  bool wantPause  = statePause;
  bool wantAuto   = (stateMode == "AUTO");
  bool wantManual = (stateMode == "MANUAL");
  bool allowPumpToggle = (stateMode == "HOME" && !statePause && !stateStop);
  bool wantPump1  = allowPumpToggle ? stateP1 : false;
  bool wantPump2  = allowPumpToggle ? stateP2 : false;

  virtualWriteIfChanged(VPIN_STOP,   wantStop,   uiStopSent);
  virtualWriteIfChanged(VPIN_PAUSE,  wantPause,  uiPauseSent);
  virtualWriteIfChanged(VPIN_AUTO,   wantAuto,   uiAutoSent);
  virtualWriteIfChanged(VPIN_MANUAL, wantManual, uiManualSent);
  virtualWriteIfChanged(VPIN_PUMP1,  wantPump1,  uiPump1Sent);
  virtualWriteIfChanged(VPIN_PUMP2,  wantPump2,  uiPump2Sent);

  uiSentInit = true;
}

// -----------------------------------------------------------------------------
// ESP -> UNO
// -----------------------------------------------------------------------------
void sendCommandToUno(const String &src, const String &act)
{
  unsigned long now = millis();
  if (now - lastCmdMs < CMD_GAP_MS)
  {
    delay(CMD_GAP_MS - (now - lastCmdMs));
  }

  String cmd = "CMD:SRC=" + src + ";ACT=" + act;
  unoSer.println(cmd);

  lastCmdSource = src;
  lastCmdAction = act;
  lastCmdMs = millis();

  Serial.print("CMD -> UNO : ");
  Serial.println(act);
}

void sendBlynkCommandIfAllowed(const String &act)
{
  if (shouldSuppressBlynkCallback()) return;
  if (isDuplicateBlynkCommand(act)) return;

  markUserActivity();

  pendingCmdEvent = act;
  pendingCmdMs = millis();

  enqueuePumpLog(SRC_BLYNK_CMD, act);
  sendCommandToUno(SRC_BLYNK, act);
  lastStatusReqMs = millis();
}

void syncNtpTime()
{
  if (WiFi.status() != WL_CONNECTED) return;

  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
  lastNtpSyncMs = millis();
  Serial.println("NTP : SYNC");
}

bool getThaiTime(int &hh, int &mm, int &ss)
{
  time_t now = time(nullptr);
  if (now < 100000) return false;

  struct tm timeInfo;
  localtime_r(&now, &timeInfo);

  hh = timeInfo.tm_hour;
  mm = timeInfo.tm_min;
  ss = timeInfo.tm_sec;
  return true;
}

void sendTimeToUno()
{
  int hh = 0;
  int mm = 0;
  int ss = 0;

  if (!getThaiTime(hh, mm, ss)) return;
  if (ss == lastSentSecond) return;

  unsigned long nowMs = millis();
  if (nowMs - lastCmdMs < CMD_GAP_MS)
  {
    delay(CMD_GAP_MS - (nowMs - lastCmdMs));
  }

  char buf[20];
  snprintf(buf, sizeof(buf), "TIME:%02d:%02d:%02d", hh, mm, ss);
  unoSer.println(buf);

  lastCmdMs = millis();
  lastSentSecond = ss;
}

// -----------------------------------------------------------------------------
// UNO -> ESP
// -----------------------------------------------------------------------------
void processStateMessage(const String &msg)
{
  String oldSource = stateSource;
  String oldEvent  = stateEvent;
  String oldMode   = stateMode;
  bool oldP1       = stateP1;
  bool oldP2       = stateP2;
  bool oldPause    = statePause;
  bool oldStop     = stateStop;
  bool oldUnoOnline = unoOnline;

  String newSource = getFieldValue(msg, "SRC");
  String newEvent  = getFieldValue(msg, "EVENT");
  String newMode   = getFieldValue(msg, "MODE");
  String newP1     = getFieldValue(msg, "P1");
  String newP2     = getFieldValue(msg, "P2");
  String newPause  = getFieldValue(msg, "PAUSE");
  String newStop   = getFieldValue(msg, "STOP");
  String newL1     = getFieldValue(msg, "L1");
  String newL2     = getFieldValue(msg, "L2");

  if (newSource.length() > 0) stateSource = newSource;
  if (newEvent.length()  > 0) stateEvent  = newEvent;
  if (newMode.length()   > 0) stateMode   = normalizeMode(newMode, newEvent);
  if (newP1.length()     > 0) stateP1     = textToBool(newP1);
  if (newP2.length()     > 0) stateP2     = textToBool(newP2);
  if (newPause.length()  > 0) statePause  = textToBool(newPause);
  if (newStop.length()   > 0) stateStop   = textToBool(newStop);
  if (newL1.length()     > 0) lcdLine1    = newL1;
  if (newL2.length()     > 0) lcdLine2    = newL2;

  unoOnline = true;
  lastUnoStateMs = millis();
  lastUnoRxMs = lastUnoStateMs;
  unoMissCount = 0;

  bool stateChanged = didStateActuallyChange(
    oldSource, oldEvent, oldMode, oldP1, oldP2, oldPause, oldStop, oldUnoOnline,
    stateSource, stateEvent, stateMode, stateP1, stateP2, statePause, stateStop, unoOnline
  );

  if (stateChanged)
  {
    updateBlynkUI();
    printImportantStateIfChanged();
  }

  if (stateChanged && shouldLogSource(stateSource, stateEvent) && !isRecentBlynkEcho(stateSource, stateEvent))
  {
    enqueuePumpLog(stateSource, stateEvent);
  }
}

void processUnoMessage(String msg)
{
  msg.trim();
  if (msg.length() == 0) return;

  int firstState = msg.indexOf("STATE:");
  int lastState  = msg.lastIndexOf("STATE:");

  if (firstState > 0)
  {
    msg = msg.substring(firstState);
  }

  if (lastState > 0 && lastState < (int)msg.length())
  {
    String tail = msg.substring(lastState);
    if (tail.startsWith("STATE:"))
    {
      msg = tail;
    }
  }

  if (msg.startsWith("STATE:"))
  {
    processStateMessage(msg);
  }
}

void readUnoSerial()
{
  while (unoSer.available())
  {
    char c = unoSer.read();
    lastUnoRxMs = millis();

    if (c == '\r') continue;

    if (c == '\n')
    {
      if (rxLine.length() > 0)
      {
        processUnoMessage(rxLine);
        rxLine = "";
      }
    }
    else
    {
      rxLine += c;
      if (rxLine.length() > 180)
      {
        rxLine = "";
      }
    }
  }
}

// -----------------------------------------------------------------------------
// UNO offline supervision
// -----------------------------------------------------------------------------
void updateUnoOnlineState()
{
  unsigned long now = millis();

  if ((now - lastUnoStateMs) > UNO_OFFLINE_MS)
  {
    if ((now - lastUnoRxMs) > UNO_RX_GRACE_MS)
    {
      if (unoMissCount < UNO_MISS_LIMIT) unoMissCount++;
    }
  }
  else
  {
    unoMissCount = 0;
    return;
  }

  if (unoOnline && unoMissCount >= UNO_MISS_LIMIT)
  {
    unoOnline = false;
    stateSource = SRC_ESP;
    stateEvent = "UNO_OFFLINE";
    stateMode = "ERROR";
    lcdLine1 = "UNO OFFLINE";
    lcdLine2 = "LAST STATE HELD";

    updateBlynkUI();
    printImportantStateIfChanged();
  }
}

// -----------------------------------------------------------------------------
// Blynk callbacks
// -----------------------------------------------------------------------------
BLYNK_CONNECTED()
{
  Serial.println("BLYNK : CONNECTED");
  suppressBlynkUntilMs = millis() + 1500UL;
  uiSentInit = false;
  updateBlynkUI();
  lastStatusReqMs = millis();
  sendCommandToUno(SRC_ESP, ACT_STATUS_REQ);
}

BLYNK_WRITE(VPIN_PUMP1)
{
  int value = param.asInt();
  if (shouldSuppressBlynkCallback()) return;

  if (value == 1)
  {
    if (!stateP1 || stateP2 || statePause || stateStop || stateMode != "HOME")
    {
      sendBlynkCommandIfAllowed(ACT_SINGLE_P1);
    }
  }
  else
  {
    if (stateP1)
    {
      sendBlynkCommandIfAllowed(ACT_SINGLE_OFF);
    }
  }
}

BLYNK_WRITE(VPIN_PUMP2)
{
  int value = param.asInt();
  if (shouldSuppressBlynkCallback()) return;

  if (value == 1)
  {
    if (!stateP2 || stateP1 || statePause || stateStop || stateMode != "HOME")
    {
      sendBlynkCommandIfAllowed(ACT_SINGLE_P2);
    }
  }
  else
  {
    if (stateP2)
    {
      sendBlynkCommandIfAllowed(ACT_SINGLE_OFF);
    }
  }
}

BLYNK_WRITE(VPIN_PAUSE)
{
  int value = param.asInt();
  if (shouldSuppressBlynkCallback()) return;

  if (value == 1)
  {
    if (!statePause) sendBlynkCommandIfAllowed(ACT_PAUSE_ON);
  }
  else
  {
    if (statePause) sendBlynkCommandIfAllowed(ACT_PAUSE_OFF);
  }
}

BLYNK_WRITE(VPIN_AUTO)
{
  int value = param.asInt();
  if (shouldSuppressBlynkCallback()) return;

  if (value == 1)
  {
    if (stateMode != "AUTO") sendBlynkCommandIfAllowed(ACT_AUTO_ON);
  }
  else
  {
    if (stateMode == "AUTO") sendBlynkCommandIfAllowed(ACT_AUTO_OFF);
  }
}

BLYNK_WRITE(VPIN_MANUAL)
{
  int value = param.asInt();
  if (shouldSuppressBlynkCallback()) return;

  if (value == 1)
  {
    if (stateMode != "MANUAL") sendBlynkCommandIfAllowed(ACT_MANUAL_ON);
  }
  else
  {
    if (stateMode == "MANUAL") sendBlynkCommandIfAllowed(ACT_MANUAL_OFF);
  }
}

BLYNK_WRITE(VPIN_STOP)
{
  int value = param.asInt();
  if (shouldSuppressBlynkCallback()) return;

  if (value == 1)
  {
    if (!stateStop) sendBlynkCommandIfAllowed(ACT_STOP_ON);
  }
  else
  {
    if (stateStop) sendBlynkCommandIfAllowed(ACT_STOP_OFF);
  }
}

// -----------------------------------------------------------------------------
// Setup / Loop
// -----------------------------------------------------------------------------
void setup()
{
  Serial.begin(SERIAL_BAUD);
  unoSer.begin(UNO_BAUD);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP AIR MATTRESS BRIDGE : READY");
  Serial.println("CONTROL FIRST, UPLOAD LATER");
  Serial.println("========================================");

  bootMs = millis();
  lastUserActionMs = millis();

  initMpu();

  WiFi.mode(WIFI_STA);
  Blynk.begin(auth, ssid, pass);
  syncNtpTime();

  lastUnoRxMs = millis();
  pumpLogHead = 0;
  pumpLogTail = 0;
  pumpLogCount = 0;
  nextLogRetryMs = 0;
  lastMpuSampleMs = 0;
  mpuCandidateStartMs = millis();
  lastMpuSendMs = millis();
  nextMpuRetryMs = 0;
  lastHttpSendMs = 0;
}

void loop()
{
  Blynk.run();
  readUnoSerial();
  updateUnoOnlineState();
  updateMpuPosture();
  serviceHttpScheduler();

  if (WiFi.status() == WL_CONNECTED && (millis() - lastNtpSyncMs >= NTP_RESYNC_MS))
  {
    syncNtpTime();
  }

  sendTimeToUno();

  if ((millis() - lastStatusReqMs >= STATUS_REQ_MS) && (millis() - lastBlynkCmdMs >= STATUS_HOLD_MS))
  {
    lastStatusReqMs = millis();
    sendCommandToUno(SRC_ESP, ACT_STATUS_REQ);
  }

  if (millis() - lastPrintMs >= PRINT_MS)
  {
    lastPrintMs = millis();

    Serial.print("[SYS] WIFI=");
    Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "NO");
    Serial.print(" | BLYNK=");
    Serial.print(Blynk.connected() ? "OK" : "NO");
    Serial.print(" | UNO=");
    Serial.print(unoOnline ? "OK" : "NO");
    Serial.print(" | MODE=");
    Serial.print(stateMode);
    Serial.print(" | EVT=");
    Serial.print(stateEvent);
    Serial.print(" | Q=");
    Serial.print(pumpLogCount);
    Serial.print(" | MPU=");
    Serial.print(mpuStablePosture);
    Serial.print(" | T=");
    Serial.print(String(mpuTempC, 1));
    Serial.print(" | BOOT_HOLD=");
    Serial.print((millis() - bootMs < BOOT_HTTP_HOLD_MS) ? "YES" : "NO");
    Serial.print(" | IDLE_OK=");
    Serial.print(isUserIdleForUpload() ? "YES" : "NO");
    Serial.print(" | IDLE_MS=");
    Serial.println(millis() - lastUserActionMs);
  }
}
