/*
 * OHORONA BIKE COMPUTER v4.4.2 SOUND — RELAY-GATED BLE + BROWSER TTS
 * ESP32 + NimBLE-Arduino 2.5.1 + ESP32 core 3.3.12 + AsyncWebServer
 *
 * v4.3: A (OTA auth) + C (JSON escape) + D (NVS wear 15s)
 * v4.4: убраны кнопки 🛞 и 🚨 из верхнего ряда панели,
 *       удалены диалог колеса и функции openWheel/doSetWheel.
 *       Все настройки остаются доступны через ⚙️ /setup.
 * v4.4.2: «89 минут» → «1 час 29 минут»;
 *         расстояние в формате «X километров Y метров»;
 *         в обзор добавлено текущее время (без секунд).
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <time.h>
#include <esp_wifi.h>

// ============================================================
//                        КОНСТАНТЫ
// ============================================================
const char* AP_SSID_DEFAULT = "OHORONA-BIKE";
const char* HOSTNAME   = "bike";
const char* BLE_NAME   = "ohorona";
const char* NTP_SERVER = "pool.ntp.org";
const char* TZ_KYIV    = "EET-2EEST,M3.5.0/3,M10.5.0/4";

#define CSC_SERVICE_UUID     "1816"
#define CSC_MEASUREMENT_UUID "2A5B"
#define PIN_LED              2
#define PIN_RELAY            4
#define PIN_BOOT             0

#define CFG_MAGIC   0x4F425633UL
#define CFG_VERSION 6

const uint32_t SAVE_INTERVAL_MS  = 15000;
const uint32_t SENSOR_TIMEOUT_MS = 3000;

// ============================================================
//                        КОНФИГ
// ============================================================
struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t _pad0;

  char   wifiSSID[33];
  char   wifiPass[65];
  char   apPass[64];
  char   webUser[17];
  char   webPass[33];
  uint8_t _pad1[3];

  char   staticIP[16];
  char   staticGW[16];

  float  wheelCircumference;

  uint32_t armDelaySec;
  uint32_t staBootTimeoutSec;
  uint32_t staCheckIntervalSec;
  float    armSpeedIgnoreKmh;

  int32_t  sleepHour, sleepMin;
  int32_t  wakeHour,  wakeMin;
  uint32_t sleepGraceSec;

  uint32_t lastDailyResetDay;

  uint32_t bleScanIntervalSec;
  uint32_t bleScanDurationMs;
  uint32_t speedStopMs;

  uint8_t flags;
  uint8_t _pad2[3];

  char   apSSID[33];
};
Config cfg;
Preferences prefs;

struct OldConfigV5 {
  uint32_t magic; uint16_t version; uint16_t _pad0;
  char   wifiSSID[33]; char   wifiPass[65]; char   apPass[64];
  char   webUser[17]; char   webPass[33]; uint8_t _pad1[3];
  char   staticIP[16]; char   staticGW[16];
  float  wheelCircumference;
  uint32_t armDelaySec; uint32_t staBootTimeoutSec; uint32_t staCheckIntervalSec;
  float    armSpeedIgnoreKmh;
  int32_t  sleepHour, sleepMin; int32_t  wakeHour,  wakeMin;
  uint32_t sleepGraceSec; uint32_t lastDailyResetDay;
  uint32_t bleScanIntervalSec; uint32_t bleScanDurationMs; uint32_t speedStopMs;
  uint8_t flags; uint8_t _pad2[3];
};

struct OldConfigV4 {
  uint32_t magic; uint16_t version; uint16_t _pad0;
  char   wifiSSID[33]; char   wifiPass[65]; char   apPass[64];
  char   webUser[17]; char   webPass[33]; uint8_t _pad1[3];
  float  wheelCircumference;
  uint32_t armDelaySec; uint32_t staBootTimeoutSec; uint32_t staCheckIntervalSec;
  float    armSpeedIgnoreKmh;
  int32_t  sleepHour, sleepMin; int32_t  wakeHour,  wakeMin;
  uint32_t sleepGraceSec; uint32_t lastDailyResetDay;
  uint32_t bleScanIntervalSec; uint32_t bleScanDurationMs; uint32_t speedStopMs;
  uint8_t flags; uint8_t _pad2[3];
};

struct OldConfigV3 {
  uint32_t magic; uint16_t version; uint16_t _pad0;
  char   wifiSSID[33]; char   wifiPass[65]; char   apPass[64];
  char   webUser[17]; char   webPass[33]; uint8_t _pad1[3];
  float  wheelCircumference;
  uint32_t armDelaySec; uint32_t staBootTimeoutSec; uint32_t staCheckIntervalSec;
  float    armSpeedIgnoreKmh;
  int32_t  sleepHour, sleepMin; int32_t  wakeHour,  wakeMin;
  uint32_t sleepGraceSec; uint32_t lastDailyResetDay;
  uint8_t flags; uint8_t _pad2[3];
};

struct OldConfigV2 {
  char  wifiSSID[33]; char  wifiPass[65]; char  webUser[17]; char  webPass[33];
  float wheelCircumference; bool  configured;
};

#define FLAG_CONFIGURED      0x01
#define FLAG_ALARM_ENABLED   0x02
#define FLAG_MANUAL_MODE     0x04
#define FLAG_MANUAL_RELAY    0x08
#define FLAG_MANUAL_COUNT_KM 0x10
#define FLAG_SLEEP_ENABLED   0x20
#define FLAG_STATIC_IP       0x40

inline bool cfgConfigured()    { return cfg.flags & FLAG_CONFIGURED; }
inline bool cfgAlarmEnabled()  { return cfg.flags & FLAG_ALARM_ENABLED; }
inline bool cfgManualMode()    { return cfg.flags & FLAG_MANUAL_MODE; }
inline bool cfgManualRelay()   { return cfg.flags & FLAG_MANUAL_RELAY; }
inline bool cfgManualCountKm() { return cfg.flags & FLAG_MANUAL_COUNT_KM; }
inline bool cfgSleepEnabled()  { return cfg.flags & FLAG_SLEEP_ENABLED; }
inline bool cfgStaticIP()      { return cfg.flags & FLAG_STATIC_IP; }
inline void setFlag(uint8_t f, bool on) { if (on) cfg.flags |= f; else cfg.flags &= ~f; }

// ============================================================
//                        ДАННЫЕ
// ============================================================
struct BikeData {
  volatile float    speed           = 0.0f;
  volatile float    dailyDistance   = 0.0f;
  volatile float    totalDistance   = 0.0f;
  volatile float    maxSpeed        = 0.0f;
  volatile uint32_t rideTime        = 0;
  volatile uint32_t totalRideTime   = 0;
  volatile bool     sensorConnected = false;
};
BikeData bike;

// ============================================================
//                        ГЛОБАЛЬНОЕ
// ============================================================
AsyncWebServer server(80);

volatile bool relayIsOn = false;
volatile bool bleConnected = false;
bool scanning = false;
volatile bool forceScanRequested = false;
NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pMeasurementCharacteristic = nullptr;
volatile bool          wheelDataValid    = false;
volatile unsigned long lastWheelPacketMs = 0;
volatile unsigned long lastRevolutionMs  = 0;
volatile bool          bleClientDeletePending = false;

bool timeSynced = false;
unsigned long bootMs = 0;
bool bikeDirty = false;
unsigned long pendingRestart = 0;
bool sleeping = false;

// --- v4.2 SOUND ---
char     pendingAnnouncement[250] = "";
volatile bool pendingAnnouncementUrgent = false;
uint32_t lastAnnouncedKm          = 0;
uint32_t lastAnnouncedHour        = 0;
float    lastAnnouncedMaxSpeed    = 15.0f;
bool dailyResetPending = false;
bool isMoving = false;
unsigned long lastMovementMs = 0;
bool sysNormalAnnounced = false;

enum AlarmState { ALARM_READY, ALARM_ARMING, ALARM_ARMED, ALARM_MANUAL };
AlarmState alarmState = ALARM_READY;
unsigned long ownerGoneSince = 0;
unsigned long testRelayUntil = 0;

static inline bool timeReached(unsigned long now, unsigned long target) { return (int32_t)(now - target) >= 0; }

// Forward declarations
int apClientCount();
bool ownerNearby();
void setRelay(bool on, const char* src);
void saveBikeDataNow();
void saveConfig();
uint32_t currentLocalDay();
void sendSetupPage(AsyncWebServerRequest* r);

// ============================================================
//                        LED
// ============================================================
void initLed() { pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW); }
void startupBlink() { for (int i = 0; i < 3; i++) { digitalWrite(PIN_LED, HIGH); delay(100); digitalWrite(PIN_LED, LOW);  delay(100); } }

void updateLed() {
  if (sleeping) { digitalWrite(PIN_LED, LOW); return; }
  unsigned long now = millis();
  if (!cfgConfigured()) {
    unsigned long p = now % 2000;
    digitalWrite(PIN_LED, ((p < 1000) ? ((p % 200) < 100) : false) ? HIGH : LOW);
    return;
  }
  if (cfgManualMode()) {
    unsigned long p = now % 1000;
    digitalWrite(PIN_LED, ((p < 200) || (p >= 400 && p < 600)) ? HIGH : LOW);
    return;
  }
  if (!cfgAlarmEnabled()) {
    unsigned long p = now % 2500;
    digitalWrite(PIN_LED, (p < 500) ? HIGH : LOW);
    return;
  }
  if (!relayIsOn) {
    unsigned long p = now % 2300;
    bool on;
    if (p < 1000)       on = true;
    else if (p < 1100)  on = false;
    else                on = (((p - 1100) % 400) < 200);
    digitalWrite(PIN_LED, on ? HIGH : LOW);
    return;
  }
  bool sensorActive = (bike.sensorConnected && (millis() - lastRevolutionMs < SENSOR_TIMEOUT_MS));
  digitalWrite(PIN_LED, sensorActive ? ((now % 2000) < 1000 ? HIGH : LOW) : ((now % 1000) < 500 ? HIGH : LOW));
}

// ============================================================
//                        УТИЛИТЫ
// ============================================================
void formatTime(char* buf, size_t n) { time_t now = time(nullptr); struct tm ti; localtime_r(&now, &ti); snprintf(buf, n, "%02d:%02d:%02d", ti.tm_hour, ti.tm_min, ti.tm_sec); }
void formatDate(char* buf, size_t n) { time_t now = time(nullptr); struct tm ti; localtime_r(&now, &ti); snprintf(buf, n, "%02d.%02d.%04d", ti.tm_mday, ti.tm_mon + 1, ti.tm_year + 1900); }

void streamEscaped(AsyncResponseStream* s, const char* str) {
  if (!s || !str) return;
  for (; *str; ++str) {
    switch (*str) {
      case '&':  s->print(F("&amp;"));  break;
      case '<':  s->print(F("&lt;"));   break;
      case '>':  s->print(F("&gt;"));   break;
      case '"':  s->print(F("&quot;")); break;
      case '\'': s->print(F("&#39;"));  break;
      default:   s->write(*str);        break;
    }
  }
}

// v4.3: экранирует строку для вставки в JSON-значение
static void jsonEscape(char* dst, size_t dstSize, const char* src) {
  if (!dst || !dstSize) return;
  size_t j = 0;
  for (size_t i = 0; src[i] && j + 7 < dstSize; i++) {
    unsigned char c = src[i];
    if      (c == '"')  { dst[j++]='\\'; dst[j++]='"';  }
    else if (c == '\\') { dst[j++]='\\'; dst[j++]='\\'; }
    else if (c == '\n') { dst[j++]='\\'; dst[j++]='n';  }
    else if (c == '\r') { dst[j++]='\\'; dst[j++]='r';  }
    else if (c == '\t') { dst[j++]='\\'; dst[j++]='t';  }
    else if (c < 0x20)  { j += snprintf(dst+j, dstSize-j, "\\u%04x", c); }
    else                { dst[j++] = c; }
  }
  dst[j] = 0;
}

// ============================================================
//        v4.2 SOUND — формирование текста для озвучки
// ============================================================
void setAnnouncement(const char* msg, bool urgent = false) {
  strlcpy(pendingAnnouncement, msg, sizeof(pendingAnnouncement));
  pendingAnnouncementUrgent = urgent;
}

// v4.4.2: «89 минут» → «1 час 29 минут»
void formatRideTime(char* buf, size_t n, uint32_t sec) {
  uint32_t h = sec / 3600;
  uint32_t m = (sec % 3600) / 60;
  uint32_t s = sec % 60;
  if (h > 0) {
    const char* hw = (h % 10 == 1 && h % 100 != 11) ? "час" :
                     (h % 10 >= 2 && h % 10 <= 4 && (h % 100 < 10 || h % 100 >= 20)) ? "часа" : "часов";
    if (m == 0) { snprintf(buf, n, "%lu %s", (unsigned long)h, hw); return; }
    const char* mw = (m % 10 == 1 && m % 100 != 11) ? "минута" :
                     (m % 10 >= 2 && m % 10 <= 4 && (m % 100 < 10 || m % 100 >= 20)) ? "минуты" : "минут";
    snprintf(buf, n, "%lu %s %lu %s", (unsigned long)h, hw, (unsigned long)m, mw);
  } else if (m > 0) {
    const char* mw = (m % 10 == 1 && m % 100 != 11) ? "минута" :
                     (m % 10 >= 2 && m % 10 <= 4 && (m % 100 < 10 || m % 100 >= 20)) ? "минуты" : "минут";
    if (s == 0) { snprintf(buf, n, "%lu %s", (unsigned long)m, mw); return; }
    const char* sw = (s % 10 == 1 && s % 100 != 11) ? "секунда" :
                     (s % 10 >= 2 && s % 10 <= 4 && (s % 100 < 10 || s % 100 >= 20)) ? "секунды" : "секунд";
    snprintf(buf, n, "%lu %s %lu %s", (unsigned long)m, mw, (unsigned long)s, sw);
  } else {
    const char* sw = (s % 10 == 1 && s % 100 != 11) ? "секунда" :
                     (s % 10 >= 2 && s % 10 <= 4 && (s % 100 < 10 || s % 100 >= 20)) ? "секунды" : "секунд";
    snprintf(buf, n, "%lu %s", (unsigned long)s, sw);
  }
}

// v4.4.2: «X километров Y метров» — надёжно проговаривается TTS
void formatDistance(char* buf, size_t n, float meters) {
  uint32_t totalM = (uint32_t)(meters + 0.5f);
  uint32_t kM = totalM / 1000;
  uint32_t mM = totalM % 1000;
  const char* kmWord = (kM % 10 == 1 && kM % 100 != 11) ? "километр" :
                       (kM % 10 >= 2 && kM % 10 <= 4 && (kM % 100 < 10 || kM % 100 >= 20)) ? "километра" :
                       "километров";
  if (mM == 0) {
    snprintf(buf, n, "%lu %s", (unsigned long)kM, kmWord);
  } else {
    const char* mWord = (mM % 10 == 1 && mM % 100 != 11) ? "метр" :
                        (mM % 10 >= 2 && mM % 10 <= 4 && (mM % 100 < 10 || mM % 100 >= 20)) ? "метра" :
                        "метров";
    snprintf(buf, n, "%lu %s %lu %s", (unsigned long)kM, kmWord, (unsigned long)mM, mWord);
  }
}

void checkAnnounce() {
  if (!bike.sensorConnected || !relayIsOn) return;
  if (cfgManualMode() && !cfgManualCountKm()) return;

  uint32_t km = (uint32_t)(bike.dailyDistance / 1000.0f);
  if (km > lastAnnouncedKm && km > 0) {
    lastAnnouncedKm = km;
    uint32_t rt = bike.rideTime;
    float    avg = (rt > 0) ? (bike.dailyDistance / 1000.0f) / (rt / 3600.0f) : 0.0f;
    float    maxSp = bike.maxSpeed;
    char distBuf[48]; formatDistance(distBuf, sizeof(distBuf), bike.dailyDistance);
    char rideBuf[48]; formatRideTime(rideBuf, sizeof(rideBuf), rt);
    snprintf(pendingAnnouncement, sizeof(pendingAnnouncement),
      "Проехал за сегодня %s. Время в пути %s. Средняя %.1f километров в час. Максимальная %.1f километров в час.",
      distBuf, rideBuf, avg, maxSp);
  }

  uint32_t currentHour = bike.rideTime / 3600;
  if (currentHour > lastAnnouncedHour && currentHour > 0) {
    lastAnnouncedHour = currentHour;
    const char* hourWord;
    if (currentHour % 10 == 1 && currentHour % 100 != 11) hourWord = "час";
    else if (currentHour % 10 >= 2 && currentHour % 10 <= 4 && (currentHour % 100 < 10 || currentHour % 100 >= 20)) hourWord = "часа";
    else hourWord = "часов";
    uint32_t rt = bike.rideTime;
    float    avg = (rt > 0) ? (bike.dailyDistance / 1000.0f) / (rt / 3600.0f) : 0.0f;
    char distBuf[48]; formatDistance(distBuf, sizeof(distBuf), bike.dailyDistance);
    snprintf(pendingAnnouncement, sizeof(pendingAnnouncement),
      "Вы в пути уже %lu %s. Пробег %s. Средняя %.1f километров в час.",
      (unsigned long)currentHour, hourWord, distBuf, avg);
  }

  if (bike.maxSpeed > 15.0f && bike.maxSpeed > lastAnnouncedMaxSpeed) {
    lastAnnouncedMaxSpeed = bike.maxSpeed;
    snprintf(pendingAnnouncement, sizeof(pendingAnnouncement),
      "Максимальная сегодня %.1f километров в час.", lastAnnouncedMaxSpeed);
  }
}

void movementTick() {
  if (!relayIsOn) { isMoving = false; return; }
  if (bike.speed > 1.0f) {
    lastMovementMs = millis();
    if (!isMoving) { isMoving = true; setAnnouncement("Поехали."); }
  } else {
    if (isMoving && (millis() - lastMovementMs > 30000)) { isMoving = false; setAnnouncement("Остановка."); }
  }
}

// ============================================================
//                        BLE CSC
// ============================================================
class MyClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient*) override {
    Serial.println(F("BLE: подключено"));
    wheelDataValid = false; bike.sensorConnected = true;
    if (relayIsOn) setAnnouncement("Датчик подключен.");
  }
  void onDisconnect(NimBLEClient*, int reason) override {
    Serial.print(F("BLE: отключено (")); Serial.print(reason); Serial.println(F(")"));
    bike.sensorConnected = false; bike.speed = 0.0f; bleConnected = false;
    pMeasurementCharacteristic = nullptr; wheelDataValid = false;
    bleClientDeletePending = true;
    if (relayIsOn) setAnnouncement("Нет связи с датчиком.", true);
  }
};

void notifyCallback(NimBLERemoteCharacteristic*, uint8_t* pData, size_t length, bool) {
  if (!relayIsOn) return;
  if (length < 7) return;
  uint8_t flags = pData[0]; if (!(flags & 0x01)) return;
  uint32_t wheelRevs = (uint32_t)pData[1] | ((uint32_t)pData[2] << 8) | ((uint32_t)pData[3] << 16) | ((uint32_t)pData[4] << 24);
  uint16_t wheelTime = (uint16_t)pData[5] | ((uint16_t)pData[6] << 8);
  static uint32_t lastRevs = 0; static uint16_t lastTime = 0;
  if (wheelDataValid) {
    uint16_t timeDelta = wheelTime - lastTime; uint32_t revDelta  = wheelRevs - lastRevs;
    if (revDelta > 0 && revDelta < 100 && timeDelta > 0) {
      float dt = timeDelta / 1024.0f;
      float meters = revDelta * cfg.wheelCircumference;
      float kmh = (meters / dt) * 3.6f;
      if (kmh < 150.0f) {
        bike.speed = kmh;
        if (kmh > bike.maxSpeed) bike.maxSpeed = kmh;
        lastRevolutionMs = millis();
        bool countIt = !cfgManualMode() || cfgManualCountKm();
        if (countIt) { bike.dailyDistance += meters; bike.totalDistance += meters; bikeDirty = true; }
      }
    }
  }
  lastRevs = wheelRevs; lastTime = wheelTime; wheelDataValid = true; lastWheelPacketMs = millis();
}

static void cleanupClient() {
  if (!pClient) return;
  if (pClient->isConnected()) pClient->disconnect();
  NimBLEDevice::deleteClient(pClient);
  pClient = nullptr; pMeasurementCharacteristic = nullptr;
  wheelDataValid = false; bleConnected = false; bike.sensorConnected = false;
}

bool connectToSensor(const NimBLEAdvertisedDevice* device) {
  if (pClient) cleanupClient();
  bleClientDeletePending = false;
  pClient = NimBLEDevice::createClient();
  if (!pClient) return false;
  pClient->setClientCallbacks(new MyClientCallback(), true);
  if (!pClient->connect(device)) { Serial.println(F("BLE: connect failed")); cleanupClient(); return false; }
  NimBLERemoteService* svc = pClient->getService(CSC_SERVICE_UUID);
  if (!svc) { Serial.println(F("BLE: CSC не найден")); cleanupClient(); return false; }
  pMeasurementCharacteristic = svc->getCharacteristic(CSC_MEASUREMENT_UUID);
  if (!pMeasurementCharacteristic) { Serial.println(F("BLE: хар-ка не найдена")); cleanupClient(); return false; }
  if (!pMeasurementCharacteristic->canNotify()) { Serial.println(F("BLE: notify не подд.")); cleanupClient(); return false; }
  pMeasurementCharacteristic->subscribe(true, notifyCallback);
  Serial.println(F("BLE: подписка активна"));
  return true;
}

void scanForSensor() {
  if (bleConnected || scanning) return;
  scanning = true;
  setAnnouncement("Поиск датчика.");
  Serial.print(F("BLE: скан (")); Serial.print(cfg.bleScanDurationMs); Serial.println(F(" мс)..."));
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true); pScan->setInterval(100); pScan->setWindow(99);
  NimBLEScanResults results = pScan->getResults(cfg.bleScanDurationMs, false);
  int count = results.getCount();
  for (int i = 0; i < count; i++) {
    const NimBLEAdvertisedDevice* dev = results.getDevice(i);
    if (!dev) continue;
    if (dev->haveServiceUUID() && dev->isAdvertisingService(NimBLEUUID(CSC_SERVICE_UUID))) {
      if (connectToSensor(dev)) { bleConnected = true; scanning = false; return; }
    }
  }
  scanning = false;
}

// ============================================================
//                        NVS
// ============================================================
void configDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CFG_MAGIC; cfg.version = CFG_VERSION;
  strlcpy(cfg.apSSID, AP_SSID_DEFAULT, sizeof(cfg.apSSID));
  strlcpy(cfg.webUser, "admin", sizeof(cfg.webUser));
  cfg.wheelCircumference = 2.105f; cfg.armDelaySec = 1;
  cfg.staBootTimeoutSec = 30; cfg.staCheckIntervalSec = 5; cfg.armSpeedIgnoreKmh = 1.0f;
  cfg.sleepHour = 22; cfg.sleepMin = 0; cfg.wakeHour = 6; cfg.wakeMin = 0;
  cfg.sleepGraceSec = 300; cfg.bleScanIntervalSec = 15;
  cfg.bleScanDurationMs = 4000; cfg.speedStopMs = 3000;
}

void saveConfig() { prefs.begin("bike", false); prefs.putBytes("cfg", &cfg, sizeof(cfg)); prefs.end(); }

void loadConfig() {
  prefs.begin("bike", true);
  size_t n = prefs.getBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  if (n == sizeof(cfg) && cfg.magic == CFG_MAGIC && cfg.version == CFG_VERSION) {
    if (strlen(cfg.apSSID) == 0) strlcpy(cfg.apSSID, AP_SSID_DEFAULT, sizeof(cfg.apSSID));
    Serial.println(F("CFG: загружен v6")); return;
  }
  if (n == sizeof(OldConfigV5)) {
    Serial.println(F("CFG: миграция v5 -> v6"));
    OldConfigV5 old;
    prefs.begin("bike", true); prefs.getBytes("cfg", &old, sizeof(old)); prefs.end();
    if (old.magic == CFG_MAGIC && old.version == 5) {
      configDefaults();
      memcpy(cfg.wifiSSID, old.wifiSSID, sizeof(cfg.wifiSSID));
      memcpy(cfg.wifiPass, old.wifiPass, sizeof(cfg.wifiPass));
      memcpy(cfg.apPass, old.apPass, sizeof(cfg.apPass));
      memcpy(cfg.webUser, old.webUser, sizeof(cfg.webUser));
      memcpy(cfg.webPass, old.webPass, sizeof(cfg.webPass));
      memcpy(cfg.staticIP, old.staticIP, sizeof(cfg.staticIP));
      memcpy(cfg.staticGW, old.staticGW, sizeof(cfg.staticGW));
      cfg.wheelCircumference = old.wheelCircumference; cfg.armDelaySec = old.armDelaySec;
      cfg.staBootTimeoutSec = old.staBootTimeoutSec; cfg.staCheckIntervalSec = old.staCheckIntervalSec;
      cfg.armSpeedIgnoreKmh = old.armSpeedIgnoreKmh;
      cfg.sleepHour = old.sleepHour; cfg.sleepMin = old.sleepMin; cfg.wakeHour = old.wakeHour; cfg.wakeMin = old.wakeMin;
      cfg.sleepGraceSec = old.sleepGraceSec; cfg.lastDailyResetDay = old.lastDailyResetDay;
      cfg.bleScanIntervalSec = old.bleScanIntervalSec; cfg.bleScanDurationMs = old.bleScanDurationMs;
      cfg.speedStopMs = old.speedStopMs; cfg.flags = old.flags;
      saveConfig(); return;
    }
  }
  Serial.println(F("CFG: дефолт"));
  configDefaults(); saveConfig();
}

void loadBikeData() {
  prefs.begin("bike", true);
  bike.totalDistance = prefs.getFloat("totalDist", 0.0f);
  bike.dailyDistance = prefs.getFloat("dailyDist", 0.0f);
  bike.maxSpeed = prefs.getFloat("maxSpeed", 0.0f);
  bike.rideTime = prefs.getULong("rideTime", 0);
  bike.totalRideTime = prefs.getULong("totalRideTime", 0);
  prefs.end();
}

void saveBikeDataNow() {
  float td = bike.totalDistance, dd = bike.dailyDistance;
  float ms = bike.maxSpeed;
  uint32_t rt = bike.rideTime, trt = bike.totalRideTime;
  prefs.begin("bike", false);
  prefs.putFloat("totalDist", td); prefs.putFloat("dailyDist", dd);
  prefs.putFloat("maxSpeed", ms); prefs.putULong("rideTime", rt);
  prefs.putULong("totalRideTime", trt); prefs.end();
}

void wipeAll() {
  prefs.begin("bike", false); prefs.clear(); prefs.end();
  configDefaults(); saveConfig();
  bike.totalDistance = 0; bike.dailyDistance = 0; bike.maxSpeed = 0;
  bike.rideTime = 0; bike.totalRideTime = 0; bike.speed = 0;
  lastAnnouncedKm = 0; lastAnnouncedHour = 0; lastAnnouncedMaxSpeed = 15.0f;
  pendingAnnouncement[0] = 0; pendingAnnouncementUrgent = false;
  dailyResetPending = false;
}

// ============================================================
//                        ВРЕМЯ И ОХРАНА
// ============================================================
uint32_t currentLocalDay() {
  time_t now = time(nullptr); struct tm ti; localtime_r(&now, &ti);
  return (uint32_t)((ti.tm_year + 1900) * 10000 + (ti.tm_mon + 1) * 100 + ti.tm_mday);
}

void checkDailyReset() {
  if (!timeSynced) return;
  uint32_t today = currentLocalDay();
  if (today < 20240101UL || today > 21001231UL) return;
  if (cfg.lastDailyResetDay == today) return;
  bike.dailyDistance = 0; bike.maxSpeed = 0; bike.rideTime = 0;
  lastAnnouncedKm = 0; lastAnnouncedHour = 0; lastAnnouncedMaxSpeed = 15.0f;
  cfg.lastDailyResetDay = today; dailyResetPending = true;
  saveBikeDataNow(); saveConfig();
}

void timeTick() {
  if (timeSynced) return;
  if (WiFi.status() != WL_CONNECTED) return;
  static unsigned long last = 0;
  if (millis() - last < 3000) return;
  last = millis();
  if (time(nullptr) > 1600000000) {
    timeSynced = true; setAnnouncement("Время синхронизировано.");
    Serial.println(F("NTP: OK"));
  }
}

int apClientCount() { wifi_sta_list_t list; if (esp_wifi_ap_get_sta_list(&list) != ESP_OK) return 0; return list.num; }

bool applyStaStaticConfig() {
  if (!cfgStaticIP()) return false;
  if (strlen(cfg.staticIP) == 0 || strlen(cfg.staticGW) == 0) return false;
  IPAddress ip, gw, mask(255, 255, 255, 0);
  if (!ip.fromString(cfg.staticIP)) return false;
  if (!gw.fromString(cfg.staticGW)) return false;
  if (!WiFi.config(ip, gw, mask, gw)) return false;
  return true;
}

void wifiBoot() {
  WiFi.setHostname(HOSTNAME); WiFi.softAPsetHostname(HOSTNAME); WiFi.setSleep(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  const char* ssid = strlen(cfg.apSSID) ? cfg.apSSID : AP_SSID_DEFAULT;
  if (strlen(cfg.apPass) >= 8) WiFi.softAP(ssid, cfg.apPass); else WiFi.softAP(ssid);
  if (cfgConfigured() && strlen(cfg.wifiSSID) > 0) { applyStaStaticConfig(); WiFi.begin(cfg.wifiSSID, cfg.wifiPass); }
}

void wifiTick() {
  if (sleeping) return;
  static int lastApCount = 0;
  int nowApCount = apClientCount();
  if (nowApCount > lastApCount) {
    setAnnouncement("Внимание. К вело компьютеру подключено устройство.", true);
  }
  lastApCount = nowApCount;
  bool staNow = (WiFi.status() == WL_CONNECTED);
  if (staNow && !sysNormalAnnounced) {
    sysNormalAnnounced = true; MDNS.end(); delay(50);
    if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
    setAnnouncement("Система активирована и работает в норме.");
  } else if (!staNow) { sysNormalAnnounced = false; }
  if (cfgConfigured() && strlen(cfg.wifiSSID) > 0 && !staNow) {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > cfg.staCheckIntervalSec * 1000UL) {
      lastRetry = millis(); applyStaStaticConfig(); WiFi.begin(cfg.wifiSSID, cfg.wifiPass);
    }
  }
}

bool ownerNearby() { return (WiFi.status() == WL_CONNECTED) || (apClientCount() > 0); }

void setRelay(bool on, const char* src) { digitalWrite(PIN_RELAY, on ? HIGH : LOW); relayIsOn = on; }

void alarmTick() {
  unsigned long now = millis();
  if (testRelayUntil) {
    if (!timeReached(now, testRelayUntil)) { digitalWrite(PIN_RELAY, HIGH); relayIsOn = true; return; }
    testRelayUntil = 0;
  }
  if (cfgManualMode()) { alarmState = ALARM_MANUAL; setRelay(cfgManualRelay(), "ручной"); return; }
  if (!cfgConfigured()) { setRelay(false, "не настроено"); return; }
  if (!cfgAlarmEnabled()) { ownerGoneSince = 0; alarmState = ALARM_READY; setRelay(false, "выкл"); return; }
  bool owner = ownerNearby(), riding = (bike.speed > cfg.armSpeedIgnoreKmh);
  if (owner || riding) {
    if (alarmState == ALARM_ARMED) setAnnouncement("Охрана снята.");
    ownerGoneSince = 0; alarmState = ALARM_READY; setRelay(true, owner ? "хозяин" : "движение"); return;
  }
  if (ownerGoneSince == 0) { ownerGoneSince = now; alarmState = ALARM_ARMING; setRelay(true, "arm"); return; }
  unsigned long gone = now - ownerGoneSince;
  if (gone < cfg.armDelaySec * 1000UL) { alarmState = ALARM_ARMING; setRelay(true, "arm"); return; }
  alarmState = ALARM_ARMED; setRelay(false, "охрана");
}

const char* alarmStateStr() {
  switch (alarmState) {
    case ALARM_READY:  return "ready";
    case ALARM_ARMING: return "arming";
    case ALARM_ARMED:  return "armed";
    case ALARM_MANUAL: return "manual";
  }
  return "?";
}

// ============================================================
//                        СОН
// ============================================================
bool inSleepWindow() {
  if (cfgManualMode() || !cfgSleepEnabled() || !timeSynced) return false;
  time_t now = time(nullptr); struct tm ti; localtime_r(&now, &ti);
  int nowMin = ti.tm_hour * 60 + ti.tm_min;
  int sleepMin = cfg.sleepHour * 60 + cfg.sleepMin, wakeMin = cfg.wakeHour * 60 + cfg.wakeMin;
  if (sleepMin == wakeMin) return false;
  if (sleepMin < wakeMin) return (nowMin >= sleepMin && nowMin < wakeMin);
  return (nowMin >= sleepMin || nowMin < wakeMin);
}

bool canSleep() {
  if (cfgManualMode() || !cfgSleepEnabled() || !timeSynced) return false;
  if (millis() - bootMs < cfg.sleepGraceSec * 1000UL) return false;
  if (!inSleepWindow() || ownerNearby()) return false;
  if (bike.speed > cfg.armSpeedIgnoreKmh || relayIsOn) return false;
  return true;
}

void enterSleep() {
  setRelay(false, "сон"); MDNS.end(); delay(50);
  WiFi.disconnect(true); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF);
  sleeping = true; digitalWrite(PIN_LED, LOW);
}

void wakeFromSleep() {
  sleeping = false; bootMs = millis(); wifiBoot();
  MDNS.begin(HOSTNAME); MDNS.addService("http", "tcp", 80);
  if (cfgConfigured() && strlen(cfg.wifiSSID) > 0) { configTime(0, 0, NTP_SERVER); setenv("TZ", TZ_KYIV, 1); tzset(); }
}

// ============================================================
//          HTML: SETUP (STREAMED, ESCAPED)
// ============================================================
void sendSetupPage(AsyncWebServerRequest* r) {
  AsyncResponseStream *response = r->beginResponseStream("text/html; charset=utf-8");
  response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  response->print(F("<!DOCTYPE html><html lang='ru'><head><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>OHORONA BIKE 4.0</title><style>"
    "body{font-family:Arial,sans-serif;background:#111;color:#eee;max-width:560px;margin:0 auto;padding:16px}"
    "h1{color:#00ff88;text-align:center;font-size:1.4em}"
    ".warn{background:#531;border:1px solid #a63;border-radius:8px;padding:12px;margin:14px 0;font-size:.9em}"
    ".tabs{display:flex;gap:3px;flex-wrap:wrap;margin:14px 0}"
    ".tab{flex:1;min-width:60px;padding:8px 3px;text-align:center;background:#222;border:1px solid #444;"
    "border-radius:6px 6px 0 0;cursor:pointer;font-size:.75em;font-weight:bold;color:#aaa}"
    ".tab.active{background:#2a6;color:#fff;border-color:#2a6}"
    ".pane{display:none;border:1px solid #444;border-radius:0 6px 6px 6px;padding:14px}"
    ".pane.active{display:block}"
    "fieldset{border:0;margin:0 0 14px;padding:0}"
    "legend{color:#00ff88;font-weight:bold;margin-bottom:8px}"
    "label{display:block;margin:8px 0 3px;font-size:.9em;color:#bbb}"
    "input[type=text],input[type=password],input[type=number]{width:100%;padding:9px;background:#222;"
    "color:#eee;border:1px solid #555;border-radius:5px;box-sizing:border-box;font-size:15px}"
    "input[type=checkbox]{margin-right:6px}"
    ".m{color:#888;font-size:.8em;margin-top:3px;line-height:1.4}"
    "button{width:100%;padding:14px;font-size:16px;background:#00ff88;border:none;color:#000;"
    "border-radius:8px;font-weight:bold;margin-top:16px;cursor:pointer}"
    "a{color:#00ff88}"
    "</style></head><body><h1>🚴 OHORONA BIKE 4.0</h1>"));

  if (!cfgConfigured()) response->print(F("<div class='warn'><b>Первый запуск.</b> Заполните все поля.</div>"));

  response->print(F("<div class='tabs'>"
    "<div class='tab active' data-t='wifi'>Wi-Fi</div>"
    "<div class='tab' data-t='sensor'>Датчик</div>"
    "<div class='tab' data-t='alarm'>Охрана</div>"
    "<div class='tab' data-t='sleep'>Сон</div>"
    "<div class='tab' data-t='wheel'>Колесо</div>"
    "<div class='tab' data-t='pass'>Пароли</div>"
    "</div><form action='/setup' method='POST' onsubmit=\"localStorage.setItem('ohorona-setup-saved','1');\">"));

  response->print(F("<div class='pane active' id='pane-wifi'><fieldset><legend>Точка доступа телефона</legend>"
    "<label>SSID хотспота</label><input type='text' name='wifiSSID' maxlength='32' value='"));
  streamEscaped(response, cfg.wifiSSID);
  response->print(F("'><label>Пароль хотспота</label><input type='password' name='wifiPass' maxlength='64' placeholder='оставь пустым — не менять'>"
    "<label>Период попыток, сек</label><input type='number' name='staCheckIntervalSec' min='2' max='120' value='"));
  response->print(cfg.staCheckIntervalSec);
  response->print(F("'></fieldset><fieldset><legend>Статический IP</legend>"
    "<label><input type='checkbox' name='useStaticIP'"));
  if (cfgStaticIP()) response->print(F(" checked"));
  response->print(F("> Использовать статический IP</label><label>IP-адрес</label><input type='text' name='staticIP' maxlength='15' value='"));
  streamEscaped(response, cfg.staticIP);
  response->print(F("'><label>Шлюз</label><input type='text' name='staticGW' maxlength='15' value='"));
  streamEscaped(response, cfg.staticGW);
  response->print(F("'></fieldset></div>"));

  response->print(F("<div class='pane' id='pane-sensor'><fieldset><legend>Поиск BLE</legend>"
    "<label>Отдых, сек</label><input type='number' name='bleScanIntervalSec' min='1' max='600' value='"));
  response->print(cfg.bleScanIntervalSec);
  response->print(F("'><label>Время поиска, мс</label><input type='number' name='bleScanDurationMs' min='500' max='15000' value='"));
  response->print(cfg.bleScanDurationMs);
  response->print(F("'></fieldset></div>"));

  response->print(F("<div class='pane' id='pane-alarm'><fieldset><legend>Сигнализация</legend>"
    "<label><input type='checkbox' name='alarmEnabled'"));
  if (cfgAlarmEnabled()) response->print(F(" checked"));
  response->print(F("> Включить охрану</label><label>Задержка, сек</label><input type='number' name='armDelaySec' min='1' max='60' value='"));
  response->print(cfg.armDelaySec);
  response->print(F("'><label>Скорость остановки, км/ч</label><input type='number' step='0.1' name='armSpeedIgnoreKmh' min='0' max='10' value='"));
  response->print(cfg.armSpeedIgnoreKmh, 1);
  response->print(F("'><label><input type='checkbox' name='manualMode'"));
  if (cfgManualMode()) response->print(F(" checked"));
  response->print(F("> Ручной режим</label><label><input type='checkbox' name='manualRelay'"));
  if (cfgManualRelay()) response->print(F(" checked"));
  response->print(F("> Реле ВКЛ</label><label><input type='checkbox' name='manualCountKm'"));
  if (cfgManualCountKm()) response->print(F(" checked"));
  response->print(F("> Считать км</label></fieldset></div>"));

  response->print(F("<div class='pane' id='pane-sleep'><fieldset><legend>Окно сна</legend>"
    "<label><input type='checkbox' name='sleepEnabled'"));
  if (cfgSleepEnabled()) response->print(F(" checked"));
  response->print(F("> Сон включён</label><label>Уснуть (ч:м)</label><input type='number' name='sleepHour' min='0' max='23' value='"));
  response->print(cfg.sleepHour);
  response->print(F("' style='width:80px'> : <input type='number' name='sleepMin' min='0' max='59' value='"));
  response->print(cfg.sleepMin);
  response->print(F("' style='width:80px'><label>Проснуться (ч:м)</label><input type='number' name='wakeHour' min='0' max='23' value='"));
  response->print(cfg.wakeHour);
  response->print(F("' style='width:80px'> : <input type='number' name='wakeMin' min='0' max='59' value='"));
  response->print(cfg.wakeMin);
  response->print(F("' style='width:80px'><label>Grace, сек</label><input type='number' name='sleepGraceSec' min='0' max='3600' value='"));
  response->print(cfg.sleepGraceSec);
  response->print(F("'></fieldset></div>"));

  response->print(F("<div class='pane' id='pane-wheel'><fieldset><legend>Велосипед</legend>"
    "<label>Окружность, мм</label><input type='number' name='wheelMm' min='500' max='4000' value='"));
  response->print((int)(cfg.wheelCircumference * 1000.0f + 0.5f));
  response->print(F("'><label>Остановка, сек</label><input type='number' name='speedStopSec' min='1' max='30' value='"));
  response->print(cfg.speedStopMs / 1000);
  response->print(F("'></fieldset></div>"));

  response->print(F("<div class='pane' id='pane-pass'><fieldset><legend>Пароли</legend>"
    "<label>Имя точки доступа</label><input type='text' name='apSSID' maxlength='32' value='"));
  streamEscaped(response, cfg.apSSID);
  response->print(F("'><label>Пароль точки (мин. 8)</label><input type='password' name='apPass' maxlength='63' placeholder='не менять'>"
    "<label>Логин</label><input type='text' name='webUser' maxlength='16' value='"));
  streamEscaped(response, cfg.webUser);
  response->print(F("'><label>Пароль панели</label><input type='password' name='webPass' maxlength='32' placeholder='не менять'></fieldset></div>"));

  response->print(F("<button type='submit'>Сохранить и перезагрузить</button></form>"
    "<p style='text-align:center;margin-top:20px'><a href='/update'>📦 Прошивка (OTA)</a> · <a href='/'>← на панель</a></p>"
    "<script>document.querySelectorAll('.tab').forEach(t=>{t.onclick=()=>{document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.pane').forEach(x=>x.classList.remove('active'));t.classList.add('active');document.getElementById('pane-'+t.dataset.t).classList.add('active');};});</script>"
    "</body></html>"));
  r->send(response);
}

// ============================================================
//          HTML: ПАНЕЛЬ (PROGMEM)
// ============================================================
const char BIKE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="ru"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>OHORONA BIKE 4.0</title>
<style>
:root{ --bg:#f4f1d0; --ink:#001a6e; --cell-bg:#f0efe5; --cell-border:#d8d5b8; --lbl:#666; --pink:#a0195c; }
body.night{ --bg:#0a0a2e; --ink:#f4f1d0; --cell-bg:#1a1a3a; --cell-border:#2a2a4a; --lbl:#999; --pink:#ff6fa5; }
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent;-webkit-user-select:none;user-select:none}
html,body{height:100%;overflow:hidden;font-family:'Arial Narrow','Roboto Condensed',Arial,sans-serif;background:#111;color:var(--ink)}
.wrap{height:100vh;height:100dvh;display:flex;flex-direction:column;background:var(--bg);transition:background .25s}
.top{flex:1 1 50%;position:relative;min-height:0;overflow:hidden;background:var(--bg);transition:background .25s}
.speedo-svg{width:100%;height:100%;display:block;color:var(--ink)}
.clockbox{position:absolute;top:6px;left:10px;line-height:1;pointer-events:none}
.clock{font-size:clamp(18px,4.2vw,28px);font-weight:bold;letter-spacing:1px;font-variant-numeric:tabular-nums;color:var(--ink);opacity:.9;line-height:1}
.date{font-size:clamp(11px,2.2vw,15px);font-weight:bold;letter-spacing:1.5px;font-variant-numeric:tabular-nums;color:var(--ink);opacity:.65;line-height:1;margin-top:3px}
.topright{position:absolute;top:6px;right:8px;display:flex;flex-direction:column;align-items:flex-end;gap:3px}
.icons{display:flex;align-items:center;gap:4px;font-size:13px;font-weight:bold;color:var(--lbl)}
.ip{color:#999;font-weight:normal;font-size:11px;letter-spacing:.5px;text-align:right}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block;background:#cc2222;transition:background .3s}
.dot.ok{background:#2a9d2a}.dot.scan{background:#e08600;animation:blink .5s infinite}.dot.none{background:#cc2222;animation:blink .7s infinite}
.sq{width:9px;height:9px;border-radius:2px;display:inline-block;margin-left:2px;vertical-align:middle}
.sq.ready{background:#2a9d2a}.sq.armed{background:#cc2222;animation:blink .7s infinite}.sq.arming{background:#e08600;animation:blink .5s infinite}.sq.manual{background:#0066cc}
@keyframes blink{50%{opacity:.2}}
.scan-btn{background:transparent;border:1.5px solid var(--ink);color:var(--ink);border-radius:5px;font-size:13px;padding:2px 6px;font-weight:bold;cursor:pointer;line-height:1.2;text-decoration:none;display:inline-block;font-family:inherit;min-width:26px;text-align:center}
.scan-btn:active{background:var(--ink);color:var(--bg)}.scan-btn.alert{border-color:var(--pink);color:var(--pink)}.scan-btn.alert:active{background:var(--pink);color:var(--bg)}
.alarm-btn{display:inline-flex;align-items:center;gap:2px}
.big-speed{position:absolute;right:4%;bottom:2%;text-align:right;line-height:.8;pointer-events:none}
.big-speed .val{font-size:clamp(90px,26vw,280px);font-weight:900;color:var(--ink);letter-spacing:-6px;line-height:.8;font-family:'Arial Narrow',Impact,sans-serif;font-variant-numeric:tabular-nums}
.big-speed .unit{font-size:clamp(14px,3vw,28px);font-weight:bold;color:var(--ink);letter-spacing:3px;margin-top:2px}
.night-btn{position:absolute;bottom:12px;left:12px;z-index:5;background:transparent;border:1.5px solid var(--ink);color:var(--ink);border-radius:50%;width:38px;height:38px;font-size:18px;cursor:pointer;padding:0;line-height:1;display:flex;align-items:center;justify-content:center;font-family:inherit}
.night-btn:active{background:var(--ink);color:var(--bg)}
.grid{flex:1 1 50%;display:grid;grid-template-columns:1fr 1fr;grid-template-rows:32fr 32fr 36fr;gap:4px;padding:4px;background:#111}
.cell{background:var(--cell-bg);border-radius:6px;padding:6px 8px;display:flex;flex-direction:column;justify-content:center;align-items:center;text-align:center;position:relative;overflow:hidden;border:1px solid var(--cell-border);transition:background .25s, border-color .25s}
.cell .lbl{font-size:clamp(11px,2.4vw,14px);letter-spacing:2px;text-transform:uppercase;color:var(--lbl);font-weight:bold;margin-bottom:4px;line-height:1}
.cell .days{font-size:clamp(11px,2.4vw,14px);color:var(--lbl);font-weight:bold;line-height:1;margin-bottom:2px;min-height:14px}
.cell .num{font-size:clamp(40px,15vw,100px);font-weight:900;color:var(--ink);line-height:1;font-family:'Arial Narrow',Impact,sans-serif;letter-spacing:-1px;font-variant-numeric:tabular-nums;white-space:nowrap}
.cell .num small{font-size:.35em;color:var(--lbl);font-weight:bold;margin-left:4px;letter-spacing:1px}
.cell.pink .num{color:var(--pink)}.cell.pink .lbl{color:var(--pink)}
.cell.tappable{cursor:pointer}.cell.tappable:active{filter:brightness(.92)}
.dialog-bg{display:none;position:fixed;inset:0;background:rgba(0,0,0,.75);z-index:100;align-items:center;justify-content:center;padding:12px}
.dialog-bg.show{display:flex}
.dialog{background:#f4f1d0;padding:20px;border-radius:10px;max-width:360px;width:100%;color:#001a6e}
.dialog h3{margin-bottom:12px;font-size:18px}.dialog .current{font-size:14px;color:#555;margin-bottom:10px}
.dialog input{width:100%;padding:10px;background:#fff;color:#001a6e;border:1px solid #888;border-radius:6px;font-size:16px;margin-bottom:10px}
.dialog button{width:100%;padding:11px;font-size:15px;background:#a0195c;color:#fff;border:none;border-radius:6px;font-weight:bold;margin-top:6px;cursor:pointer;font-family:inherit}
.dialog button.alt{background:#001a6e}.dialog button.ok{background:#2a9d2a}.dialog button.cancel{background:#888}
.dialog .err{color:#a0195c;font-size:13px;min-height:16px;margin-top:6px}
.dialog .stat{background:#fff;border:1px solid #ccc;border-radius:6px;padding:10px;margin-bottom:10px;font-size:14px}
.dialog .stat b{color:#001a6e}
</style></head><body>
<div class="wrap">
  <div class="top">
    <svg class="speedo-svg" viewBox="0 0 400 260" preserveAspectRatio="xMidYMid meet">
      <g id="ticks"></g>
      <g id="numbers" font-family="'Arial Narrow', Impact, sans-serif" font-weight="900" fill="currentColor" text-anchor="middle" dominant-baseline="middle"></g>
      <g id="needle" transform="rotate(-90 240 220)"><polygon points="234,220 240,45 246,220" fill="currentColor"/></g>
      <circle cx="240" cy="220" r="9" fill="currentColor"/><circle cx="240" cy="220" r="3" fill="var(--bg)"/>
    </svg>
    <div class="clockbox"><div class="clock" id="clock">--:--:--</div><div class="date" id="date">--.--.----</div></div>
    <div class="topright">
      <div class="icons">
        <button class="scan-btn alarm-btn" onclick="openAlarm()" title="Охрана"><span id="lockIcon">🔓</span><span class="sq ready" id="alarmSq"></span></button>
        <span class="dot" id="sensorDot"></span><span id="sensorStatus">нет</span>
        <button class="scan-btn" onclick="forceScan()" title="Скан">🔍</button>
        <button class="scan-btn" id="voiceBtn" onclick="toggleVoice()" title="Голос">🔊</button>
        <a href="/setup" class="scan-btn" title="Настройки">⚙️</a>
      </div>
      <div class="ip" id="ipDisplay"></div>
    </div>
    <button class="night-btn" id="nightBtn" onclick="toggleNight()" title="Ночной">🌙</button>
    <div class="big-speed"><div class="val" id="speedVal">0</div><div class="unit">КМ/Ч</div></div>
  </div>
  <div class="grid">
    <div class="cell pink tappable" id="dailyCell"><div class="lbl">Суточный</div><div class="num" id="dailyVal">0.00<small>км</small></div></div>
    <div class="cell"><div class="lbl">Время</div><div class="num" id="rideVal">0:00</div></div>
    <div class="cell"><div class="lbl">Средняя</div><div class="num" id="avgVal">0.0<small>км/ч</small></div></div>
    <div class="cell"><div class="lbl">Макс. сегодня</div><div class="num" id="maxVal">0.0<small>км/ч</small></div></div>
    <div class="cell tappable" id="totalCell"><div class="lbl">Общий</div><div class="num" id="totalVal">0.0<small>км</small></div></div>
    <div class="cell"><div class="lbl">Всего время</div><div class="days" id="totalRideDays"></div><div class="num" id="totalRideVal">0:00</div></div>
  </div>
</div>
<div class="dialog-bg" id="resetTotalBg"><div class="dialog"><h3>Сброс общего пробега</h3><input type="password" id="resetTotalPass" placeholder="Пароль"><button onclick="doResetTotal()">Сбросить</button><button class="cancel" onclick="closeDialog('resetTotalBg')">Отмена</button><div class="err" id="resetTotalErr"></div></div></div>
<div class="dialog-bg" id="resetDailyBg"><div class="dialog"><h3>Сброс суточного</h3><input type="password" id="resetDailyPass" placeholder="Пароль"><button onclick="doResetDaily()">Сбросить</button><button class="cancel" onclick="closeDialog('resetDailyBg')">Отмена</button><div class="err" id="resetDailyErr"></div></div></div>
<div class="dialog-bg" id="alarmBg"><div class="dialog"><h3>Сигнализация</h3><div class="stat" id="alarmStat">—</div><input type="password" id="alarmPass" placeholder="Пароль"><button class="alt" onclick="toggleManual()">Ручной режим</button><button class="alt" onclick="toggleRelay()">Реле</button><button class="ok" onclick="testRelay()">Тест реле 5 сек</button><button class="cancel" onclick="closeDialog('alarmBg')">Закрыть</button><div class="err" id="alarmErr"></div></div></div>
<script>
const SCALE_MAX = 50, PIVOT_X = 240, PIVOT_Y = 220;
function outerPos(v){if(v<=20){const t=v/20;return{x:6,y:218-t*178}}else if(v<=25){const t=(v-20)/5,a=(180+t*90)*Math.PI/180;return{x:40+Math.cos(a)*34,y:40+Math.sin(a)*34}}else{const t=(v-25)/25;return{x:40+t*330,y:6}}}
(function(){const NS='http://www.w3.org/2000/svg',ticks=document.getElementById('ticks'),nums=document.getElementById('numbers'),lenMaj=27,lenMid=19,lenMin=12;
for(let v=0;v<=SCALE_MAX;v++){const p=outerPos(v),isMaj=(v%10===0),isMid=(v%5===0),len=isMaj?lenMaj:(isMid?lenMid:lenMin);let xi,yi;
if(v<=20){xi=p.x+len;yi=p.y}else if(v<=25){const dx=40-p.x,dy=40-p.y,d=Math.hypot(dx,dy);xi=p.x+(dx/d)*len;yi=p.y+(dy/d)*len}else{xi=p.x;yi=p.y+len}
const line=document.createElementNS(NS,'line');line.setAttribute('x1',p.x.toFixed(1));line.setAttribute('y1',p.y.toFixed(1));line.setAttribute('x2',xi.toFixed(1));line.setAttribute('y2',yi.toFixed(1));line.setAttribute('stroke','currentColor');line.setAttribute('stroke-width',isMaj?3.5:(isMid?2.5:1.8));line.setAttribute('stroke-linecap','butt');ticks.appendChild(line)}
for(let v=0;v<=SCALE_MAX;v+=10){const p=outerPos(v);let tx,ty;if(v<=10){tx=p.x+lenMaj+20;ty=p.y}else if(v===20){tx=53;ty=53}else{tx=p.x;ty=p.y+lenMaj+20}
const el=document.createElementNS(NS,'text');el.setAttribute('x',tx.toFixed(1));el.setAttribute('y',ty.toFixed(1));el.setAttribute('font-size','30');el.textContent=v;nums.appendChild(el)}})();
function toggleNight(){const on=!document.body.classList.contains('night');document.body.classList.toggle('night',on);document.getElementById('nightBtn').textContent=on?'☀️':'🌙';try{localStorage.setItem('ohorona-night',on?'1':'0')}catch(e){}} 
(function(){try{if(localStorage.getItem('ohorona-night')==='1'){document.body.classList.add('night');document.getElementById('nightBtn').textContent='☀️'}}catch(e){}})();
function setNeedle(kmh){const s=Math.max(0,Math.min(kmh,SCALE_MAX)),target=outerPos(s),dx=target.x-PIVOT_X,dy=target.y-PIVOT_Y,theta=Math.atan2(dx,-dy)*180/Math.PI;document.getElementById('needle').setAttribute('transform','rotate('+theta.toFixed(1)+' '+PIVOT_X+' '+PIVOT_Y+')')}
function fmtRide(sec){const h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60),s=sec%60;return h>0?h+':'+String(m).padStart(2,'0')+':'+String(s).padStart(2,'0'):m+':'+String(s).padStart(2,'0')}
function pluralDays(n){const m100=n%100,m10=n%10;if(m100>=11&&m100<=14)return'дней';if(m10===1)return'день';if(m10>=2&&m10<=4)return'дня';return'дней'}
function pluralRu(n,one,few,many){const m100=n%100,m10=n%10;if(m100>=11&&m100<=14)return many;if(m10===1)return one;if(m10>=2&&m10<=4)return few;return many}
function fmtTotalRide(sec){const days=Math.floor(sec/86400),rem=sec%86400;if(days===0)return{days:'',time:fmtRide(sec)};const h=Math.floor(rem/3600),m=Math.floor((rem%3600)/60),s=rem%60,time=String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(s).padStart(2,'0');return{days:days+' '+pluralDays(days),time:time}}
function fitNum(el){if(!el)return;const cell=el.parentElement;if(!cell)return;el.style.fontSize='';let topH=0;for(const child of cell.children){if(child===el)break;topH+=child.offsetHeight+4}const availW=cell.clientWidth-12,availH=cell.clientHeight-topH-8;let size=parseFloat(getComputedStyle(el).fontSize),guard=40;while(size>10&&guard-->0){el.style.fontSize=size+'px';if(el.scrollWidth<=availW&&el.scrollHeight<=availH)break;size-=2}}
function fitAll(){['dailyVal','rideVal','avgVal','maxVal','totalVal','totalRideVal'].forEach(id=>fitNum(document.getElementById(id)))}

// v4.4.2: «89 минут» → «1 час 29 минут»
function fmtRideHuman(sec){
  const h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60),s=sec%60;
  if(h>0){
    const hw=pluralRu(h,'час','часа','часов');
    if(m===0)return h+' '+hw;
    return h+' '+hw+' '+m+' '+pluralRu(m,'минута','минуты','минут');
  } else if(m>0){
    const mw=pluralRu(m,'минута','минуты','минут');
    if(s===0)return m+' '+mw;
    return m+' '+mw+' '+s+' '+pluralRu(s,'секунда','секунды','секунд');
  } else {
    return s+' '+pluralRu(s,'секунда','секунды','секунд');
  }
}

// v4.4.2: «X километров Y метров» — то же, что formatDistance в C++
function fmtDistHuman(meters){
  const totalM=Math.round(meters);
  const km=Math.floor(totalM/1000),m=totalM%1000;
  const kmWord=pluralRu(km,'километр','километра','километров');
  if(m===0)return km+' '+kmWord;
  return km+' '+kmWord+' '+m+' '+pluralRu(m,'метр','метра','метров');
}

// v4.4.2: «HH:MM:SS» → «HH часов MM минут» (без секунд)
function fmtTimeHuman(timeStr){
  if(!timeStr||timeStr.length<5)return "";
  const h=parseInt(timeStr.substr(0,2),10);
  const m=parseInt(timeStr.substr(3,2),10);
  if(isNaN(h)||isNaN(m))return "";
  return "Сейчас "+h+" "+pluralRu(h,'час','часа','часов')+" "+m+" "+pluralRu(m,'минута','минуты','минут')+".";
}

// ============================================================
//   v4.2 SOUND — БИМ-БОМ / СИРЕНА / ОЗВУЧКА
// ============================================================
let audioCtx=null;
function initAudio(){if(audioCtx)return;try{audioCtx=new(window.AudioContext||window.webkitAudioContext)()}catch(e){audioCtx=null}}
document.addEventListener('touchstart',initAudio,{once:true});document.addEventListener('click',initAudio,{once:true});

function playBimBom(){
  if(!audioCtx)initAudio();
  if(!audioCtx)return;
  if(audioCtx.state==='suspended')audioCtx.resume();
  const t=audioCtx.currentTime;
  const o1=audioCtx.createOscillator(),g1=audioCtx.createGain();
  o1.type='sine';o1.frequency.value=350;
  g1.gain.setValueAtTime(0,t);
  g1.gain.linearRampToValueAtTime(0.22,t+0.02);
  g1.gain.linearRampToValueAtTime(0.20,t+0.30);
  g1.gain.linearRampToValueAtTime(0,t+0.40);
  o1.connect(g1);g1.connect(audioCtx.destination);
  o1.start(t);o1.stop(t+0.42);
  const t2=t+0.45;
  const o2=audioCtx.createOscillator(),g2=audioCtx.createGain();
  o2.type='sine';o2.frequency.value=700;
  g2.gain.setValueAtTime(0,t2);
  g2.gain.linearRampToValueAtTime(0.22,t2+0.02);
  g2.gain.linearRampToValueAtTime(0.20,t2+0.42);
  g2.gain.linearRampToValueAtTime(0,t2+0.55);
  o2.connect(g2);g2.connect(audioCtx.destination);
  o2.start(t2);o2.stop(t2+0.57);
}

function playSiren(){
  if(!audioCtx)initAudio();
  if(!audioCtx)return;
  if(audioCtx.state==='suspended')audioCtx.resume();
  const t=audioCtx.currentTime;
  const o=audioCtx.createOscillator(),g=audioCtx.createGain();
  o.type='sine';
  const CYCLES=3, HALF=0.35;
  g.gain.setValueAtTime(0,t);
  g.gain.linearRampToValueAtTime(0.20,t+0.05);
  let ct=t;
  for(let i=0;i<CYCLES*2;i++){
    if(i%2===0){ o.frequency.setValueAtTime(500,ct); o.frequency.linearRampToValueAtTime(1000,ct+HALF); }
    else       { o.frequency.setValueAtTime(1000,ct); o.frequency.linearRampToValueAtTime(500,ct+HALF); }
    ct+=HALF;
  }
  g.gain.setValueAtTime(0.20,ct-0.1);
  g.gain.linearRampToValueAtTime(0,ct);
  o.connect(g);g.connect(audioCtx.destination);
  o.start(t);o.stop(ct+0.02);
}

function formatSpeech(text){
  return text.replace(/(\d+)\.(\d+)/g,function(match,intPart,decPart){if(decPart==='0'||decPart==='00')return intPart;return intPart+' и '+decPart});
}

function speakText(text, onEnd){
  if(!('speechSynthesis'in window)){if(onEnd)setTimeout(onEnd,100);return;}
  const u=new SpeechSynthesisUtterance(formatSpeech(text));
  u.lang='ru-RU';u.rate=1.0;u.pitch=1.0;u.volume=1.0;
  if(onEnd){
    let fired=false;
    const fire=()=>{if(fired)return;fired=true;onEnd();};
    u.onend=fire; u.onerror=fire;
    setTimeout(fire,8000);
  }
  try{window.speechSynthesis.cancel()}catch(e){}
  window.speechSynthesis.speak(u);
}

let voiceEnabled=localStorage.getItem('ohorona-voice')!=='false';
function updateVoiceBtn(){const btn=document.getElementById('voiceBtn');if(!btn)return;if(voiceEnabled){btn.textContent='🔊';btn.style.opacity='1'}else{btn.textContent='🔇';btn.style.opacity='0.5'}}
document.addEventListener('DOMContentLoaded',updateVoiceBtn);

function buildOverview(){
  const d=lastData;
  if(!d) return "Данные ещё не получены.";
  const rt=d.rideTime||0;
  const avg=d.average||0;
  const max=d.maxSpeed||0;
  const distStr=fmtDistHuman(d.daily);
  const rideStr=fmtRideHuman(rt);
  const timeStr=fmtTimeHuman(d.time);
  let s="Проехал за сегодня "+distStr+". Время в пути "+rideStr+". Средняя "+avg.toFixed(1)+" километров в час. Максимальная "+max.toFixed(1)+" километров в час.";
  if(timeStr) s+=" "+timeStr;
  return s;
}

function toggleVoice(){
  voiceEnabled=!voiceEnabled;
  localStorage.setItem('ohorona-voice',voiceEnabled?'true':'false');
  updateVoiceBtn();
  try{window.speechSynthesis.cancel()}catch(e){}
  if(voiceEnabled){
    playBimBom();
    setTimeout(()=>{
      speakText("Голосовые уведомления включены.", ()=>{
        setTimeout(()=>{
          playBimBom();
          setTimeout(()=>speakText(buildOverview()),1100);
        }, 600);
      });
    }, 1100);
  } else {
    playBimBom();
    setTimeout(()=>speakText("Голосовые уведомления выключены."),1100);
  }
}

function announceVoice(text, urgent){
  if(!text)return;
  try{window.speechSynthesis.cancel()}catch(e){}
  if(urgent){
    playSiren();
    setTimeout(()=>speakText(text),2300);
  } else {
    playBimBom();
    setTimeout(()=>speakText(text),1200);
  }
}

document.addEventListener('DOMContentLoaded',()=>{
  if(localStorage.getItem('ohorona-setup-saved')==='1'){
    localStorage.removeItem('ohorona-setup-saved');
    setTimeout(()=>{
      playBimBom();
      setTimeout(()=>speakText("Настройки сохранены."),1200);
    },1000);
  }
});

let lastAlarmState='', lastData=null;
function fetchData(){fetch('/api/bike').then(r=>r.json()).then(d=>{
lastData=d;
document.getElementById('speedVal').textContent=Math.round(d.speed);setNeedle(d.speed);
document.getElementById('clock').textContent=d.time;document.getElementById('date').textContent=d.date||'';
document.getElementById('dailyVal').innerHTML=(d.daily/1000).toFixed(2)+'<small>км</small>';
document.getElementById('rideVal').textContent=fmtRide(d.rideTime);
document.getElementById('avgVal').innerHTML=d.average.toFixed(1)+'<small>км/ч</small>';
document.getElementById('maxVal').innerHTML=d.maxSpeed.toFixed(1)+'<small>км/ч</small>';
document.getElementById('totalVal').innerHTML=(d.total/1000).toFixed(1)+'<small>км</small>';
const tr=fmtTotalRide(d.totalRideTime||0);document.getElementById('totalRideDays').textContent=tr.days;document.getElementById('totalRideVal').textContent=tr.time;
document.getElementById('ipDisplay').textContent=d.ip||'';lastAlarmState=d.alarm;
if(voiceEnabled&&d.announce&&d.announce.length>0){announceVoice(d.announce, d.urgent)}fitAll();
const dot=document.getElementById('sensorDot'),txt=document.getElementById('sensorStatus');dot.className='dot';
if(d.connected){dot.classList.add('ok');txt.textContent='есть'}else if(d.scanning){dot.classList.add('scan');txt.textContent='поиск'}else{dot.classList.add('none');txt.textContent='нет'}
const lock=document.getElementById('lockIcon'),sq=document.getElementById('alarmSq');sq.className='sq';
if(d.alarm==='ready'){lock.textContent='🔓';sq.classList.add('ready')}else if(d.alarm==='arming'){lock.textContent='🔒';sq.classList.add('arming')}else if(d.alarm==='armed'){lock.textContent='🔒';sq.classList.add('armed')}else if(d.alarm==='manual'){lock.textContent='🔓';sq.classList.add('manual')}
}).catch(()=>{})}
setInterval(fetchData,500);fetchData();window.addEventListener('resize',fitAll);

function forceScan(){fetch('/scan',{method:'POST'}).catch(()=>{})}
function closeDialog(id){document.getElementById(id).classList.remove('show')}
function openResetTotal(){document.getElementById('resetTotalBg').classList.add('show');document.getElementById('resetTotalPass').value='';document.getElementById('resetTotalErr').textContent='';setTimeout(()=>document.getElementById('resetTotalPass').focus(),50)}
function doResetTotal(){const pass=document.getElementById('resetTotalPass').value;fetch('/api/reset-total',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'password='+encodeURIComponent(pass)}).then(r=>{if(r.ok){closeDialog('resetTotalBg');fetchData()}else document.getElementById('resetTotalErr').textContent='Неверный пароль'}).catch(()=>document.getElementById('resetTotalErr').textContent='Ошибка сети')}
document.getElementById('totalCell').addEventListener('click',openResetTotal);
function openResetDaily(){document.getElementById('resetDailyBg').classList.add('show');document.getElementById('resetDailyPass').value='';document.getElementById('resetDailyErr').textContent='';setTimeout(()=>document.getElementById('resetDailyPass').focus(),50)}
function doResetDaily(){const pass=document.getElementById('resetDailyPass').value;fetch('/api/reset-daily',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'password='+encodeURIComponent(pass)}).then(r=>{if(r.ok){closeDialog('resetDailyBg');fetchData()}else document.getElementById('resetDailyErr').textContent='Неверный пароль'}).catch(()=>document.getElementById('resetDailyErr').textContent='Ошибка сети')}
document.getElementById('dailyCell').addEventListener('click',openResetDaily);
function openAlarm(){const st={ready:'Снято',arming:'Задержка',armed:'Охрана активна',manual:'Ручной'};document.getElementById('alarmStat').innerHTML='Статус: <b>'+(st[lastAlarmState]||'—')+'</b>';document.getElementById('alarmErr').textContent='';document.getElementById('alarmBg').classList.add('show')}
function alarmAction(action){const pass=document.getElementById('alarmPass').value;fetch('/api/alarm',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'password='+encodeURIComponent(pass)+'&action='+action}).then(r=>{if(r.ok){document.getElementById('alarmErr').textContent='OK';fetchData()}else document.getElementById('alarmErr').textContent='Неверный пароль'}).catch(()=>document.getElementById('alarmErr').textContent='Ошибка сети')}
function toggleManual(){alarmAction('toggle-manual')}function toggleRelay(){alarmAction('toggle-relay')}function testRelay(){alarmAction('test-relay')}
</script></body></html>
)rawliteral";

// ============================================================
//                        АВТОРИЗАЦИЯ
// ============================================================
bool requireAuth(AsyncWebServerRequest* r) {
  if (strlen(cfg.webPass) == 0) return true;
  if (r->authenticate(cfg.webUser, cfg.webPass)) return true;
  r->requestAuthentication(); return false;
}
bool checkPassParam(AsyncWebServerRequest* r) {
  if (strlen(cfg.webPass) == 0) return true;
  if (!r->hasParam("password", true)) return false;
  return r->getParam("password", true)->value() == String(cfg.webPass);
}

// ============================================================
//                        ВЕБ-СЕРВЕР
// ============================================================
void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    if (!cfgConfigured()) { sendSetupPage(r); return; }
    r->send_P(200, "text/html", BIKE_HTML);
  });

  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (cfgConfigured() && !requireAuth(r)) return;
    sendSetupPage(r);
  });

  server.on("/setup", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (cfgConfigured() && !requireAuth(r)) return;
    auto getS = [&](const char* name) -> String { return r->hasParam(name, true) ? r->getParam(name, true)->value() : String(""); };
    auto getI = [&](const char* name, int def) -> int { String v = getS(name); return v.length() ? v.toInt() : def; };
    auto hasC = [&](const char* name) -> bool { return r->hasParam(name, true); };

    String s = getS("wifiSSID"); if (s.length() > 0 && s.length() <= 32) strlcpy(cfg.wifiSSID, s.c_str(), sizeof(cfg.wifiSSID));
    s = getS("wifiPass"); if (s.length() > 0) { if (s.length() > 64) { r->send(400, "text/plain", "too long"); return; } strlcpy(cfg.wifiPass, s.c_str(), sizeof(cfg.wifiPass)); }
    s = getS("apSSID"); if (s.length() > 0 && s.length() <= 32) strlcpy(cfg.apSSID, s.c_str(), sizeof(cfg.apSSID));
    s = getS("apPass"); if (s.length() > 0) { if (s.length() < 8) { r->send(400, "text/plain", "min 8"); return; } if (s.length() > 63) { r->send(400, "text/plain", "too long"); return; } strlcpy(cfg.apPass, s.c_str(), sizeof(cfg.apPass)); }
    s = getS("webUser"); if (s.length() > 0 && s.length() <= 16) strlcpy(cfg.webUser, s.c_str(), sizeof(cfg.webUser));
    s = getS("webPass"); if (s.length() > 0) { if (s.length() > 32) { r->send(400, "text/plain", "too long"); return; } strlcpy(cfg.webPass, s.c_str(), sizeof(cfg.webPass)); }

    bool useStatic = hasC("useStaticIP"); String ipS = getS("staticIP"), gwS = getS("staticGW");
    if (useStatic) {
      IPAddress ip, gw;
      if (ipS.length() == 0 || gwS.length() == 0 || !ip.fromString(ipS.c_str()) || !gw.fromString(gwS.c_str())) { r->send(400, "text/plain; charset=utf-8", "Неверный IP"); return; }
      strlcpy(cfg.staticIP, ipS.c_str(), sizeof(cfg.staticIP)); strlcpy(cfg.staticGW, gwS.c_str(), sizeof(cfg.staticGW)); setFlag(FLAG_STATIC_IP, true);
    } else setFlag(FLAG_STATIC_IP, false);

    if (r->hasParam("wheelMm", true)) { int mm = getI("wheelMm", 0); if (mm >= 500 && mm <= 4000) cfg.wheelCircumference = mm / 1000.0f; }
    cfg.armDelaySec = constrain(getI("armDelaySec", cfg.armDelaySec), 1, 60);
    cfg.staCheckIntervalSec = constrain(getI("staCheckIntervalSec", cfg.staCheckIntervalSec), 2, 120);
    { String v = getS("armSpeedIgnoreKmh"); if (v.length()) cfg.armSpeedIgnoreKmh = constrain(v.toFloat(), 0.0f, 10.0f); }
    setFlag(FLAG_ALARM_ENABLED, hasC("alarmEnabled")); setFlag(FLAG_MANUAL_MODE, hasC("manualMode"));
    setFlag(FLAG_MANUAL_RELAY, hasC("manualRelay")); setFlag(FLAG_MANUAL_COUNT_KM, hasC("manualCountKm"));
    setFlag(FLAG_SLEEP_ENABLED, hasC("sleepEnabled"));
    cfg.sleepHour = constrain(getI("sleepHour", cfg.sleepHour), 0, 23); cfg.sleepMin = constrain(getI("sleepMin", cfg.sleepMin), 0, 59);
    cfg.wakeHour = constrain(getI("wakeHour", cfg.wakeHour), 0, 23); cfg.wakeMin = constrain(getI("wakeMin", cfg.wakeMin), 0, 59);
    cfg.sleepGraceSec = constrain(getI("sleepGraceSec", cfg.sleepGraceSec), 0, 3600);
    cfg.bleScanIntervalSec = constrain(getI("bleScanIntervalSec", cfg.bleScanIntervalSec), 1, 600);
    cfg.bleScanDurationMs = constrain(getI("bleScanDurationMs", cfg.bleScanDurationMs), 500, 15000);
    cfg.speedStopMs = constrain(getI("speedStopSec", cfg.speedStopMs / 1000), 1, 30) * 1000UL;

    bool ok = (strlen(cfg.apPass) >= 8) && (strlen(cfg.webUser) > 0) && (strlen(cfg.webPass) > 0) && (strlen(cfg.wifiSSID) > 0);
    setFlag(FLAG_CONFIGURED, ok); saveConfig();
    const char* msg = ok ? "Сохранено. Перезагрузка..." : "Не все поля заполнены. Перезагрузка...";
    char buf[256];
    snprintf(buf, sizeof(buf), "<!DOCTYPE html><html><head><meta charset='utf-8'><meta http-equiv='refresh' content='4;url=/'></head><body style='background:#111;color:#0f0;text-align:center;padding:50px;font-family:sans-serif'><h1>%s</h1></body></html>", msg);
    r->send(200, "text/html; charset=utf-8", buf); pendingRestart = millis();
  });

  server.on("/api/bike", HTTP_GET, [](AsyncWebServerRequest* r) {
    if (!cfgConfigured()) return r->send(403, "text/plain", "not configured");
    char timeBuf[12]; formatTime(timeBuf, sizeof(timeBuf)); char dateBuf[12]; formatDate(dateBuf, sizeof(dateBuf));
    float speed = bike.speed, daily = bike.dailyDistance, total = bike.totalDistance, maxSp = bike.maxSpeed;
    uint32_t rt = bike.rideTime, trt = bike.totalRideTime; bool connected = bike.sensorConnected;
    int wheelMm = (int)(cfg.wheelCircumference * 1000.0f + 0.5f);
    float avg = (rt > 0) ? (daily / 1000.0f) / (rt / 3600.0f) : 0.0f;
    IPAddress ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : WiFi.softAPIP();

    if (dailyResetPending && pendingAnnouncement[0] == '\0') {
      setAnnouncement("Наступил новый день. Суточный пробег сброшен.");
      dailyResetPending = false;
    }

    char announceEsc[500];
    jsonEscape(announceEsc, sizeof(announceEsc), pendingAnnouncement);

    char json[1200];
    snprintf(json, sizeof(json),
      "{\"speed\":%.1f,\"daily\":%.1f,\"total\":%.1f,\"maxSpeed\":%.1f,"
      "\"rideTime\":%lu,\"totalRideTime\":%lu,\"average\":%.1f,"
      "\"connected\":%s,\"scanning\":%s,\"wheelMm\":%d,"
      "\"alarm\":\"%s\",\"ip\":\"%s\",\"time\":\"%s\",\"date\":\"%s\","
      "\"urgent\":%s,\"announce\":\"%s\"}",
      speed, daily, total, maxSp, (unsigned long)rt, (unsigned long)trt, avg,
      connected ? "true" : "false", scanning ? "true" : "false", wheelMm,
      alarmStateStr(), ip.toString().c_str(), timeBuf, dateBuf,
      pendingAnnouncementUrgent ? "true" : "false",
      announceEsc);

    pendingAnnouncement[0] = 0;
    pendingAnnouncementUrgent = false;
    r->send(200, "application/json; charset=utf-8", json);
  });

  server.on("/scan", HTTP_POST, [](AsyncWebServerRequest* r) { forceScanRequested = true; r->send(200, "text/plain", "OK"); });

  server.on("/api/wheel", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (!cfgConfigured()) return r->send(403, "text/plain", "not configured");
    if (!checkPassParam(r)) return r->send(401, "text/plain", "Wrong password");
    if (!r->hasParam("mm", true)) return r->send(400, "text/plain", "no mm");
    int mm = r->getParam("mm", true)->value().toInt();
    if (mm < 500 || mm > 4000) return r->send(400, "text/plain", "range");
    cfg.wheelCircumference = mm / 1000.0f; saveConfig(); r->send(200, "text/plain", "OK");
  });

  server.on("/api/reset-total", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (!cfgConfigured()) return r->send(403, "text/plain", "not configured");
    if (!checkPassParam(r)) return r->send(401, "text/plain", "Wrong password");
    bike.totalDistance = 0; bike.totalRideTime = 0; saveBikeDataNow(); r->send(200, "text/plain", "OK");
  });

  server.on("/api/reset-daily", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (!cfgConfigured()) return r->send(403, "text/plain", "not configured");
    if (!checkPassParam(r)) return r->send(401, "text/plain", "Wrong password");
    bike.dailyDistance = 0; bike.maxSpeed = 0; bike.rideTime = 0; lastAnnouncedKm = 0;
    lastAnnouncedHour = 0; lastAnnouncedMaxSpeed = 15.0f;
    cfg.lastDailyResetDay = timeSynced ? currentLocalDay() : 0; saveBikeDataNow(); saveConfig(); r->send(200, "text/plain", "OK");
  });

  server.on("/api/alarm", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (!cfgConfigured()) return r->send(403, "text/plain", "not configured");
    if (!checkPassParam(r)) return r->send(401, "text/plain", "Wrong password");
    if (!r->hasParam("action", true)) return r->send(400, "text/plain", "no action");
    String act = r->getParam("action", true)->value();
    if (act == "toggle-manual") { setFlag(FLAG_MANUAL_MODE, !cfgManualMode()); saveConfig(); }
    else if (act == "toggle-relay") { setFlag(FLAG_MANUAL_RELAY, !cfgManualRelay()); saveConfig(); }
    else if (act == "test-relay") { testRelayUntil = millis() + 5000; }
    else return r->send(400, "text/plain", "unknown");
    r->send(200, "text/plain", "OK");
  });

  server.on("/api/wipe", HTTP_POST, [](AsyncWebServerRequest* r) {
    if (!checkPassParam(r)) { r->send(401, "text/plain; charset=utf-8", "Неверный пароль"); return; }
    wipeAll();
    r->send_P(200, "text/html; charset=utf-8",
      "<!DOCTYPE html><html><head><meta charset='utf-8'><meta http-equiv='refresh' content='5;url=/'></head>"
      "<body style='background:#111;color:#f66;text-align:center;padding:50px;font-family:sans-serif'>"
      "<h1>Сброс выполнен</h1><p>ESP перезагрузится.</p></body></html>");
    delay(500); ESP.restart();
  });

  server.on("/update", HTTP_GET, [](AsyncWebServerRequest* r) {
    if (!requireAuth(r)) return;

    AsyncResponseStream *response = r->beginResponseStream("text/html; charset=utf-8");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Pragma", "no-cache");
    response->addHeader("Expires", "0");

    response->print(F("<!DOCTYPE html><html lang='ru'><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>OHORONA BIKE 4.0 — прошивка</title><style>"
      "body{font-family:sans-serif;background:#111;color:#eee;max-width:560px;margin:20px auto;padding:0 12px}"
      "h1{color:#00ff88;text-align:center;font-size:1.4em}"
      "h2{color:#8cf;margin-top:24px;font-size:1.2em}"
      "input[type=file],input[type=password]{width:100%;padding:10px;background:#222;color:#eee;"
      "border:1px solid #555;border-radius:5px;box-sizing:border-box;margin:8px 0;font-size:15px}"
      "button{padding:14px;font-size:16px;background:#2a6;border:0;color:#fff;border-radius:8px;"
      "margin-top:8px;cursor:pointer;width:100%;font-weight:bold}"
      "button.danger{background:#c33}"
      "a{color:#00ff88;text-decoration:none}"
      ".m{color:#aaa;font-size:.85em;line-height:1.4}"
      ".warn{background:#511;border:1px solid #a33;border-radius:6px;padding:12px;margin:12px 0;"
      "color:#faa;font-size:.9em}"
      ".msg{color:#0f0;font-weight:bold;min-height:20px;margin-top:10px;text-align:center}"
      "</style></head><body>"
      "<h1>🚴 OHORONA BIKE 4.0</h1>"
      "<h2>📦 Обновление прошивки (OTA)</h2>"
      "<form method='POST' action='/update' enctype='multipart/form-data'>"
      "<label style='display:block;color:#8cf;font-size:14px;margin:12px 0 4px;font-weight:bold'>🔑 Пароль панели</label>"
      "<input type='password' name='password' placeholder='Пароль панели' required>"
      "<label style='display:block;color:#8cf;font-size:14px;margin:12px 0 4px;font-weight:bold'>📁 Файл прошивки (.bin)</label>"
      "<input type='file' name='firmware' accept='.bin' required>"
      "<button>Загрузить и прошить</button></form>"
      "<p class='m'>Файл .bin берется из Arduino IDE → Скетч → Экспорт бинарного файла.</p>"
      "<h2>⚠️ Сброс до заводских</h2>"
      "<div class='warn'>Стирает ВСЁ: Wi-Fi, пароли, пробег, настройки охраны.</div>"
      "<input type='password' id='wipePass' placeholder='Пароль панели'>"
      "<button class='danger' onclick='doWipe()'>Сбросить всё</button>"
      "<div class='msg' id='wipeMsg'></div>"
      "<p style='text-align:center;margin-top:30px'><a href='/'>← Вернуться на панель</a></p>"
      "<script>"
      "function doWipe(){"
      "  if(!confirm('Вы уверены, что хотите стереть все данные?')) return;"
      "  const p=document.getElementById('wipePass').value;"
      "  const m=document.getElementById('wipeMsg');"
      "  m.textContent='...';"
      "  fetch('/api/wipe',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},"
      "    body:'password='+encodeURIComponent(p)})"
      "  .then(r=>{"
      "    if(r.ok){m.textContent='OK! Перезагрузка...'; m.style.color='#0f0';}"
      "    else{m.textContent='Неверный пароль'; m.style.color='#f66';}"
      "  }).catch(()=>{m.textContent='Ошибка сети'; m.style.color='#f66';});"
      "}"
      "</script></body></html>"));

    r->send(response);
  });

  server.on("/update", HTTP_POST,
    [](AsyncWebServerRequest* r) {
      if (!requireAuth(r)) return;
      bool ok = !Update.hasError();
      AsyncWebServerResponse* resp = r->beginResponse(200, "text/plain", ok ? "OK" : "FAIL");
      resp->addHeader("Connection", "close");
      r->send(resp);
      if (ok) { delay(500); ESP.restart(); }
    },
    [](AsyncWebServerRequest* r, String filename, size_t index, uint8_t* data, size_t len, bool final) {
      bool authed = (strlen(cfg.webPass) == 0)
                 || r->authenticate(cfg.webUser, cfg.webPass)
                 || checkPassParam(r);
      if (!authed) {
        if (!index) {
          Update.abort();
          Serial.println(F("OTA: отклонено — неверный пароль"));
        }
        return;
      }
      if (!index) { if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial); }
      if (Update.write(data, len) != len) Update.printError(Serial);
      if (final) { if (Update.end(true)) { } else { Update.printError(Serial); } }
    });

  server.onNotFound([](AsyncWebServerRequest* r) { r->send(404, "text/plain", "Not found"); });
  server.begin();
}

// ============================================================
//                        SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n\nOHORONA BIKE v4.4.2 SOUND — BROWSER TTS"));
  bootMs = millis();
  initLed(); startupBlink();
  pinMode(PIN_RELAY, OUTPUT); digitalWrite(PIN_RELAY, LOW); relayIsOn = false;
  pinMode(PIN_BOOT, INPUT_PULLUP);
  loadConfig(); loadBikeData();
  lastAnnouncedKm = (uint32_t)(bike.dailyDistance / 1000.0f);
  lastAnnouncedHour = bike.rideTime / 3600;
  lastAnnouncedMaxSpeed = (bike.maxSpeed > 15.0f) ? bike.maxSpeed : 15.0f;

  if (digitalRead(PIN_BOOT) == LOW) {
    uint32_t t0 = millis();
    while (digitalRead(PIN_BOOT) == LOW && (millis() - t0) < 10000) { digitalWrite(PIN_LED, (millis() / 100) % 2); delay(10); }
    if (digitalRead(PIN_BOOT) == LOW) { wipeAll(); digitalWrite(PIN_LED, HIGH); delay(1000); digitalWrite(PIN_LED, LOW); }
  }

  configTime(0, 0, NTP_SERVER); setenv("TZ", TZ_KYIV, 1); tzset();
  wifiBoot();
  if (MDNS.begin(HOSTNAME)) { MDNS.addService("http", "tcp", 80); }
  setupWebServer();
  NimBLEDevice::init(BLE_NAME);
  Serial.println(F("Готово."));
}

// ============================================================
//                        LOOP
// ============================================================
void loop() {
  if (pendingRestart && millis() - pendingRestart > 2000) { saveBikeDataNow(); ESP.restart(); }
  if (bleClientDeletePending) { bleClientDeletePending = false; if (pClient) cleanupClient(); }
  if (!sleeping) { wifiTick(); timeTick(); checkDailyReset(); checkAnnounce(); movementTick(); }
  if (!sleeping && canSleep()) { enterSleep(); }
  if (sleeping) {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 30000) { lastCheck = millis(); if (!inSleepWindow()) wakeFromSleep(); }
    delay(500); return;
  }
  if (cfgConfigured() && relayIsOn && !bleConnected && !scanning) {
    static unsigned long lastScanAttempt = 0;
    unsigned long intervalMs = cfg.bleScanIntervalSec * 1000UL;
    if (millis() - lastScanAttempt > intervalMs || forceScanRequested) {
      lastScanAttempt = millis(); if (forceScanRequested) forceScanRequested = false; scanForSensor();
    }
  } else if (forceScanRequested) { forceScanRequested = false; }
  if (bike.speed > 0.0f && (millis() - lastRevolutionMs > cfg.speedStopMs)) { bike.speed = 0.0f; }
  static unsigned long lastRideTick = 0;
  if (millis() - lastRideTick >= 1000) {
    lastRideTick += 1000;
    bool countIt = !cfgManualMode() || cfgManualCountKm();
    if (countIt && bike.sensorConnected && bike.speed > 0.5f) { bike.rideTime++; bike.totalRideTime++; bikeDirty = true; }
  }
  static unsigned long lastSave = 0;
  if (bikeDirty && millis() - lastSave > SAVE_INTERVAL_MS) { lastSave = millis(); bikeDirty = false; saveBikeDataNow(); }
  alarmTick(); updateLed();
  delay(20);
}