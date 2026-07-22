/*
  ESP32-S3 Room Thermostat ULTRA PRO

  Hardware:
  - ESP32-S3 N16R8 Dev Module
  - ST7789 1.9" IPS TFT 170x320 SPI display
  - SHT41 temperature/humidity sensor on I2C
  - DS3231 RTC on I2C
  - HW-040 rotary encoder with push button
  - 1-channel 5 V relay module, HIGH-level trigger
  - Optional Wi-Fi reset jumper/button: GPIO7 -> GND, internal pull-up

  Required Arduino libraries:
  - Adafruit GFX Library
  - Adafruit ST7735 and ST7789 Library
  - Adafruit SHT4x Library
  - RTClib
  - PubSubClient

  Board: ESP32S3 Dev Module
  Flash: 16MB
  PSRAM: OPI PSRAM
  USB CDC On Boot: Enabled
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <time.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_SHT4x.h>
#include <RTClib.h>
#include <PubSubClient.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#include <esp_task_wdt.h>
#include <driver/gpio.h>
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

// ----------------------------- PINOUT ---------------------------------
// I2C bus: SHT41 and DS3231 in parallel.
static const uint8_t PIN_I2C_SDA = 8;
static const uint8_t PIN_I2C_SCL = 9;

// TFT ST7789, 170x320. Display labels often say SCL/SDA, but this is SPI SCK/MOSI.
static const uint8_t PIN_TFT_CS   = 10;
static const uint8_t PIN_TFT_MOSI = 11;  // display SDA / DIN / MOSI
static const uint8_t PIN_TFT_SCLK = 12;  // display SCL / CLK / SCK
static const uint8_t PIN_TFT_DC   = 13;
static const uint8_t PIN_TFT_RST  = 14;
static const uint8_t PIN_TFT_BL   = 21;  // backlight; driven constantly HIGH to avoid PWM shimmer
static const uint32_t TFT_BL_PWM_FREQ = 25000;  // kept only for compatibility with older config fields
static const uint8_t TFT_BL_PWM_RES = 8;
static const uint8_t TFT_BL_PWM_CH = 0;

// Relay. For the shown module ordered as 5V high-level trigger.
// VCC = 5V, GND common, IN = GPIO16. Heating contact: COM + NO.
static const uint8_t PIN_RELAY = 16;
static const bool RELAY_ACTIVE_HIGH = true;

// Rotary encoder HW-040.
static const uint8_t PIN_ENC_CLK = 4;
static const uint8_t PIN_ENC_DT  = 5;
static const uint8_t PIN_ENC_SW  = 6;

// Wi-Fi reset jumper/button: short GPIO7 to GND for at least 4 seconds.
// Do not use GPIO0 here, because GPIO0 is the ESP32 boot/flash strap.
static const uint8_t PIN_WIFI_RESET = 7;

// -------------------------- BASIC SETTINGS -----------------------------
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 12000;
static const uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
static const uint16_t MQTT_SOCKET_TIMEOUT_SEC = 4;
static const uint16_t MQTT_KEEPALIVE_SEC = 30;
static const uint32_t SENSOR_INTERVAL_MS = 2000;
static const uint32_t CONTROL_INTERVAL_MS = 300;
static const uint32_t MQTT_PUBLISH_INTERVAL_MS = 10000;
static const uint32_t NTP_SYNC_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;
static const uint32_t BUTTON_LONG_PRESS_MS = 5000;
static const uint32_t WIFI_RESET_HOLD_MS = 4000;
static const uint32_t UI_FRAME_MS = 35;       // faster UI response for encoder; still stable on ST7789
static const uint32_t WDT_TIMEOUT_MS = 8000;
static const uint8_t MAX_SCHEDULE = 24;

static const float MIN_SETPOINT = 5.0f;
static const float MAX_SETPOINT = 35.0f;
static const float SETPOINT_STEP = 0.5f;
static const float DEFAULT_SETPOINT = 21.0f;
static const float DEFAULT_HYSTERESIS = 1.0f;

// Europe/Warsaw with DST.
static const char *DEFAULT_TZ = "CET-1CEST,M3.5.0/2,M10.5.0/3";
static const char *FW_VERSION = "2026.07.11-premium-r1";
static const char *HA_DISCOVERY_PREFIX = "homeassistant";

// Landscape after rotation(1): 320 x 170.
static const int TFT_W = 320;
static const int TFT_H = 170;

// Many 1.9" ST7789 IPS modules need display inversion ON.
// Symptoms when wrong: black becomes white and orange flame looks blue.
static const bool TFT_COLOR_INVERT = true;

// ------------------------------ OBJECTS --------------------------------
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GFXcanvas16 *fb = nullptr;  // pseudo framebuffer: draw to RAM, then push one complete frame
Adafruit_SHT4x sht4;
RTC_DS3231 rtc;
Preferences prefs;
WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

TaskHandle_t uiTaskHandle = nullptr;
TaskHandle_t logicTaskHandle = nullptr;

// ------------------------------- TYPES ---------------------------------
enum RunMode : uint8_t {
  MODE_AUTO = 0,
  MODE_MANUAL = 1,
  MODE_OFF = 2
};

enum RelayOverride : uint8_t {
  OVR_NONE = 0,
  OVR_FORCE_ON = 1,
  OVR_FORCE_OFF = 2
};

enum ScreenMode : uint8_t {
  SCREEN_MAIN = 0,
  SCREEN_SETTINGS = 1
};

struct ScheduleEntry {
  bool enabled = false;
  uint16_t startMin = 0;
  uint16_t endMin = 0;
  float setpoint = DEFAULT_SETPOINT;
  float hysteresis = DEFAULT_HYSTERESIS;
};

struct DeviceConfig {
  String deviceName;
  String wifiSsid;
  String wifiPass;

  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser;
  String mqttPass;
  String mqttPrefix;
  bool mqttDiscovery = true;

  bool ntpEnabled = true;
  String tzString;

  uint8_t lcdBrightness = 80;  // percent
  float manualHysteresis = DEFAULT_HYSTERESIS;
};

struct RuntimeState {
  bool sensorOk = false;
  bool rtcOk = false;
  bool wifiOk = false;
  bool apActive = false;
  bool mqttOk = false;

  float temperature = NAN;
  float humidity = NAN;
  float target = DEFAULT_SETPOINT;
  float activeHysteresis = DEFAULT_HYSTERESIS;
  float manualSetpoint = DEFAULT_SETPOINT;

  RunMode mode = MODE_AUTO;
  RelayOverride relayOverride = OVR_NONE;
  bool relayOn = false;

  uint32_t heatingCyclesToday = 0;
  uint32_t currentDayOrdinal = 0;

  ScreenMode screen = SCREEN_MAIN;
  int settingsPage = 0;
};

struct UiSnapshot {
  bool sensorOk;
  bool rtcOk;
  bool wifiOk;
  bool apActive;
  bool mqttOk;
  float temperature;
  float humidity;
  float target;
  float activeHysteresis;
  float manualSetpoint;
  RunMode mode;
  RelayOverride relayOverride;
  bool relayOn;
  uint32_t heatingCyclesToday;
  ScreenMode screen;
  int settingsPage;
  uint8_t lcdBrightness;
  String ip;
  String ssid;
  String timeShort;
  String dateShort;
};

struct Palette {
  uint16_t bgTop;
  uint16_t bgBottom;
  uint16_t panel;
  uint16_t text;
  uint16_t muted;
  uint16_t accent;
  uint16_t accent2;
  uint16_t danger;
  uint16_t ringDim;
};

DeviceConfig config;
RuntimeState state;
ScheduleEntry scheduleEntries[MAX_SCHEDULE];
uint8_t scheduleCount = 0;
String scheduleRaw;

uint32_t lastSensorMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastMqttPublishMs = 0;
uint32_t lastMqttReconnectMs = 0;
uint32_t lastNtpSyncMs = 0;
bool wifiReconnectRequested = false;
bool mdnsStarted = false;
String mqttLastError = "MQTT nie uruchomione";
uint32_t mqttLastAttemptMs = 0;
bool pendingManualSetpointSave = false;
uint32_t lastManualSetpointChangeMs = 0;

// Encoder ISR state.
static const int8_t DRAM_ATTR ENC_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t encDelta = 0;
volatile int8_t encAcc = 0;
volatile uint8_t encLastState = 0;

// Button and reset jumper state.
bool btnLastRaw = false;
bool btnStable = false;
uint32_t btnDebounceMs = 0;
uint32_t btnPressStartMs = 0;
bool btnLongHandled = false;
uint32_t wifiResetStartMs = 0;
bool wifiResetHandled = false;

// UI animation state lives only on UI task/core.
float uiTemp = NAN;
float uiTarget = DEFAULT_SETPOINT;
float uiHum = NAN;
float uiPhase = 0.0f;

// --------------------------- SMALL HELPERS ------------------------------
String twoDigits(int v) {
  if (v < 10) return String("0") + String(v);
  return String(v);
}

String macSuffix() {
  uint64_t mac = ESP.getEfuseMac();
  char b[7];
  snprintf(b, sizeof(b), "%06X", (uint32_t)(mac & 0xFFFFFF));
  return String(b);
}

String sanitizeTopicPart(String s) {
  s.trim();
  s.toLowerCase();
  String out;
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-') out += c;
    else if (c == ' ' || c == '.') out += '_';
  }
  if (out.length() == 0) out = "thermostat";
  return out;
}

String htmlEscape(const String &s) {
  String out;
  out.reserve(s.length() + 16);
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else out += c;
  }
  return out;
}

String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 8);
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\') out += F("\\\\");
    else if (c == '"') out += F("\\\"");
    else if (c == '\n') out += F("\\n");
    else if (c == '\r') out += F("\\r");
    else if (c == '\t') out += F("\\t");
    else out += c;
  }
  return out;
}

float clampFloat(float value, float mn, float mx) {
  if (value < mn) return mn;
  if (value > mx) return mx;
  return value;
}

uint8_t clampBrightness(int value) {
  if (value < 0) return 0;
  if (value > 100) return 100;
  return (uint8_t)value;
}

String modeToText(RunMode mode) {
  switch (mode) {
    case MODE_AUTO: return "auto";
    case MODE_MANUAL: return "manual";
    case MODE_OFF: return "off";
  }
  return "auto";
}

String modeToDisplayText(RunMode mode) {
  switch (mode) {
    case MODE_AUTO: return "AUTO";
    case MODE_MANUAL: return "RECZNY";
    case MODE_OFF: return "OFF";
  }
  return "AUTO";
}

String modeToHaClimateText(RunMode mode) {
  switch (mode) {
    case MODE_AUTO: return "auto";
    case MODE_MANUAL: return "heat";
    case MODE_OFF: return "off";
  }
  return "auto";
}

String overrideToText(RelayOverride ovr) {
  switch (ovr) {
    case OVR_NONE: return "auto";
    case OVR_FORCE_ON: return "force_on";
    case OVR_FORCE_OFF: return "force_off";
  }
  return "auto";
}

uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint8_t r565(uint16_t c) { return ((c >> 11) & 0x1F) * 255 / 31; }
uint8_t g565(uint16_t c) { return ((c >> 5) & 0x3F) * 255 / 63; }
uint8_t b565(uint16_t c) { return (c & 0x1F) * 255 / 31; }

uint16_t mix565(uint16_t a, uint16_t b, float t) {
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  uint8_t r = (uint8_t)(r565(a) + (r565(b) - r565(a)) * t);
  uint8_t g = (uint8_t)(g565(a) + (g565(b) - g565(a)) * t);
  uint8_t bl = (uint8_t)(b565(a) + (b565(b) - b565(a)) * t);
  return rgb565(r, g, bl);
}

uint16_t scale565(uint16_t c, uint8_t percent) {
  // Software LCD brightness. Backlight stays constant HIGH to avoid PWM shimmer,
  // but the UI palette is scaled so the WebGUI brightness control has visible effect.
  percent = clampBrightness(percent);
  float k = percent / 100.0f;
  uint8_t r = (uint8_t)(r565(c) * k);
  uint8_t g = (uint8_t)(g565(c) * k);
  uint8_t b = (uint8_t)(b565(c) * k);
  return rgb565(r, g, b);
}

Palette paletteFor(const UiSnapshot &s) {
  Palette p;
  // Pure black base: no bright background, no animated bands, no gradients.
  p.bgTop = ST77XX_BLACK;
  p.bgBottom = ST77XX_BLACK;
  p.panel = rgb565(13, 16, 22);
  p.text = rgb565(246, 248, 252);
  p.muted = rgb565(232, 236, 244);  // very light gray: readable on the menu pages
  p.danger = rgb565(255, 91, 91);
  p.ringDim = rgb565(35, 40, 48);

  if (s.mode == MODE_OFF) {
    p.accent = rgb565(140, 148, 160);
    p.accent2 = rgb565(75, 82, 94);
  } else if (s.relayOn) {
    p.accent = rgb565(255, 142, 50);   // warm orange for heating
    p.accent2 = rgb565(255, 72, 72);
  } else if (s.mode == MODE_MANUAL) {
    p.accent = rgb565(160, 118, 255);
    p.accent2 = rgb565(72, 210, 255);
  } else {
    p.accent = rgb565(35, 224, 200);
    p.accent2 = rgb565(64, 145, 255);
  }

  // No hardware PWM on BL pin: it caused visible shimmer on this LCD.
  // Brightness from WebGUI now dims the rendered palette instead.
  uint8_t b = s.lcdBrightness;
  uint8_t b20 = (b < 20) ? 20 : b;
  uint8_t b28 = (b < 45) ? 45 : b;
  p.panel = scale565(p.panel, b20);
  p.text = scale565(p.text, b28);
  p.muted = scale565(p.muted, b28);
  p.danger = scale565(p.danger, b28);
  p.ringDim = scale565(p.ringDim, b20);
  p.accent = scale565(p.accent, b28);
  p.accent2 = scale565(p.accent2, b28);
  return p;
}

void setupBacklightPwm() {
  // W ESP32 Core 3.0+ używamy nowej funkcji ledcAttach:
  // Składnia: ledcAttach(PIN, CZĘSTOTLIWOŚĆ, ROZDZIELCZOŚĆ);
  // Zwraca automatycznie przypisany kanał, ale nie musimy go znać do sterowania pinem.
  ledcAttach(PIN_TFT_BL, TFT_BL_PWM_FREQ, TFT_BL_PWM_RES);
  
  // Ustawienie początkowej jasności przy starcie
  uint32_t duty = (clampBrightness(config.lcdBrightness) * 255) / 100;
  ledcWrite(PIN_TFT_BL, duty);
}

void setBacklight(uint8_t percent) {
  percent = clampBrightness(percent);
  
  // Zabezpieczenie przed programowym migotaniem
  if (config.lcdBrightness == percent) {
    return; 
  }
  
  config.lcdBrightness = percent;
  
  // Przeliczenie procentów na wartość 8-bit (0-255)
  uint32_t duty = (percent * 255) / 100;
  
  // W nowym API ledcWrite przyjmuje bezpośrednio NUMER PINU, a nie numer kanału!
  ledcWrite(PIN_TFT_BL, duty);
}

void setRelayPhysical(bool on) {
  digitalWrite(PIN_RELAY, (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

// ----------------------------- WATCHDOG ---------------------------------
void initWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_MS;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  esp_task_wdt_init(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
#endif
}

void wdtAddCurrentTask() {
  esp_task_wdt_add(NULL);
}

void wdtFeed() {
  esp_task_wdt_reset();
}

// ---------------------------- PREFERENCES -------------------------------
String defaultScheduleText() {
  return String(
    "00:00-06:00,19,1\n"
    "06:00-08:00,21,1\n"
    "08:00-16:00,19,1\n"
    "16:00-22:30,21,1\n"
    "22:30-00:00,19,1\n"
  );
}

void loadConfig() {
  prefs.begin("thermostat", false);

  String defaultName = "termostat_" + macSuffix();
  config.deviceName = prefs.getString("devName", defaultName);
  config.wifiSsid = prefs.getString("wifiSsid", "");
  config.wifiPass = prefs.getString("wifiPass", "");

  config.mqttHost = prefs.getString("mqttHost", "");
  config.mqttHost.trim();
  config.mqttPort = prefs.getUShort("mqttPort", 1883);
  config.mqttUser = prefs.getString("mqttUser", "");
  config.mqttPass = prefs.getString("mqttPass", "");
  config.mqttPrefix = prefs.getString("mqttPrefix", "home/thermostat");
  config.mqttPrefix.trim();
  if (config.mqttPrefix.length() == 0) config.mqttPrefix = "home/thermostat";
  config.mqttDiscovery = prefs.getBool("mqttDisc", true);

  config.ntpEnabled = prefs.getBool("ntp", true);
  config.tzString = prefs.getString("tz", DEFAULT_TZ);
  config.lcdBrightness = prefs.getUChar("bright", 80);
  config.manualHysteresis = prefs.getFloat("manHyst", DEFAULT_HYSTERESIS);
  state.manualSetpoint = prefs.getFloat("lastSet", DEFAULT_SETPOINT);
  state.manualSetpoint = clampFloat(state.manualSetpoint, MIN_SETPOINT, MAX_SETPOINT);

  scheduleRaw = prefs.getString("schedule", defaultScheduleText());
}

void saveConfig() {
  prefs.putString("devName", config.deviceName);
  prefs.putString("wifiSsid", config.wifiSsid);
  prefs.putString("wifiPass", config.wifiPass);
  prefs.putString("mqttHost", config.mqttHost);
  prefs.putUShort("mqttPort", config.mqttPort);
  prefs.putString("mqttUser", config.mqttUser);
  prefs.putString("mqttPass", config.mqttPass);
  prefs.putString("mqttPrefix", config.mqttPrefix);
  prefs.putBool("mqttDisc", config.mqttDiscovery);
  prefs.putBool("ntp", config.ntpEnabled);
  prefs.putString("tz", config.tzString);
  prefs.putUChar("bright", config.lcdBrightness);
  prefs.putFloat("manHyst", config.manualHysteresis);
  prefs.putFloat("lastSet", state.manualSetpoint);
}

void saveScheduleRaw(const String &raw) {
  scheduleRaw = raw;
  prefs.putString("schedule", scheduleRaw);
}

void clearWifiCredentials() {
  config.wifiSsid = "";
  config.wifiPass = "";
  prefs.putString("wifiSsid", "");
  prefs.putString("wifiPass", "");
}

// ----------------------------- SCHEDULE ---------------------------------
bool parseTimeToMinutes(String token, uint16_t &minutes) {
  token.trim();
  token.replace('.', ':');
  int colon = token.indexOf(':');
  int h = 0;
  int m = 0;

  if (colon >= 0) {
    h = token.substring(0, colon).toInt();
    m = token.substring(colon + 1).toInt();
  } else {
    h = token.toInt();
    m = 0;
  }

  if (h == 24 && m == 0) {
    minutes = 0;  // 24:00 as midnight; range crossing midnight handles it.
    return true;
  }
  if (h < 0 || h > 23 || m < 0 || m > 59) return false;
  minutes = (uint16_t)(h * 60 + m);
  return true;
}

bool lineToScheduleEntry(String line, ScheduleEntry &entry) {
  line.trim();
  if (line.length() == 0) return false;
  if (line.startsWith("#")) return false;

  int dash = line.indexOf('-');
  int comma1 = line.indexOf(',', dash + 1);
  int comma2 = line.indexOf(',', comma1 + 1);
  if (dash < 0 || comma1 < 0 || comma2 < 0) return false;

  String startStr = line.substring(0, dash);
  String endStr = line.substring(dash + 1, comma1);
  String tempStr = line.substring(comma1 + 1, comma2);
  String hystStr = line.substring(comma2 + 1);
  tempStr.trim();
  hystStr.trim();

  uint16_t startMin = 0;
  uint16_t endMin = 0;
  if (!parseTimeToMinutes(startStr, startMin)) return false;
  if (!parseTimeToMinutes(endStr, endMin)) return false;

  float temp = tempStr.toFloat();
  float hyst = hystStr.toFloat();
  if (temp < MIN_SETPOINT || temp > MAX_SETPOINT) return false;
  if (hyst < 0.1f || hyst > 5.0f) return false;

  entry.enabled = true;
  entry.startMin = startMin;
  entry.endMin = endMin;
  entry.setpoint = temp;
  entry.hysteresis = hyst;
  return true;
}

void parseScheduleText(const String &raw) {
  scheduleCount = 0;
  String copy = raw;
  copy.replace("\r", "");

  uint16_t pos = 0;
  while (pos < copy.length() && scheduleCount < MAX_SCHEDULE) {
    int nl = copy.indexOf('\n', pos);
    String line;
    if (nl < 0) {
      line = copy.substring(pos);
      pos = copy.length();
    } else {
      line = copy.substring(pos, nl);
      pos = nl + 1;
    }

    ScheduleEntry e;
    if (lineToScheduleEntry(line, e)) {
      scheduleEntries[scheduleCount++] = e;
    }
  }

  if (scheduleCount == 0) {
    ScheduleEntry fallback;
    fallback.enabled = true;
    fallback.startMin = 0;
    fallback.endMin = 0;
    fallback.setpoint = DEFAULT_SETPOINT;
    fallback.hysteresis = DEFAULT_HYSTERESIS;
    scheduleEntries[0] = fallback;
    scheduleCount = 1;
  }
}

bool minuteInRange(uint16_t nowMin, uint16_t startMin, uint16_t endMin) {
  if (startMin == endMin) return true;  // full day
  if (startMin < endMin) return nowMin >= startMin && nowMin < endMin;
  return nowMin >= startMin || nowMin < endMin;  // crosses midnight
}

ScheduleEntry currentScheduleEntry(uint16_t nowMin) {
  for (uint8_t i = 0; i < scheduleCount; i++) {
    if (scheduleEntries[i].enabled && minuteInRange(nowMin, scheduleEntries[i].startMin, scheduleEntries[i].endMin)) {
      return scheduleEntries[i];
    }
  }
  ScheduleEntry fallback;
  fallback.enabled = true;
  fallback.setpoint = DEFAULT_SETPOINT;
  fallback.hysteresis = DEFAULT_HYSTERESIS;
  return fallback;
}

// ------------------------------ TIME ------------------------------------
DateTime nowDateTime() {
  if (state.rtcOk) return rtc.now();

  time_t raw = time(nullptr);
  struct tm t;
  if (raw > 1600000000 && localtime_r(&raw, &t)) {
    return DateTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
  }
  return DateTime(2025, 1, 1, 0, 0, 0);
}

String dateTimeToString(const DateTime &dt) {
  return String(dt.year()) + "-" + twoDigits(dt.month()) + "-" + twoDigits(dt.day()) + " " +
         twoDigits(dt.hour()) + ":" + twoDigits(dt.minute()) + ":" + twoDigits(dt.second());
}

bool parseDateTimeString(String s, DateTime &out) {
  s.trim();
  if (s.length() < 16) return false;
  s.replace('T', ' ');
  int year = s.substring(0, 4).toInt();
  int mon = s.substring(5, 7).toInt();
  int day = s.substring(8, 10).toInt();
  int hour = s.substring(11, 13).toInt();
  int minute = s.substring(14, 16).toInt();
  int second = 0;
  if (s.length() >= 19) second = s.substring(17, 19).toInt();
  if (year < 2024 || mon < 1 || mon > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59) return false;
  out = DateTime(year, mon, day, hour, minute, second);
  return true;
}

void initRtc() {
  state.rtcOk = rtc.begin();
  if (!state.rtcOk) {
    Serial.println(F("RTC DS3231 not found."));
    return;
  }
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power; setting to compile time."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void syncRtcFromNtp(bool force) {
  if (!config.ntpEnabled) return;
  if (!state.wifiOk) return;
  if (!force && millis() - lastNtpSyncMs < NTP_SYNC_INTERVAL_MS) return;

  configTzTime(config.tzString.c_str(), "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 7000)) {
    if (state.rtcOk) {
      rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
    }
    lastNtpSyncMs = millis();
    Serial.println(F("NTP sync OK."));
  } else {
    Serial.println(F("NTP sync failed."));
  }
}

// ------------------------------ SENSORS ---------------------------------
void initSensors() {
  state.sensorOk = sht4.begin();
  if (state.sensorOk) {
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    Serial.println(F("SHT4x OK."));
  } else {
    Serial.println(F("SHT4x not found."));
  }
}

void readSensors() {
  if (!state.sensorOk) {
    state.sensorOk = sht4.begin();
    if (!state.sensorOk) return;
  }

  sensors_event_t humidityEvent;
  sensors_event_t tempEvent;
  if (sht4.getEvent(&humidityEvent, &tempEvent)) {
    state.temperature = tempEvent.temperature;
    state.humidity = humidityEvent.relative_humidity;
  } else {
    state.sensorOk = false;
    state.temperature = NAN;
    state.humidity = NAN;
  }
}

// ------------------------------- CONTROL --------------------------------
void updateDailyCounter(const DateTime &now) {
  uint32_t day = now.unixtime() / 86400UL;
  if (state.currentDayOrdinal == 0) state.currentDayOrdinal = day;
  if (day != state.currentDayOrdinal) {
    state.currentDayOrdinal = day;
    state.heatingCyclesToday = 0;
  }
}

void calculateTarget() {
  DateTime now = nowDateTime();
  updateDailyCounter(now);

  if (state.mode == MODE_AUTO) {
    uint16_t nowMin = (uint16_t)(now.hour() * 60 + now.minute());
    ScheduleEntry e = currentScheduleEntry(nowMin);
    state.target = e.setpoint;
    state.activeHysteresis = e.hysteresis;
  } else {
    state.target = state.manualSetpoint;
    state.activeHysteresis = config.manualHysteresis;
  }
}

void updateRelayControl() {
  calculateTarget();

  bool desired = false;

  if (state.relayOverride == OVR_FORCE_ON) {
    desired = true;
  } else if (state.relayOverride == OVR_FORCE_OFF) {
    desired = false;
  } else if (state.mode == MODE_OFF) {
    desired = false;
  } else if (state.sensorOk && !isnan(state.temperature)) {
    // Heating hysteresis: ON at target - hysteresis, OFF at target.
    if (state.relayOn) desired = state.temperature < state.target;
    else desired = state.temperature <= (state.target - state.activeHysteresis);
  }

  if (desired != state.relayOn) {
    if (!state.relayOn && desired) state.heatingCyclesToday++;
    state.relayOn = desired;
    setRelayPhysical(state.relayOn);
    lastMqttPublishMs = 0;  // publish immediately on state change
  }
}

String climateActionText() {
  if (state.mode == MODE_OFF) return "off";
  if (state.relayOn) return "heating";
  return "idle";
}

// ------------------------------- DISPLAY --------------------------------
void initDisplay() {
  pinMode(PIN_TFT_BL, OUTPUT);
  setupBacklightPwm();
  setBacklight(config.lcdBrightness);

  SPI.begin(PIN_TFT_SCLK, -1, PIN_TFT_MOSI, PIN_TFT_CS);
  tft.init(170, 320);
  tft.setRotation(3); // rotacja ekranu 1 lub 3 
  tft.invertDisplay(TFT_COLOR_INVERT);
  tft.setSPISpeed(40000000);
  tft.fillScreen(ST77XX_BLACK);
  delay(50);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);

  fb = new GFXcanvas16(TFT_W, TFT_H);
  if (!fb || !fb->getBuffer()) {
    Serial.println(F("FATAL: framebuffer allocation failed."));
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("Framebuffer ERR");
    while (true) delay(1000);
  }
}

void drawGradient(GFXcanvas16 &c, const Palette &p) {
  // Hard black background. This removes the visible panel-wide colour bands.
  c.fillScreen(ST77XX_BLACK);
}

void drawSoftPanel(GFXcanvas16 &c, int x, int y, int w, int h, uint16_t color) {
  c.fillRoundRect(x, y, w, h, 14, color);
  c.drawRoundRect(x, y, w, h, 14, mix565(color, rgb565(255, 255, 255), 0.11f));
}

void drawCenteredText(GFXcanvas16 &c, const String &txt, const GFXfont *font, uint16_t color, int baselineY) {
  c.setFont(font);
  c.setTextSize(1);
  c.setTextColor(color);
  int16_t x1, y1;
  uint16_t w, h;
  c.getTextBounds(txt, 0, baselineY, &x1, &y1, &w, &h);
  int x = (TFT_W - (int)w) / 2 - x1;
  c.setCursor(x, baselineY);
  c.print(txt);
}

void drawPill(GFXcanvas16 &c, int x, int y, int w, int h, const String &txt, uint16_t bg, uint16_t fg) {
  c.fillRoundRect(x, y, w, h, h / 2, bg);
  c.drawRoundRect(x, y, w, h, h / 2, mix565(bg, fg, 0.22f));
  c.setFont(NULL);
  c.setTextSize(1);
  c.setTextColor(fg);
  int tx = x + 8;
  int ty = y + (h - 8) / 2;
  c.setCursor(tx, ty);
  c.print(txt);
}

void drawStatusLabel(GFXcanvas16 &c, int x, int y, const char *label, bool ok, uint16_t okColor, uint16_t offColor) {
  uint16_t col = ok ? okColor : offColor;
  c.setFont(NULL);
  c.setTextSize(1);
  c.setTextColor(col);
  c.setCursor(x, y);
  c.print(label);
  c.fillCircle(x + strlen(label) * 6 + 6, y + 4, 3, col);
}

void drawHandIcon(GFXcanvas16 &c, int x, int y, uint16_t color) {
  // Small custom manual-mode hand glyph. Avoids emoji/font problems on GFX fonts.
  c.fillRoundRect(x + 6, y + 7, 13, 12, 4, color);      // palm
  c.fillRoundRect(x + 3, y + 10, 7, 5, 3, color);       // thumb
  c.fillRoundRect(x + 6, y + 0, 4, 10, 2, color);       // fingers
  c.fillRoundRect(x + 11, y + 1, 4, 10, 2, color);
  c.fillRoundRect(x + 16, y + 3, 4, 9, 2, color);
  c.fillRoundRect(x + 21, y + 6, 4, 8, 2, color);
  c.drawRoundRect(x + 1, y + 0, 25, 20, 6, mix565(color, ST77XX_WHITE, 0.35f));
}

void drawDegreeC(GFXcanvas16 &c, int x, int y, uint16_t color) {
  // Separate compact degree/C mark. Large gap prevents overlap with big digits.
  c.drawCircle(x, y - 19, 4, color);
  c.setFont(&FreeSansBold12pt7b);
  c.setTextSize(1);
  c.setTextColor(color);
  c.setCursor(x + 12, y - 2);
  c.print("C");
}

void drawFlameIcon(GFXcanvas16 &c, int x, int y, uint16_t hot, uint16_t core) {
  // Original orange heating glyph made from primitives. No emoji fonts needed.
  uint16_t rim = rgb565(255, 86, 18);
  uint16_t body = rgb565(255, 143, 31);
  uint16_t inner = rgb565(255, 218, 93);
  c.fillCircle(x, y + 22, 13, rim);
  c.fillTriangle(x - 14, y + 23, x + 14, y + 23, x - 1, y + 1, rim);
  c.fillTriangle(x - 5, y + 16, x + 12, y + 24, x + 5, y - 2, body);
  c.fillCircle(x, y + 23, 8, body);
  c.fillCircle(x, y + 25, 5, inner);
  c.fillTriangle(x - 5, y + 24, x + 5, y + 24, x, y + 11, inner);
}

void drawMetricTile(GFXcanvas16 &c, int x, int y, int w, int h, const char *label, const String &value, uint16_t labelColor, uint16_t valueColor, uint16_t borderColor) {
  uint16_t bg = rgb565(10, 13, 18);
  c.fillRoundRect(x, y, w, h, 12, bg);
  c.drawRoundRect(x, y, w, h, 12, mix565(borderColor, rgb565(255, 255, 255), 0.05f));
  c.setFont(NULL);
  c.setTextSize(1);
  c.setTextColor(labelColor);
  c.setCursor(x + 10, y + 7);
  c.print(label);
  c.setTextSize(2);
  c.setTextColor(valueColor);
  c.setCursor(x + 10, y + 20);
  c.print(value);
}

void drawHeatFlowRing(GFXcanvas16 &c, const Palette &p, bool heating, float phase) {
  int cx = 160;
  int cy = 78;
  int r = 63;

  for (int i = 0; i < 5; i++) {
    uint16_t col = mix565(p.ringDim, p.accent, 0.10f + i * 0.08f);
    c.drawCircle(cx, cy, r - i * 4, col);
  }

  int dots = heating ? 14 : 8;
  for (int i = 0; i < dots; i++) {
    float a = phase + (float)i * (2.0f * M_PI / dots);
    float breath = 0.5f + 0.5f * sinf(phase * 1.4f + i * 0.8f);
    int rr = r - 3 - (i % 3);
    int x = cx + (int)(cosf(a) * rr);
    int y = cy + (int)(sinf(a) * rr);
    uint16_t col = mix565(p.accent2, p.accent, breath);
    int rad = heating ? 3 + (i % 2) : 2;
    c.fillCircle(x, y, rad, col);
  }
}

UiSnapshot makeSnapshot() {
  UiSnapshot s;
  s.sensorOk = state.sensorOk;
  s.rtcOk = state.rtcOk;
  s.wifiOk = state.wifiOk;
  s.apActive = state.apActive;
  s.mqttOk = state.mqttOk;
  s.temperature = state.temperature;
  s.humidity = state.humidity;
  s.target = (state.mode == MODE_MANUAL) ? state.manualSetpoint : state.target;
  s.activeHysteresis = state.activeHysteresis;
  s.manualSetpoint = state.manualSetpoint;
  s.mode = state.mode;
  s.relayOverride = state.relayOverride;
  s.relayOn = state.relayOn;
  s.heatingCyclesToday = state.heatingCyclesToday;
  s.screen = state.screen;
  s.settingsPage = state.settingsPage;
  s.lcdBrightness = config.lcdBrightness;
  s.ip = state.wifiOk ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  s.ssid = state.wifiOk ? WiFi.SSID() : String("AP: Termostat-") + macSuffix();
  DateTime now = nowDateTime();
  s.timeShort = twoDigits(now.hour()) + ":" + twoDigits(now.minute());
  s.dateShort = String(now.year()) + "-" + twoDigits(now.month()) + "-" + twoDigits(now.day());
  return s;
}

void renderMain(GFXcanvas16 &c, const UiSnapshot &s) {
  // -------------------------------------------------------------------------
  // 1. OBSŁUGA CAŁKOWITEGO WYŁĄCZENIA (0%)
  // -------------------------------------------------------------------------
  if (s.lcdBrightness == 0) {
    c.fillScreen(rgb565(0, 0, 0)); // Czyszczenie ekranu na czarno
    return; // Przerywamy dalsze rysowanie – brak napisów, brak ikon
  }

  // 2. LOGIKA CZUJNIKÓW I PASMA
  if (s.sensorOk && !isnan(s.temperature)) {
    if (isnan(uiTemp)) uiTemp = s.temperature;
    uiTemp += (s.temperature - uiTemp) * 0.22f;
  }
  if (s.sensorOk && !isnan(s.humidity)) {
    if (isnan(uiHum)) uiHum = s.humidity;
    uiHum += (s.humidity - uiHum) * 0.16f;
  }
  uiTarget = s.target;

  Palette p = paletteFor(s);
  drawGradient(c, p);

  // Liniowe skalowanie dla ikon i kolorów w pełnym zakresie 1-100%
  // map(wartość, od_min, od_max, do_min, do_max)
  uint8_t colorDimFactor = map(s.lcdBrightness, 1, 100, 20, 255);

  // 3. GÓRNY PASEK STANU
  const uint16_t statusOn = rgb565(0, 255, 72);
  const uint16_t statusOff = rgb565(128, 136, 148);
  drawStatusLabel(c, 12, 11, "WiFi", s.wifiOk, statusOn, statusOff);
  drawStatusLabel(c, 64, 11, "HA", s.mqttOk, statusOn, statusOff);
  
  if (s.mode == MODE_MANUAL) {
    // Liniowe ściemnianie ikony trybu manualnego
    uint16_t handColor = scale565(rgb565(255, 204, 92), colorDimFactor);
    drawHandIcon(c, 112, 5, handColor);
  } else if (s.mode == MODE_OFF) {
    drawPill(c, 112, 6, 38, 16, "OFF", mix565(p.panel, p.danger, 0.42f), p.text);
  }

  c.setFont(NULL);
  c.setTextSize(1);
  c.setTextColor(p.muted);
  c.setCursor(270, 11);
  c.print(s.timeShort);

  // Liniowe ściemnianie ikony płomienia
  if (s.relayOn) {
    uint16_t flame1 = scale565(rgb565(255, 143, 31), colorDimFactor);
    uint16_t flame2 = scale565(rgb565(255, 218, 93), colorDimFactor);
    drawFlameIcon(c, 291, 36, flame1, flame2);
  }

  // -------------------------------------------------------------------------
  // 4. GŁÓWNY BLOK TEMPERATURY Z LINIOWĄ KOREKTĄ
  // -------------------------------------------------------------------------
  String tempTxt = (s.sensorOk && !isnan(uiTemp)) ? String(uiTemp, 1) : String("--.-");
  uint16_t tempColor = p.text;
  
  if (s.sensorOk && !isnan(s.temperature)) {
    const float band = max(0.0f, s.activeHysteresis);
    
    if (s.temperature > (s.target + band + 0.05f)) {
      // Liniowe ściemnianie czerwieni od 1% do 100% jasności
      tempColor = scale565(rgb565(255, 82, 82), colorDimFactor);
    } 
    else if (s.temperature < (s.target - band - 0.05f)) {
      // Liniowe ściemnianie niebieskiego od 1% do 100% jasności
      tempColor = scale565(rgb565(74, 164, 255), colorDimFactor);
    } 
    else {
      // Biały tekst ściemnia się dobrze naturalnie za sprawą PWM podświetlenia
      tempColor = p.text;
    }
  }

  c.setFont(&FreeSansBold24pt7b);
  c.setTextSize(2);
  c.setTextColor(tempColor);
  int16_t x1, y1;
  uint16_t w, h;
  c.getTextBounds(tempTxt, 0, 0, &x1, &y1, &w, &h);
  int totalW = (int)w + 42;
  int tempX = (TFT_W - totalW) / 2 - x1;
  if (tempX < 4) tempX = 4;
  int tempBase = 103;
  c.setCursor(tempX, tempBase);
  c.print(tempTxt);

  // Znak stopnia i jednostka "C"
  int degX = tempX + (int)w + 13;
  c.drawCircle(degX, tempBase - 67, 5, tempColor);
  c.setFont(&FreeSansBold12pt7b);
  c.setTextSize(1);
  c.setTextColor(tempColor);
  c.setCursor(degX + 14, tempBase - 51);
  c.print("C");

  // -------------------------------------------------------------------------
  // 5. DOLNA SEKCJA / KAFLIKI
  // -------------------------------------------------------------------------
  if (s.mode == MODE_OFF) {
    drawMetricTile(c, 89, 116, 142, 40, "", "OFF", p.text, p.text, p.panel);
    return;
  }

  String humTxt = (s.sensorOk && !isnan(uiHum)) ? String(uiHum, 0) + "%" : "--%";
  String targetTxt = String(uiTarget, 1) + " C";

  drawMetricTile(c, 14, 114, 142, 42, "WILGOTNOSC", humTxt, p.text, p.text, p.panel);
  drawMetricTile(c, 164, 114, 142, 42, "CEL", targetTxt, p.muted, p.text, p.panel);

  if (s.relayOverride != OVR_NONE) {
    drawPill(c, 204, 146, 102, 18, s.relayOverride == OVR_FORCE_ON ? "PRZEK. ON" : "PRZEK. OFF", mix565(p.panel, p.danger, 0.35f), p.text);
  }
}

void renderSettings(GFXcanvas16 &c, const UiSnapshot &s) {
  Palette p = paletteFor(s);

  // Full canvas redraw only. Do not call tft.fillScreen() from the menu renderer,
  // because physical clearing on every frame causes visible blinking on this ST7789.
  drawGradient(c, p);
  uint16_t panelBg = mix565(p.panel, p.bgTop, 0.18f);
  drawSoftPanel(c, 8, 8, 304, 154, panelBg);

  // No large "MENU" label: on this panel it was leaving a visible edge glyph,
  // seen as a single letter on the WiFi status page.
  c.setFont(NULL);
  c.setTextSize(1);
  c.setTextColor(mix565(p.muted, p.text, 0.65f));
  c.setCursor(252, 18);
  c.print("str ");
  c.print(s.settingsPage + 1);
  c.print("/4");

  c.drawFastHLine(18, 36, 284, mix565(p.ringDim, p.accent, 0.45f));

  // Brighter menu text. These pages are informational, so contrast is more important
  // than decorative grey.
  const uint16_t menuText = p.text;
  const uint16_t menuMuted = mix565(p.muted, p.text, 0.55f);
  c.setFont(NULL);
  c.setTextSize(2);
  c.setTextColor(menuText);

  if (s.settingsPage == 0) {
    c.setCursor(22, 52); c.print("WiFi: "); c.print(s.wifiOk ? "POLACZONE" : (s.apActive ? "Haslo:12345678" : "BRAK"));
    c.setCursor(22, 78); c.print("SSID: "); c.print(s.ssid.substring(0, 19));
    c.setCursor(22, 104); c.print("IP: "); c.print(s.ip);
    c.setCursor(22, 130); c.print("RTC: "); c.print(s.rtcOk ? "OK" : "BRAK");
  } else if (s.settingsPage == 1) {
    c.setCursor(22, 52); c.print("AUTO temp.: "); c.print(String(s.target, 1));
    c.drawCircle(212, 52, 2, menuText);
    c.setCursor(22, 80); c.print("AUTO hist.: "); c.print(String(s.activeHysteresis, 1));
    c.drawCircle(212, 80, 2, menuText);
    c.setCursor(22, 108); c.print("Manual: "); c.print(String(s.manualSetpoint, 1));
    c.drawCircle(165, 108, 2, menuText);
  } else if (s.settingsPage == 2) {
    c.setCursor(22, 52); c.print("Cykle dzis: "); c.print(s.heatingCyclesToday);
    c.setCursor(22, 80); c.print("Przekaznik: "); c.print(s.relayOn ? "ON" : "OFF");
    c.setCursor(22, 108); c.print("Wymuszenie: "); c.print(overrideToText(s.relayOverride));
  } else {
    c.setCursor(22, 52); c.print("Czas: "); c.print(s.timeShort);
    c.setCursor(22, 80); c.print("Data: "); c.print(s.dateShort);
    c.setCursor(22, 108); c.print("Jasnosc LCD: "); c.print(s.lcdBrightness); c.print("%");
    // Reset WiFi information is intentionally hidden from the device screen.
  }

  // Soft footer hint, bright enough to read but not dominant.
  c.setTextSize(1);
  c.setTextColor(menuMuted);
  c.setCursor(22, 148);
  c.print("obrot: strona   klik: wyjscie");

  c.drawRoundRect(8, 8, 304, 154, 14, mix565(panelBg, rgb565(255, 255, 255), 0.13f));
}

void renderFrame() {
  if (!fb) return;
  static ScreenMode lastScreen = SCREEN_MAIN;
  static bool firstFrame = true;
  UiSnapshot s = makeSnapshot();
  uiPhase += s.relayOn ? 0.12f : 0.035f;
  if (uiPhase > 2.0f * M_PI) uiPhase -= 2.0f * M_PI;

  if (firstFrame || s.screen != lastScreen) {
    // One-time physical clear removes possible residual glyphs in the ST7789 edge area.
    tft.fillScreen(ST77XX_BLACK);
    lastScreen = s.screen;
    firstFrame = false;
  }

  if (s.screen == SCREEN_MAIN) renderMain(*fb, s);
  else renderSettings(*fb, s);

  tft.drawRGBBitmap(0, 0, fb->getBuffer(), TFT_W, TFT_H);
}

void ensureMdnsStarted();

// ------------------------------ WIFI ------------------------------------
void startAccessPoint() {
  String apName = "Termostat-" + macSuffix();
  WiFi.mode(WIFI_AP_STA);
  bool ok = WiFi.softAP(apName.c_str(), "12345678");
  state.apActive = ok;
  Serial.print(F("AP: "));
  Serial.print(apName);
  Serial.print(F("  IP: "));
  Serial.println(WiFi.softAPIP());
}

void connectWiFiBlocking() {
  if (config.wifiSsid.length() == 0) {
    WiFi.mode(WIFI_AP_STA);
    startAccessPoint();
    state.wifiOk = false;
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPass.c_str());

  Serial.print(F("Connecting WiFi"));
  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  state.wifiOk = (WiFi.status() == WL_CONNECTED);
  if (state.wifiOk) {
    state.apActive = false;
    Serial.print(F("WiFi OK, IP: "));
    Serial.println(WiFi.localIP());
    ensureMdnsStarted();
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println(F("WiFi failed; starting configuration AP."));
    startAccessPoint();
  }
}

void reconnectWiFiNow() {
  Serial.println(F("WiFi reconnect requested."));
  mqtt.disconnect();
  WiFi.disconnect(true, true);
  mdnsStarted = false;
  delay(250);
  state.wifiOk = false;
  state.apActive = false;
  connectWiFiBlocking();
  if (state.wifiOk) syncRtcFromNtp(true);
}

void updateWiFiState() {
  bool was = state.wifiOk;
  state.wifiOk = (WiFi.status() == WL_CONNECTED);
  if (was != state.wifiOk) lastMqttPublishMs = 0;
}

void handleWifiResetPin() {
  bool low = (digitalRead(PIN_WIFI_RESET) == LOW);
  if (!low) {
    wifiResetStartMs = 0;
    wifiResetHandled = false;
    return;
  }

  if (wifiResetStartMs == 0) wifiResetStartMs = millis();
  if (!wifiResetHandled && millis() - wifiResetStartMs >= WIFI_RESET_HOLD_MS) {
    wifiResetHandled = true;
    clearWifiCredentials();
    saveConfig();
    wifiReconnectRequested = true;
    Serial.println(F("WiFi credentials cleared by GPIO7 reset pin."));
  }
}

// ------------------------------ WEB UI ----------------------------------
String stateJson() {
  DateTime now = nowDateTime();
  String json = "{";
  json += "\"time\":\"" + dateTimeToString(now) + "\",";

  json += "\"temperature\":";
  if (state.sensorOk && !isnan(state.temperature)) json += String(state.temperature, 2); else json += "null";
  json += ",\"humidity\":";
  if (state.sensorOk && !isnan(state.humidity)) json += String(state.humidity, 2); else json += "null";

  json += ",\"target\":"; json += String(state.target, 2);
  json += ",\"hysteresis\":"; json += String(state.activeHysteresis, 2);
  json += ",\"manualSetpoint\":"; json += String(state.manualSetpoint, 2);
  json += ",\"manualHysteresis\":"; json += String(config.manualHysteresis, 2);
  json += ",\"mode\":\"" + modeToText(state.mode) + "\"";
  json += ",\"relay\":"; json += (state.relayOn ? "true" : "false");
  json += ",\"relayOverride\":\"" + overrideToText(state.relayOverride) + "\"";
  json += ",\"cyclesToday\":"; json += String(state.heatingCyclesToday);
  json += ",\"wifi\":"; json += (state.wifiOk ? "true" : "false");
  json += ",\"ap\":"; json += (state.apActive ? "true" : "false");
  json += ",\"ip\":\"" + (state.wifiOk ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\"";
  json += ",\"ssid\":\"" + jsonEscape(state.wifiOk ? WiFi.SSID() : String("Termostat-") + macSuffix()) + "\"";
  json += ",\"rssi\":"; json += String(state.wifiOk ? WiFi.RSSI() : 0);
  json += ",\"mqtt\":"; json += (state.mqttOk ? "true" : "false");
  json += ",\"mqttError\":\"" + jsonEscape(mqttLastError) + "\"";
  json += ",\"rtc\":"; json += (state.rtcOk ? "true" : "false");
  json += ",\"brightness\":"; json += String(config.lcdBrightness);
  json += "}";
  return json;
}

String configJson() {
  String json = "{";
  json += "\"devName\":\"" + jsonEscape(config.deviceName) + "\",";
  json += "\"wifiSsid\":\"" + jsonEscape(config.wifiSsid) + "\",";
  json += "\"mqttHost\":\"" + jsonEscape(config.mqttHost) + "\",";
  json += "\"mqttPort\":" + String(config.mqttPort) + ",";
  json += "\"mqttUser\":\"" + jsonEscape(config.mqttUser) + "\",";
  json += "\"mqttPrefix\":\"" + jsonEscape(config.mqttPrefix) + "\",";
  json += "\"mqttDiscovery\":"; json += (config.mqttDiscovery ? "true" : "false");
  json += ",\"ntp\":"; json += (config.ntpEnabled ? "true" : "false");
  json += ",\"tz\":\"" + jsonEscape(config.tzString) + "\",";
  json += "\"brightness\":" + String(config.lcdBrightness) + ",";
  json += "\"manualHysteresis\":" + String(config.manualHysteresis, 2) + ",";
  json += "\"schedule\":\"" + jsonEscape(scheduleRaw) + "\"";
  json += "}";
  return json;
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="pl"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Panel</title>
<style>
:root{--bg:#05070d;--panel:#111722;--panel2:#18202d;--text:#f4f7fb;--muted:#8e9aac;--a:#23e0c8;--b:#ff8b3a;--d:#ff5b5b;--ok:#37d67a}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 30% 0%,#102a35 0%,#05070d 38%,#020309 100%);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}main{max-width:980px;margin:0 auto;padding:16px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}.card{background:rgba(17,23,34,.86);border:1px solid rgba(255,255,255,.08);border-radius:22px;padding:16px;margin:12px 0;box-shadow:0 12px 35px rgba(0,0,0,.25);backdrop-filter:blur(10px)}.tile{background:linear-gradient(180deg,rgba(255,255,255,.06),rgba(255,255,255,.02));border:1px solid rgba(255,255,255,.07);border-radius:18px;padding:14px}.label{color:var(--muted);font-size:13px}.value{font-size:30px;font-weight:800;margin-top:4px}.row{display:grid;grid-template-columns:210px 1fr;gap:12px;align-items:center;margin:10px 0}input,textarea,select{width:100%;border:1px solid rgba(255,255,255,.12);border-radius:12px;background:#090d15;color:var(--text);padding:10px;font:inherit}textarea{min-height:150px;font-family:ui-monospace,Consolas,monospace}button{border:0;border-radius:14px;background:linear-gradient(135deg,var(--a),#358bff);color:#001214;font-weight:800;padding:11px 14px;margin:5px;cursor:pointer}.heat{background:linear-gradient(135deg,var(--b),#ff3d4e);color:#1b0600}.danger{background:linear-gradient(135deg,#ff5b5b,#ff294d);color:white}.ghost{background:#202a3a;color:var(--text)}.ok{color:var(--ok)}.bad{color:var(--d)}code{color:#c7f7ef}.tabs{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0}.tab{background:#121a27;color:var(--text)}.tab.active{background:var(--text);color:#05070d}@media(max-width:640px){.row{grid-template-columns:1fr}.value{font-size:26px}}
</style></head><body><main>
<section class="card"><div class="grid">
<div class="tile"><div class="label">Aktualna temperatura</div><div class="value" id="temp">--</div></div>
<div class="tile"><div class="label">Wilgotność</div><div class="value" id="hum">--</div></div>
<div class="tile"><div class="label">Żądana temperatura</div><div class="value" id="target">--</div></div>
<div class="tile"><div class="label">Tryb pracy</div><div class="value" id="mode">--</div></div>
<div class="tile"><div class="label">Przekaźnik grzania</div><div class="value" id="relay">--</div></div>
<div class="tile"><div class="label">Cykle grzania dziś</div><div class="value" id="cycles">--</div></div>
</div></section>
<section class="card"><div class="tabs"><button class="tab active" onclick="showTab('control',this)">Sterowanie</button><button class="tab" onclick="showTab('config',this)">Konfiguracja</button><button class="tab" onclick="showTab('service',this)">Serwis</button></div>
<div id="control">
<div class="row"><label>Temperatura ręczna [°C]</label><input id="manualSetpoint" type="number" step="0.5" min="5" max="35"></div>
<button onclick="setManual()">Ustaw tryb RĘCZNY</button><button class="ghost" onclick="setMode('auto')">Tryb AUTO</button><button class="ghost" onclick="setMode('manual')">Tryb RĘCZNY</button><button class="danger" onclick="setMode('off')">Termostat OFF</button><br>
<button class="ghost" onclick="setRelay('AUTO')">Przekaźnik AUTO</button><button class="heat" onclick="setRelay('ON')">Przekaźnik ON</button><button class="danger" onclick="setRelay('OFF')">Przekaźnik OFF</button>
<div class="row"><label>Jasność LCD [%]</label><input id="brightness" type="range" min="0" max="100" oninput="setQuick('brightness',this.value)"></div>
</div>
<div id="config" style="display:none">
<div class="row"><label>Nazwa MQTT/HA</label><input id="devName"></div>
<div class="row"><label>SSID Wi‑Fi</label><input id="wifiSsid"></div>
<div class="row"><label>Hasło Wi‑Fi</label><input id="wifiPass" type="password" placeholder="zostaw puste, jeśli bez zmian"></div>
<div class="row"><label>MQTT host/IP</label><input id="mqttHost"></div>
<div class="row"><label>MQTT port</label><input id="mqttPort" type="number"></div>
<div class="row"><label>MQTT użytkownik</label><input id="mqttUser"></div>
<div class="row"><label>MQTT hasło</label><input id="mqttPass" type="password" placeholder="zostaw puste, jeśli bez zmian"></div>
<div class="row"><label>MQTT prefix</label><input id="mqttPrefix"></div>
<div class="row"><label>Discovery Home Assistant</label><select id="mqttDiscovery"><option value="1">włączone</option><option value="0">wyłączone</option></select></div>
<div class="row"><label>Histereza ręczna [°C]</label><input id="manualHysteresis" type="number" step="0.1" min="0.1" max="5"></div>
<div class="row"><label>NTP</label><select id="ntp"><option value="1">włączone</option><option value="0">wyłączone</option></select></div>
<div class="row"><label>Strefa TZ POSIX</label><input id="tz"></div>
<div class="row"><label>Harmonogram AUTO<br><code>HH:MM-HH:MM,temp,hist</code></label><textarea id="schedule"></textarea></div>
<button onclick="saveConfig()">Zapisz bez restartu</button>
</div>
<div id="service" style="display:none">
<div class="grid"><div class="tile"><div class="label">Wi‑Fi</div><div class="value" id="wifi">--</div></div><div class="tile"><div class="label">MQTT</div><div class="value" id="mqtt">--</div></div><div class="tile"><div class="label">RTC</div><div class="value" id="rtc">--</div></div><div class="tile"><div class="label">IP</div><div class="value" id="ip" style="font-size:20px">--</div></div></div>
<p>Kasowanie zapamiętanego Wi‑Fi sprzętowo: zewrzyj GPIO7 do GND przez minimum 4 sekundy. Pin ma włączone podciąganie programowe.</p>
<div class="row"><label>Ustaw RTC</label><input id="rtcSet" placeholder="YYYY-MM-DD HH:MM:SS"></div><button onclick="setRtc()">Ustaw RTC</button><button onclick="ntpNow()">Synchronizuj NTP teraz</button><br>
<button onclick="haDiscovery()">Wyślij discovery HA</button><button class="danger" onclick="resetWifi()">Skasuj Wi‑Fi i uruchom hotspot</button><button class="ghost" onclick="restartEsp()">Restart ESP</button>
</div></section>
</main><script>
function q(id){return document.getElementById(id)}function f(v,d=1){return v==null?'--':Number(v).toFixed(d)}
function showTab(id,btn){['control','config','service'].forEach(x=>q(x).style.display=x==id?'block':'none');document.querySelectorAll('.tab').forEach(b=>b.classList.remove('active'));btn.classList.add('active')}
async function api(path, data){let opt={method:data?'POST':'GET'};if(data){opt.headers={'Content-Type':'application/x-www-form-urlencoded'};opt.body=new URLSearchParams(data)}let r=await fetch(path,opt);return await r.json().catch(()=>({ok:false}))}
function setIdleValue(id,val){let e=q(id);if(e&&document.activeElement!==e)e.value=val}
async function poll(){try{let s=await api('/api/state');q('temp').textContent=f(s.temperature,1)+' °C';q('hum').textContent=f(s.humidity,0)+' %';q('target').textContent=String(s.mode)=='off'?'OFF':f(s.target,1)+' °C';q('mode').textContent=String(s.mode).toUpperCase();q('relay').textContent=s.relay?'ON':'OFF';q('cycles').textContent=s.cyclesToday;q('wifi').textContent=s.wifi?'OK':(s.ap?'HOTSPOT':'BRAK');q('mqtt').textContent=s.mqtt?'OK':'OFF';q('rtc').textContent=s.rtc?'OK':'BRAK';q('ip').textContent=s.ip;setIdleValue('manualSetpoint',s.manualSetpoint);setIdleValue('brightness',s.brightness);setIdleValue('rtcSet',s.time)}catch(e){}}
async function loadCfg(){let c=await api('/api/config');['devName','wifiSsid','mqttHost','mqttPort','mqttUser','mqttPrefix','manualHysteresis','tz','schedule'].forEach(k=>{if(q(k)&&c[k]!==undefined)q(k).value=c[k]});q('mqttDiscovery').value=c.mqttDiscovery?'1':'0';q('ntp').value=c.ntp?'1':'0'}
async function setManual(){await api('/api/set',{mode:'manual',setpoint:q('manualSetpoint').value});poll()}
async function setMode(m){await api('/api/set',{mode:m});poll()}
async function setRelay(o){await api('/api/set',{relayOverride:o});poll()}
async function setQuick(k,v){let d={};d[k]=v;await api('/api/set',d);poll()}
async function saveConfig(){let d={devName:q('devName').value,wifiSsid:q('wifiSsid').value,wifiPass:q('wifiPass').value,mqttHost:q('mqttHost').value,mqttPort:q('mqttPort').value,mqttUser:q('mqttUser').value,mqttPass:q('mqttPass').value,mqttPrefix:q('mqttPrefix').value,mqttDiscovery:q('mqttDiscovery').value,manualHysteresis:q('manualHysteresis').value,ntp:q('ntp').value,tz:q('tz').value,schedule:q('schedule').value};let r=await api('/api/config',d);alert(r.message||'Zapisano')}
async function setRtc(){await api('/api/rtc',{dt:q('rtcSet').value});poll()}async function ntpNow(){await api('/api/ntp',{});poll()}async function resetWifi(){if(confirm('Skasować Wi‑Fi? Urządzenie uruchomi hotspot konfiguracyjny.'))await api('/api/resetwifi',{})}async function haDiscovery(){let r=await api('/api/ha_discovery',{});alert(r.message||'Discovery wyslane')}async function restartEsp(){if(confirm('Restart ESP?'))await api('/api/restart',{})}
loadCfg();poll();setInterval(poll,1000);
</script></body></html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleApiSet() {
  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    m.toLowerCase();
    if (m == "auto") state.mode = MODE_AUTO;
    else if (m == "manual" || m == "heat") state.mode = MODE_MANUAL;
    else if (m == "off") state.mode = MODE_OFF;
    state.relayOverride = OVR_NONE;
  }
  if (server.hasArg("setpoint")) {
    state.manualSetpoint = clampFloat(server.arg("setpoint").toFloat(), MIN_SETPOINT, MAX_SETPOINT);
    prefs.putFloat("lastSet", state.manualSetpoint);
    state.mode = MODE_MANUAL;
    state.relayOverride = OVR_NONE;
  }
  if (server.hasArg("relayOverride")) {
    String o = server.arg("relayOverride");
    o.toUpperCase();
    if (o == "ON") state.relayOverride = OVR_FORCE_ON;
    else if (o == "OFF") state.relayOverride = OVR_FORCE_OFF;
    else state.relayOverride = OVR_NONE;
  }
  if (server.hasArg("brightness")) {
    setBacklight(clampBrightness(server.arg("brightness").toInt()));
    prefs.putUChar("bright", config.lcdBrightness);
  }
  if (server.hasArg("manualHysteresis")) {
    config.manualHysteresis = clampFloat(server.arg("manualHysteresis").toFloat(), 0.1f, 5.0f);
    prefs.putFloat("manHyst", config.manualHysteresis);
  }
  lastMqttPublishMs = 0;
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiConfigSave() {
  bool wifiChanged = false;
  bool mqttChanged = false;

  if (server.hasArg("devName")) {
    String v = server.arg("devName");
    v.trim();
    if (v.length() > 0 && v != config.deviceName) { config.deviceName = v; mqttChanged = true; }
  }
  if (server.hasArg("wifiSsid")) {
    String v = server.arg("wifiSsid");
    if (v != config.wifiSsid) { config.wifiSsid = v; wifiChanged = true; }
  }
  if (server.hasArg("wifiPass")) {
    String v = server.arg("wifiPass");
    if (v.length() > 0 && v != config.wifiPass) { config.wifiPass = v; wifiChanged = true; }
  }
  if (server.hasArg("mqttHost")) { String v = server.arg("mqttHost"); v.trim(); if (v != config.mqttHost) { config.mqttHost = v; mqttChanged = true; } }
  if (server.hasArg("mqttPort")) { uint16_t v = (uint16_t)server.arg("mqttPort").toInt(); if (v == 0) v = 1883; if (v != config.mqttPort) { config.mqttPort = v; mqttChanged = true; } }
  if (server.hasArg("mqttUser")) { String v = server.arg("mqttUser"); v.trim(); if (v != config.mqttUser) { config.mqttUser = v; mqttChanged = true; } }
  if (server.hasArg("mqttPass")) { String v = server.arg("mqttPass"); if (v.length() > 0 && v != config.mqttPass) { config.mqttPass = v; mqttChanged = true; } }
  if (server.hasArg("mqttPrefix")) { String v = server.arg("mqttPrefix"); v.trim(); if (v.length() == 0) v = "home/thermostat"; if (v != config.mqttPrefix) { config.mqttPrefix = v; mqttChanged = true; } }
  if (server.hasArg("mqttDiscovery")) { bool v = server.arg("mqttDiscovery") == "1"; if (v != config.mqttDiscovery) { config.mqttDiscovery = v; mqttChanged = true; } }
  if (server.hasArg("manualHysteresis")) config.manualHysteresis = clampFloat(server.arg("manualHysteresis").toFloat(), 0.1f, 5.0f);
  if (server.hasArg("ntp")) config.ntpEnabled = server.arg("ntp") == "1";
  if (server.hasArg("tz")) config.tzString = server.arg("tz");
  if (server.hasArg("schedule")) {
    saveScheduleRaw(server.arg("schedule"));
    parseScheduleText(scheduleRaw);
  }

  saveConfig();
  if (mqttChanged) {
    mqtt.disconnect();
    state.mqttOk = false;
    mqttLastError = "konfiguracja MQTT zmieniona";
    initMqtt();
    lastMqttReconnectMs = 0;
  }
  if (wifiChanged) wifiReconnectRequested = true;

  String msg = wifiChanged ? "Zapisano. Wi-Fi zostanie przełączone bez restartu." : "Zapisano bez restartu.";
  server.send(200, "application/json", String("{\"ok\":true,\"message\":\"") + jsonEscape(msg) + "\"}");
}

void handleApiRtc() {
  if (server.hasArg("dt")) {
    DateTime dt;
    if (parseDateTimeString(server.arg("dt"), dt) && state.rtcOk) {
      rtc.adjust(dt);
      server.send(200, "application/json", "{\"ok\":true}");
      return;
    }
  }
  server.send(400, "application/json", "{\"ok\":false}");
}

void handleApiNtp() {
  syncRtcFromNtp(true);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleApiResetWifi() {
  clearWifiCredentials();
  saveConfig();
  wifiReconnectRequested = true;
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"WiFi skasowane. Uruchamiam hotspot konfiguracyjny.\"}");
}

void handleApiHaDiscovery() {
  if (mqtt.connected()) {
    publishDiscovery();
    publishMqttState();
    server.send(200, "application/json", "{\"ok\":true,\"message\":\"Discovery Home Assistant wyslane.\"}");
  } else {
    server.send(503, "application/json", String("{\"ok\":false,\"message\":\"") + jsonEscape(mqttLastError) + "\"}");
  }
}

void handleApiRestart() {
  server.send(200, "application/json", "{\"ok\":true}");
  delay(250);
  ESP.restart();
}

void initWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, []() { server.send(200, "application/json", stateJson()); });
  server.on("/api/config", HTTP_GET, []() { server.send(200, "application/json", configJson()); });
  server.on("/api/set", HTTP_POST, handleApiSet);
  server.on("/api/config", HTTP_POST, handleApiConfigSave);
  server.on("/api/rtc", HTTP_POST, handleApiRtc);
  server.on("/api/ntp", HTTP_POST, handleApiNtp);
  server.on("/api/resetwifi", HTTP_POST, handleApiResetWifi);
  server.on("/api/ha_discovery", HTTP_POST, handleApiHaDiscovery);
  server.on("/api/restart", HTTP_POST, handleApiRestart);
  server.onNotFound([]() { server.send(404, "application/json", "{\"ok\":false,\"error\":\"404\"}"); });
  server.begin();
}

// ------------------------------- MQTT -----------------------------------
String mqttStateName(int rc) {
  switch (rc) {
    case MQTT_CONNECTION_TIMEOUT: return "timeout - broker nie odpowiada";
    case MQTT_CONNECTION_LOST: return "connection lost";
    case MQTT_CONNECT_FAILED: return "connect failed - port/firewall/adres";
    case MQTT_DISCONNECTED: return "disconnected";
    case MQTT_CONNECTED: return "connected";
    case MQTT_CONNECT_BAD_PROTOCOL: return "bad protocol";
    case MQTT_CONNECT_BAD_CLIENT_ID: return "bad client id";
    case MQTT_CONNECT_UNAVAILABLE: return "broker unavailable";
    case MQTT_CONNECT_BAD_CREDENTIALS: return "bledny login/haslo";
    case MQTT_CONNECT_UNAUTHORIZED: return "unauthorized";
    default: return "rc=" + String(rc);
  }
}

void mqttSetError(const String &msg) {
  mqttLastError = msg;
  Serial.print(F("MQTT: "));
  Serial.println(msg);
}

void ensureMdnsStarted() {
  if (!state.wifiOk || mdnsStarted) return;
  String host = sanitizeTopicPart(config.deviceName);
  if (host.length() < 3) host = "termostat-" + macSuffix();
  if (MDNS.begin(host.c_str())) {
    mdnsStarted = true;
    Serial.print(F("mDNS started as: "));
    Serial.print(host);
    Serial.println(F(".local"));
  } else {
    Serial.println(F("mDNS start failed; normal DNS/IP MQTT will still work."));
  }
}

bool resolveMqttHost(IPAddress &ipOut) {
  String host = config.mqttHost;
  host.trim();
  if (host.length() == 0) {
    mqttSetError(F("brak MQTT host/IP w konfiguracji"));
    return false;
  }

  String lower = host;
  lower.toLowerCase();
  if (lower == "localhost" || lower == "127.0.0.1" || lower == "0.0.0.0") {
    mqttSetError(F("MQTT host nie moze byc localhost/127.0.0.1; wpisz IP brokera w sieci LAN"));
    return false;
  }

  if (ipOut.fromString(host)) return true;

  if (lower.endsWith(".local")) {
    ensureMdnsStarted();
    String mdnsName = host.substring(0, host.length() - 6);
    IPAddress mdnsIp = MDNS.queryHost(mdnsName.c_str(), 2500);
    if (mdnsIp != IPAddress(0, 0, 0, 0)) {
      ipOut = mdnsIp;
      return true;
    }
    Serial.print(F("MQTT mDNS resolve failed for "));
    Serial.println(host);
  }

  if (WiFi.hostByName(host.c_str(), ipOut) == 1 && ipOut != IPAddress(0, 0, 0, 0)) return true;

  mqttSetError(String(F("nie moge rozwiazac MQTT host: ")) + host);
  return false;
}

String mqttBase() {
  // Stable topic root. Do not derive MQTT topics from editable display name,
  // otherwise Home Assistant can keep stale retained discovery topics after rename.
  return config.mqttPrefix + "/esp32s3_thermostat_" + macSuffix();
}

String deviceId() {
  return "esp32s3_thermostat_" + macSuffix();
}

bool mqttPublish(const String &topic, const String &payload, bool retained = true) {
  if (!mqtt.connected()) return false;
  bool ok = mqtt.publish(topic.c_str(), payload.c_str(), retained);
  if (!ok) {
    Serial.print(F("MQTT publish FAILED topic="));
    Serial.print(topic);
    Serial.print(F(" bytes="));
    Serial.println(payload.length());
  }
  return ok;
}

String mqttDeviceJson() {
  String dev = "{\"identifiers\":[\"" + deviceId() + "\"],";
  dev += "\"connections\":[[\"mac\",\"" + WiFi.macAddress() + "\"]],";
  dev += "\"name\":\"" + jsonEscape(config.deviceName) + "\",";
  dev += "\"manufacturer\":\"Margo-Tom DIY\",";
  dev += "\"model\":\"ESP32-S3 Room Thermostat\",";
  dev += "\"hw_version\":\"ESP32-S3 N16R8 + ST7789 + SHT41 + DS3231\",";
  dev += "\"sw_version\":\"" + String(FW_VERSION) + "\"";
  if (state.wifiOk) dev += ",\"configuration_url\":\"http://" + WiFi.localIP().toString() + "/\"";
  dev += "}";
  return dev;
}

String mqttOriginJson() {
  String o = "{\"name\":\"ESP32S3 Thermostat Firmware\",";
  o += "\"sw\":\"" + String(FW_VERSION) + "\",";
  o += "\"url\":\"http://" + (state.wifiOk ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "/\"}";
  return o;
}

String discoveryTopic(const String &component, const String &objectId) {
  return String(HA_DISCOVERY_PREFIX) + "/" + component + "/" + objectId + "/config";
}

void appendAvailabilityJson(String &payload) {
  payload += "\"availability_topic\":\"" + mqttBase() + "/availability\",";
  payload += "\"payload_available\":\"online\",";
  payload += "\"payload_not_available\":\"offline\",";
}

void publishSensorDiscovery(const String &object, const String &name, const String &topic,
                            const String &deviceClass, const String &unit, const String &stateClass) {
  String id = deviceId() + "_" + object;
  String cfgTopic = discoveryTopic("sensor", id);
  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + id + "\",";
  payload += "\"state_topic\":\"" + topic + "\",";
  appendAvailabilityJson(payload);
  payload += "\"origin\":" + mqttOriginJson() + ",";
  if (deviceClass.length()) payload += "\"device_class\":\"" + deviceClass + "\",";
  if (unit.length()) payload += "\"unit_of_measurement\":\"" + unit + "\",";
  if (stateClass.length()) payload += "\"state_class\":\"" + stateClass + "\",";
  if (object == "temperature") payload += "\"suggested_display_precision\":1,";
  if (object == "humidity") payload += "\"suggested_display_precision\":0,";
  payload += "\"device\":" + mqttDeviceJson();
  payload += "}";
  mqttPublish(cfgTopic, payload, true);
}

void publishBinaryDiscovery(const String &object, const String &name, const String &topic) {
  String id = deviceId() + "_" + object;
  String cfgTopic = discoveryTopic("binary_sensor", id);
  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + id + "\",";
  payload += "\"state_topic\":\"" + topic + "\",";
  payload += "\"payload_on\":\"ON\",\"payload_off\":\"OFF\",";
  appendAvailabilityJson(payload);
  payload += "\"origin\":" + mqttOriginJson() + ",";
  payload += "\"device\":" + mqttDeviceJson();
  payload += "}";
  mqttPublish(cfgTopic, payload, true);
}

void publishNumberDiscovery(const String &object, const String &name, const String &stateTopic,
                            const String &cmdTopic, float mn, float mx, float step, const String &unit) {
  String id = deviceId() + "_" + object;
  String cfgTopic = discoveryTopic("number", id);
  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + id + "\",";
  payload += "\"state_topic\":\"" + stateTopic + "\",";
  payload += "\"command_topic\":\"" + cmdTopic + "\",";
  payload += "\"min\":" + String(mn, 1) + ",\"max\":" + String(mx, 1) + ",\"step\":" + String(step, 1) + ",";
  if (unit.length()) payload += "\"unit_of_measurement\":\"" + unit + "\",";
  payload += "\"mode\":\"slider\",";
  appendAvailabilityJson(payload);
  payload += "\"origin\":" + mqttOriginJson() + ",";
  payload += "\"device\":" + mqttDeviceJson();
  payload += "}";
  mqttPublish(cfgTopic, payload, true);
}

void publishSwitchDiscovery(const String &object, const String &name, const String &stateTopic, const String &cmdTopic,
                            const String &payloadOn, const String &payloadOff, const String &stateOn, const String &stateOff) {
  String id = deviceId() + "_" + object;
  String cfgTopic = discoveryTopic("switch", id);
  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + id + "\",";
  payload += "\"state_topic\":\"" + stateTopic + "\",";
  payload += "\"command_topic\":\"" + cmdTopic + "\",";
  payload += "\"payload_on\":\"" + payloadOn + "\",\"payload_off\":\"" + payloadOff + "\",";
  payload += "\"state_on\":\"" + stateOn + "\",\"state_off\":\"" + stateOff + "\",";
  appendAvailabilityJson(payload);
  payload += "\"origin\":" + mqttOriginJson() + ",";
  payload += "\"device\":" + mqttDeviceJson();
  payload += "}";
  mqttPublish(cfgTopic, payload, true);
}

void publishButtonDiscovery(const String &object, const String &name, const String &cmdTopic, const String &payloadPress) {
  String id = deviceId() + "_" + object;
  String cfgTopic = discoveryTopic("button", id);
  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + id + "\",";
  payload += "\"command_topic\":\"" + cmdTopic + "\",";
  payload += "\"payload_press\":\"" + payloadPress + "\",";
  appendAvailabilityJson(payload);
  payload += "\"origin\":" + mqttOriginJson() + ",";
  payload += "\"device\":" + mqttDeviceJson();
  payload += "}";
  mqttPublish(cfgTopic, payload, true);
}

void clearLegacyDiscoveryTopics(const String &id) {
  // Older firmware revisions used this climate config topic with the same unique_id.
  // Clearing it avoids duplicated/invalid retained discovery payloads in Home Assistant.
  mqttPublish(String(HA_DISCOVERY_PREFIX) + "/climate/" + id + "/config", "", true);
}

void publishDiscovery() {
  if (!config.mqttDiscovery || !mqtt.connected()) return;

  String base = mqttBase();
  String id = deviceId();

  clearLegacyDiscoveryTopics(id);

  String climateTopic = discoveryTopic("climate", id + "_climate");
  String climate = "{";
  climate += "\"name\":null,";
  climate += "\"object_id\":\"" + id + "_climate\",";
  climate += "\"unique_id\":\"" + id + "_climate\",";
  appendAvailabilityJson(climate);
  climate += "\"current_temperature_topic\":\"" + base + "/temperature/state\",";
  climate += "\"current_humidity_topic\":\"" + base + "/humidity/state\",";
  climate += "\"temperature_state_topic\":\"" + base + "/setpoint/state\",";
  climate += "\"temperature_command_topic\":\"" + base + "/setpoint/set\",";
  climate += "\"mode_state_topic\":\"" + base + "/mode/state\",";
  climate += "\"mode_command_topic\":\"" + base + "/mode/set\",";
  climate += "\"action_topic\":\"" + base + "/action/state\",";
  climate += "\"modes\":[\"off\",\"heat\",\"auto\"],";
  climate += "\"temperature_unit\":\"C\",\"min_temp\":5,\"max_temp\":35,\"temp_step\":0.5,\"precision\":0.1,";
  climate += "\"optimistic\":false,";
  climate += "\"origin\":" + mqttOriginJson() + ",";
  climate += "\"device\":" + mqttDeviceJson();
  climate += "}";
  mqttPublish(climateTopic, climate, true);

  publishSensorDiscovery("temperature", "Temperatura", base + "/temperature/state", "temperature", "°C", "measurement");
  publishSensorDiscovery("humidity", "Wilgotnosc", base + "/humidity/state", "humidity", "%", "measurement");
  publishSensorDiscovery("wifi_rssi", "WiFi RSSI", base + "/wifi_rssi/state", "signal_strength", "dBm", "measurement");
  publishSensorDiscovery("cycles_today", "Cykle grzania dzisiaj", base + "/cycles_today/state", "", "", "measurement");
  publishBinaryDiscovery("relay", "Przekaznik grzania ON/OFF", base + "/relay/state");
  publishNumberDiscovery("manual_hysteresis", "Histereza reczna", base + "/hysteresis/state", base + "/hysteresis/set", 0.1f, 5.0f, 0.1f, "°C");
  publishNumberDiscovery("lcd_brightness", "Jasnosc LCD", base + "/lcd_brightness/state", base + "/lcd_brightness/set", 0.0f, 100.0f, 1.0f, "%");
  publishSwitchDiscovery("relay_force_on", "Przekaznik wymus ON", base + "/relay_override/state", base + "/relay_override/set", "ON", "AUTO", "force_on", "auto");
  publishSwitchDiscovery("relay_force_off", "Przekaznik wymus OFF", base + "/relay_override/state", base + "/relay_override/set", "OFF", "AUTO", "force_off", "auto");
 // publishButtonDiscovery("reset_wifi", "Skasuj WiFi", base + "/reset_wifi/set", "ON"); // możliwość skasowania ustawień wifi z poziomu HA 
  //publishButtonDiscovery("republish_discovery", "Ponow discovery HA", base + "/ha_discovery/set", "ON"); // to nie potrzebne z poziomu HA skoro już go znalazło 
}


void publishMqttState() {
  if (!mqtt.connected()) return;
  String base = mqttBase();

  mqttPublish(base + "/availability", "online", true);

  if (state.sensorOk && !isnan(state.temperature)) mqttPublish(base + "/temperature/state", String(state.temperature, 2), true);
  if (state.sensorOk && !isnan(state.humidity)) mqttPublish(base + "/humidity/state", String(state.humidity, 2), true);
  mqttPublish(base + "/setpoint/state", String(state.target, 2), true);
  mqttPublish(base + "/hysteresis/state", String(config.manualHysteresis, 2), true);
  mqttPublish(base + "/mode/state", modeToHaClimateText(state.mode), true);
  mqttPublish(base + "/action/state", climateActionText(), true);
  mqttPublish(base + "/relay/state", state.relayOn ? "ON" : "OFF", true);
  mqttPublish(base + "/relay_override/state", overrideToText(state.relayOverride), true);
  mqttPublish(base + "/lcd_brightness/state", String(config.lcdBrightness), true);
  mqttPublish(base + "/cycles_today/state", String(state.heatingCyclesToday), true);
  mqttPublish(base + "/wifi_rssi/state", String(state.wifiOk ? WiFi.RSSI() : 0), true);
  mqttPublish(base + "/json/state", stateJson(), true);
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String t(topic);
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  String base = mqttBase();
  Serial.print(F("MQTT cmd "));
  Serial.print(t);
  Serial.print(F(" = "));
  Serial.println(msg);

  if (t == String(HA_DISCOVERY_PREFIX) + "/status") {
    msg.toLowerCase();
    if (msg == "online") {
      // Home Assistant explicitly recommends devices republish discovery when HA sends its birth message.
      publishDiscovery();
      publishMqttState();
    }
    return;
  }

  if (t == base + "/setpoint/set") {
    state.manualSetpoint = clampFloat(msg.toFloat(), MIN_SETPOINT, MAX_SETPOINT);
    prefs.putFloat("lastSet", state.manualSetpoint);
    state.mode = MODE_MANUAL;
    state.relayOverride = OVR_NONE;
  } else if (t == base + "/mode/set") {
    msg.toLowerCase();
    if (msg == "auto") state.mode = MODE_AUTO;
    else if (msg == "heat" || msg == "manual") state.mode = MODE_MANUAL;
    else if (msg == "off") state.mode = MODE_OFF;
    state.relayOverride = OVR_NONE;
  } else if (t == base + "/relay_override/set") {
    msg.toUpperCase();
    if (msg == "ON" || msg == "FORCE_ON") state.relayOverride = OVR_FORCE_ON;
    else if (msg == "OFF" || msg == "FORCE_OFF") state.relayOverride = OVR_FORCE_OFF;
    else state.relayOverride = OVR_NONE;
  } else if (t == base + "/hysteresis/set") {
    config.manualHysteresis = clampFloat(msg.toFloat(), 0.1f, 5.0f);
    prefs.putFloat("manHyst", config.manualHysteresis);
  } else if (t == base + "/lcd_brightness/set") {
    setBacklight(clampBrightness(msg.toInt()));
    prefs.putUChar("bright", config.lcdBrightness);
  } else if (t == base + "/schedule/set") {
    saveScheduleRaw(msg);
    parseScheduleText(scheduleRaw);
  } else if (t == base + "/rtc/set") {
    DateTime dt;
    if (state.rtcOk && parseDateTimeString(msg, dt)) rtc.adjust(dt);
  } else if (t == base + "/reset_wifi/set") {
    if (msg == "1" || msg == "ON" || msg == "on") {
      clearWifiCredentials();
      saveConfig();
      wifiReconnectRequested = true;
    }
  } else if (t == base + "/ha_discovery/set") {
    if (msg == "1" || msg == "ON" || msg == "on") {
      publishDiscovery();
      publishMqttState();
    }
  }

  lastMqttPublishMs = 0;
}

void initMqtt() {
  mqtt.setBufferSize(8192);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SEC);
  wifiClient.setTimeout(MQTT_SOCKET_TIMEOUT_SEC * 1000UL);
  mqtt.setCallback(mqttCallback);

  config.mqttHost.trim();
  config.mqttUser.trim();
  config.mqttPrefix.trim();
  if (config.mqttPrefix.length() == 0) config.mqttPrefix = "home/thermostat";
  if (config.mqttPort == 0) config.mqttPort = 1883;

  Serial.print(F("MQTT config: host="));
  Serial.print(config.mqttHost.length() ? config.mqttHost : String(F("<empty>")));
  Serial.print(F(" port="));
  Serial.print(config.mqttPort);
  Serial.print(F(" user="));
  Serial.print(config.mqttUser.length() ? String(F("set")) : String(F("none")));
  Serial.print(F(" prefix="));
  Serial.println(config.mqttPrefix);
}

void subscribeMqttCommands() {
  String base = mqttBase();
  mqtt.subscribe((base + "/setpoint/set").c_str());
  mqtt.subscribe((base + "/mode/set").c_str());
  mqtt.subscribe((base + "/relay_override/set").c_str());
  mqtt.subscribe((base + "/hysteresis/set").c_str());
  mqtt.subscribe((base + "/lcd_brightness/set").c_str());
  mqtt.subscribe((base + "/schedule/set").c_str());
  mqtt.subscribe((base + "/rtc/set").c_str());
  mqtt.subscribe((base + "/reset_wifi/set").c_str());
  mqtt.subscribe((base + "/ha_discovery/set").c_str());
  mqtt.subscribe((String(HA_DISCOVERY_PREFIX) + "/status").c_str());
}


void connectMqttIfNeeded() {
  if (!state.wifiOk) {
    if (state.mqttOk) mqtt.disconnect();
    state.mqttOk = false;
    mqttLastError = "WiFi offline";
    return;
  }

  config.mqttHost.trim();
  if (config.mqttHost.length() == 0) {
    state.mqttOk = false;
    mqttLastError = "brak MQTT host/IP w konfiguracji";
    return;
  }

  if (mqtt.connected()) {
    state.mqttOk = true;
    return;
  }

  state.mqttOk = false;
  if (millis() - lastMqttReconnectMs < MQTT_RECONNECT_INTERVAL_MS) return;
  lastMqttReconnectMs = millis();
  mqttLastAttemptMs = millis();

  IPAddress brokerIp;
  if (!resolveMqttHost(brokerIp)) return;

  mqtt.setServer(brokerIp, config.mqttPort);
  String clientId = sanitizeTopicPart(config.deviceName) + "_" + macSuffix();
  String base = mqttBase();
  String willTopic = base + "/availability";

  Serial.print(F("MQTT connect -> host=")); Serial.print(config.mqttHost);
  Serial.print(F(" ip=")); Serial.print(brokerIp);
  Serial.print(F(" port=")); Serial.print(config.mqttPort);
  Serial.print(F(" clientId=")); Serial.print(clientId);
  Serial.print(F(" user=")); Serial.print(config.mqttUser.length() ? F("set") : F("none"));
  Serial.print(F(" wifiIP=")); Serial.println(WiFi.localIP());

  bool ok = false;
  if (config.mqttUser.length()) {
    ok = mqtt.connect(clientId.c_str(), config.mqttUser.c_str(), config.mqttPass.c_str(),
                      willTopic.c_str(), 1, true, "offline");
  } else {
    ok = mqtt.connect(clientId.c_str(), willTopic.c_str(), 1, true, "offline");
  }

  if (ok) {
    state.mqttOk = true;
    mqttLastError = "connected";
    mqttPublish(willTopic, "online", true);
    subscribeMqttCommands();
    publishDiscovery();
    publishMqttState();
    Serial.println(F("MQTT connected and HA discovery/state published."));
  } else {
    int rc = mqtt.state();
    state.mqttOk = false;
    mqttSetError(String(F("polaczenie nieudane: ")) + mqttStateName(rc));
  }
}

// ------------------------------ INPUT -----------------------------------
void IRAM_ATTR encoderISR() {
  uint8_t a = gpio_get_level((gpio_num_t)PIN_ENC_CLK) ? 1 : 0;
  uint8_t b = gpio_get_level((gpio_num_t)PIN_ENC_DT) ? 1 : 0;
  uint8_t current = (a << 1) | b;
  uint8_t idx = (encLastState << 2) | current;
  int8_t movement = ENC_TABLE[idx & 0x0F];
  encLastState = current;

  if (movement != 0) {
    portENTER_CRITICAL_ISR(&encMux);
    encAcc += movement;
    //if (encAcc >= 4) {  // Enkoder co 4 impulsy 
    if (encAcc >= 2) {   // enkoder co 2 impulsy ( tańszy)
      encDelta++;
      encAcc = 0;
    //} else if (encAcc <= -4) { // w drugą stronę 
    } else if (encAcc <= -2) {
      encDelta--;
      encAcc = 0;
    }
    portEXIT_CRITICAL_ISR(&encMux);
  }
}

int32_t takeEncoderDelta() {
  portENTER_CRITICAL(&encMux);
  int32_t d = encDelta;
  encDelta = 0;
  portEXIT_CRITICAL(&encMux);
  return d;
}

void handleEncoderMove(int32_t delta) {
  if (delta == 0) return;

  if (state.screen == SCREEN_MAIN) {
    if (state.mode != MODE_MANUAL) {
      state.manualSetpoint = state.target;
      state.mode = MODE_MANUAL;
      state.relayOverride = OVR_NONE;
    }
    state.manualSetpoint += delta * SETPOINT_STEP;
    state.manualSetpoint = clampFloat(state.manualSetpoint, MIN_SETPOINT, MAX_SETPOINT);
    pendingManualSetpointSave = true;
    lastManualSetpointChangeMs = millis();
    lastMqttPublishMs = 0;
  } else {
    state.settingsPage += delta;
    while (state.settingsPage < 0) state.settingsPage += 4;
    while (state.settingsPage > 3) state.settingsPage -= 4;
  }
}

void handleShortClick() {
  if (state.screen == SCREEN_MAIN) {
    state.mode = MODE_AUTO;
    state.relayOverride = OVR_NONE;
    lastMqttPublishMs = 0;
  } else {
    state.screen = SCREEN_MAIN;
  }
}

void handleLongClick() {
  state.screen = SCREEN_SETTINGS;
  state.settingsPage = 0;
}

void updateEncoderButton() {
  int32_t d = takeEncoderDelta();
  if (d != 0) handleEncoderMove(d);

  bool rawPressed = (digitalRead(PIN_ENC_SW) == LOW);
  if (rawPressed != btnLastRaw) {
    btnDebounceMs = millis();
    btnLastRaw = rawPressed;
  }

  if (millis() - btnDebounceMs > 35) {
    if (rawPressed != btnStable) {
      btnStable = rawPressed;
      if (btnStable) {
        btnPressStartMs = millis();
        btnLongHandled = false;
      } else {
        if (!btnLongHandled) handleShortClick();
      }
    }
  }

  if (btnStable && !btnLongHandled && millis() - btnPressStartMs >= BUTTON_LONG_PRESS_MS) {
    btnLongHandled = true;
    handleLongClick();
  }
}

// ------------------------------- TASKS ----------------------------------
void taskUI(void *p) {
  wdtAddCurrentTask();
  uint32_t lastFrame = 0;
  while (true) {
    // Main screen stays responsive for encoder changes. Settings pages are almost static,
    // so they are refreshed more slowly to remove the visible menu shimmer/blink.
    uint32_t frameMs = (state.screen == SCREEN_SETTINGS) ? 180 : UI_FRAME_MS;
    if (millis() - lastFrame >= frameMs) {
      lastFrame = millis();
      renderFrame();
    }
    wdtFeed();
    vTaskDelay(1);
  }
}

void taskLogic(void *p) {
  wdtAddCurrentTask();
  while (true) {
    server.handleClient();
    updateEncoderButton();
    if (pendingManualSetpointSave && millis() - lastManualSetpointChangeMs > 900) {
      pendingManualSetpointSave = false;
      prefs.putFloat("lastSet", state.manualSetpoint);
    }
    handleWifiResetPin();

    if (wifiReconnectRequested) {
      wifiReconnectRequested = false;
      reconnectWiFiNow();
    }

    updateWiFiState();

    if (millis() - lastSensorMs >= SENSOR_INTERVAL_MS) {
      lastSensorMs = millis();
      readSensors();
    }

    if (millis() - lastControlMs >= CONTROL_INTERVAL_MS) {
      lastControlMs = millis();
      updateRelayControl();
    }

    connectMqttIfNeeded();
    if (mqtt.connected()) mqtt.loop();

    if (millis() - lastMqttPublishMs >= MQTT_PUBLISH_INTERVAL_MS) {
      lastMqttPublishMs = millis();
      publishMqttState();
    }

    syncRtcFromNtp(false);
    wdtFeed();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ------------------------------- SETUP/LOOP -----------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println();
  Serial.print(F("ESP32-S3 thermostat premium starting, FW ")); Serial.println(FW_VERSION);

  pinMode(PIN_RELAY, OUTPUT);
  state.relayOn = false;
  setRelayPhysical(false);

  pinMode(PIN_WIFI_RESET, INPUT_PULLUP);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  encLastState = ((digitalRead(PIN_ENC_CLK) ? 1 : 0) << 1) | (digitalRead(PIN_ENC_DT) ? 1 : 0);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT), encoderISR, CHANGE);

  loadConfig();
  if (digitalRead(PIN_WIFI_RESET) == LOW) {
    Serial.println(F("GPIO7 held low at boot: clearing WiFi credentials."));
    clearWifiCredentials();
    saveConfig();
  }
  parseScheduleText(scheduleRaw);

  initDisplay();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  initSensors();
  initRtc();

  connectWiFiBlocking();
  initWebServer();
  syncRtcFromNtp(true);
  initMqtt();

  // Device must return to automatic schedule after power loss.
  state.mode = MODE_AUTO;
  state.relayOverride = OVR_NONE;
  calculateTarget();
  readSensors();
  updateRelayControl();

  initWatchdog();

  xTaskCreatePinnedToCore(taskUI, "ui", 8192, NULL, 2, &uiTaskHandle, 0);
  xTaskCreatePinnedToCore(taskLogic, "logic", 12288, NULL, 3, &logicTaskHandle, 1);
}

void loop() {
  // Real work is split into FreeRTOS tasks. Keep loop task idle.
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
