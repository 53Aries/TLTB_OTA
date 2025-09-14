// ESP32-S3 Trailer Lighting Test Box (TLTB)
// Run Status page, interactive OPEN/SHORT popups (Back=Cancel, PUSH=Enable),
// "Back" wording, Wi-Fi scan/select/password UI, OTA (GitHub),
// TFT + encoder + Back button, Relays with pulse-test + OCP/open/short,
// INA226, CC1101 RF (learn 6 buttons), buzzer, NVS prefs.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// ------------------- Pin Map -------------------
static constexpr int PIN_FSPI_SCK  = 36;
static constexpr int PIN_FSPI_MOSI = 35;
static constexpr int PIN_FSPI_MISO = 37;

static constexpr int PIN_TFT_CS   = 5;
static constexpr int PIN_TFT_DC   = 2;
static constexpr int PIN_TFT_RST  = 4;
static constexpr int PIN_TFT_BL   = 21;

static constexpr int PIN_CC1101_CS   = 10;
static constexpr int PIN_CC1101_GDO0 = 7;

static constexpr int PIN_I2C_SDA   = 8;
static constexpr int PIN_I2C_SCL   = 9;
static constexpr int PIN_INA_ALERT = 16;

static constexpr int PIN_SW_POS1 = 34;
static constexpr int PIN_SW_POS2 = 35;
static constexpr int PIN_SW_POS3 = 36;
static constexpr int PIN_SW_POS4 = 37;
static constexpr int PIN_SW_POS5 = 38;
static constexpr int PIN_SW_POS6 = 39;
static constexpr int PIN_SW_POS7 = 40;
static constexpr int PIN_SW_POS8 = 41;

static constexpr int PIN_ENC_A   = 32;
static constexpr int PIN_ENC_B   = 33;
static constexpr int PIN_ENC_BTN = 25;  // encoder PUSH
static constexpr int PIN_ENC_KO  = 26;  // physical Back button

static constexpr int PIN_RLY_LEFT   = 11;
static constexpr int PIN_RLY_RIGHT  = 12;
static constexpr int PIN_RLY_BRAKE  = 13;
static constexpr int PIN_RLY_TAIL   = 14;
static constexpr int PIN_RLY_MARKER = 17;
static constexpr int PIN_RLY_AUX    = 22;

static constexpr int PIN_BUZZER = 27;

// ------------------- OTA URL -------------------
#ifndef OTA_LATEST_ASSET_URL
#define OTA_LATEST_ASSET_URL "https://github.com/53Aries/TLTB_OTA/releases/latest/download/firmware.bin"
#endif

// ------------------- INA226 Config -------------------
static float OCP_LIMIT_A   = 20.0f;     // editable via menu
static constexpr float SHUNT_OHMS    = 0.0025f; // 2.5 mΩ (30A/75mV)
static constexpr float CURRENT_LSB_A = 0.001f;  // 1 mA/bit
static constexpr float FAST_SHORT_A  = 40.0f;   // instant trip
static constexpr float OPEN_THRESH_A = 0.15f;   // open load detect

// ------------------- Timings -------------------
static constexpr uint32_t PULSE_MS        = 80;
static constexpr uint32_t POST_PULSE_MS   = 40;
static constexpr uint32_t DOUBLE_PRESS_MS = 500;

// ------------------- TFT -------------------
SPIClass spiTFT(FSPI);
Adafruit_ST7735 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

// ------------------- Preferences -------------------
Preferences prefs;
static const char* NVS_NS = "net";

// Wi-Fi creds in NVS
static const char* KEY_WIFI_SSID = "wifi_ssid";
static const char* KEY_WIFI_PASS = "wifi_pass";

// ------------------- Relay Enum -------------------
enum RelayId { R_NONE=-1, R_LEFT, R_RIGHT, R_BRAKE, R_TAIL, R_MARKER, R_AUX, R_COUNT };
static const int RELAY_PIN[R_COUNT] = {PIN_RLY_LEFT, PIN_RLY_RIGHT, PIN_RLY_BRAKE, PIN_RLY_TAIL, PIN_RLY_MARKER, PIN_RLY_AUX};
static bool relayState[R_COUNT] = {false,false,false,false,false,false};

// Labels + Preference keys for RF learn
static const char* RELAY_LABELS[R_COUNT] = {"LEFT","RIGHT","BRAKE","TAIL","MARKER","AUX"};
static const char* RF_PREF_KEYS[R_COUNT] = {"rf_left","rf_right","rf_brake","rf_tail","rf_marker","rf_aux"};

// ------------------- Flash Mode -------------------
static bool     flashMode   = false;
static RelayId  flashTarget = R_NONE;
static RelayId  lastRfRelay = R_NONE;

// ------------------- Buzzer -------------------
static void buzzerBeep(uint16_t ms=60){ digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW); }
static void buzzerAlarm(uint16_t ms=800){ digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW); }

// ------------------- Relay Helpers -------------------
static inline void relayOn(RelayId r){ if(r>=0&&r<R_COUNT){ digitalWrite(RELAY_PIN[r],HIGH); relayState[r]=true; } }
static inline void relayOff(RelayId r){ if(r>=0&&r<R_COUNT){ digitalWrite(RELAY_PIN[r],LOW);  relayState[r]=false;} }
static inline void relayOffAll(){ for(int i=0;i<R_COUNT;i++) relayOff((RelayId)i); }

// ------------------- Name Helpers -------------------
static const char* relayName(RelayId r){
  switch(r){
    case R_LEFT:   return "LEFT";
    case R_RIGHT:  return "RIGHT";
    case R_BRAKE:  return "BRAKE";
    case R_TAIL:   return "TAIL";
    case R_MARKER: return "MARKER";
    case R_AUX:    return "AUX";
    default:       return "NONE";
  }
}
static RelayId currentActiveRelay(){ for(int i=0;i<R_COUNT;i++) if(relayState[i]) return (RelayId)i; return R_NONE; }

// ------------------- Forward declarations used across sections ---
static int8_t readEncoderStep();
static bool   readButtonPressed();
static bool   readKoPressed();               // physical "Back" button
static void   drawMenu();
static void   drawStatusPage(bool force=false);
static void   refreshStatusIfChanged();

// ------------------- UI: Status (Run) page -------------------
static bool uiInMenu = false;
static RelayId  _lastShownRelay = R_NONE;
static bool     _lastShownFlash = false;

static void drawStatusPage(bool force){
  RelayId act = currentActiveRelay();
  bool flash = flashMode;

  if (!force && act==_lastShownRelay && flash==_lastShownFlash) return;

  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("TLTB - Run");

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(0, 18);
  tft.print("Active Relay: ");
  tft.print(relayName(act));

  tft.setCursor(0, 34);
  tft.print("Flash: ");
  tft.print(flash ? "ON" : "OFF");

  // Hint line
  tft.setCursor(0, 52);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("PUSH=Menu   Back=Exit");

  _lastShownRelay = act;
  _lastShownFlash = flash;
}

static inline void refreshStatusIfChanged(){
  if (!uiInMenu) drawStatusPage(false);
}

// ------------------- Fault choice popup (interactive) -------------------
enum FaultType { FAULT_OPEN=0, FAULT_SHORT=1 };

// Returns true if user presses PUSH to enable anyway, false if Back to cancel
static bool showFaultChoicePopup(FaultType ft, RelayId r) {
  bool wasInMenu = uiInMenu;

  for (;;) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);

    if (ft == FAULT_OPEN) {
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.print("OPEN detected");
    } else {
      tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
      tft.print("SHORT detected");
    }

    tft.setCursor(0, 20);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print("On relay: ");
    tft.print(relayName(r));

    tft.setCursor(0, 42);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.print("Back = Cancel");

    tft.setCursor(0, 56);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.print("PUSH = Enable");

    if (readButtonPressed()) {             // PUSH → enable anyway
      if (wasInMenu) drawMenu(); else drawStatusPage(true);
      return true;
    }
    if (readKoPressed()) {                 // Back → cancel
      if (wasInMenu) drawMenu(); else drawStatusPage(true);
      return false;
    }
    delay(10);
  }
}

// ------------------- INA226 -------------------
namespace INA226 {
  static uint8_t addr = 0x40;

  static void wr16(uint8_t r, uint16_t v){
    Wire.beginTransmission(addr); Wire.write(r);
    Wire.write((uint8_t)(v>>8)); Wire.write((uint8_t)(v&0xFF));
    Wire.endTransmission();
  }
  static uint16_t rd16(uint8_t r){
    Wire.beginTransmission(addr); Wire.write(r); Wire.endTransmission(false);
    Wire.requestFrom((int)addr,2);
    return (Wire.read()<<8)|Wire.read();
  }

  static void setOcpLimit(float amps); // forward-declare

  static void begin(){
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    // Reset
    wr16(0x00, 0x8000); delay(2);

    // Config: AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, MODE=Shunt+Bus continuous
    wr16(0x00, (0b010<<9)|(0b100<<6)|(0b100<<3)|0b111);

    // Calibration for 2.5 mΩ, 1mA/bit
    wr16(0x05, 0x0800);

    pinMode(PIN_INA_ALERT, INPUT_PULLUP);

    setOcpLimit(OCP_LIMIT_A);   // program alert threshold + enable SOL
  }

  static float currentA(){
    int16_t r = (int16_t)rd16(0x04);
    return r * CURRENT_LSB_A;
  }

  static bool overCurrent(){ return digitalRead(PIN_INA_ALERT) == LOW; }

  static void setOcpLimit(float amps){
    OCP_LIMIT_A = amps;
    uint16_t limit = (uint16_t)round(OCP_LIMIT_A / CURRENT_LSB_A);
    wr16(0x06, limit);   // Alert Limit
    wr16(0x07, 0x0002);  // Mask/Enable: enable SOL (bit1)
  }
}

// ------------------- Pulse Test -------------------
static bool pulseTestAndEngage(RelayId rly) {
  relayOn(rly);
  delay(PULSE_MS);

  float ia = INA226::currentA();
  delay(POST_PULSE_MS);

  // Short-circuit detection
  if (ia >= FAST_SHORT_A || INA226::overCurrent()) {
    relayOff(rly);
    buzzerAlarm();

    if (showFaultChoicePopup(FAULT_SHORT, rly)) {
      relayOn(rly);
      buzzerBeep();
      refreshStatusIfChanged();
      return true;
    } else {
      refreshStatusIfChanged();
      return false;
    }
  }

  // Open-circuit detection
  if (ia < OPEN_THRESH_A) {
    relayOff(rly);
    buzzerAlarm();

    if (showFaultChoicePopup(FAULT_OPEN, rly)) {
      relayOn(rly);
      buzzerBeep();
      refreshStatusIfChanged();
      return true;
    } else {
      refreshStatusIfChanged();
      return false;
    }
  }

  // Normal engage
  buzzerBeep();
  refreshStatusIfChanged();
  return true;
}

// ------------------- Rotary -------------------
static int readRotaryPos(){
  if(!digitalRead(PIN_SW_POS1))return 1;
  if(!digitalRead(PIN_SW_POS2))return 2;
  if(!digitalRead(PIN_SW_POS3))return 3;
  if(!digitalRead(PIN_SW_POS4))return 4;
  if(!digitalRead(PIN_SW_POS5))return 5;
  if(!digitalRead(PIN_SW_POS6))return 6;
  if(!digitalRead(PIN_SW_POS7))return 7;
  if(!digitalRead(PIN_SW_POS8))return 8;
  return 0;
}
static RelayId relayFromRotary(int pos){
  switch(pos){
    case 3:return R_LEFT;   case 4:return R_RIGHT;
    case 5:return R_BRAKE;  case 6:return R_TAIL;
    case 7:return R_MARKER; case 8:return R_AUX;
    default:return R_NONE;
  }
}
static bool rfEnabled=false;
static void applyRotaryMode(int pos){
  switch(pos){
    case 1: rfEnabled=false; flashMode=false; relayOffAll(); break;
    case 2: rfEnabled=true;  break;
    case 3: case 4: case 5: case 6: case 7: case 8:{
      rfEnabled=false; flashMode=false; relayOffAll();
      RelayId tgt=relayFromRotary(pos);
      if (tgt!=R_NONE) pulseTestAndEngage(tgt);
      flashTarget=tgt;
    } break;
  }
  refreshStatusIfChanged();
}

// ------------------- CC1101: init -------------------
static void rfInit(){
  ELECHOUSE_cc1101.setSpiPin(PIN_FSPI_SCK, PIN_FSPI_MISO, PIN_FSPI_MOSI, PIN_CC1101_CS);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(433.92);
  pinMode(PIN_CC1101_GDO0, INPUT);
}

// ------------------- RF "hash" capture (learn + runtime) -------------------
static inline uint32_t fnv1a(uint32_t h, uint32_t x){ h ^= x; return h * 16777619UL; }

// Capture an RF burst from GDO0 and convert to a stable 32-bit "fingerprint".
static uint32_t captureRfHashBlocking(uint32_t arm_ms=5000) {
  uint32_t start = millis();
  int last = digitalRead(PIN_CC1101_GDO0);

  // Wait for first falling edge (button press)
  while (millis() - start < arm_ms) {
    int cur = digitalRead(PIN_CC1101_GDO0);
    if (last == HIGH && cur == LOW) break;
    last = cur;
    if (!digitalRead(PIN_ENC_KO)) return 0; // Back cancels
    delay(1);
  }
  if (millis() - start >= arm_ms) return 0;

  // Record edge durations until a gap
  static uint16_t dur[128];
  int idx = 0;
  uint32_t tEdge = micros();
  const uint32_t GAP_US = 8000;

  while (idx < (int)(sizeof(dur)/sizeof(dur[0]))) {
    int cur = digitalRead(PIN_CC1101_GDO0);
    if (cur != last) {
      uint32_t now = micros();
      uint32_t d = now - tEdge;
      tEdge = now;
      dur[idx++] = (uint16_t)min<uint32_t>(d, 65535);
      last = cur;
    }
    if (micros() - tEdge > GAP_US) break;
    if (!digitalRead(PIN_ENC_KO)) return 0; // cancel
  }
  if (idx < 8) return 0;

  // Normalize to buckets and hash
  uint32_t sum=0; for (int i=0;i<idx;i++) sum += dur[i];
  uint16_t avg = (uint16_t)(sum / idx);
  uint16_t thr = (avg > 400 ? avg : 400);

  uint32_t h = 2166136261UL;
  for (int i=0;i<idx;i++) {
    uint8_t bucket = (dur[i] > thr*2) ? 2 : (dur[i] > thr ? 1 : 0);
    h = fnv1a(h, (uint32_t)bucket + 0x9E);
  }
  h = fnv1a(h, (uint32_t)idx ^ 0xA5A5A5A5UL);
  return h ? h : 0xFFFFFFFF;
}

// ------------------- RF service (uses learned codes) -------------------
static void rfService(){
  if (!rfEnabled) return;

  static int last = HIGH;
  int cur = digitalRead(PIN_CC1101_GDO0);
  if (last == HIGH && cur == LOW) {
    // Capture this press quickly
    uint32_t code = captureRfHashBlocking(200);
    if (code) {
      // Map to learned relay
      RelayId target = R_NONE;
      for (int i=0;i<R_COUNT;i++){
        if (code == prefs.getULong(RF_PREF_KEYS[i], 0)) { target = (RelayId)i; break; }
      }

      if (target != R_NONE) {
        static uint32_t lastPressMs[R_COUNT] = {0};
        uint32_t now = millis();
        bool isDouble = (now - lastPressMs[target]) < DOUBLE_PRESS_MS;
        lastPressMs[target] = now;

        if (isDouble) {
          flashMode = !flashMode;
          flashTarget = target;
          if (!flashMode) relayOff(flashTarget);
        } else {
          if (relayState[target]) { relayOff(target); buzzerBeep(); }
          else if (pulseTestAndEngage(target)) { /* engaged */ }
          lastRfRelay = target;
          flashTarget = target;
        }
        refreshStatusIfChanged();
      } else {
        // Unknown button → short chirp
        buzzerBeep(30);
      }
    }
  }
  last = cur;
}

static void serviceFlashMode(){
  static uint32_t last=0; static bool on=false;
  if (!flashMode || flashTarget==R_NONE) return;
  int pos = readRotaryPos();
  if (pos>=3 && pos<=8) flashTarget = relayFromRotary(pos);
  if (millis()-last > 400) {
    last = millis(); on = !on;
    if (on) relayOn(flashTarget); else relayOff(flashTarget);
    refreshStatusIfChanged();
  }
}

// ------------------- Wi-Fi + OTA -------------------
WebServer server(80);

// Connect using saved creds (returns true on success)
static bool wifiConnectSaved(uint32_t timeout_ms=20000){
  String ssid = prefs.getString(KEY_WIFI_SSID, "");
  String pass = prefs.getString(KEY_WIFI_PASS, "");
  if (ssid.isEmpty()) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<timeout_ms) delay(200);
  return WiFi.status()==WL_CONNECTED;
}

// Simple scrollable list UI on TFT. Returns selected index or -1 on cancel (Back).
static int tftSelectFromList(const String* items, int count, const char* title) {
  if (count <= 0) return -1;
  int idx = 0;
  for (;;) {
    // draw
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0,0); tft.setTextColor(ST77XX_CYAN); tft.print(title);
    tft.setTextColor(ST77XX_WHITE);
    int first = max(0, min(idx-3, count-6));               // show up to 6 lines
    int last  = min(count, first+6);
    for (int i=first, row=0; i<last; ++i, ++row) {
      if (i==idx) tft.setTextColor(ST77XX_BLACK, ST77XX_YELLOW);
      else         tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor(0, 14 + row*12);
      tft.print(items[i]);
    }

    // Back hint
    tft.setCursor(0, 90);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Back = cancel");

    // input
    int8_t step = readEncoderStep();
    if (step) idx = (idx + step + count) % count;
    if (readButtonPressed()) return idx;
    if (readKoPressed()) return -1;
    delay(60);
  }
}

// Rotary password entry. Returns true if confirmed, false if cancelled.
// Writes into 'outPass' (buffer size outCap). Stars by default, toggle show/hide.
static bool tftEnterPassword(char* outPass, size_t outCap, const char* ssid) {
  static const char* ROW1 = "abcdefghijklm";
  static const char* ROW2 = "nopqrstuvwxyz";
  static const char* ROW3 = "ABCDEFGHJKLMN";
  static const char* ROW4 = "OPQRSTUVWXYZ";
  static const char* ROW5 = "0123456789";
  static const char* ROW6 = "!@#$%^&*()-_=+[]{};:',./?";
  const String keys[] = {"<Bksp>","<Space>","<Show>","<Done>"};

  String charset = ROW1; charset += ROW2; charset += ROW3; charset += ROW4;
  charset += ROW5; charset += ROW6;
  const int baseCount = charset.length();
  const int total = baseCount + 4;

  String preview;
  int cur = 0;
  bool show = false;
  size_t len = 0; outPass[0] = 0;

  for (;;) {
    // draw
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0,0); tft.setTextColor(ST77XX_CYAN);
    tft.print("Enter password:\n"); tft.print(ssid);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0,26);
    preview = "";
    for (size_t i=0;i<len;i++) preview += (show ? (char)outPass[i] : '*');
    tft.print(preview);

    tft.setCursor(0, 42);
    tft.setTextColor(ST77XX_BLACK, ST77XX_YELLOW);
    if (cur < baseCount) {
      tft.printf(" [%c] ", charset[cur]);
    } else {
      tft.printf(" %s ", keys[cur - baseCount].c_str());
    }

    tft.setCursor(0, 90);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Back = cancel");

    // input
    int8_t step = readEncoderStep();
    if (step) cur = (cur + step + total) % total;

    if (readButtonPressed()) {
      if (cur < baseCount) {                   // append char
        if (len + 1 < outCap) { outPass[len++] = charset[cur]; outPass[len] = 0; }
      } else {
        int k = cur - baseCount;
        if (k == 0) {                          // <Bksp>
          if (len) { len--; outPass[len] = 0; }
        } else if (k == 1) {                   // <Space>
          if (len + 1 < outCap) { outPass[len++] = ' '; outPass[len] = 0; }
        } else if (k == 2) {                   // <Show>
          show = !show;
        } else if (k == 3) {                   // <Done>
          return true;
        }
      }
    }
    if (readKoPressed()) return false;         // cancel
    delay(50);
  }
}

// Scan networks, select one, (optionally) enter password, save & connect.
static void wifiScanAndConnectUI(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,0); tft.setTextColor(ST77XX_WHITE);
  tft.print("Scanning Wi-Fi...");
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks(/*async=*/false,true);
  if (n <= 0) { tft.setCursor(0,14); tft.print("No networks found"); delay(1000); return; }

  // Build list: "SSID  (RSSI) [OPEN/SEC]" (limit to first 12)
  const int MAX_SHOW = min(n, 12);
  String items[12];
  for (int i=0;i<MAX_SHOW;i++){
    String ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    uint8_t enc  = WiFi.encryptionType(i);
    const char* sec = (enc==WIFI_AUTH_OPEN)?"OPEN":"SEC";
    items[i] = ssid + " (" + String(rssi) + "dBm) " + sec;
  }

  int sel = tftSelectFromList(items, MAX_SHOW, "Select network");
  if (sel < 0) return;

  String ssid = WiFi.SSID(sel);
  uint8_t enc  = WiFi.encryptionType(sel);

  String pass = "";
  if (enc != WIFI_AUTH_OPEN) {
    char buf[65]; buf[0]=0;
    if (!tftEnterPassword(buf, sizeof(buf), ssid.c_str())) return; // cancelled
    pass = String(buf);
  }

  // Try connect
  tft.fillScreen(ST77XX_BLACK); tft.setCursor(0,0);
  tft.printf("Connecting to\n%s\n", ssid.c_str());
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<20000) delay(200);

  if (WiFi.status()==WL_CONNECTED) {
    prefs.putString(KEY_WIFI_SSID, ssid);
    prefs.putString(KEY_WIFI_PASS, pass);
    tft.setCursor(0,30); tft.print("Connected!");
    tft.setCursor(0,42); tft.print(WiFi.localIP());
    buzzerBeep(90);
  } else {
    tft.setCursor(0,30); tft.print("Failed.");
    buzzerAlarm(300);
  }
  delay(1200);
}

// Forget creds
static void wifiForget(){
  prefs.remove(KEY_WIFI_SSID);
  prefs.remove(KEY_WIFI_PASS);
  WiFi.disconnect(true,true);
  tft.fillScreen(ST77XX_BLACK); tft.setCursor(0,0);
  tft.print("Wi-Fi creds cleared");
  delay(900);
}

static esp_err_t runGithubOta(){
  if (WiFi.status()!=WL_CONNECTED) return ESP_ERR_INVALID_STATE;
  WiFiClientSecure client; client.setInsecure();
  HTTPUpdate updater; updater.rebootOnUpdate(true);
  tft.fillScreen(ST77XX_BLACK); tft.setCursor(0,0); tft.print("OTA updating…");
  return (updater.update(client, OTA_LATEST_ASSET_URL)==HTTP_UPDATE_OK)?ESP_OK:ESP_FAIL;
}

// ------------------- Menu UI -------------------
static const char* menuItems[] = {
  "Wi-Fi Scan & Connect",
  "Wi-Fi Forget",
  "OTA Update",
  "All Relays OFF",
  "Learn Remote",
  "Set OCP Limit",
  "Brightness"
};
static int menuCount = sizeof(menuItems)/sizeof(menuItems[0]);
static int menuIndex=0;

static void drawMenu(){
  uiInMenu = true;
  tft.fillScreen(ST77XX_BLACK); tft.setCursor(0,0);
  tft.setTextColor(ST77XX_CYAN); tft.print("Menu");
  for (int i=0;i<menuCount;i++){
    if (i==menuIndex) tft.setTextColor(ST77XX_BLACK, ST77XX_CYAN);
    else              tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(0, i*12+14); tft.print(menuItems[i]);
  }
  tft.setCursor(0, 14 + menuCount*12 + 4);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Back = Exit");
}

static void exitMenuToStatus(){
  uiInMenu = false;
  drawStatusPage(true);
}

// ------------------- Encoder + Back button helpers -------------------
static bool btn_last=true;
static int8_t readEncoderStep(){
  static uint8_t last=0;
  uint8_t a=digitalRead(PIN_ENC_A), b=digitalRead(PIN_ENC_B), cur=(a<<1)|b;
  int8_t s=0;
  if((last==0&&cur==1)||(last==1&&cur==3)||(last==3&&cur==2)||(last==2&&cur==0))s=+1;
  if((last==0&&cur==2)||(last==2&&cur==3)||(last==3&&cur==1)||(last==1&&cur==0))s=-1;
  last=cur; return s;
}
static bool readButtonPressed(){ bool cur=!digitalRead(PIN_ENC_BTN); bool p=(cur && !btn_last); btn_last=cur; return p; }
static bool readKoPressed(){ return !digitalRead(PIN_ENC_KO); }  // Physical "Back"

// ------------------- Menu handlers ----
static void startRfLearn(){
  // 6-step wizard: LEFT, RIGHT, BRAKE, TAIL, MARKER, AUX
  for (int i=0;i<R_COUNT;i++){
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0,0);  tft.setTextColor(ST77XX_WHITE);
    tft.printf("Learn %s\n", RELAY_LABELS[i]);
    tft.setCursor(0,14); tft.print("Press remote button");
    tft.setCursor(0,26); tft.print("Back = cancel");

    uint32_t code = captureRfHashBlocking(8000);
    if (code == 0){
      tft.fillScreen(ST77XX_BLACK); tft.setCursor(0,0); tft.print("Learning cancelled");
      delay(1000); return;
    }

    prefs.putULong(RF_PREF_KEYS[i], code);
    tft.setCursor(0,40); tft.print("Saved: 0x"); tft.print(code, HEX);
    buzzerBeep(80); delay(800);
  }
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,0); tft.print("All 6 saved!");
  buzzerBeep(120); delay(1000);
}

static void adjustOcpLimit(){
  float cur = OCP_LIMIT_A;
  while (!readKoPressed()){
    int8_t s = readEncoderStep();
    if (s) cur += s;           // 1 A steps
    cur = max(5.0f, min(cur, 30.0f));
    INA226::setOcpLimit(cur);
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0,0); tft.printf("OCP: %.1f A\n", cur);
    tft.setCursor(0,16); tft.setTextColor(ST77XX_YELLOW); tft.print("Back = Exit");
    delay(140);
  }
  prefs.putFloat("ocp", cur);
}

static void adjustBrightness(){
  int val = prefs.getInt("bright", 200);
  ledcAttachPin(PIN_TFT_BL, 0); ledcSetup(0, 5000, 8);
  while (!readKoPressed()){
    int8_t s = readEncoderStep();
    if (s) val += s*10;
    val = max(0, min(val, 255));
    ledcWrite(0, val);
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0,0); tft.printf("Brightness: %d\n", val);
    tft.setCursor(0,16); tft.setTextColor(ST77XX_YELLOW); tft.print("Back = Exit");
    delay(100);
  }
  prefs.putInt("bright", val);
}

static void doMenuAction(int idx){
  switch(idx){
    case 0: wifiScanAndConnectUI(); break;
    case 1: wifiForget(); break;
    case 2: runGithubOta(); break;
    case 3: relayOffAll(); flashMode=false; break;
    case 4: startRfLearn(); break;
    case 5: adjustOcpLimit(); break;
    case 6: adjustBrightness(); break;
  }
}

// ------------------- Setup & Loop -------------------
static void initPins(){
  pinMode(PIN_ENC_A,INPUT_PULLUP); pinMode(PIN_ENC_B,INPUT_PULLUP);
  pinMode(PIN_ENC_BTN,INPUT_PULLUP); pinMode(PIN_ENC_KO,INPUT_PULLUP);
  pinMode(PIN_BUZZER,OUTPUT); digitalWrite(PIN_BUZZER,LOW);

  pinMode(PIN_SW_POS1,INPUT_PULLUP); pinMode(PIN_SW_POS2,INPUT_PULLUP);
  pinMode(PIN_SW_POS3,INPUT_PULLUP); pinMode(PIN_SW_POS4,INPUT_PULLUP);
  pinMode(PIN_SW_POS5,INPUT_PULLUP); pinMode(PIN_SW_POS6,INPUT_PULLUP);
  pinMode(PIN_SW_POS7,INPUT_PULLUP); pinMode(PIN_SW_POS8,INPUT_PULLUP);

  for(int i=0;i<R_COUNT;i++){ pinMode(RELAY_PIN[i],OUTPUT); digitalWrite(RELAY_PIN[i],LOW); }
}

void setup(){
  Serial.begin(115200);
  prefs.begin(NVS_NS, false);

  spiTFT.begin(PIN_FSPI_SCK, PIN_FSPI_MISO, PIN_FSPI_MOSI, -1);
  tft.initR(INITR_BLACKTAB); tft.setRotation(1);
  pinMode(PIN_TFT_BL,OUTPUT); digitalWrite(PIN_TFT_BL,HIGH);

  initPins();
  INA226::begin();
  rfInit();

  // Restore saved OCP / brightness on boot
  float ocp = prefs.getFloat("ocp", OCP_LIMIT_A);
  INA226::setOcpLimit(ocp);
  int bright = prefs.getInt("bright", 255);
  ledcAttachPin(PIN_TFT_BL, 0); ledcSetup(0, 5000, 8); ledcWrite(0, bright);

  // Auto-connect Wi-Fi if saved
  if (wifiConnectSaved(8000)) {
    tft.fillScreen(ST77XX_BLACK); tft.setCursor(0,0);
    tft.print("Wi-Fi connected\n");
    tft.print(WiFi.localIP());
    delay(700);
  }

  // Start on Run Status page
  drawStatusPage(true);
}

void loop(){
  int8_t step = readEncoderStep();

  if (uiInMenu) {
    if (step){ menuIndex=(menuIndex+step+menuCount)%menuCount; drawMenu(); delay(120); }
    if (readButtonPressed()){ doMenuAction(menuIndex); drawMenu(); }
    if (readKoPressed()){ exitMenuToStatus(); }  // Back exits menu
  } else {
    // On status page
    if (readButtonPressed()){ drawMenu(); }
    if (readKoPressed()){ drawStatusPage(true); } // Back refresh (or wire E-stop here)
  }

  static int lastPos=0; int pos=readRotaryPos();
  if (pos && pos!=lastPos){ applyRotaryMode(pos); lastPos=pos; }

  if (INA226::overCurrent()){
    flashMode = false;
    RelayId culprit = currentActiveRelay();  // best guess
    relayOffAll();
    buzzerAlarm();
    // Hard OCP trip = SHORT; no bypass here
    refreshStatusIfChanged();
  }

  rfService();
  serviceFlashMode();

  delay(5);
}
