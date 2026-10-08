// Battery-life tester for the LilyGo T-Display-S3 Pro.
//
// How to use: charge the battery fully, flash this, then UNPLUG USB. The timer
// only counts time spent on battery. Leave it running until it dies. Plug USB back
// in later and the screen shows how long that run lasted. The last 3 runs are kept.
//
// Buttons: BTN2 (GPIO12) = backlight level, BTN3 (GPIO16) = screen off/on.
// RESET starts a new run (the unfinished one is saved to the history first).

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <XPowersLib.h>
#include <Arduino_GFX_Library.h>

// ---- T-Display-S3 Pro pins (from LilyGo's utilities.h) ----
#define I2C_SDA 5
#define I2C_SCL 6
#define SPI_MISO 8
#define SPI_MOSI 17
#define SPI_SCK 18
#define TFT_CS 39
#define TFT_RST 47
#define TFT_DC 9
#define TFT_BL 48
#define BTN_LEVEL 12
#define BTN_SCREEN 16

#ifdef USING_DISPLAY_PRO_V1
#define MAX_LEVEL 255   // PWM backlight
#else
#define MAX_LEVEL 16    // constant-current backlight, 16 steps
#endif

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, SPI_SCK, SPI_MOSI, SPI_MISO);
// Portrait (rotation 0) exactly as in LilyGo's own example: 222 x 480 pixels.
Arduino_GFX *gfx = new Arduino_ST7796(bus, TFT_RST, 0, true /* IPS */, 222, 480, 49, 0);
PowersSY6970 PMU;
Preferences prefs;

struct Run {
  uint32_t secs;       // seconds spent on battery
  uint16_t startMv;    // battery voltage when the run started
  uint16_t endMv;      // last battery voltage seen
  uint8_t level;       // backlight level used (0 = screen off)
};

static Run cur = {0, 0, 0, 0};
static Run hist[3];
static bool pmuOk = false;

// ---------------- backlight ----------------
static uint8_t blLevel = 0;   // current hardware level (0 = off)

static void setBacklight(uint8_t value) {
#ifdef USING_DISPLAY_PRO_V1
  ledcWrite(1, value);
  blLevel = value;
#else
  const uint8_t steps = 16;
  if (value == 0) { digitalWrite(TFT_BL, 0); delay(3); blLevel = 0; return; }
  if (blLevel == 0) { digitalWrite(TFT_BL, 1); blLevel = steps; delayMicroseconds(30); }
  int from = steps - blLevel, to = steps - value;
  int num = (steps + to - from) % steps;
  for (int i = 0; i < num; i++) { digitalWrite(TFT_BL, 0); digitalWrite(TFT_BL, 1); }
  blLevel = value;
#endif
}

// ---------------- storage ----------------
static void loadHistory() {
  prefs.begin("battest", false);
  prefs.getBytes("h0", &hist[0], sizeof(Run));
  prefs.getBytes("h1", &hist[1], sizeof(Run));
  prefs.getBytes("h2", &hist[2], sizeof(Run));
  Run unfinished;
  memset(&unfinished, 0, sizeof(unfinished));
  prefs.getBytes("cur", &unfinished, sizeof(Run));
  if (unfinished.secs >= 60) {            // last run really ran: push it into the history
    hist[2] = hist[1]; hist[1] = hist[0]; hist[0] = unfinished;
    prefs.putBytes("h0", &hist[0], sizeof(Run));
    prefs.putBytes("h1", &hist[1], sizeof(Run));
    prefs.putBytes("h2", &hist[2], sizeof(Run));
  }
  Run none;
  memset(&none, 0, sizeof(none));
  prefs.putBytes("cur", &none, sizeof(Run));
}

static void saveCurrent() { prefs.putBytes("cur", &cur, sizeof(Run)); }

// ---------------- helpers ----------------
static void hms(char *out, size_t n, uint32_t s) {
  snprintf(out, n, "%02u:%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
}

// Rough single-cell LiPo state of charge from resting voltage (approximate; voltage is the real number).
static int approxPercent(int mv) {
  static const int v[] = {3300, 3500, 3600, 3700, 3750, 3800, 3850, 3900, 3950, 4000, 4100, 4200};
  static const int p[] = {0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
  if (mv <= v[0]) return 0;
  if (mv >= v[11]) return 100;
  for (int i = 1; i < 12; i++)
    if (mv < v[i]) return p[i - 1] + (p[i] - p[i - 1]) * (mv - v[i - 1]) / (v[i] - v[i - 1]);
  return 100;
}

// Draw one line of text at pixel row y, padded with spaces to the full width so old text is overwritten.
static void put(int y, int size, uint16_t color, const char *fmt, ...) {
  char buf[40];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  int cols = 222 / (6 * size);
  char padded[40];
  snprintf(padded, sizeof(padded), "%-*.*s", cols, cols, buf);
  gfx->setTextSize(size);
  gfx->setTextColor(color, BLACK);
  gfx->setCursor(0, y);
  gfx->print(padded);
}

// ---------------- setup / loop ----------------
static uint8_t level = 8;        // backlight step 1..16 (shown to the user)
static bool screenOn = true;

static void applyBacklight() {
#ifdef USING_DISPLAY_PRO_V1
  setBacklight(screenOn ? (uint8_t)(level * 16 - 1) : 0);
#else
  setBacklight(screenOn ? level : 0);
#endif
}

void setup() {
  loadHistory();

  pinMode(BTN_LEVEL, INPUT_PULLUP);
  pinMode(BTN_SCREEN, INPUT_PULLUP);

#ifdef USING_DISPLAY_PRO_V1
  ledcSetup(1, 5000, 8);
  ledcAttachPin(TFT_BL, 1);
#else
  pinMode(TFT_BL, OUTPUT);
#endif
  gfx->begin();
  gfx->fillScreen(BLACK);
  applyBacklight();

  Wire.begin(I2C_SDA, I2C_SCL);
  pmuOk = PMU.init(Wire, I2C_SDA, I2C_SCL, SY6970_SLAVE_ADDRESS);
  if (pmuOk) {
    PMU.setInputCurrentLimit(1000);
    PMU.setPrechargeCurr(64);
    PMU.setChargerConstantCurr(320);   // gentle charge (LilyGo's own example value)
    PMU.enableStatLed();
    PMU.enableADCMeasure();            // needed to read battery voltage
    PMU.enableCharge();
  }
}

void loop() {
  static uint32_t lastTick = 0, lastSave = 0, lastBtn = 0;
  static bool onBatteryEver = false;

  // Buttons (debounced)
  if (millis() - lastBtn > 250) {
    if (digitalRead(BTN_LEVEL) == LOW) {
      lastBtn = millis();
      level = (level >= 16) ? 1 : level + 1;
      screenOn = true;
      applyBacklight();
    } else if (digitalRead(BTN_SCREEN) == LOW) {
      lastBtn = millis();
      screenOn = !screenOn;
      applyBacklight();
    }
  }

  if (millis() - lastTick < 1000) { delay(20); return; }
  lastTick = millis();

  bool usb = pmuOk && PMU.isVbusIn();
  int mv = pmuOk ? (int)PMU.getBattVoltage() : 0;

  if (!usb && pmuOk) {
    if (!onBatteryEver) { onBatteryEver = true; cur.startMv = mv; }
    cur.secs++;
    cur.endMv = mv;
    cur.level = screenOn ? level : 0;
    if (millis() - lastSave >= 30000) { lastSave = millis(); saveCurrent(); }
  }

  if (!screenOn) return;   // nothing to draw (timing and saving continue)

  char t[16];
  put(0, 2, CYAN, "BATTERY TEST");
  if (!pmuOk) { put(30, 2, RED, "Power chip not found"); return; }
  if (usb) {
    put(30, 2, YELLOW, "USB plugged in");
    put(50, 2, WHITE, "Unplug USB to");
    put(70, 2, WHITE, "start timing");
    put(100, 2, WHITE, "Chg: %s", PMU.getChargeStatusString());
    put(120, 2, 0x7BEF, "(battery V is not");
    put(140, 2, 0x7BEF, " valid on USB)");
    put(170, 3, BLACK, " ");
    put(200, 3, BLACK, " ");
  } else {
    hms(t, sizeof(t), cur.secs);
    put(30, 2, GREEN, "ON BATTERY");
    put(50, 4, WHITE, "%s", t);
    put(90, 3, WHITE, "%d.%03dV ~%d%%", mv / 1000, mv % 1000, approxPercent(mv));
    put(120, 2, WHITE, "Start %d.%03d V", cur.startMv / 1000, cur.startMv % 1000);
    put(140, 2, WHITE, "Backlight %u/16", (unsigned)level);
    put(160, 2, 0x7BEF, "%-18s", " ");
    put(180, 2, 0x7BEF, "%-18s", " ");
  }
  put(210, 2, CYAN, "Previous runs:");
  for (int i = 0; i < 3; i++) {
    int y = 232 + i * 52;
    if (hist[i].secs < 60) { put(y, 2, 0x7BEF, "-%d: none", i + 1); put(y + 20, 2, BLACK, " "); continue; }
    hms(t, sizeof(t), hist[i].secs);
    put(y, 2, ORANGE, "-%d: %s", i + 1, t);
    put(y + 20, 2, 0x7BEF, "%d.%02d>%d.%02dV bl%u", hist[i].startMv / 1000, (hist[i].startMv % 1000) / 10, hist[i].endMv / 1000, (hist[i].endMv % 1000) / 10, hist[i].level);
  }
}
