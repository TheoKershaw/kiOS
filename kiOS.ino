#define USE_BLE 1

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <WiFi.h>
#include <SD.h>
#include <SPI.h>
#include <Preferences.h>
#include <vector>
#include <algorithm>
#include <time.h>
#if USE_BLE
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#endif

#define T_CLK  25
#define T_MOSI 32
#define T_MISO 39
#define T_CS   33

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;
public:
  LGFX() {
    {
      auto c = _bus.config();
      c.spi_host = SPI2_HOST;
      c.spi_mode = 0;
      c.freq_write = 40000000;
      c.freq_read = 16000000;
      c.spi_3wire = false;
      c.use_lock = true;
      c.dma_channel = 1;
      c.pin_sclk = 14;
      c.pin_mosi = 13;
      c.pin_miso = 12;
      c.pin_dc = 2;
      _bus.config(c);
      _panel.setBus(&_bus);
    }
    {
      auto c = _panel.config();
      c.pin_cs = 15;
      c.pin_rst = -1;
      c.pin_busy = -1;
      c.panel_width = 240;
      c.panel_height = 320;
      c.offset_x = 0;
      c.offset_y = 0;
      c.offset_rotation = 0;
      c.readable = true;
      c.invert = false;
      c.rgb_order = false;
      c.dlen_16bit = false;
      c.bus_shared = false;
      _panel.config(c);
    }
    {
      auto c = _light.config();
      c.pin_bl = 21;
      c.invert = false;
      c.freq = 44100;
      c.pwm_channel = 7;
      _light.config(c);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

LGFX tft;
Preferences prefs;
SPIClass sdSPI(VSPI);

const uint16_t BG    = TFT_BLACK;
const uint16_t FG    = 0x07E0;
const uint16_t DIM   = 0x0320;
const uint16_t BTN   = 0x0260;
const uint16_t WARN  = 0xFD20;
const uint16_t OPC   = 0x8200;

struct TouchCal {
  float ax, bx;
  float ay, by;
  bool swap;
};
TouchCal tc;

void setDefaultCal() {
  tc.swap = false;
  tc.ax = 320.0f / 3500.0f;  tc.bx = -200.0f * tc.ax;
  tc.ay = 240.0f / 3560.0f;  tc.by = -240.0f * tc.ay;
}

enum App { HOME, WIFI, BTSCAN, FILES, NOTES, CLOCK, CALC, SNAKE, PONG, TTT, PAINT, INFO, SETTINGS, KEYB };
App app = HOME;
int homePage = 0;
uint8_t brightness = 200;
bool sdOK = false;
bool wasDown = false;
uint32_t lastTick = 0;

uint16_t xptRead(uint8_t cmd) {
  digitalWrite(T_CS, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(T_MOSI, (cmd >> i) & 1);
    digitalWrite(T_CLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(T_CLK, LOW);
    delayMicroseconds(1);
  }
  uint16_t v = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(T_CLK, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | digitalRead(T_MISO);
    digitalWrite(T_CLK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(T_CS, HIGH);
  return (v >> 3) & 0x0FFF;
}

int median5(int *a) {
  for (int i = 1; i < 5; i++) {
    int k = a[i], j = i - 1;
    while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; j--; }
    a[j + 1] = k;
  }
  return a[2];
}

bool readRaw(int &rx, int &ry) {
  int z1 = xptRead(0xB1);
  int z2 = xptRead(0xC1);
  int z = z1 + 4095 - z2;
  if (z < 400) {
    xptRead(0x90);
    return false;
  }
  int xs[5], ys[5];
  for (int i = 0; i < 5; i++) {
    xs[i] = xptRead(0xD1);
    ys[i] = xptRead(0x91);
  }
  xptRead(0x90);
  rx = median5(xs);
  ry = median5(ys);
  if (rx < 50 || ry < 50 || rx > 4090 || ry > 4090) return false;
  return true;
}

bool readTouch(int32_t &x, int32_t &y) {
  static int miss = 0, cnt = 0;
  static bool down = false;
  static int32_t lx = 0, ly = 0;
  int rx, ry;
  if (readRaw(rx, ry)) {
    miss = 0;
    if (cnt < 2) cnt++;
    if (cnt >= 2) {
      down = true;
      float a = tc.swap ? ry : rx;
      float b = tc.swap ? rx : ry;
      lx = constrain((int)(tc.ax * a + tc.bx), 0, 319);
      ly = constrain((int)(tc.ay * b + tc.by), 0, 239);
    }
  } else if (down && miss < 3) {
    miss++;
  } else {
    down = false;
    cnt = 0;
    miss = 0;
  }
  x = lx;
  y = ly;
  return down;
}

bool hit(int x, int y, int bx, int by, int bw, int bh) {
  return x >= bx && x < bx + bw && y >= by && y < by + bh;
}

void button(int x, int y, int w, int h, const char* label, uint16_t col = BTN) {
  tft.fillRoundRect(x, y, w, h, 6, col);
  tft.drawRoundRect(x, y, w, h, 6, FG);
  tft.setTextColor(TFT_WHITE, col);
  tft.setTextDatum(lgfx::textdatum_t::middle_center);
  tft.drawString(label, x + w / 2, y + h / 2);
  tft.setTextDatum(lgfx::textdatum_t::top_left);
}

void centerText(const char* s, int x, int y, int size, uint16_t fg, uint16_t bg) {
  tft.setTextSize(size);
  tft.setTextColor(fg, bg);
  tft.setTextDatum(lgfx::textdatum_t::middle_center);
  tft.drawString(s, x, y);
  tft.setTextDatum(lgfx::textdatum_t::top_left);
}

void header(const char* title) {
  tft.fillScreen(BG);
  tft.fillRect(0, 0, 320, 26, DIM);
  tft.setTextSize(1);
  button(2, 2, 60, 22, "< Home");
  tft.setTextSize(2);
  tft.setTextColor(FG, DIM);
  tft.setCursor(72, 5);
  tft.print(title);
  tft.setTextSize(1);
}

bool captureRaw(float &outX, float &outY) {
  int rx, ry;
  while (readRaw(rx, ry)) delay(10);
  while (!readRaw(rx, ry)) {
    if (digitalRead(0) == LOW) return false;
    delay(10);
  }
  delay(150);
  long sx = 0, sy = 0;
  int n = 0, miss = 0;
  while (n < 30 && miss < 3) {
    if (readRaw(rx, ry)) { sx += rx; sy += ry; n++; miss = 0; }
    else miss++;
    delay(8);
  }
  while (readRaw(rx, ry)) delay(10);
  if (n == 0) return captureRaw(outX, outY);
  outX = sx / (float)n;
  outY = sy / (float)n;
  return true;
}

void drawTarget(int x, int y, int step) {
  tft.fillScreen(BG);
  tft.setTextSize(2);
  tft.setTextColor(FG, BG);
  tft.setCursor(60, 100);
  tft.print("Press the cross");
  tft.setCursor(100, 124);
  tft.printf("%d / 4", step);
  tft.setTextSize(1);
  tft.setTextColor(WARN, BG);
  tft.setCursor(70, 225);
  tft.print("Press firmly. BOOT = skip");
  tft.drawCircle(x, y, 12, TFT_WHITE);
  tft.drawFastHLine(x - 16, y, 33, TFT_WHITE);
  tft.drawFastVLine(x, y - 16, 33, TFT_WHITE);
  tft.fillCircle(x, y, 2, TFT_RED);
}

void calibrate() {
  const int px[4] = {30, 290, 30, 290};
  const int py[4] = {30, 30, 210, 210};
  float rx[4], ry[4];

  for (int i = 0; i < 4; i++) {
    drawTarget(px[i], py[i], i + 1);
    if (!captureRaw(rx[i], ry[i])) {
      while (digitalRead(0) == LOW) delay(10);
      setDefaultCal();
      prefs.putBytes("tc3", &tc, sizeof(tc));
      return;
    }
  }

  float dRawX = fabsf(rx[1] - rx[0]) + fabsf(rx[3] - rx[2]);
  float dRawY = fabsf(ry[1] - ry[0]) + fabsf(ry[3] - ry[2]);
  bool sw = dRawX < dRawY;

  float A[4], B[4];
  for (int i = 0; i < 4; i++) {
    A[i] = sw ? ry[i] : rx[i];
    B[i] = sw ? rx[i] : ry[i];
  }
  float spanA = ((A[1] - A[0]) + (A[3] - A[2])) / 2.0f;
  float spanB = ((B[2] - B[0]) + (B[3] - B[1])) / 2.0f;
  float meanA = (A[0] + A[1] + A[2] + A[3]) / 4.0f;
  float meanB = (B[0] + B[1] + B[2] + B[3]) / 4.0f;

  if (fabsf(spanA) < 200 || fabsf(spanB) < 200) {
    setDefaultCal();
  } else {
    tc.swap = sw;
    tc.ax = 260.0f / spanA;
    tc.bx = 160.0f - tc.ax * meanA;
    tc.ay = 180.0f / spanB;
    tc.by = 120.0f - tc.ay * meanB;
  }
  prefs.putBytes("tc3", &tc, sizeof(tc));
}

String kbText, kbPrompt;
int kbLayer = 0;
int kbMax = 64;
void (*kbCb)(bool, String) = nullptr;

const char* kbRows[3][4] = {
  {"1234567890", "qwertyuiop", "asdfghjkl", "^zxcvbnm<"},
  {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "^ZXCVBNM<"},
  {"1234567890", "!@#$%^&*()", "-_=+;:'\"/", "^.,?!~|\\<"}
};

void kbDrawField() {
  tft.fillRect(0, 26, 320, 28, DIM);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, DIM);
  String shown = kbText;
  if (shown.length() > 24) shown = shown.substring(shown.length() - 24);
  tft.setCursor(4, 32);
  tft.print(shown);
  tft.print("_");
}

void kbDraw() {
  tft.fillScreen(BG);
  tft.fillRect(0, 0, 320, 26, DIM);
  tft.setTextSize(1);
  tft.setTextColor(FG, DIM);
  tft.setCursor(6, 9);
  tft.print(kbPrompt);
  kbDrawField();
  for (int r = 0; r < 4; r++) {
    const char* row = kbRows[kbLayer][r];
    int len = strlen(row);
    int x0 = (320 - len * 32) / 2;
    int y = 58 + r * 34;
    for (int k = 0; k < len; k++) {
      char ch = row[k];
      String label;
      uint16_t col = BTN;
      if (ch == '^') {
        label = (kbLayer == 2) ? " " : "Sh";
        if (kbLayer == 1) col = OPC;
      } else if (ch == '<') {
        label = "Del";
        col = OPC;
      } else {
        label = String(ch);
      }
      tft.fillRoundRect(x0 + k * 32 + 1, y, 30, 32, 4, col);
      tft.drawRoundRect(x0 + k * 32 + 1, y, 30, 32, 4, FG);
      tft.setTextColor(TFT_WHITE, col);
      tft.setTextSize(label.length() > 1 ? 1 : 2);
      tft.setTextDatum(lgfx::textdatum_t::middle_center);
      tft.drawString(label, x0 + k * 32 + 16, y + 16);
      tft.setTextDatum(lgfx::textdatum_t::top_left);
    }
  }
  tft.setTextSize(1);
  button(0, 198, 64, 38, kbLayer == 2 ? "abc" : "123/#");
  button(66, 198, 120, 38, "space");
  button(188, 198, 62, 38, "Cancel", OPC);
  button(252, 198, 68, 38, "OK", 0x0400);
}

void startKeyboard(String prompt, String initial, int maxLen, void (*cb)(bool, String)) {
  kbPrompt = prompt;
  kbText = initial;
  kbMax = maxLen;
  kbCb = cb;
  kbLayer = 0;
  app = KEYB;
  kbDraw();
}

void kbFinish(bool ok) {
  void (*cb)(bool, String) = kbCb;
  kbCb = nullptr;
  String t = kbText;
  if (cb) cb(ok, t);
}

void kbTap(int x, int y) {
  for (int r = 0; r < 4; r++) {
    int ry = 58 + r * 34;
    if (y >= ry && y < ry + 32) {
      const char* row = kbRows[kbLayer][r];
      int len = strlen(row);
      int x0 = (320 - len * 32) / 2;
      if (x < x0 || x >= x0 + len * 32) return;
      char ch = row[(x - x0) / 32];
      if (ch == '^') {
        if (kbLayer < 2) { kbLayer = 1 - kbLayer; kbDraw(); }
      } else if (ch == '<') {
        if (kbText.length()) kbText.remove(kbText.length() - 1);
        kbDrawField();
      } else {
        if ((int)kbText.length() < kbMax) kbText += ch;
        kbDrawField();
        if (kbLayer == 1) { kbLayer = 0; kbDraw(); }
      }
      return;
    }
  }
  if (y >= 198 && y < 236) {
    if (x < 64) { kbLayer = (kbLayer == 2) ? 0 : 2; kbDraw(); }
    else if (x < 186) { if ((int)kbText.length() < kbMax) kbText += ' '; kbDrawField(); }
    else if (x < 250) kbFinish(false);
    else kbFinish(true);
  }
}

struct Net { String ssid; int rssi; bool open; int ch; };
Net nets[8];
int netCount = 0;
String pendingSsid;

void drawWifiList() {
  app = WIFI;
  header("WiFi");
  tft.setTextSize(1);
  tft.setTextColor(FG, BG);
  tft.setCursor(4, 32);
  tft.printf("%d networks - tap one to connect", netCount);
  for (int i = 0; i < netCount; i++) {
    int y = 46 + i * 20;
    String ssid = nets[i].ssid;
    if (ssid.length() == 0) ssid = "<hidden>";
    if (ssid.length() > 18) ssid = ssid.substring(0, 18);
    int rssi = nets[i].rssi;
    uint16_t c = rssi > -60 ? TFT_GREEN : (rssi > -75 ? TFT_YELLOW : TFT_RED);
    tft.setTextSize(2);
    tft.setTextColor(c, BG);
    tft.setCursor(4, y);
    tft.print(ssid);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, BG);
    tft.setCursor(250, y);
    tft.printf("%ddB", rssi);
    tft.setCursor(250, y + 9);
    tft.printf("ch%d %s", nets[i].ch, nets[i].open ? "open" : "lock");
  }
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, BG);
  tft.setCursor(4, 206);
  if (WiFi.status() == WL_CONNECTED) {
    String s = WiFi.SSID();
    if (s.length() > 16) s = s.substring(0, 16);
    tft.print(s);
    tft.setCursor(4, 218);
    tft.print(WiFi.localIP().toString());
  } else {
    tft.print("Not connected");
  }
  button(130, 210, 92, 26, "Disconnect");
  button(228, 210, 88, 26, "Rescan");
}

void wifiScan() {
  netCount = 0;
  int n = WiFi.scanNetworks();
  if (n < 0) n = 0;
  netCount = n > 8 ? 8 : n;
  for (int i = 0; i < netCount; i++) {
    nets[i].ssid = WiFi.SSID(i);
    nets[i].rssi = WiFi.RSSI(i);
    nets[i].open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    nets[i].ch = WiFi.channel(i);
  }
  WiFi.scanDelete();
}

void drawWifi() {
  app = WIFI;
  header("WiFi");
  centerText("Scanning...", 160, 120, 2, WARN, BG);
  wifiScan();
  drawWifiList();
}

void connectWifi(String ssid, String pw) {
  tft.fillScreen(BG);
  centerText("Connecting...", 160, 100, 2, WARN, BG);
  centerText(ssid.c_str(), 160, 130, 1, FG, BG);
  WiFi.disconnect();
  WiFi.begin(ssid.c_str(), pw.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    configTzTime("GMT0BST,M3.5.0/1,M10.5.0", "pool.ntp.org", "time.nist.gov");
    centerText("Connected!", 160, 160, 2, TFT_GREEN, BG);
  } else {
    WiFi.disconnect();
    centerText("Failed", 160, 160, 2, TFT_RED, BG);
  }
  delay(1000);
  drawWifiList();
}

void wifiPwDone(bool ok, String text) {
  if (ok) connectWifi(pendingSsid, text);
  else drawWifiList();
}

void wifiTap(int x, int y) {
  if (hit(x, y, 228, 210, 88, 26)) { drawWifi(); return; }
  if (hit(x, y, 130, 210, 92, 26)) { WiFi.disconnect(); drawWifiList(); return; }
  if (y >= 46 && y < 46 + 8 * 20) {
    int i = (y - 46) / 20;
    if (i < netCount) {
      pendingSsid = nets[i].ssid;
      if (nets[i].open) connectWifi(pendingSsid, "");
      else startKeyboard("Password for " + pendingSsid, "", 64, wifiPwDone);
    }
  }
}

#if USE_BLE
struct BleDev { String name; String addr; int rssi; };
std::vector<BleDev> bleList;
bool bleReady = false;

class BleCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) {
    String a = String(d.getAddress().toString().c_str());
    for (auto &e : bleList) {
      if (e.addr == a) { e.rssi = d.getRSSI(); return; }
    }
    BleDev b;
    b.name = d.haveName() ? String(d.getName().c_str()) : String("(no name)");
    b.addr = a;
    b.rssi = d.getRSSI();
    bleList.push_back(b);
  }
};

void drawBleList() {
  app = BTSCAN;
  header("Bluetooth");
  tft.setTextSize(1);
  tft.setTextColor(FG, BG);
  tft.setCursor(4, 31);
  tft.printf("%d devices found", (int)bleList.size());
  int rows = bleList.size() > 8 ? 8 : bleList.size();
  for (int i = 0; i < rows; i++) {
    int y = 42 + i * 21;
    String n = bleList[i].name;
    if (n.length() > 12) n = n.substring(0, 12);
    int rssi = bleList[i].rssi;
    uint16_t c = rssi > -60 ? TFT_GREEN : (rssi > -80 ? TFT_YELLOW : TFT_RED);
    tft.setTextSize(2);
    tft.setTextColor(c, BG);
    tft.setCursor(4, y);
    tft.print(n);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, BG);
    tft.setCursor(180, y + 4);
    tft.print(bleList[i].addr);
    tft.setCursor(180, y + 13);
    tft.printf("%d dBm", rssi);
  }
  button(230, 210, 86, 26, "Rescan");
}

void drawBle() {
  app = BTSCAN;
  header("Bluetooth");
  centerText("Scanning 4s...", 160, 120, 2, WARN, BG);
  if (!bleReady) {
    BLEDevice::init("kiOS");
    BLEScan* s = BLEDevice::getScan();
    s->setAdvertisedDeviceCallbacks(new BleCB());
    s->setActiveScan(true);
    s->setInterval(100);
    s->setWindow(99);
    bleReady = true;
  }
  bleList.clear();
  BLEScan* scan = BLEDevice::getScan();
  scan->start(4, false);
  scan->clearResults();
  std::sort(bleList.begin(), bleList.end(), [](const BleDev &a, const BleDev &b) { return a.rssi > b.rssi; });
  drawBleList();
}
#else
void drawBle() {
  app = BTSCAN;
  header("Bluetooth");
  centerText("BLE disabled", 160, 110, 2, WARN, BG);
  centerText("Set USE_BLE to 1", 160, 140, 2, FG, BG);
}
#endif

String path = "/";
String viewFile = "";
int fileScroll = 0;
std::vector<String> names;
std::vector<bool> isDir;

void loadDir() {
  names.clear();
  isDir.clear();
  fileScroll = 0;
  if (!sdOK) return;
  File d = SD.open(path);
  if (!d || !d.isDirectory()) return;
  File f;
  while ((f = d.openNextFile())) {
    names.push_back(String(f.name()));
    isDir.push_back(f.isDirectory());
    f.close();
  }
  d.close();
}

void drawFiles() {
  app = FILES;
  header("Files");
  tft.setTextSize(1);
  tft.setTextColor(FG, BG);
  tft.setCursor(4, 31);
  if (!sdOK) {
    centerText("No SD card found", 160, 120, 2, WARN, BG);
    return;
  }
  if (viewFile.length()) {
    tft.print(viewFile);
    File f = SD.open(viewFile);
    tft.setCursor(4, 46);
    tft.setTextColor(TFT_WHITE, BG);
    int count = 0;
    while (f && f.available() && count < 900) {
      char ch = f.read();
      if (ch == '\r') continue;
      if ((ch >= 32 && ch < 127) || ch == '\n') tft.print(ch);
      else tft.print('.');
      count++;
    }
    if (f) f.close();
    return;
  }
  tft.print(path);
  for (int i = 0; i < 7; i++) {
    int idx = fileScroll + i;
    if (idx >= (int)names.size()) break;
    int y = 44 + i * 23;
    tft.setTextSize(2);
    tft.setTextColor(isDir[idx] ? TFT_CYAN : TFT_WHITE, BG);
    tft.setCursor(4, y);
    String n = names[idx];
    if (n.length() > 24) n = n.substring(0, 24);
    tft.print(isDir[idx] ? "/" + n : n);
  }
  tft.setTextSize(1);
  button(4, 208, 60, 28, "Up");
  button(70, 208, 60, 28, "Down");
  button(136, 208, 60, 28, "..");
}

void filesTap(int x, int y) {
  if (!sdOK) return;
  if (viewFile.length()) {
    viewFile = "";
    drawFiles();
    return;
  }
  if (hit(x, y, 4, 208, 60, 28)) {
    if (fileScroll > 0) fileScroll--;
    drawFiles();
  } else if (hit(x, y, 70, 208, 60, 28)) {
    if (fileScroll + 7 < (int)names.size()) fileScroll++;
    drawFiles();
  } else if (hit(x, y, 136, 208, 60, 28)) {
    if (path != "/") {
      int s = path.lastIndexOf('/');
      path = s <= 0 ? "/" : path.substring(0, s);
      loadDir();
      drawFiles();
    }
  } else if (y >= 44 && y < 205) {
    int idx = fileScroll + (y - 44) / 23;
    if (idx < (int)names.size()) {
      String full = (path == "/" ? "" : path) + "/" + names[idx];
      if (isDir[idx]) {
        path = full;
        loadDir();
      } else {
        viewFile = full;
      }
      drawFiles();
    }
  }
}

int noteEditing = 0;

String noteGet(int i) { return prefs.getString((String("n") + i).c_str(), ""); }
void notePut(int i, String s) { prefs.putString((String("n") + i).c_str(), s); }

void drawNotes() {
  app = NOTES;
  header("Notes");
  for (int i = 0; i < 6; i++) {
    int y = 32 + i * 34;
    tft.fillRoundRect(4, y, 312, 30, 5, BTN);
    tft.drawRoundRect(4, y, 312, 30, 5, FG);
    String t = noteGet(i);
    tft.setTextSize(2);
    tft.setTextColor(FG, BTN);
    tft.setCursor(10, y + 7);
    tft.print(i + 1);
    if (t.length() == 0) {
      tft.setTextColor(0x8410, BTN);
      tft.setCursor(34, y + 7);
      tft.print("(empty)");
    } else {
      if (t.length() > 22) t = t.substring(0, 22);
      tft.setTextColor(TFT_WHITE, BTN);
      tft.setCursor(34, y + 7);
      tft.print(t);
    }
  }
  tft.setTextSize(1);
}

void noteDone(bool ok, String text) {
  if (ok) notePut(noteEditing, text);
  drawNotes();
}

void notesTap(int x, int y) {
  if (y >= 32 && y < 32 + 6 * 34) {
    int i = (y - 32) / 34;
    if (i < 6) {
      noteEditing = i;
      startKeyboard("Note " + String(i + 1), noteGet(i), 120, noteDone);
    }
  }
}

uint32_t swStart = 0, swElapsed = 0;
bool swRun = false;

void drawClockButtons() {
  tft.setTextSize(2);
  button(8, 176, 150, 44, swRun ? "Stop" : "Start", swRun ? OPC : 0x0400);
  button(168, 176, 144, 44, "Reset");
  tft.setTextSize(1);
}

void drawClock() {
  app = CLOCK;
  header("Clock");
  tft.setTextSize(1);
  tft.setTextColor(FG, BG);
  tft.setCursor(124, 112);
  tft.print("STOPWATCH");
  drawClockButtons();
  lastTick = 0;
}

void clockTick() {
  if (millis() - lastTick < 150) return;
  lastTick = millis();
  struct tm ti;
  tft.setTextColor(FG, BG);
  tft.setTextSize(4);
  tft.setCursor(64, 36);
  if (getLocalTime(&ti, 0)) {
    tft.printf("%02d:%02d:%02d", ti.tm_hour, ti.tm_min, ti.tm_sec);
    tft.setTextSize(2);
    tft.setCursor(88, 78);
    tft.printf("%02d/%02d/%04d  ", ti.tm_mday, ti.tm_mon + 1, ti.tm_year + 1900);
  } else {
    uint32_t s = millis() / 1000;
    tft.printf("%02d:%02d:%02d", (int)((s / 3600) % 100), (int)((s / 60) % 60), (int)(s % 60));
    tft.setTextSize(2);
    tft.setCursor(88, 78);
    tft.print("(uptime)    ");
  }
  uint32_t e = swElapsed + (swRun ? (millis() - swStart) : 0);
  tft.setTextSize(3);
  tft.setTextColor(TFT_CYAN, BG);
  tft.setCursor(97, 128);
  tft.printf("%02d:%02d.%d", (int)((e / 60000) % 100), (int)((e / 1000) % 60), (int)((e / 100) % 10));
  tft.setTextSize(1);
}

void clockTap(int x, int y) {
  if (hit(x, y, 8, 176, 150, 44)) {
    if (swRun) { swElapsed += millis() - swStart; swRun = false; }
    else { swStart = millis(); swRun = true; }
    drawClockButtons();
  } else if (hit(x, y, 168, 176, 144, 44)) {
    swElapsed = 0;
    swRun = false;
    drawClockButtons();
  }
}

String cStr = "";
double cAcc = 0;
char cOp = 0;
bool cResult = false;
const char calcKeys[17] = "789/456*123-0.=+";

String fmtNum(double v) {
  if (isnan(v) || isinf(v)) return "Err";
  String s = String(v, 6);
  if (s.indexOf('.') >= 0) {
    while (s.endsWith("0")) s.remove(s.length() - 1);
    if (s.endsWith(".")) s.remove(s.length() - 1);
  }
  if (s.length() > 13) s = String(v, 2);
  return s;
}

double calcApply(double a, double b, char op) {
  switch (op) {
    case '+': return a + b;
    case '-': return a - b;
    case '*': return a * b;
    case '/': return b == 0 ? NAN : a / b;
  }
  return b;
}

void calcDisplay() {
  tft.fillRect(8, 30, 246, 40, DIM);
  tft.drawRect(8, 30, 246, 40, FG);
  String s = cStr.length() ? cStr : fmtNum(cAcc);
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, DIM);
  tft.setTextDatum(lgfx::textdatum_t::middle_right);
  tft.drawString(s, 248, 51);
  if (cOp) {
    tft.setTextSize(2);
    tft.setTextColor(WARN, DIM);
    tft.setTextDatum(lgfx::textdatum_t::middle_left);
    tft.drawString(String(cOp), 14, 51);
  }
  tft.setTextDatum(lgfx::textdatum_t::top_left);
  tft.setTextSize(1);
}

void drawCalc() {
  app = CALC;
  header("Calc");
  cStr = "";
  cAcc = 0;
  cOp = 0;
  cResult = false;
  calcDisplay();
  tft.setTextSize(2);
  button(258, 30, 54, 40, "C", OPC);
  for (int i = 0; i < 16; i++) {
    int r = i / 4, c = i % 4;
    char k = calcKeys[i];
    char lab[2] = {k, 0};
    button(8 + c * 78, 78 + r * 40, 72, 36, lab, strchr("/*-+=", k) ? OPC : BTN);
  }
  tft.setTextSize(1);
}

void calcPress(char k) {
  if ((k >= '0' && k <= '9') || k == '.') {
    if (cResult) { cStr = ""; cResult = false; }
    if (k == '.' && cStr.indexOf('.') >= 0) return;
    if (cStr.length() < 12) cStr += k;
  } else if (k == 'C') {
    cStr = ""; cAcc = 0; cOp = 0; cResult = false;
  } else {
    if (cStr.length()) {
      double v = cStr.toDouble();
      cAcc = cOp ? calcApply(cAcc, v, cOp) : v;
    }
    if (k == '=') {
      cOp = 0;
      cStr = fmtNum(cAcc);
      cResult = true;
    } else {
      cOp = k;
      cStr = "";
      cResult = false;
    }
  }
  calcDisplay();
}

void calcTap(int x, int y) {
  if (hit(x, y, 258, 30, 54, 40)) { calcPress('C'); return; }
  for (int i = 0; i < 16; i++) {
    int r = i / 4, c = i % 4;
    if (hit(x, y, 8 + c * 78, 78 + r * 40, 72, 36)) { calcPress(calcKeys[i]); return; }
  }
}

const int SC = 12, SW = 26, SH = 17, SX = 4, SY = 30;
int snx[SW * SH], sny[SW * SH];
int snLen, sdx, sdy, nsdx, nsdy, fx, fy, snScore;
bool snDead = false;
uint32_t snLast = 0;

void snakeCell(int x, int y, uint16_t col) {
  tft.fillRect(SX + x * SC, SY + y * SC, SC - 1, SC - 1, col);
}

void snakeScore() {
  tft.fillRect(240, 0, 80, 26, DIM);
  tft.setTextSize(2);
  tft.setTextColor(FG, DIM);
  tft.setCursor(250, 5);
  tft.print(snScore);
  tft.setTextSize(1);
}

void snakeFood() {
  bool bad;
  do {
    fx = random(SW);
    fy = random(SH);
    bad = false;
    for (int i = 0; i < snLen; i++) if (snx[i] == fx && sny[i] == fy) { bad = true; break; }
  } while (bad);
  snakeCell(fx, fy, TFT_RED);
}

void snakeStart() {
  header("Snake");
  tft.drawRect(SX - 2, SY - 2, SW * SC + 3, SH * SC + 3, FG);
  snLen = 3;
  for (int i = 0; i < 3; i++) { snx[i] = SW / 2 - i; sny[i] = SH / 2; }
  sdx = 1; sdy = 0; nsdx = 1; nsdy = 0;
  snDead = false;
  snScore = 0;
  snLast = millis();
  for (int i = 0; i < snLen; i++) snakeCell(snx[i], sny[i], FG);
  snakeFood();
  snakeScore();
}

void snakeDie() {
  snDead = true;
  tft.fillRect(60, 90, 200, 70, BG);
  tft.drawRect(60, 90, 200, 70, TFT_RED);
  centerText("GAME OVER", 160, 112, 2, TFT_RED, BG);
  centerText("Tap to retry", 160, 140, 2, TFT_WHITE, BG);
}

void snakeTouch(int x, int y) {
  if (snDead) return;
  int hx = SX + snx[0] * SC + SC / 2, hy = SY + sny[0] * SC + SC / 2;
  int dx = x - hx, dy = y - hy;
  if (abs(dx) < 6 && abs(dy) < 6) return;
  int wx = 0, wy = 0;
  if (abs(dx) > abs(dy)) wx = dx > 0 ? 1 : -1;
  else wy = dy > 0 ? 1 : -1;
  if (wx == -sdx && wy == -sdy) return;
  nsdx = wx; nsdy = wy;
}

void snakeTick() {
  if (snDead) return;
  uint32_t speed = 140 - min(snScore * 3, 70);
  if (millis() - snLast < speed) return;
  snLast = millis();
  sdx = nsdx; sdy = nsdy;
  int nx = snx[0] + sdx, ny = sny[0] + sdy;
  if (nx < 0 || ny < 0 || nx >= SW || ny >= SH) { snakeDie(); return; }
  bool eat = (nx == fx && ny == fy);
  int checkLen = eat ? snLen : snLen - 1;
  for (int i = 0; i < checkLen; i++) {
    if (snx[i] == nx && sny[i] == ny) { snakeDie(); return; }
  }
  if (!eat) snakeCell(snx[snLen - 1], sny[snLen - 1], BG);
  else snLen++;
  for (int i = snLen - 1; i > 0; i--) { snx[i] = snx[i - 1]; sny[i] = sny[i - 1]; }
  snx[0] = nx; sny[0] = ny;
  snakeCell(nx, ny, FG);
  if (eat) {
    snScore++;
    snakeScore();
    snakeFood();
  }
}

const int PY0 = 28, PY1 = 238, PH = 40;
float bx, by, bvx, bvy;
int py, ay_, ps, as_;
bool pongOver = false;
uint32_t pongLast = 0, pongPause = 0;

void pongScore() {
  tft.fillRect(240, 0, 80, 26, DIM);
  tft.setTextSize(2);
  tft.setTextColor(FG, DIM);
  tft.setCursor(250, 5);
  tft.printf("%d:%d", ps, as_);
  tft.setTextSize(1);
}

void pongServe(int dir) {
  bx = 157; by = 130;
  bvx = 3.0f * dir;
  bvy = random(-20, 21) / 10.0f;
  if (fabsf(bvy) < 0.8f) bvy = 1.0f;
}

void pongStart() {
  header("Pong");
  py = 130 - PH / 2;
  ay_ = py;
  ps = 0; as_ = 0;
  pongOver = false;
  pongScore();
  pongServe(random(2) ? 1 : -1);
  tft.fillRect(6, py, 6, PH, FG);
  tft.fillRect(308, ay_, 6, PH, FG);
  pongPause = millis() + 700;
}

void pongTouch(int y) {
  if (pongOver) return;
  int np = constrain(y - PH / 2, PY0, PY1 - PH);
  if (np != py) {
    tft.fillRect(6, py, 6, PH, BG);
    py = np;
    tft.fillRect(6, py, 6, PH, FG);
  }
}

void pongEnd(const char* msg, uint16_t col) {
  pongOver = true;
  tft.fillRect(60, 90, 200, 70, BG);
  tft.drawRect(60, 90, 200, 70, col);
  centerText(msg, 160, 112, 2, col, BG);
  centerText("Tap to restart", 160, 140, 2, TFT_WHITE, BG);
}

void pongTick() {
  if (pongOver) return;
  if (millis() < pongPause) return;
  if (millis() - pongLast < 16) return;
  pongLast = millis();

  tft.fillRect((int)bx, (int)by, 6, 6, BG);
  bx += bvx; by += bvy;
  if (by < PY0) { by = PY0; bvy = -bvy; }
  if (by > PY1 - 6) { by = PY1 - 6; bvy = -bvy; }

  int target = (int)by - PH / 2 + 3;
  int oldAy = ay_;
  if (abs(target - ay_) > 2) ay_ += (target > ay_) ? 2 : -2;
  ay_ = constrain(ay_, PY0, PY1 - PH);
  if (ay_ != oldAy) tft.fillRect(308, oldAy, 6, PH, BG);

  if (bvx < 0 && bx <= 12 && bx >= 2 && by + 6 >= py && by <= py + PH) {
    bx = 12;
    bvx = -bvx * 1.06f;
    if (bvx > 7) bvx = 7;
    bvy += ((by + 3) - (py + PH / 2)) * 0.08f;
  }
  if (bvx > 0 && bx + 6 >= 308 && bx <= 316 && by + 6 >= ay_ && by <= ay_ + PH) {
    bx = 302;
    bvx = -bvx * 1.06f;
    if (bvx < -7) bvx = -7;
    bvy += ((by + 3) - (ay_ + PH / 2)) * 0.08f;
  }
  bvy = constrain(bvy, -5.0f, 5.0f);

  tft.fillRect(6, py, 6, PH, FG);
  tft.fillRect(308, ay_, 6, PH, FG);

  if (bx < -6) {
    as_++;
    pongScore();
    if (as_ >= 7) { pongEnd("You lose", TFT_RED); return; }
    pongServe(1);
    pongPause = millis() + 700;
    return;
  }
  if (bx > 320) {
    ps++;
    pongScore();
    if (ps >= 7) { pongEnd("You win!", TFT_GREEN); return; }
    pongServe(-1);
    pongPause = millis() + 700;
    return;
  }
  tft.fillRect((int)bx, (int)by, 6, 6, TFT_WHITE);
}

const int GX = 70, GY = 30, GCELL = 60;
char tb[9];
bool tOver = false;

char tWinner() {
  static const int L[8][3] = {{0,1,2},{3,4,5},{6,7,8},{0,3,6},{1,4,7},{2,5,8},{0,4,8},{2,4,6}};
  for (int i = 0; i < 8; i++) {
    char a = tb[L[i][0]];
    if (a != ' ' && a == tb[L[i][1]] && a == tb[L[i][2]]) return a;
  }
  for (int i = 0; i < 9; i++) if (tb[i] == ' ') return 0;
  return 'T';
}

void tttStatus(const char* msg, uint16_t col) {
  tft.fillRect(0, 214, 320, 26, BG);
  centerText(msg, 160, 226, 2, col, BG);
}

void tttDraw() {
  tft.fillRect(GX - 2, GY - 2, 3 * GCELL + 4, 3 * GCELL + 4, BG);
  for (int i = 1; i < 3; i++) {
    tft.drawFastHLine(GX, GY + i * GCELL, 3 * GCELL, FG);
    tft.drawFastVLine(GX + i * GCELL, GY, 3 * GCELL, FG);
  }
  for (int i = 0; i < 9; i++) {
    int cx = GX + (i % 3) * GCELL + GCELL / 2;
    int cy = GY + (i / 3) * GCELL + GCELL / 2;
    if (tb[i] == 'X') {
      tft.drawLine(cx - 16, cy - 16, cx + 16, cy + 16, TFT_CYAN);
      tft.drawLine(cx - 16, cy + 16, cx + 16, cy - 16, TFT_CYAN);
      tft.drawLine(cx - 15, cy - 16, cx + 17, cy + 16, TFT_CYAN);
      tft.drawLine(cx - 15, cy + 16, cx + 17, cy - 16, TFT_CYAN);
    } else if (tb[i] == 'O') {
      tft.drawCircle(cx, cy, 17, WARN);
      tft.drawCircle(cx, cy, 16, WARN);
    }
  }
}

void tttStart() {
  header("TicTac");
  for (int i = 0; i < 9; i++) tb[i] = ' ';
  tOver = false;
  tttDraw();
  tttStatus("Your turn (X)", FG);
}

int tttAI() {
  for (int pass = 0; pass < 2; pass++) {
    char who = pass == 0 ? 'O' : 'X';
    for (int i = 0; i < 9; i++) {
      if (tb[i] == ' ') {
        tb[i] = who;
        bool w = (tWinner() == who);
        tb[i] = ' ';
        if (w) return i;
      }
    }
  }
  if (tb[4] == ' ') return 4;
  int corners[4] = {0, 2, 6, 8};
  int start = random(4);
  for (int k = 0; k < 4; k++) {
    int c = corners[(start + k) % 4];
    if (tb[c] == ' ') return c;
  }
  for (int i = 0; i < 9; i++) if (tb[i] == ' ') return i;
  return -1;
}

void tttTap(int x, int y) {
  if (tOver) { tttStart(); return; }
  if (!hit(x, y, GX, GY, 3 * GCELL, 3 * GCELL)) return;
  int idx = ((y - GY) / GCELL) * 3 + (x - GX) / GCELL;
  if (tb[idx] != ' ') return;
  tb[idx] = 'X';
  char r = tWinner();
  tttDraw();
  if (!r) {
    delay(250);
    int a = tttAI();
    if (a >= 0) tb[a] = 'O';
    r = tWinner();
    tttDraw();
  }
  if (r) {
    tOver = true;
    if (r == 'X') tttStatus("You win! Tap=restart", TFT_GREEN);
    else if (r == 'O') tttStatus("You lose. Tap=restart", TFT_RED);
    else tttStatus("Draw. Tap=restart", WARN);
  }
}

uint16_t paintColor = TFT_WHITE;
int lastX = -1, lastY = -1;
const uint16_t palette[8] = {TFT_WHITE, TFT_RED, TFT_ORANGE, TFT_YELLOW, TFT_GREEN, TFT_CYAN, TFT_BLUE, TFT_MAGENTA};

void drawPaint() {
  app = PAINT;
  header("Paint");
  for (int i = 0; i < 8; i++) tft.fillRect(i * 34, 216, 34, 24, palette[i]);
  tft.setTextSize(1);
  button(274, 216, 46, 24, "CLR");
  lastX = lastY = -1;
}

void paintMove(int x, int y) {
  if (y >= 28 && y < 214) {
    if (lastX >= 0) tft.drawLine(lastX, lastY, x, y, paintColor);
    tft.fillCircle(x, y, 1, paintColor);
    lastX = x;
    lastY = y;
  } else {
    lastX = lastY = -1;
  }
}

void paintTap(int x, int y) {
  if (y >= 216) {
    if (x < 272) paintColor = palette[x / 34];
    else tft.fillRect(0, 27, 320, 188, BG);
  }
}

void drawInfo() {
  app = INFO;
  tft.setTextSize(2);
  tft.setTextColor(FG, BG);
  uint32_t s = millis() / 1000;
  int y = 34;
  const int dy = 24;
  tft.setCursor(4, y);           tft.printf("%s r%d      ", ESP.getChipModel(), ESP.getChipRevision());
  tft.setCursor(4, y += dy);     tft.printf("CPU %d MHz   ", ESP.getCpuFreqMHz());
  tft.setCursor(4, y += dy);     tft.printf("Heap %d KB   ", ESP.getFreeHeap() / 1024);
  tft.setCursor(4, y += dy);     tft.printf("Flash %d MB  ", ESP.getFlashChipSize() / 1048576);
  tft.setCursor(4, y += dy);     tft.printf("Up %02d:%02d:%02d  ", (int)(s / 3600), (int)((s / 60) % 60), (int)(s % 60));
  tft.setCursor(4, y += dy);     tft.printf("SD %s       ", sdOK ? (String((uint32_t)(SD.cardSize() / 1048576)) + " MB").c_str() : "none");
  tft.setTextSize(1);
  tft.setCursor(4, y += dy + 4); tft.printf("MAC %s", WiFi.macAddress().c_str());
  tft.setCursor(4, y += 12);     tft.print("kiOS v2.0 - Kershaw Industries OS");
  lastTick = millis();
}

void startInfo() {
  header("Info");
  drawInfo();
}

void drawSettings() {
  app = SETTINGS;
  header("Setup");
  tft.setTextSize(2);
  tft.setTextColor(FG, BG);
  tft.setCursor(8, 44);
  tft.print("Brightness");
  button(8, 72, 60, 44, "-");
  button(252, 72, 60, 44, "+");
  tft.drawRect(76, 82, 168, 24, FG);
  tft.fillRect(77, 83, 166, 22, BG);
  tft.fillRect(77, 83, map(brightness, 10, 255, 0, 166), 22, FG);
  button(8, 140, 150, 44, "Recalibrate");
  button(168, 140, 144, 44, "Reboot", TFT_RED);
  tft.setTextSize(1);
  tft.setTextColor(FG, BG);
  tft.setCursor(8, 200);
  tft.print("Touch not working? Press the BOOT button on the board.");
}

void settingsTap(int x, int y) {
  if (hit(x, y, 8, 72, 60, 44)) {
    brightness = brightness > 30 ? brightness - 20 : 10;
    tft.setBrightness(brightness);
    prefs.putUChar("bl", brightness);
    drawSettings();
  } else if (hit(x, y, 252, 72, 60, 44)) {
    brightness = brightness < 235 ? brightness + 20 : 255;
    tft.setBrightness(brightness);
    prefs.putUChar("bl", brightness);
    drawSettings();
  } else if (hit(x, y, 8, 140, 150, 44)) {
    calibrate();
    drawSettings();
  } else if (hit(x, y, 168, 140, 144, 44)) {
    ESP.restart();
  }
}

const char* appNames[12] = {"WiFi", "BLE", "Files", "Notes", "Clock", "Calc",
                            "Snake", "Pong", "TicTac", "Paint", "Info", "Setup"};
const uint16_t appCols[12] = {TFT_BLUE, 0x041F, TFT_ORANGE, 0x8010, TFT_DARKCYAN, TFT_DARKGREY,
                              0x03E0, TFT_RED, 0x780F, TFT_MAGENTA, 0x0410, 0x4208};

void drawHome() {
  app = HOME;
  tft.fillScreen(BG);
  tft.fillRect(0, 0, 320, 26, DIM);
  tft.setTextSize(2);
  tft.setTextColor(FG, DIM);
  tft.setCursor(8, 5);
  tft.print("kiOS");
  tft.setTextSize(1);
  button(236, 2, 80, 22, homePage == 0 ? "More >" : "< Back");
  for (int i = 0; i < 6; i++) {
    int idx = homePage * 6 + i;
    int x = 8 + (i % 3) * 104;
    int y = 38 + (i / 3) * 98;
    tft.fillRoundRect(x, y, 96, 86, 10, appCols[idx]);
    tft.drawRoundRect(x, y, 96, 86, 10, FG);
    centerText(appNames[idx], x + 48, y + 43, 2, TFT_WHITE, appCols[idx]);
  }
  tft.setTextSize(1);
}

void launch(int idx) {
  switch (idx) {
    case 0:  drawWifi(); break;
    case 1:  drawBle(); break;
    case 2:  path = "/"; viewFile = ""; loadDir(); drawFiles(); break;
    case 3:  drawNotes(); break;
    case 4:  drawClock(); break;
    case 5:  drawCalc(); break;
    case 6:  app = SNAKE; snakeStart(); break;
    case 7:  app = PONG; pongStart(); break;
    case 8:  app = TTT; tttStart(); break;
    case 9:  drawPaint(); break;
    case 10: startInfo(); break;
    case 11: drawSettings(); break;
  }
}

void onTap(int x, int y) {
  if (app == KEYB) { kbTap(x, y); return; }

  if (app != HOME && hit(x, y, 0, 0, 66, 26)) {
    if (app == FILES && viewFile.length()) {
      viewFile = "";
      drawFiles();
    } else {
      drawHome();
    }
    return;
  }
  switch (app) {
    case HOME:
      if (hit(x, y, 236, 2, 80, 22)) { homePage = 1 - homePage; drawHome(); break; }
      for (int i = 0; i < 6; i++) {
        int bx2 = 8 + (i % 3) * 104;
        int by2 = 38 + (i / 3) * 98;
        if (hit(x, y, bx2, by2, 96, 86)) { launch(homePage * 6 + i); break; }
      }
      break;
    case WIFI:
      wifiTap(x, y);
      break;
    case BTSCAN:
#if USE_BLE
      if (hit(x, y, 230, 210, 86, 26)) drawBle();
#endif
      break;
    case FILES:    filesTap(x, y); break;
    case NOTES:    notesTap(x, y); break;
    case CLOCK:    clockTap(x, y); break;
    case CALC:     calcTap(x, y); break;
    case SNAKE:    if (snDead) snakeStart(); break;
    case PONG:     if (pongOver) pongStart(); break;
    case TTT:      tttTap(x, y); break;
    case PAINT:    paintTap(x, y); break;
    case SETTINGS: settingsTap(x, y); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(0, INPUT_PULLUP);

  pinMode(T_CS, OUTPUT);   digitalWrite(T_CS, HIGH);
  pinMode(T_CLK, OUTPUT);  digitalWrite(T_CLK, LOW);
  pinMode(T_MOSI, OUTPUT);
  pinMode(T_MISO, INPUT);

  randomSeed(esp_random());
  prefs.begin("cydos", false);
  tft.init();
  tft.setRotation(1);
  brightness = prefs.getUChar("bl", 200);
  tft.setBrightness(brightness);

  tft.fillScreen(BG);
  centerText("kiOS", 160, 100, 6, FG, BG);
  centerText("Kershaw Industries OS", 160, 150, 2, TFT_WHITE, BG);
  centerText("v2.0", 160, 180, 2, DIM, BG);
  delay(1400);

  setDefaultCal();
  bool forceCal = (digitalRead(0) == LOW);
  if (forceCal) while (digitalRead(0) == LOW) delay(10);

  if (!forceCal && prefs.getBytesLength("tc3") == sizeof(tc)) {
    prefs.getBytes("tc3", &tc, sizeof(tc));
  } else {
    calibrate();
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  sdSPI.begin(18, 19, 23, 5);
  sdOK = SD.begin(5, sdSPI);

  drawHome();
}

void loop() {
  if (digitalRead(0) == LOW) {
    delay(50);
    if (digitalRead(0) == LOW) {
      while (digitalRead(0) == LOW) delay(10);
      calibrate();
      drawHome();
      return;
    }
  }

  int32_t x, y;
  bool down = readTouch(x, y);

  if (down && !wasDown) onTap(x, y);
  if (down) {
    if (app == PAINT) paintMove(x, y);
    else if (app == SNAKE) snakeTouch(x, y);
    else if (app == PONG) pongTouch(y);
  }
  if (!down) lastX = lastY = -1;
  wasDown = down;

  switch (app) {
    case INFO:  if (millis() - lastTick > 1000) drawInfo(); break;
    case CLOCK: clockTick(); break;
    case SNAKE: snakeTick(); break;
    case PONG:  pongTick(); break;
    default: break;
  }
  delay(2);
}
