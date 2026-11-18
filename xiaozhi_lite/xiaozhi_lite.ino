/*
  Xiaozhi-Lite for ESP32-S3 CAM
  - INMP441 mic, ST7789 240x240 LCD, OV2640/OV5640 camera
  - No speaker
  - GenZi AI: voice is transcribed, then sent to the selected chat model.
    Photo-only questions use one chat request; spoken questions use two requests.
  - Wi-Fi setup AP: Xiaozhi-Setup, password: xiaozhi-setup-26
    Open http://192.168.4.1

  Libraries (Arduino Library Manager):
    Adafruit GFX Library
    Adafruit ST7735 and ST7789 Library
    ArduinoJson (v7)

  Board: ESP32S3 Dev Module, Arduino-ESP32 core 3.x, PSRAM Enabled.
  Camera pins below match the Freenove ESP32-S3 WROOM CAM example from the original sketch.
  Verify every pin against YOUR exact board before powering it.

  Security:
    - The API key is entered in the local setup page and stored in NVS Preferences.
    - Never paste an API key into this source file or share it in chat.
    - HTTPS validates the GenZi endpoint using the included ISRG Root X1 certificate.
    - Replace the key that was previously pasted in chat; assume it is compromised.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <vector>
#include <math.h>
#include <string.h>
#include "esp_camera.h"
#include "driver/i2s_std.h"
#include "mbedtls/base64.h"

// ------------------ Pin map ------------------
#define LCD_SCK   19
#define LCD_MOSI  20
#define LCD_RST   21
#define LCD_DC    47
#define LCD_BLK   38

#define I2S_WS    1
#define I2S_SCK   2
#define I2S_SD    42
#define BTN_PIN   0

#define CAM_PWDN  -1
#define CAM_RESET -1
#define CAM_XCLK  15
#define CAM_SIOD  4
#define CAM_SIOC  5
#define CAM_Y9    16
#define CAM_Y8    17
#define CAM_Y7    18
#define CAM_Y6    12
#define CAM_Y5    10
#define CAM_Y4    8
#define CAM_Y3    9
#define CAM_Y2    11
#define CAM_VSYNC 6
#define CAM_HREF  7
#define CAM_PCLK  13

// ------------------ Settings ------------------
static const char* DEFAULT_MODEL = "gemini-3.8-flash";
static const char* AP_NAME = "Xiaozhi-Setup";
static const char* AP_PASSWORD = "xiaozhi-setup-26";
static const char* GENZI_HOST = "ai.genzicorp.com";

static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr size_t CHUNK_SAMPLES = 512;
static constexpr uint32_t MAX_SECONDS = 6;
static constexpr size_t MAX_SAMPLES = SAMPLE_RATE * MAX_SECONDS;
static constexpr size_t MIN_SAMPLES = 12000;
static constexpr int SILENCE_CHUNKS = 31;
static constexpr int PRE_ROLL_CHUNKS = 4;
static constexpr int AUDIO_GAIN_SHIFT = 14;
static constexpr float VAD_MIN_LEVEL = 250.0f;

static const char SYSTEM_PROMPT[] =
    "Kamu asisten suara di perangkat ESP32 dengan layar kecil. "
    "Jawab dengan bahasa pengguna (default Indonesia), maksimal 5 kalimay "
    "atau 250 karakter. boleh pakai emoji dan kamu asisten pintar yang bisa baca saat foto dan juga harus jawab dengan lengkap.";

// ISRG Root X1, used to verify the public TLS certificate chain.
static const char GENZI_ROOT_CA[] PROGMEM = R"CERT(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----)CERT";

Adafruit_ST7789 tft(&SPI, -1, LCD_DC, LCD_RST);
Preferences prefs;
WebServer server(80);
DNSServer dns;
i2s_chan_handle_t rxChan = nullptr;

String cfgSsid;
String cfgPass;
String cfgKey;
String cfgModel;
uint8_t* pcmBuf = nullptr;
int16_t* pcm = nullptr;
float noiseFloor = 60.0f;
bool camOK = false;
bool micOK = false;
bool memoryOK = false;
bool portalRunning = false;

// ------------------ LCD ------------------
void showStatus(const char* text, uint16_t color) {
  tft.fillRect(0, 0, 240, 26, color);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(6, 5);
  tft.print(text);
}

void showBody(const String& text, uint16_t color = ST77XX_WHITE) {
  tft.fillRect(0, 28, 240, 212, ST77XX_BLACK);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(0, 34);
  tft.print(text);
}

void showAnswer(const String& answer) {
  String clean;
  clean.reserve(answer.length());
  for (size_t i = 0; i < answer.length(); i++) {
    const unsigned char c = static_cast<unsigned char>(answer[i]);
    if (c == '\n' || c == '\r' || c == '\t') clean += ' ';
    else if (c >= 32 && c < 127 && c != '*' && c != '#' && c != '`') clean += static_cast<char>(c);
  }

  const int COLS = 20;
  const int ROWS = 12;
  std::vector<String> lines;
  String current;
  int pos = 0;
  const int length = static_cast<int>(clean.length());
  while (pos < length) {
    int space = clean.indexOf(' ', pos);
    String word = (space < 0) ? clean.substring(pos) : clean.substring(pos, space);
    pos = (space < 0) ? length : space + 1;
    if (word.length() == 0) continue;
    while (static_cast<int>(word.length()) > COLS) {
      if (current.length()) {
        lines.push_back(current);
        current = "";
      }
      lines.push_back(word.substring(0, COLS));
      word = word.substring(COLS);
    }
    const int extra = current.length() ? 1 : 0;
    if (static_cast<int>(current.length() + extra + word.length()) > COLS) {
      lines.push_back(current);
      current = word;
    } else {
      if (current.length()) current += ' ';
      current += word;
    }
  }
  if (current.length()) lines.push_back(current);
  if (lines.empty()) lines.push_back("(kosong)");

  const int pages = (static_cast<int>(lines.size()) + ROWS - 1) / ROWS;
  for (int page = 0; page < pages; page++) {
    tft.fillRect(0, 28, 240, 212, ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    for (int row = 0; row < ROWS; row++) {
      const int index = page * ROWS + row;
      if (index >= static_cast<int>(lines.size())) break;
      tft.setCursor(0, 34 + row * 16);
      tft.print(lines[index]);
    }
    if (pages > 1) {
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(200, 230);
      tft.printf("%d/%d", page + 1, pages);
    }
    if (page < pages - 1) delay(3500);
  }
}

String htmlEscape(const String& value) {
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); i++) {
    switch (value[i]) {
      case '&': out += "&amp;"; break;
      case '"': out += "&quot;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '\'': out += "&#39;"; break;
      default: out += value[i]; break;
    }
  }
  return out;
}

// ------------------ Wi-Fi setup portal ------------------
String portalPage() {
  String html = R"HTML(<!doctype html><html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Setup Xiaozhi</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{font:16px system-ui,sans-serif;background:#101820;color:#edf3f5;margin:0;padding:20px}
main{max-width:440px;margin:0 auto}h1{font-size:24px;margin:0 0 8px}p{color:#b7c4ca;line-height:1.5}.hint{border-left:3px solid #43c59e;padding:8px 12px;margin:16px 0}
label{display:block;margin:17px 0 6px;color:#c5d0d5}input{width:100%;padding:13px;border:1px solid #52616b;border-radius:6px;background:#192630;color:#fff;font-size:16px}
button{width:100%;padding:14px;border:0;border-radius:6px;margin-top:22px;background:#43c59e;color:#10201b;font-size:17px;font-weight:700}
small{color:#a9b8bf}.check{display:flex;align-items:center;gap:10px}.check input{width:20px;height:20px}
</style></head><body><main><h1>Setup Xiaozhi-Lite</h1>
<p>Hubungkan perangkat ke Wi‑Fi dan masukkan kunci GenZi AI.</p>
<div class="hint">Wi‑Fi perangkat: <b>Xiaozhi-Setup</b><br>Kata sandi: <b>xiaozhi-setup-26</b><br>Buka <b>http://192.168.4.1</b></div>
<form method="POST" action="/save">
<label for="ssid">Nama Wi‑Fi (SSID)</label><input id="ssid" name="ssid" required maxlength="32" value=")HTML";
  html += htmlEscape(cfgSsid);
  html += R"HTML(">
<label for="pass">Kata sandi Wi‑Fi</label><input id="pass" name="pass" type="password" maxlength="64" autocomplete="new-password" placeholder="Isi kata sandi Wi‑Fi">
<label for="key">GenZi AI API key</label><input id="key" name="key" type="password" maxlength="256" autocomplete="new-password" placeholder="gzi_...">
<small>Kunci tersimpan tidak ditampilkan kembali. Kosongkan untuk memakai kunci yang sudah tersimpan.</small>
<label for="model">Model GenZi</label><input id="model" name="model" maxlength="100" value=")HTML";
  html += htmlEscape(cfgModel);
  html += R"HTML(">
<label class="check"><input type="checkbox" name="clear_key" value="1"> Hapus kunci GenZi tersimpan</label>
<button type="submit">Simpan dan sambungkan</button></form>
<p><small>Jika kata sandi Wi‑Fi kosong, perangkat akan mencoba jaringan terbuka.</small></p>
</main></body></html>)HTML";
  return html;
}

void runPortal() {
  portalRunning = true;
  showStatus("Setup WiFi", ST77XX_ORANGE);
  showBody(String("WiFi: ") + AP_NAME + "\nSandi: " + AP_PASSWORD + "\n\nBuka 192.168.4.1");

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_NAME, AP_PASSWORD);
  delay(250);
  dns.start(53, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html; charset=utf-8", portalPage());
  });
  server.on("/save", HTTP_POST, []() {
    String ssid = server.arg("ssid");
    ssid.trim();
    if (ssid.length() == 0) {
      server.send(400, "text/plain; charset=utf-8", "Nama Wi-Fi wajib diisi.");
      return;
    }
    prefs.putString("ssid", ssid);
    prefs.putString("pass", server.arg("pass"));

    String key = server.arg("key");
    key.trim();
    if (server.hasArg("clear_key")) prefs.putString("gkey", "");
    else if (key.length() > 0) prefs.putString("gkey", key);

    String model = server.arg("model");
    model.trim();
    prefs.putString("gmodel", model.length() ? model : DEFAULT_MODEL);

    server.send(200, "text/html; charset=utf-8",
      "<!doctype html><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<meta charset='utf-8'><body style='font:17px system-ui;background:#101820;color:#edf3f5;padding:28px'>"
      "<h2>Pengaturan tersimpan</h2><p>Perangkat akan menyambung ulang. Halaman ini boleh ditutup.</p>");
    delay(1200);
    ESP.restart();
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();

  while (true) {
    dns.processNextRequest();
    server.handleClient();
    delay(2);
  }
}

bool connectWiFi() {
  showStatus("Sambung WiFi", ST77XX_YELLOW);
  showBody(cfgSsid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 20000) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

// ------------------ Button ------------------
// 0 = none, 1 = short click. Holding BOOT for 3 seconds clears Wi-Fi settings.
int checkButton() {
  if (digitalRead(BTN_PIN) == HIGH) return 0;
  const uint32_t started = millis();
  while (digitalRead(BTN_PIN) == LOW) {
    if (millis() - started >= 3000) {
      showStatus("Reset WiFi", ST77XX_RED);
      prefs.remove("ssid");
      prefs.remove("pass");
      delay(500);
      ESP.restart();
    }
    delay(10);
  }
  return millis() - started > 30 ? 1 : 0;
}

// ------------------ Microphone / voice detection ------------------
bool micInit() {
  i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = 6;
  channelConfig.dma_frame_num = 256;
  if (i2s_new_channel(&channelConfig, nullptr, &rxChan) != ESP_OK || rxChan == nullptr) return false;

  i2s_std_config_t standardConfig = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = static_cast<gpio_num_t>(I2S_SCK),
      .ws = static_cast<gpio_num_t>(I2S_WS),
      .dout = I2S_GPIO_UNUSED,
      .din = static_cast<gpio_num_t>(I2S_SD),
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  standardConfig.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  if (i2s_channel_init_std_mode(rxChan, &standardConfig) != ESP_OK) return false;
  if (i2s_channel_enable(rxChan) != ESP_OK) return false;
  return true;
}

float readChunk(int16_t* output) {
  if (rxChan == nullptr || output == nullptr) return -1.0f;
  static int32_t raw[CHUNK_SAMPLES];
  static float dc = 0.0f;
  size_t bytesRead = 0;
  const esp_err_t result = i2s_channel_read(rxChan, raw, sizeof(raw), &bytesRead, 500);
  if (result != ESP_OK || bytesRead < sizeof(int32_t)) return -1.0f;

  const size_t count = bytesRead / sizeof(int32_t);
  float sum = 0.0f;
  for (size_t i = 0; i < count; i++) {
    float value = static_cast<float>(raw[i] >> AUDIO_GAIN_SHIFT);
    dc += (value - dc) * 0.002f;
    value -= dc;
    if (value > 32767.0f) value = 32767.0f;
    if (value < -32768.0f) value = -32768.0f;
    output[i] = static_cast<int16_t>(value);
    sum += fabsf(value);
  }
  for (size_t i = count; i < CHUNK_SAMPLES; i++) output[i] = 0;
  return sum / static_cast<float>(count);
}

void flushMic() {
  if (!micOK) return;
  int16_t temp[CHUNK_SAMPLES];
  for (int i = 0; i < 6; i++) readChunk(temp);
}

// Return captured sample count; 0 = no speech/too short, -1 = BOOT pressed.
int recordSpeech(uint32_t maxWaitMs, bool pollButton) {
  if (!micOK || pcm == nullptr) return 0;
  static int16_t preRoll[PRE_ROLL_CHUNKS][CHUNK_SAMPLES];
  int16_t chunk[CHUNK_SAMPLES];
  int preIndex = 0;
  int preCount = 0;
  int hot = 0;
  const uint32_t started = millis();
  float threshold = 0.0f;

  while (true) {
    if (pollButton && checkButton() == 1) return -1;
    if (maxWaitMs > 0 && millis() - started >= maxWaitMs) return 0;
    const float level = readChunk(chunk);
    if (level < 0.0f) continue;

    memcpy(preRoll[preIndex], chunk, sizeof(chunk));
    preIndex = (preIndex + 1) % PRE_ROLL_CHUNKS;
    if (preCount < PRE_ROLL_CHUNKS) preCount++;
    threshold = noiseFloor * 3.0f + VAD_MIN_LEVEL;
    if (level > threshold) hot++;
    else {
      hot = 0;
      noiseFloor = noiseFloor * 0.98f + level * 0.02f;
    }
    if (hot >= 3) break;
  }

  showStatus("Merekam", ST77XX_RED);
  size_t written = 0;
  const int first = (preIndex - preCount + PRE_ROLL_CHUNKS) % PRE_ROLL_CHUNKS;
  for (int i = 0; i < preCount && written + CHUNK_SAMPLES <= MAX_SAMPLES; i++) {
    memcpy(pcm + written, preRoll[(first + i) % PRE_ROLL_CHUNKS], sizeof(chunk));
    written += CHUNK_SAMPLES;
  }

  int quiet = 0;
  while (written + CHUNK_SAMPLES <= MAX_SAMPLES) {
    const float level = readChunk(chunk);
    if (level < 0.0f) continue;
    memcpy(pcm + written, chunk, sizeof(chunk));
    written += CHUNK_SAMPLES;
    if (level > threshold * 0.7f) quiet = 0;
    else quiet++;
    if (quiet > SILENCE_CHUNKS) break;
  }
  if (written < MIN_SAMPLES) return 0;
  return static_cast<int>(written);
}

void writeWavHeader(uint8_t* buffer, uint32_t dataLength) {
  uint32_t value32 = 36 + dataLength;
  uint16_t value16 = 1;
  memcpy(buffer, "RIFF", 4);
  memcpy(buffer + 4, &value32, 4);
  memcpy(buffer + 8, "WAVEfmt ", 8);
  value32 = 16;
  memcpy(buffer + 16, &value32, 4);
  memcpy(buffer + 20, &value16, 2);
  memcpy(buffer + 22, &value16, 2);
  value32 = SAMPLE_RATE;
  memcpy(buffer + 24, &value32, 4);
  value32 = SAMPLE_RATE * 2;
  memcpy(buffer + 28, &value32, 4);
  value16 = 2;
  memcpy(buffer + 32, &value16, 2);
  value16 = 16;
  memcpy(buffer + 34, &value16, 2);
  memcpy(buffer + 36, "data", 4);
  value32 = dataLength;
  memcpy(buffer + 40, &value32, 4);
}

// ------------------ Camera ------------------
bool cameraInit() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_Y2; config.pin_d1 = CAM_Y3;
  config.pin_d2 = CAM_Y4; config.pin_d3 = CAM_Y5;
  config.pin_d4 = CAM_Y6; config.pin_d5 = CAM_Y7;
  config.pin_d6 = CAM_Y8; config.pin_d7 = CAM_Y9;
  config.pin_xclk = CAM_XCLK; config.pin_pclk = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC; config.pin_href = CAM_HREF;
  config.pin_sccb_sda = CAM_SIOD; config.pin_sccb_scl = CAM_SIOC;
  config.pin_pwdn = CAM_PWDN; config.pin_reset = CAM_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 14;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&config) == ESP_OK;
}

// ------------------ GenZi network helpers ------------------
bool postGenzi(const char* path, const char* contentType,
               uint8_t* body, size_t bodyLength, String& response, int& httpCode) {
  if (WiFi.status() != WL_CONNECTED || cfgKey.length() < 10 || body == nullptr) {
    httpCode = 0;
    return false;
  }

  WiFiClientSecure tls;
  tls.setCACert(GENZI_ROOT_CA);
  HTTPClient http;
  http.setTimeout(45000);
  http.setReuse(false);
  String url = String("https://") + GENZI_HOST + path;
  if (!http.begin(tls, url)) {
    httpCode = 0;
    return false;
  }
  http.addHeader("Authorization", String("Bearer ") + cfgKey);
  http.addHeader("Content-Type", contentType);
  http.addHeader("Accept-Encoding", "identity");
  httpCode = http.POST(body, bodyLength);
  if (httpCode > 0) response = http.getString();
  else response = "";
  http.end();
  return httpCode > 0;
}

size_t base64Length(size_t length) {
  return 4 * ((length + 2) / 3);
}

bool appendString(uint8_t* destination, size_t capacity, size_t& position, const String& value) {
  if (position + value.length() > capacity) return false;
  memcpy(destination + position, value.c_str(), value.length());
  position += value.length();
  return true;
}

bool appendBase64(uint8_t* destination, size_t capacity, size_t& position,
                  const uint8_t* source, size_t sourceLength) {
  if (destination == nullptr || source == nullptr || position > capacity) return false;
  const size_t expected = base64Length(sourceLength);
  if (expected + 1 > capacity - position) return false;
  size_t written = 0;
  const int result = mbedtls_base64_encode(destination + position, expected + 1,
                                           &written, source, sourceLength);
  if (result != 0 || written != expected) return false;
  position += written;
  return true;
}

String jsonQuote(const String& value) {
  String escaped;
  escaped.reserve(value.length() + 8);
  escaped += '"';
  for (size_t i = 0; i < value.length(); i++) {
    const unsigned char c = static_cast<unsigned char>(value[i]);
    if (c == '"' || c == '\\') {
      escaped += '\\';
      escaped += static_cast<char>(c);
    } else if (c == '\n') escaped += "\\n";
    else if (c == '\r') escaped += "\\r";
    else if (c == '\t') escaped += "\\t";
    else if (c < 0x20) {
      char sequence[7];
      snprintf(sequence, sizeof(sequence), "\\u%04x", c);
      escaped += sequence;
    } else escaped += static_cast<char>(c);
  }
  escaped += '"';
  return escaped;
}

String transcribeAudio(const uint8_t* wav, size_t wavLength, String& error) {
  if (wav == nullptr || wavLength < 44) {
    error = "Rekaman audio tidak valid.";
    return "";
  }
  const String boundary = "----XiaozhiGenZiBoundary7MA4YWxk";
  const String modelPart = "--" + boundary +
      "\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\nwhisper-1\r\n";
  const String fileHead = "--" + boundary +
      "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"speech.wav\""
      "\r\nContent-Type: audio/wav\r\n\r\n";
  const String ending = "\r\n--" + boundary + "--\r\n";
  const String contentType = "multipart/form-data; boundary=" + boundary;
  const size_t capacity = modelPart.length() + fileHead.length() + wavLength + ending.length();
  uint8_t* body = static_cast<uint8_t*>(ps_malloc(capacity));
  if (body == nullptr) {
    error = "PSRAM tidak cukup untuk mengirim rekaman.";
    return "";
  }

  size_t position = 0;
  bool ok = appendString(body, capacity, position, modelPart) &&
            appendString(body, capacity, position, fileHead) &&
            position + wavLength <= capacity;
  if (ok) {
    memcpy(body + position, wav, wavLength);
    position += wavLength;
    ok = appendString(body, capacity, position, ending);
  }
  if (!ok) {
    free(body);
    error = "Gagal menyiapkan data audio.";
    return "";
  }

  String response;
  int code = 0;
  const bool connected = postGenzi("/v1/audio/transcriptions", contentType.c_str(),
                                   body, position, response, code);
  free(body);
  if (!connected) {
    error = "Tidak dapat terhubung ke GenZi untuk transkripsi audio.";
    return "";
  }
  JsonDocument document;
  if (deserializeJson(document, response)) {
    error = String("Respons transkripsi tidak valid (HTTP ") + code + ").";
    return "";
  }
  if (code < 200 || code >= 300) {
    const char* message = document["error"]["message"] | "periksa API key, saldo, dan model Whisper";
    error = String("Transkripsi GenZi HTTP ") + code + ": " + message;
    return "";
  }
  String text = document["text"] | "";
  if (text.length() == 0) error = "Suara tidak terbaca. Coba bicara lebih jelas.";
  return text;
}

String askLLM(const uint8_t* wav, size_t wavLength,
              const uint8_t* jpg, size_t jpgLength, const char* prompt) {
  if (cfgKey.length() < 10) return "Kunci GenZi belum diisi. Buka setup Wi-Fi.";
  if (WiFi.status() != WL_CONNECTED) return "Wi-Fi terputus. Sambungkan ulang perangkat.";

  String transcript;
  if (wav != nullptr && wavLength > 0) {
    showStatus("Transkripsi", ST77XX_YELLOW);
    String error;
    transcript = transcribeAudio(wav, wavLength, error);
    if (transcript.length() == 0) return error.length() ? error : "Transkripsi suara gagal.";
  }

  String userText = prompt ? String(prompt) : String("");
  if (transcript.length() > 0) userText += "\nPertanyaan pengguna: " + transcript;
  const String prefix = String("{\"model\":") + jsonQuote(cfgModel) +
      ",\"max_tokens\":512,\"messages\":[{\"role\":\"system\",\"content\":" +
      jsonQuote(String(SYSTEM_PROMPT)) +
      "},{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":" +
      jsonQuote(userText);
  const String imageHead = ",{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,";
  const String imageTail = "\"}}";
  const String suffix = "]}]}";

  size_t capacity = prefix.length() + 1 + suffix.length() + 8;
  if (jpg != nullptr && jpgLength > 0) {
    capacity += imageHead.length() + base64Length(jpgLength) + imageTail.length();
  }
  uint8_t* body = static_cast<uint8_t*>(ps_malloc(capacity));
  if (body == nullptr) return "PSRAM tidak cukup untuk menyiapkan permintaan.";

  size_t position = 0;
  bool ok = appendString(body, capacity, position, prefix);
  if (ok && position < capacity) body[position++] = '}'; // end the text block
  if (ok && jpg != nullptr && jpgLength > 0) {
    ok = appendString(body, capacity, position, imageHead) &&
         appendBase64(body, capacity, position, jpg, jpgLength) &&
         appendString(body, capacity, position, imageTail);
  }
  if (ok) ok = appendString(body, capacity, position, suffix);
  if (!ok) {
    free(body);
    return "Gagal menyiapkan data. Coba foto atau ucapan lebih singkat.";
  }

  showStatus("Berpikir", ST77XX_YELLOW);
  String response;
  int code = 0;
  const bool connected = postGenzi("/v1/chat/completions", "application/json",
                                   body, position, response, code);
  free(body);
  if (!connected) return "Koneksi GenZi gagal atau waktu tunggu habis.";

  JsonDocument document;
  if (deserializeJson(document, response)) {
    return String("Respons GenZi tidak valid (HTTP ") + code + ").";
  }
  if (code < 200 || code >= 300) {
    const char* message = document["error"]["message"] | "periksa API key, saldo, dan nama model";
    return String("GenZi HTTP ") + code + ": " + message;
  }
  const char* text = document["choices"][0]["message"]["content"] | "";
  if (text == nullptr || strlen(text) == 0) return "GenZi mengirim jawaban kosong.";
  return String(text);
}

// ------------------ User flows ------------------
void doPhoto() {
  if (!camOK) {
    showStatus("Kamera error", ST77XX_RED);
    showBody("Kamera tidak terdeteksi.\nPeriksa kabel dan pin.");
    delay(2200);
    return;
  }
  showStatus("Mengambil foto", ST77XX_CYAN);
  camera_fb_t* frame = esp_camera_fb_get();
  if (frame != nullptr) esp_camera_fb_return(frame); // discard the first stale frame
  frame = esp_camera_fb_get();
  if (frame == nullptr) {
    showStatus("Foto gagal", ST77XX_RED);
    showBody("Kamera tidak mengirim foto.");
    delay(1800);
    return;
  }

  const uint8_t* image = frame->buf;
  const size_t imageLength = frame->len;
  showStatus(micOK ? "Tanya? 5 dtk" : "Menganalisis foto", ST77XX_GREEN);
  showBody(micOK ? "Foto diambil.\n\nTanyakan sesuatu,\natau diam untuk\nmenjelaskan foto." :
                    "Foto diambil.\nSedang dianalisis.", ST77XX_CYAN);

  int samples = 0;
  if (micOK) {
    flushMic();
    samples = recordSpeech(5000, false);
  }
  String answer;
  if (samples > 0) {
    writeWavHeader(pcmBuf, static_cast<uint32_t>(samples) * 2);
    answer = askLLM(pcmBuf, 44 + static_cast<size_t>(samples) * 2,
                    image, imageLength,
                    "Ini foto dari kamera. Jawab pertanyaan pengguna tentang foto ini secara singkat.");
  } else {
    answer = askLLM(nullptr, 0, image, imageLength,
                    "Jelaskan apa yang terlihat di foto ini secara singkat.");
  }
  esp_camera_fb_return(frame);
  showStatus("Jawaban", ST77XX_GREEN);
  showAnswer(answer);
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH);

  SPI.begin(LCD_SCK, -1, LCD_MOSI, -1);
  tft.init(240, 240, SPI_MODE3);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  showStatus("Memulai", ST77XX_WHITE);

  memoryOK = psramFound();
  if (!memoryOK) {
    showStatus("PSRAM mati", ST77XX_RED);
    showBody("Aktifkan PSRAM pada\npengaturan board,\nlalu unggah ulang.", ST77XX_RED);
    Serial.println("ERROR: PSRAM is required for camera/audio buffers.");
  } else {
    pcmBuf = static_cast<uint8_t*>(ps_malloc(44 + MAX_SAMPLES * sizeof(int16_t)));
    if (pcmBuf != nullptr) pcm = reinterpret_cast<int16_t*>(pcmBuf + 44);
    else {
      showStatus("Memori error", ST77XX_RED);
      showBody("PSRAM tidak cukup.\nFoto dan suara tidak\ndapat digunakan.", ST77XX_RED);
      Serial.println("ERROR: Could not allocate audio buffer.");
    }
    camOK = cameraInit();
    Serial.println(camOK ? "Camera initialized." : "Camera initialization failed.");
  }

  prefs.begin("cfg", false);
  cfgSsid = prefs.getString("ssid", "");
  cfgPass = prefs.getString("pass", "");
  cfgKey = prefs.getString("gkey", "");
  cfgModel = prefs.getString("gmodel", DEFAULT_MODEL);
  if (cfgModel.length() == 0) cfgModel = DEFAULT_MODEL;

  if (cfgSsid.length() == 0 || !connectWiFi() || cfgKey.length() < 10) runPortal();

  if (memoryOK && pcm != nullptr) {
    micOK = micInit();
    Serial.println(micOK ? "Microphone initialized." : "Microphone initialization failed.");
  }
  showStatus("Siap", ST77XX_GREEN);
  if (!memoryOK || pcm == nullptr) {
    showBody("PSRAM error.\nPerangkat tidak dapat\nmerekam suara.", ST77XX_RED);
  } else if (!micOK && !camOK) {
    showBody("Mic dan kamera gagal.\nPeriksa pin perangkat.", ST77XX_RED);
  } else if (!micOK) {
    showBody("Mic gagal. Tekan BOOT\nuntuk mencoba foto.", ST77XX_CYAN);
  } else if (!camOK) {
    showBody("Bicara ke mikrofon.\nKamera tidak tersedia.", ST77XX_CYAN);
  } else {
    showBody("Wi-Fi tersambung.\nBicara ke mikrofon,\natau tekan BOOT\nuntuk mengambil foto.", ST77XX_CYAN);
  }
  delay(1200);
}

void loop() {
  if (portalRunning) return;
  if (WiFi.status() != WL_CONNECTED) {
    showStatus("WiFi putus", ST77XX_RED);
    WiFi.reconnect();
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 12000) delay(250);
    if (WiFi.status() != WL_CONNECTED) {
      showBody("Wi-Fi belum tersambung.\nTahan BOOT 3 dtk\nuntuk setup ulang.", ST77XX_RED);
      delay(800);
    }
    return;
  }

  if (!micOK || pcm == nullptr) {
    if (checkButton() == 1) doPhoto();
    delay(40);
    return;
  }

  showStatus("Mendengarkan", ST77XX_GREEN);
  flushMic();
  const int samples = recordSpeech(0, true);
  if (samples == -1) {
    doPhoto();
    return;
  }
  if (samples > 0) {
    writeWavHeader(pcmBuf, static_cast<uint32_t>(samples) * 2);
    String answer = askLLM(pcmBuf, 44 + static_cast<size_t>(samples) * 2,
                           nullptr, 0,
                           "Jawab langsung pertanyaan atau permintaan pengguna secara singkat.");
    showStatus("Jawaban", ST77XX_GREEN);
    showAnswer(answer);
  }
}
