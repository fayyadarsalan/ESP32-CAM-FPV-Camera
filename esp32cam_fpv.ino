/**
 * ============================================================
 *  ESP32-CAM FPV FIRMWARE  —  OV3660 Low-Latency Streamer
 *  Optimised for minimal latency on home WiFi networks
 *  
 *  Board  : AI-Thinker ESP32-CAM  (ESP32-S)
 *  Sensor : OV3660
 *  Stream : MJPEG over HTTP  (sub-50 ms typical latency)
 *  Viewer : Built-in web UI + raw stream endpoint
 * ============================================================
 *
 *  SETUP INSTRUCTIONS
 *  ------------------
 *  1. Install Arduino IDE 2.x
 *  2. Add ESP32 board package (Espressif):
 *       https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *  3. Board: "AI Thinker ESP32-CAM"
 *  4. Set your WiFi credentials below (WIFI_SSID / WIFI_PASS)
 *  5. Upload with FTDI adapter — GPIO0 → GND during flash,
 *     remove wire after upload, press RESET
 *  6. Open Serial Monitor @ 115200 to see the stream URL
 * ============================================================
 */

// ─── USER CONFIGURATION ────────────────────────────────────
#define WIFI_SSID   "YOUR_WIFI_SSID"     // <-- change this
#define WIFI_PASS   "YOUR_WIFI_PASSWORD" // <-- change this
// ───────────────────────────────────────────────────────────

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ─── AI-THINKER ESP32-CAM PIN MAP ──────────────────────────
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22
#define LED_GPIO_NUM      4  // onboard flash LED
// ───────────────────────────────────────────────────────────

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %lld\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// ─── CAMERA INIT ───────────────────────────────────────────
bool initCamera() {
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;

    // 20 MHz XCLK — sweet spot for OV3660 low-latency
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // Use PSRAM for larger frame buffers
    if (psramFound()) {
        config.frame_size    = FRAMESIZE_VGA;   // 640×480 — best FPV balance
        config.jpeg_quality  = 10;              // 0-63, lower = better quality
        config.fb_count      = 2;               // double-buffer for smoothness
        config.grab_mode     = CAMERA_GRAB_LATEST; // always grab freshest frame
    } else {
        config.frame_size    = FRAMESIZE_QVGA;  // fallback if no PSRAM
        config.jpeg_quality  = 12;
        config.fb_count      = 1;
        config.grab_mode     = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }

    // ── OV3660-specific tuning for FPV ──
    sensor_t* s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        // Flip image right-way-up (OV3660 default is inverted)
        s->set_vflip(s, 1);
        s->set_hmirror(s, 0);

        // Brightness / contrast for outdoor RC driving
        s->set_brightness(s, 1);     // slight boost  (-2 to 2)
        s->set_contrast(s, 1);       // +1 contrast   (-2 to 2)
        s->set_saturation(s, 0);     // neutral colour (-2 to 2)

        // Auto-exposure & white balance
        s->set_exposure_ctrl(s, 1);  // AEC on
        s->set_aec2(s, 1);           // AEC DSP on
        s->set_ae_level(s, 0);       // neutral AE target
        s->set_awb_gain(s, 1);       // AWB gain on
        s->set_whitebal(s, 1);       // AWB on

        // Noise reduction — off for speed
        s->set_dcw(s, 1);            // downsize crop
        s->set_bpc(s, 0);            // bad pixel correction off
        s->set_wpc(s, 1);            // white pixel correction on

        // Gain & shutter for fast-moving subjects
        s->set_agc_gain(s, 0);       // max gain = 0 (auto)
        s->set_gainceiling(s, (gainceiling_t)2); // cap at 4x

        Serial.println("[CAM] OV3660 tuning applied");
    }

    Serial.println("[CAM] Camera initialised");
    return true;
}

// ─── MJPEG STREAM HANDLER ──────────────────────────────────
static esp_err_t streamHandler(httpd_req_t* req) {
    camera_fb_t* fb = NULL;
    char part_buf[128];
    esp_err_t res = ESP_OK;

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    // Disable Nagle — critical for low latency
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");

    Serial.println("[STREAM] Client connected");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[STREAM] Frame capture failed");
            res = ESP_FAIL;
            break;
        }

        int64_t ts = esp_timer_get_time();

        // Write boundary
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        // Write part header with content-length + timestamp
        size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, fb->len, ts);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        // Write JPEG payload
        res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;
    }

    Serial.println("[STREAM] Client disconnected");
    return res;
}

// ─── CAPTURE (SINGLE JPEG) HANDLER ────────────────────────
static esp_err_t captureHandler(httpd_req_t* req) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return ESP_OK;
}

// ─── STATUS JSON HANDLER ───────────────────────────────────
static esp_err_t statusHandler(httpd_req_t* req) {
    sensor_t* s = esp_camera_sensor_get();
    char json[512];
    snprintf(json, sizeof(json),
        "{\"psram\":%s,\"framesize\":%d,\"quality\":%d,"
        "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
        "\"vflip\":%d,\"hmirror\":%d,\"agc_gain\":%d,"
        "\"free_heap\":%lu}",
        psramFound() ? "true" : "false",
        s->status.framesize,
        s->status.quality,
        s->status.brightness,
        s->status.contrast,
        s->status.saturation,
        s->status.vflip,
        s->status.hmirror,
        s->status.agc_gain,
        (unsigned long)ESP.getFreeHeap()
    );
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// ─── CONTROL HANDLER  /control?var=xxx&val=yyy ─────────────
static esp_err_t controlHandler(httpd_req_t* req) {
    char buf[64];
    char var[32], val[8];
    int  ival = 0;

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    if (httpd_query_key_value(buf, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(buf, "val", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    ival = atoi(val);

    sensor_t* s = esp_camera_sensor_get();
    int res = 0;

    if      (!strcmp(var, "framesize"))   res = s->set_framesize(s, (framesize_t)ival);
    else if (!strcmp(var, "quality"))     res = s->set_quality(s, ival);
    else if (!strcmp(var, "brightness"))  res = s->set_brightness(s, ival);
    else if (!strcmp(var, "contrast"))    res = s->set_contrast(s, ival);
    else if (!strcmp(var, "saturation"))  res = s->set_saturation(s, ival);
    else if (!strcmp(var, "vflip"))       res = s->set_vflip(s, ival);
    else if (!strcmp(var, "hmirror"))     res = s->set_hmirror(s, ival);
    else if (!strcmp(var, "agc_gain"))    res = s->set_agc_gain(s, ival);
    else if (!strcmp(var, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)ival);
    else if (!strcmp(var, "awb"))         res = s->set_whitebal(s, ival);
    else if (!strcmp(var, "aec"))         res = s->set_exposure_ctrl(s, ival);
    else if (!strcmp(var, "ae_level"))    res = s->set_ae_level(s, ival);
    else if (!strcmp(var, "flash")) {
        digitalWrite(LED_GPIO_NUM, ival ? HIGH : LOW);
    }
    else { httpd_resp_send_404(req); return ESP_FAIL; }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, res == 0 ? "OK" : "FAIL", -1);
    return ESP_OK;
}

// ─── WEB UI HANDLER ────────────────────────────────────────
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[]   asm("_binary_index_html_gz_end");

static esp_err_t indexHandler(httpd_req_t* req) {
    // Serve the embedded gzipped HTML viewer
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req,
        (const char*)index_html_gz_start,
        index_html_gz_end - index_html_gz_start);
    return ESP_OK;
}

// ─── SERVER STARTUP ────────────────────────────────────────
void startStreamServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Stream server on port 81
    config.server_port       = 81;
    config.ctrl_port         = 32769;
    config.max_uri_handlers  = 4;
    config.max_open_sockets  = 3;
    config.stack_size        = 8192;
    config.send_wait_timeout = 5;
    config.recv_wait_timeout = 5;
    config.lru_purge_enable  = true;

    httpd_uri_t stream_uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = streamHandler,
        .user_ctx = NULL
    };

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("[SERVER] Stream server started on :81");
    }
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port       = 80;
    config.ctrl_port         = 32768;
    config.max_uri_handlers  = 8;
    config.max_open_sockets  = 5;
    config.stack_size        = 8192;
    config.lru_purge_enable  = true;

    httpd_uri_t index_uri    = { "/",        HTTP_GET, indexHandler,   NULL };
    httpd_uri_t capture_uri  = { "/capture", HTTP_GET, captureHandler, NULL };
    httpd_uri_t status_uri   = { "/status",  HTTP_GET, statusHandler,  NULL };
    httpd_uri_t control_uri  = { "/control", HTTP_GET, controlHandler, NULL };

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &control_uri);
        Serial.println("[SERVER] Camera server started on :80");
    }
}

// ─── SETUP ─────────────────────────────────────────────────
void setup() {
    // Disable brownout detector (camera draws heavy current bursts)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    Serial.println("\n[BOOT] ESP32-CAM FPV Firmware v1.0");

    // Flash LED pin
    pinMode(LED_GPIO_NUM, OUTPUT);
    digitalWrite(LED_GPIO_NUM, LOW);

    // Init camera
    if (!initCamera()) {
        Serial.println("[BOOT] Camera init failed — halting");
        while (true) {
            digitalWrite(LED_GPIO_NUM, HIGH); delay(100);
            digitalWrite(LED_GPIO_NUM, LOW);  delay(100);
        }
    }

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);          // disable WiFi modem sleep → lower latency
    WiFi.setTxPower(WIFI_POWER_19_5dBm); // max TX power for range
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WIFI] Connection failed — restarting");
        ESP.restart();
    }

    Serial.println();
    Serial.printf("[WIFI] Connected! RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("[WIFI] IP Address: %s\n", WiFi.localIP().toString().c_str());

    startCameraServer();
    startStreamServer();

    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.printf( "║  FPV Viewer  http://%s       \n", WiFi.localIP().toString().c_str());
    Serial.printf( "║  Raw Stream  http://%s:81/stream\n", WiFi.localIP().toString().c_str());
    Serial.printf( "║  Snapshot    http://%s/capture \n", WiFi.localIP().toString().c_str());
    Serial.println("╚══════════════════════════════════════╝");

    // Flash LED 3× to signal ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_GPIO_NUM, HIGH); delay(80);
        digitalWrite(LED_GPIO_NUM, LOW);  delay(80);
    }
}

// ─── LOOP ──────────────────────────────────────────────────
void loop() {
    // Watchdog: reconnect if WiFi drops
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 5000) {
        lastCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WIFI] Connection lost — reconnecting...");
            WiFi.reconnect();
        }
    }
    delay(100);
}
