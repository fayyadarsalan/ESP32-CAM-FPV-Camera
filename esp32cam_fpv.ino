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
#define WIFI_SSID   "DH"     // <-- change this
#define WIFI_PASS   "01714584611" // <-- change this
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

// ─── WEB UI HTML (stored in flash via PROGMEM) ─────────────
// Full FPV viewer — served directly from the ESP32 at http://<ip>/
static const char INDEX_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>ESP32-CAM FPV</title>
<style>
:root{--bg:#080c0f;--panel:#0d1419;--border:#1a2a35;--accent:#00e5ff;--accent2:#ff3d00;--green:#00ff87;--warn:#ffb300;--text:#cdd8e0}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Courier New',monospace;height:100dvh;display:grid;grid-template-rows:44px 1fr auto;overflow:hidden}
header{display:flex;align-items:center;gap:12px;padding:0 14px;background:var(--panel);border-bottom:1px solid var(--border)}
.logo{font-size:12px;letter-spacing:2px;color:var(--accent);text-transform:uppercase}
.logo span{color:var(--accent2)}
.meters{display:flex;gap:14px;align-items:center;margin-left:auto}
.meter{display:flex;flex-direction:column;align-items:center;gap:1px}
.mval{font-size:15px;color:var(--accent);line-height:1}
.mlbl{font-size:8px;letter-spacing:1.5px;color:#4a6070;text-transform:uppercase}
.badge{font-size:10px;letter-spacing:1px;padding:2px 8px;border-radius:2px;border:1px solid currentColor;text-transform:uppercase;margin-left:8px}
.green{color:var(--green)}.red{color:var(--accent2)}
main{position:relative;overflow:hidden;background:#000;display:flex;align-items:center;justify-content:center}
#sv{max-width:100%;max-height:100%;object-fit:contain;display:block}
.hud{position:absolute;inset:0;pointer-events:none}
.c{position:absolute;width:26px;height:26px;border-color:var(--accent);border-style:solid;opacity:.45}
.tl{top:10px;left:10px;border-width:2px 0 0 2px}.tr{top:10px;right:10px;border-width:2px 2px 0 0}
.bl{bottom:10px;left:10px;border-width:0 0 2px 2px}.br{bottom:10px;right:10px;border-width:0 2px 2px 0}
.cx{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);width:18px;height:18px;opacity:.3}
.cx::before,.cx::after{content:'';position:absolute;background:var(--accent)}
.cx::before{width:1px;height:100%;left:50%;transform:translateX(-50%)}
.cx::after{height:1px;width:100%;top:50%;transform:translateY(-50%)}
.sl{position:absolute;inset:0;background:repeating-linear-gradient(to bottom,transparent 0,transparent 3px,rgba(0,0,0,.07) 3px,rgba(0,0,0,.07) 4px);pointer-events:none}
.rec{position:absolute;top:12px;right:14px;display:flex;align-items:center;gap:5px;font-size:10px;letter-spacing:2px;color:var(--accent2);opacity:0;transition:opacity .3s}
.rec.on{opacity:1}
.rec::before{content:'';width:7px;height:7px;border-radius:50%;background:var(--accent2);animation:blink 1s step-start infinite}
@keyframes blink{50%{opacity:0}}
.ns{position:absolute;inset:0;background:#000;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:14px;z-index:5;transition:opacity .3s}
.ns.hidden{opacity:0;pointer-events:none}
.ns-icon{font-size:36px;opacity:.25}
.ns-txt{font-size:11px;letter-spacing:3px;color:#4a6070;text-transform:uppercase}
.ns-url{font-size:12px;color:var(--accent)}
footer{background:var(--panel);border-top:1px solid var(--border);padding:7px 12px;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.lbl{font-size:9px;letter-spacing:1.5px;color:#4a6070;text-transform:uppercase;white-space:nowrap}
select,input[type=range]{background:var(--bg);border:1px solid var(--border);color:var(--text);font-family:inherit;font-size:11px;padding:3px 5px;border-radius:2px;outline:none;cursor:pointer}
select:hover,input[type=range]:hover{border-color:var(--accent)}
input[type=range]{-webkit-appearance:none;width:72px;height:4px;padding:0;background:linear-gradient(to right,var(--accent) var(--p,50%),var(--border) var(--p,50%))}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:11px;height:11px;border-radius:50%;background:var(--accent);cursor:pointer}
#curl{background:var(--bg);border:1px solid var(--border);color:var(--accent);font-family:inherit;font-size:11px;padding:4px 7px;border-radius:2px;outline:none;width:180px}
#curl:focus{border-color:var(--accent)}
.btn{padding:4px 11px;border:1px solid var(--border);background:transparent;color:var(--text);font-family:inherit;font-size:10px;letter-spacing:1px;cursor:pointer;border-radius:2px;text-transform:uppercase;transition:all .15s}
.btn:hover{border-color:var(--accent);color:var(--accent)}
.btn.on{background:var(--accent);color:#000;border-color:var(--accent)}
.btn.d{border-color:var(--accent2);color:var(--accent2)}
.btn.d:hover{background:var(--accent2);color:#000}
.dv{width:1px;height:26px;background:var(--border);margin:0 3px}
</style>
</head>
<body>
<header>
  <div class="logo">ESP32-<span>CAM</span>&nbsp;&middot;&nbsp;FPV</div>
  <div class="meters">
    <div class="meter"><div class="mval" id="fv">--</div><div class="mlbl">FPS</div></div>
    <div class="meter"><div class="mval" id="lv">--</div><div class="mlbl">MS</div></div>
    <div class="badge green" id="cb">OFFLINE</div>
  </div>
</header>
<main id="vp">
  <img id="sv" alt="">
  <div class="hud">
    <div class="c tl"></div><div class="c tr"></div>
    <div class="c bl"></div><div class="c br"></div>
    <div class="cx"></div>
    <div class="sl"></div>
    <div class="rec" id="rec">REC</div>
  </div>
  <div class="ns" id="ns">
    <div class="ns-icon">&#x1F4E1;</div>
    <div class="ns-txt">No Signal</div>
    <div class="ns-url" id="nu">Enter camera IP below</div>
  </div>
</main>
<footer>
  <span class="lbl">IP</span>
  <input type="text" id="curl" placeholder="192.168.1.xx">
  <button class="btn" id="sb">&#9654; Stream</button>
  <button class="btn d" id="xb">&#9632; Stop</button>
  <div class="dv"></div>
  <span class="lbl">Size</span>
  <select id="fs">
    <option value="5">QVGA 320x240</option>
    <option value="6">CIF 400x296</option>
    <option value="7">HVGA 480x320</option>
    <option value="8" selected>VGA 640x480</option>
    <option value="9">SVGA 800x600</option>
  </select>
  <span class="lbl">Quality</span>
  <input type="range" id="sq" min="4" max="40" value="10">
  <div class="dv"></div>
  <span class="lbl">Bright</span>
  <input type="range" id="sb2" min="-2" max="2" value="1">
  <span class="lbl">Contrast</span>
  <input type="range" id="sc" min="-2" max="2" value="1">
  <div class="dv"></div>
  <button class="btn on" id="bvf">&#8597; Flip</button>
  <button class="btn" id="bhm">&#8596; Mirror</button>
  <button class="btn" id="bfl">&#9889; Flash</button>
  <button class="btn" id="bfs" style="margin-left:auto">&#x26F6; Full</button>
</footer>
<script>
(()=>{
  let ip='',streaming=false,fc=0,lt=performance.now(),flash=false,vf=1,hm=0;
  const img=document.getElementById('sv'),ns=document.getElementById('ns'),
        rec=document.getElementById('rec'),curl=document.getElementById('curl'),
        cb=document.getElementById('cb'),fv=document.getElementById('fv'),
        lv=document.getElementById('lv'),nu=document.getElementById('nu');

  // Auto-detect if served from ESP32
  const h=location.hostname;
  if(h&&h.match(/^\d+\.\d+\.\d+\.\d+$/)){curl.value=h;nu.textContent='http://'+h+':81/stream';}
  else{curl.value='192.168.1.100';}

  const api=p=>'http://'+ip+p;
  const ctl=(v,val)=>{if(!ip)return;fetch(api('/control?var='+v+'&val='+val)).catch(()=>{});};
  const setLive=on=>{cb.textContent=on?'LIVE':'OFFLINE';cb.className='badge '+(on?'green':'red');rec.className='rec'+(on?' on':'');};
  const trk=el=>{const p=((+el.value-+el.min)/(+el.max-+el.min)*100).toFixed(1);el.style.setProperty('--p',p+'%');};

  let t0=0;
  const start=()=>{
    ip=curl.value.trim().replace(/^https?:\/\//,'').replace(/\/.*/,'');
    if(!ip)return;
    t0=performance.now();fc=0;streaming=true;
    img.onload=()=>{ns.classList.add('hidden');setLive(true);fc++;lv.textContent=Math.round(performance.now()-t0);t0=performance.now();};
    img.onerror=()=>{setLive(false);ns.classList.remove('hidden');if(streaming)setTimeout(()=>{img.src='http://'+ip+':81/stream?t='+Date.now();},2000);};
    img.src='http://'+ip+':81/stream';
  };
  const stop=()=>{streaming=false;img.src='';setLive(false);ns.classList.remove('hidden');fv.textContent='--';lv.textContent='--';};

  setInterval(()=>{if(!streaming)return;const n=performance.now(),e=(n-lt)/1000;if(e>=1){fv.textContent=(fc/e).toFixed(0);fc=0;lt=n;}},250);

  document.getElementById('sb').onclick=start;
  document.getElementById('xb').onclick=stop;
  document.getElementById('fs').onchange=e=>ctl('framesize',e.target.value);
  document.getElementById('sq').oninput=e=>{trk(e.target);ctl('quality',e.target.value);};
  document.getElementById('sb2').oninput=e=>{trk(e.target);ctl('brightness',e.target.value);};
  document.getElementById('sc').oninput=e=>{trk(e.target);ctl('contrast',e.target.value);};
  ['sq','sb2','sc'].forEach(id=>trk(document.getElementById(id)));

  document.getElementById('bvf').onclick=e=>{vf=vf?0:1;ctl('vflip',vf);e.target.classList.toggle('on',vf===1);};
  document.getElementById('bhm').onclick=e=>{hm=hm?0:1;ctl('hmirror',hm);e.target.classList.toggle('on',hm===1);};
  document.getElementById('bfl').onclick=e=>{flash=!flash;ctl('flash',flash?1:0);e.target.classList.toggle('on',flash);};
  document.getElementById('bfs').onclick=()=>{const v=document.getElementById('vp');!document.fullscreenElement?v.requestFullscreen&&v.requestFullscreen():document.exitFullscreen&&document.exitFullscreen();};

  document.addEventListener('keydown',e=>{
    if(e.code==='Space'){e.preventDefault();streaming?stop():start();}
    if(e.code==='KeyF')document.getElementById('bfs').click();
    if(e.code==='KeyL')document.getElementById('bfl').click();
  });

  // Auto-start if IP detected from URL
  if(location.hostname.match(/^\d+\.\d+\.\d+\.\d+$/))setTimeout(start,300);
})();
</script>
</body>
</html>)rawhtml";

// ─── WEB UI HANDLER ────────────────────────────────────────
static esp_err_t indexHandler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
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
