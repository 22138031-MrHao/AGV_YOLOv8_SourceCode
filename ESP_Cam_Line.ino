
#include "esp_camera.h"
#include <Arduino.h>
#include <math.h>

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define UART_BAUD 115200

static const framesize_t CAM_FRAME_SIZE   = FRAMESIZE_QQVGA; // 160x120
static const pixformat_t CAM_PIXEL_FORMAT = PIXFORMAT_GRAYSCALE;

// =====================================================
// LINE DETECTOR – vùng quét (ưu tiên gần xe)
// =====================================================
static const int NUM_ROWS = 6;
static const float ROW_RATIOS[NUM_ROWS] = {
  0.84f, 0.78f, 0.72f, 0.66f,   // bumper – gần xe nhất (trọng số cao)
  0.54f, 0.42f                   // preview – xa hơn (feedforward)
};
static const float ROW_WEIGHTS[NUM_ROWS] = {
  1.60f, 1.40f, 1.20f, 1.00f,
  0.85f, 0.70f
};
static const int ADAPTIVE_DELTA     = 20;
static const int MIN_SEGMENT_WIDTH  = 6;
static const int MAX_SEGMENT_WIDTH  = 100;
static const int MAX_JUMP_FROM_PREV = 42;

// =====================================================
// STOP MARKER
// =====================================================
static const int STOP_Y_START          = 28;
static const int STOP_Y_END            = 88;
static const int STOP_MIN_BAR_WIDTH    = 55;
static const int STOP_MAX_CENTER_OFFSET= 26;
static const int STOP_MIN_BAR_HEIGHT   = 2;
static const int STOP_MIN_GAP          = 6;

static int prevCx[NUM_ROWS] = { -1, -1, -1, -1, -1, -1 };

static float filtBumper  = 0.0f;
static float filtPreview = 0.0f;
static float filtWidth   = 24.0f;

static int detOut = 0, confOut = 0;
static int bumperErrOut = 0, previewErrOut = 0, widthOut = 0;
static int stopDetOut = 0, stopStrengthOut = 0;
static int stopY1Out = -1, stopY2Out = -1;

static const uint32_t SEND_INTERVAL_MS = 25; // Gửi mỗi 25ms (40Hz)
static uint32_t lastSendMs = 0;

// =====================================================
// HELPERS
// =====================================================
int clampInt(int v, int mn, int mx) {
  if (v < mn) return mn;
  if (v > mx) return mx;
  return v;
}

// =====================================================
// CAMERA INIT (NƠI SỬA LỖI TỤT FPS)
// =====================================================
bool setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = CAM_FRAME_SIZE;
  config.pixel_format = CAM_PIXEL_FORMAT;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 2; // 2 buffer để grab_latest hoạt động tốt hơn

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("# Camera init failed: 0x%x\n", (uint32_t)err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 2);
    s->set_saturation(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 1);       // Camera gắn ngược
    
    // [ĐÃ SỬA] - ÉP FRAME RATE CAO, TẮT TỰ ĐỘNG PHƠI SÁNG
    s->set_exposure_ctrl(s, 0); // 0 = Tắt Auto Exposure
    s->set_aec2(s, 0);          // 0 = Tắt AEC2 (Thuật toán làm tụt FPS)
    
    // Set độ sáng thủ công. Mức 300 là mặc định tốt. 
    // Nếu nhà màng TỐI -> Tăng lên 400 hoặc 500 (Tối đa ~1200)
    // Nếu nhà màng NẮNG CHÓI -> Giảm xuống 150 hoặc 200
    s->set_aec_value(s, 300);   
  }
  return true;
}

// =====================================================
// DARK SEGMENT ON A ROW
// =====================================================
bool detectBestSegmentOnRow(
  const uint8_t *img, int w, int y, int prevCenter,
  int &bestCx, int &bestWidth
) {
  long rowSum = 0;
  for (int x = 0; x < w; x++) rowSum += img[y * w + x];
  int meanRow = (int)(rowSum / w);
  int threshold = clampInt(meanRow - ADAPTIVE_DELTA, 18, 180);
  int imageCenter = w / 2;
  bool inSeg = false, found = false;
  int segStart = 0;
  float bestScore = -1e9f;
  
  for (int x = 0; x <= w; x++) {
    bool dark = (x < w) && (img[y * w + x] < threshold);
    if (dark && !inSeg)  { inSeg = true; segStart = x; }

    if ((!dark || x == w) && inSeg) {
      int segEnd = x - 1;
      inSeg = false;
      int segWidth = segEnd - segStart + 1;
      if (segWidth < MIN_SEGMENT_WIDTH || segWidth > MAX_SEGMENT_WIDTH) continue;
      long segSum = 0;
      for (int xi = segStart; xi <= segEnd; xi++) segSum += img[y * w + xi];
      float segMean = (float)segSum / segWidth;
      int cx = (segStart + segEnd) / 2;
      float darkness = (float)(threshold - segMean);
      float widthBonus    =  0.05f * segWidth;
      float centerPenalty =  0.02f * fabsf((float)cx - imageCenter);
      float historyPenalty = (prevCenter >= 0) ? 0.09f * fabsf((float)cx - prevCenter) : 0.0f;
      float score = 5.8f * darkness + widthBonus - centerPenalty - historyPenalty;
      
      if (!found || score > bestScore) {
        found = true; bestScore = score;
        bestCx = cx; bestWidth = segWidth;
      }
    }
  }
  return found;
}

// =====================================================
// LINE DETECT
// =====================================================
bool detectLineGeometry(camera_fb_t *fb) {
  if (!fb || fb->format != PIXFORMAT_GRAYSCALE) return false;
  const int w = fb->width, h = fb->height, imageCenter = w / 2;
  const uint8_t *img = fb->buf;

  float err[NUM_ROWS];
  int width[NUM_ROWS]; bool valid[NUM_ROWS];
  int validCount = 0;

  for (int i = 0; i < NUM_ROWS; i++) {
    int y = clampInt((int)roundf(h * ROW_RATIOS[i]), 0, h - 1);
    int cx = 0, segWidth = 0;
    bool ok = detectBestSegmentOnRow(img, w, y, prevCx[i], cx, segWidth);

    valid[i] = false;
    err[i] = 0.0f; width[i] = 0;
    if (!ok) continue;
    if (prevCx[i] >= 0 && abs(cx - prevCx[i]) > MAX_JUMP_FROM_PREV) continue;
    
    prevCx[i]  = cx;
    err[i]     = (float)(cx - imageCenter);
    width[i]   = segWidth;
    valid[i]   = true;
    validCount++;
  }

  confOut = validCount;
  if (validCount < 3) {
    detOut = 0; bumperErrOut = 0; previewErrOut = 0; widthOut = 0;
    return false;
  }

  float bumperSum = 0, bumperW = 0, previewSum = 0, previewW = 0;
  float widthSum = 0, widthW = 0;

  for (int i = 0; i < NUM_ROWS; i++) {
    if (!valid[i]) continue;
    widthSum += ROW_WEIGHTS[i] * width[i];
    widthW   += ROW_WEIGHTS[i];
    if (i <= 3) { bumperSum  += ROW_WEIGHTS[i] * err[i]; bumperW  += ROW_WEIGHTS[i]; }
    else        { previewSum += ROW_WEIGHTS[i] * err[i]; previewW += ROW_WEIGHTS[i]; }
  }

  float bumperErr  = bumperSum  / bumperW;
  float previewErr = (previewW > 0) ? previewSum / previewW : bumperErr;
  float avgWidth   = widthSum   / widthW;
  
  filtBumper  = 0.65f * filtBumper  + 0.35f * bumperErr;
  filtPreview = 0.72f * filtPreview + 0.28f * previewErr;
  filtWidth   = 0.80f * filtWidth   + 0.20f * avgWidth;
  
  detOut       = 1;
  bumperErrOut  = (int)roundf(filtBumper);
  previewErrOut = (int)roundf(filtPreview);
  widthOut      = (int)roundf(filtWidth);
  return true;
}

// =====================================================
// STOP DETECTOR
// =====================================================
bool detectStopMarker(camera_fb_t *fb) {
  if (!fb || fb->format != PIXFORMAT_GRAYSCALE) return false;
  const int w = fb->width;
  const uint8_t *img = fb->buf;
  const int imageCenter = w / 2;

  stopDetOut = 0;
  stopStrengthOut = 0; stopY1Out = -1; stopY2Out = -1;

  bool rowHit[120]; int rowWidth[120]; int rowCenter[120];
  for (int y = 0; y < 120; y++) { rowHit[y] = false; rowWidth[y] = 0; rowCenter[y] = 0; }

  for (int y = STOP_Y_START; y <= STOP_Y_END; y++) {
    long rowSum = 0;
    for (int x = 0; x < w; x++) rowSum += img[y * w + x];
    int meanRow = (int)(rowSum / w);
    int threshold = clampInt(meanRow - ADAPTIVE_DELTA, 18, 180);

    bool inSeg = false;
    int segStart = 0, bestWidth = 0, bestCx = -999;
    for (int x = 0; x <= w; x++) {
      bool dark = (x < w) && (img[y * w + x] < threshold);
      if (dark && !inSeg) { inSeg = true; segStart = x; }
      if ((!dark || x == w) && inSeg) {
        int segEnd = x - 1;
        inSeg = false;
        int segWidth = segEnd - segStart + 1;
        int cx = (segStart + segEnd) / 2;
        if (segWidth >= STOP_MIN_BAR_WIDTH && abs(cx - imageCenter) <= STOP_MAX_CENTER_OFFSET) {
          if (segWidth > bestWidth) { bestWidth = segWidth; bestCx = cx; }
        }
      }
    }
    if (bestWidth > 0) { rowHit[y] = true; rowWidth[y] = bestWidth; rowCenter[y] = bestCx; }
  }

  struct Blob { int yStart, yEnd, maxWidth, avgCenter, samples; };
  Blob blobs[8]; int blobCount = 0;

  int y = STOP_Y_START;
  while (y <= STOP_Y_END) {
    if (!rowHit[y]) { y++; continue; }
    int ys = y, ye = y, maxW = rowWidth[y]; long cxSum = rowCenter[y];
    int n = 1; y++;
    while (y <= STOP_Y_END && rowHit[y]) {
      ye = y;
      if (rowWidth[y] > maxW) maxW = rowWidth[y];
      cxSum += rowCenter[y]; n++; y++;
    }
    if ((ye - ys + 1) >= STOP_MIN_BAR_HEIGHT && blobCount < 8) {
      blobs[blobCount++] = { ys, ye, maxW, (int)(cxSum / n), n };
    }
  }

  if (blobCount < 2) return false;

  int bestI = -1, bestJ = -1, bestScore = -999999;
  for (int i = 0; i < blobCount; i++) {
    for (int j = i + 1; j < blobCount; j++) {
      int gap = blobs[j].yStart - blobs[i].yEnd - 1;
      if (gap < STOP_MIN_GAP) continue;
      int centerDiff = abs(blobs[i].avgCenter - blobs[j].avgCenter);
      if (centerDiff > 18) continue;
      int score = blobs[i].maxWidth + blobs[j].maxWidth
                + 2 * (blobs[i].samples + blobs[j].samples) - 2 * centerDiff;
      if (score > bestScore) { bestScore = score; bestI = i; bestJ = j; }
    }
  }

  if (bestI < 0) return false;
  stopDetOut = 1; stopStrengthOut = bestScore;
  stopY1Out = (blobs[bestI].yStart + blobs[bestI].yEnd) / 2;
  stopY2Out = (blobs[bestJ].yStart + blobs[bestJ].yEnd) / 2;
  return true;
}

// =====================================================
// SEND
// =====================================================
void sendLinePacket() {
  Serial.print("LINEGATE,"); Serial.print(detOut);
  Serial.print(",");         Serial.print(confOut);
  Serial.print(",");         Serial.print(bumperErrOut);
  Serial.print(",");         Serial.print(previewErrOut);
  Serial.print(",");         Serial.println(widthOut);
}
void sendStopPacket() {
  Serial.print("STOPMK,");  Serial.print(stopDetOut);
  Serial.print(",");         Serial.print(stopStrengthOut);
  Serial.print(",");         Serial.print(stopY1Out);
  Serial.print(",");         Serial.println(stopY2Out);
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(UART_BAUD);
  delay(1000);
  if (!setupCamera()) { while (true) delay(1000); }
  Serial.println("# cam_optimized boot OK (Manual Exp, HIGH FPS)");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
      detOut = 0; confOut = 0; bumperErrOut = 0; previewErrOut = 0;
      sendLinePacket(); sendStopPacket(); lastSendMs = millis();
    }
    delay(3);
    return;
  }

  detectLineGeometry(fb);
  detectStopMarker(fb);
  esp_camera_fb_return(fb);

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    sendLinePacket(); sendStopPacket(); lastSendMs = millis();
  }
}