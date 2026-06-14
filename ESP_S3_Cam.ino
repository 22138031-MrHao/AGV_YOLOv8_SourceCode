#include <Arduino.h>
#include "esp_camera.h"

// ===================================================
// CẤU HÌNH TỐC ĐỘ SERIAL TỐI ĐA (CÁP USB + GIAO TIẾP XE)
// ===================================================
const unsigned long LAPTOP_BAUD = 2000000; // Cáp USB nối máy tính
const unsigned long AGV_BAUD = 115200;     // Dây nối mạch ESP32 Main

#define S3_RX_PIN 1 
#define S3_TX_PIN 2 
HardwareSerial MainSerial(1); 

unsigned long lastCaptureTime = 0; 
const unsigned long COOLDOWN_TIME = 1000; 

// Chân Camera ESP32-S3 WROOM
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

void setup() {
  setCpuFrequencyMhz(240); 

  Serial.begin(LAPTOP_BAUD);
  MainSerial.begin(AGV_BAUD, SERIAL_8N1, S3_RX_PIN, S3_TX_PIN);

  Serial.setTimeout(5);
  MainSerial.setTimeout(5);

  delay(1000);

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; 
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    // =======================================================
    // BẢN GIẢM 1 BẬC: ĐỘ NÉT SXGA (1280x1024) - CÂN BẰNG TỐT NHẤT
    // =======================================================
    config.frame_size = FRAMESIZE_SXGA; 
    config.jpeg_quality = 10;           
    config.fb_count = 2;               
    config.grab_mode = CAMERA_GRAB_LATEST; 
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.println("# [LỖI] Khởi tạo Camera thất bại!");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_whitebal(s, 1);       
    s->set_awb_gain(s, 1);       
    s->set_exposure_ctrl(s, 1);  
    s->set_aec2(s, 1);           
    s->set_gain_ctrl(s, 1);      
  }
  
  Serial.println("# [ESP32-S3-CAM] He thong da khoi dong xong! San sang nhan lenh...");
}

void loop() {
  bool needCapture = false;

  // 1. NHẬN LỆNH TỪ XE AGV GỬI LÊN (Qua cổng MainSerial)
  while (MainSerial.available()) {
    String data = MainSerial.readStringUntil('\n');
    data.trim();
    if (data.indexOf("chup") >= 0 || data.indexOf("CHUP") >= 0) {
      needCapture = true;
      // --- BÁO CÁO TRẠNG THÁI LÊN LAPTOP ---
      Serial.println("\n# [ESP32-S3-CAM] ✅ DA NHAN LENH 'CHUP' TU ESP32 MAIN! Dang chup anh...");
    }
  }

  // 2. NHẬN LỆNH TỪ LAPTOP GỬI XUỐNG (Qua cổng Serial USB)
  while (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    // Nếu Laptop bấm chụp thủ công
    if (data.indexOf("chup") >= 0 || data.indexOf("CHUP") >= 0) {
      needCapture = true;
      Serial.println("\n# [ESP32-S3-CAM] ✅ DA NHAN LENH 'CHUP' THU CONG TU LAPTOP!");
    }
    
    // Giao thức trung chuyển: Bắt được chữ DONE từ Laptop -> Đẩy xuống ESP32 Main
    else if (data.indexOf("DONE") >= 0) {
      MainSerial.println("DONE"); 
      Serial.println("# [ESP32-S3-CAM] ⏩ DA CHUYEN TIEP LENH 'DONE' XUONG ESP32 MAIN!");
    }
  }

  // 3. THỰC THI CHỤP ẢNH
  if (needCapture) {
    if (millis() - lastCaptureTime < COOLDOWN_TIME) return;

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      MainSerial.println("{\"error\": \"Camera failed\"}");
      Serial.println("# [ESP32-S3-CAM] ❌ LOI: Khong the chup anh!");
      return;
    }

    // Gửi dữ liệu ảnh lên Laptop
    Serial.printf("START:%u\n", fb->len);
    Serial.write(fb->buf, fb->len);
    Serial.print("\nEND\n");
    Serial.flush(); 

    esp_camera_fb_return(fb);
    lastCaptureTime = millis();
  }
}