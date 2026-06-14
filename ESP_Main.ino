// =====================================================
// main_straight_v1.ino – AGV THẲNG CHUYÊN DỤNG (BẢN CHUẨN)
// - Giữ nguyên 100% sức mạnh bám line thẳng tuyệt đối.
// - Tích hợp 2 Servo quét 2 luống cây (Trái - Phải).
// - CẬP NHẬT: Góc tâm P100/T100, Đợi Servo quay 1.5s!
// =====================================================
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>  

// -------------------- PIN MAP --------------------
#define MPU_SDA 21
#define MPU_SCL 22
#define MPU_ADDR 0x68

#define MOTOR_L_RPWM 25
#define MOTOR_L_LPWM 26
#define MOTOR_R_RPWM 14
#define MOTOR_R_LPWM 12
#define MOTOR_ENABLE  2

#define ENC_L_A 34
#define ENC_R_A 32

// --- PIN GIAO TIẾP ---
#define CAM_UART_RX   16
#define CAM_UART_TX   17
#define CAM_UART_BAUD 115200
HardwareSerial LineCamSerial(2);

#define DISEASE_CAM_RX 35  
#define DISEASE_CAM_TX 33  
HardwareSerial DiseaseSerial(1);

// --- PIN SERVO CAMERA ---
#define PAN_PIN 5       // Servo quay trái/phải
#define TILT_PIN 23     // Servo ngẩng lên/xuống
Servo panServo;
Servo tiltServo;
// ----------------------------------------

#define PWM_FREQ       20000
#define PWM_RESOLUTION 8
#define PWM_MAX        255

// -------------------- CẢM BIẾN & THÔNG SỐ --------------------
#define IR_LEFT_PIN   27
#define IR_RIGHT_PIN  13
#define IR_DETECT_STATE HIGH

#define TARGET_RPM          24.0f  
#define LINE_STOP_TIMEOUT   3000
#define STARTUP_GRACE_MS    3000
#define GYRO_DIR_CORRECTION -1.0f

#define LINE_MSG_TIMEOUT_MS 200

// -------------------- TIMING & STATE --------------------
const uint32_t CONTROL_INTERVAL_US = 20000;
const uint32_t STATUS_INTERVAL_MS  = 200;

enum CarState { STATE_RUNNING, STATE_TASK, STATE_RESUMING };
CarState currentState  = STATE_RUNNING;
uint32_t taskStartTime = 0, resumeStartTime = 0;

// CỜ HIỆU ĐỢI CAMERA VÀ STATE PHỤ CHO SERVO
bool isCameraDone = false;
int taskSubState = 0; 

// =====================================================
// BIẾN TELEMETRY
// =====================================================
float p_term = 0.0f;
float i_term = 0.0f;
float d_term = 0.0f;
int current_dt_ms = 20;

// =====================================================
// ENCODER – TÍNH TOÁN VẬN TỐC (RPM)
// =====================================================
volatile long    encLCount = 0, encRCount = 0;
volatile uint32_t lastMicrosLA = 0, lastMicrosRA = 0;
const uint32_t   MIN_PULSE_US = 200;
float rpmL = 0.0f, rpmR = 0.0f;
const float encoderCPR_L = 749.4f, encoderCPR_R = 748.4f;

#define ENC_HIST_SIZE 5
long    hist_encL[ENC_HIST_SIZE] = {0};
long    hist_encR[ENC_HIST_SIZE] = {0};
uint8_t enc_idx = 0;

void IRAM_ATTR isrEncLA() {
  uint32_t now = micros();
  if (now - lastMicrosLA < MIN_PULSE_US) return;
  lastMicrosLA = now; encLCount++;
}
void IRAM_ATTR isrEncRA() {
  uint32_t now = micros();
  if (now - lastMicrosRA < MIN_PULSE_US) return;
  lastMicrosRA = now; encRCount++;
}

void updateSpeedEstimate(float dt_s) {
  long curL, curR;
  noInterrupts(); curL = encLCount; curR = encRCount; interrupts();

  long oldL = hist_encL[enc_idx];
  long oldR = hist_encR[enc_idx];
  hist_encL[enc_idx] = curL;
  hist_encR[enc_idx] = curR;
  enc_idx = (enc_idx + 1) % ENC_HIST_SIZE;

  float win = ENC_HIST_SIZE * dt_s;
  if (win > 0.01f) {
    float nL = ((curL - oldL) / encoderCPR_L) / win * 60.0f;
    float nR = ((curR - oldR) / encoderCPR_R) / win * 60.0f;
    rpmL = 0.72f * rpmL + 0.28f * max(0.0f, nL);
    rpmR = 0.72f * rpmR + 0.28f * max(0.0f, nR);
  }
}

// =====================================================
// MPU6050
// =====================================================
float yawAngle = 0.0f, gyroZ_offset = 0.0f;
float targetYaw = 0.0f, lastGoodYaw = 0.0f;

void setupMPU() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);

  Serial.println("# Hieu chuan Gyro...");
  long gz_sum = 0; int samples = 0; int16_t last_gz = 0;
  while (samples < 500) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    int16_t gz = (Wire.read() << 8) | Wire.read();
    if (abs(gz - last_gz) > 100 && samples > 0) { samples = 0; gz_sum = 0; delay(50); }
    else { gz_sum += gz; samples++; delay(2); }
    last_gz = gz;
  }
  gyroZ_offset = (float)gz_sum / 500.0f;
  Serial.println("# Gyro OK.");
}

void updateYaw(float dt_s) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_ADDR, 2, true) == 2) {
    int16_t gz_raw = (Wire.read() << 8) | Wire.read();
    float rate = (gz_raw - gyroZ_offset) / 131.0f;
    if (fabsf(rate) > 0.15f) yawAngle += rate * dt_s;
  }
}

// =====================================================
// UART CAMERA
// =====================================================
bool     lineDet = false;
int      lineConf = 0, bumperErrRx = 0, previewErrRx = 0;
uint32_t lastLineMsgMs = 0, lastLineSeenMs = 0, lastStopMarkerMs = 0;

void setupCamUART() {
  LineCamSerial.begin(CAM_UART_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);
  DiseaseSerial.begin(115200, SERIAL_8N1, DISEASE_CAM_RX, DISEASE_CAM_TX);
  DiseaseSerial.setTimeout(2);
}

void parseLinePacket(const String& msg) {
  int vals[5] = {}, idx = 0, start = 9;
  while (idx < 5) {
    int c = msg.indexOf(',', start);
    if (c < 0) { vals[idx++] = msg.substring(start).toInt(); break; }
    vals[idx++] = msg.substring(start, c).toInt();
    start = c + 1;
  }
  if (idx < 4) return;
  lineDet = vals[0]; lineConf = vals[1];
  bumperErrRx = vals[2]; previewErrRx = vals[3];
  lastLineMsgMs = millis();
  if (lineDet) lastLineSeenMs = millis();
}

void parseStopPacket(const String& msg) {
  int val = msg.substring(7, msg.indexOf(',', 7)).toInt();
  if (val > 0) lastStopMarkerMs = millis();
}

void updateCamUART() {
  while (LineCamSerial.available()) {
    String msg = LineCamSerial.readStringUntil('\n');
    msg.trim();
    if (msg.startsWith("LINEGATE,")) parseLinePacket(msg);
    else if (msg.startsWith("STOPMK,"))  parseStopPacket(msg);
  }
  if (millis() - lastLineMsgMs > LINE_MSG_TIMEOUT_MS) lineDet = false;

  while (DiseaseSerial.available()) {
    String msg = DiseaseSerial.readStringUntil('\n');
    msg.trim();
    if (msg == "DONE") isCameraDone = true;
  }
}

// =====================================================
// MOTOR & PWM RAMPING
// =====================================================
void setupMotors() {
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, HIGH);
  ledcAttach(MOTOR_L_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_L_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_R_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_R_LPWM, PWM_FREQ, PWM_RESOLUTION);
}

void setMotorLeftPWM_hw(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0) { ledcWrite(MOTOR_L_RPWM, pwm); ledcWrite(MOTOR_L_LPWM, 0); }
  else         { ledcWrite(MOTOR_L_RPWM, 0);   ledcWrite(MOTOR_L_LPWM, -pwm); }
}
void setMotorRightPWM_hw(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0) { ledcWrite(MOTOR_R_RPWM, pwm); ledcWrite(MOTOR_R_LPWM, 0); }
  else         { ledcWrite(MOTOR_R_RPWM, 0);   ledcWrite(MOTOR_R_LPWM, -pwm); }
}
void stopMotors() { setMotorLeftPWM_hw(0); setMotorRightPWM_hw(0); }

const int PWM_RAMP_UP   = 40;
const int PWM_RAMP_DOWN = 50; 
static int rampedL = 0, rampedR = 0;

void applyRampedPWM(int targetL, int targetR) {
  rampedL += constrain(targetL - rampedL, -PWM_RAMP_DOWN, PWM_RAMP_UP);
  rampedR += constrain(targetR - rampedR, -PWM_RAMP_DOWN, PWM_RAMP_UP);
  setMotorLeftPWM_hw(rampedL);
  setMotorRightPWM_hw(rampedR);
}

// =====================================================
// PID CONTROL (GIỮ NGUYÊN BẢN CHUẨN)
// =====================================================
float Kp_high = 2.00f;
float Kd      = 1.00f;
float Ki      = 0.008f;
float STEER_STEP_UP   = 25.0f;
float STEER_STEP_DOWN = 35.0f;
float STEER_MAX       = 20.0f;
float GYRO_KP         = 1.2f;

int FAST_BASE     = 85;
int MIN_BASE      = 65;
int MAX_BASE      = 250;
const int MIN_WHEEL_PWM = 60;

float SPEED_KP_W  = 3.5f;
float SPEED_KI_W  = 0.8f;

float errFilt = 0.0f, previewFilt = 0.0f;
float errIntegral = 0.0f, prevErr = 0.0f;
float steerCmd = 0.0f;
int   prevPwmTargetL = 0, prevPwmTargetR = 0;
float softBasePwm = 0.0f, currentRpmTarget = 0.0f;
float speedErrIntL = 0.0f, speedErrIntR = 0.0f;
float speedErrInt  = 0.0f;

float dErrFilt    = 0.0f;
uint32_t stallTimer = 0;
static bool isBlindCrawling = false;
static float stallBoostPwm = 0.0f;
uint32_t resumeSoftEndMs = 0;

float steerRateLimit(float cur, float tgt, float stepUp, float stepDown) {
  float diff = tgt - cur;
  if (fabsf(diff) < 0.01f) return cur;
  bool goingOut = (fabsf(tgt) > fabsf(cur)) && (cur * tgt >= 0);
  float step = goingOut ? stepUp : stepDown;
  if (diff > 0) { cur += step; if (cur > tgt) cur = tgt; }
  else          { cur -= step; if (cur < tgt) cur = tgt; }
  return cur;
}

void resetController() {
  errFilt = 0; previewFilt = 0; errIntegral = 0; prevErr = 0;
  dErrFilt = 0; steerCmd = 0; softBasePwm = 0;
  prevPwmTargetL = 0; prevPwmTargetR = 0;
  currentRpmTarget = 0; isBlindCrawling = false;
  speedErrInt = 0; speedErrIntL = 0; speedErrIntR = 0;   
  stallBoostPwm = 0;
  lastGoodYaw = yawAngle;
  stallTimer = millis(); rampedL = 0; rampedR = 0;
  
  // Trả servo về mặc định khi bắt đầu chạy (P100, T100)
  taskSubState = 0;
  panServo.write(100);
  tiltServo.write(100);
}

// =====================================================
// LOG SYSTEM
// =====================================================
#define LOG_SIZE 800
struct Telemetry {
  uint32_t ts; float yaw, tYaw; int8_t bE, steer;
  uint8_t pL, pR; float rL, rR; uint8_t mode;
};
Telemetry logBuf[LOG_SIZE];
uint16_t  logHead = 0, logCount = 0;

void recordTelemetry(uint8_t mode, int8_t bE, float sCmd, int pL, int pR, float rL, float rR) {
  Telemetry &e = logBuf[logHead];
  e.ts    = millis(); e.yaw = yawAngle; e.tYaw = targetYaw;
  e.bE    = bE;
  e.steer = (int8_t)constrain(sCmd, -127, 127);
  e.pL    = (uint8_t)constrain(abs(pL), 0, 255);
  e.pR    = (uint8_t)constrain(abs(pR), 0, 255);
  e.rL    = rL; e.rR = rR; e.mode = mode;
  logHead = (logHead + 1) % LOG_SIZE; if (logCount < LOG_SIZE) logCount++;
}

// =====================================================
// MAIN CONTROL LOOP 
// =====================================================
void updateLineControl(float dt_s) {
  uint32_t now = millis();
  updateYaw(dt_s);
  uint8_t logMode = 0;

  // =================================================================
  // CƠ CHẾ 7 BƯỚC: CHỤP 2 LUỐNG CÂY
  // =================================================================
  if (currentState == STATE_TASK) {
    stopMotors();
    rampedL = 0; rampedR = 0;

    // Bước 0: Mới dừng xe, thiết lập mốc thời gian và góc mặc định (P100, T100)
    if (taskSubState == 0) {
      panServo.write(100);
      tiltServo.write(100);
      taskStartTime = now;
      taskSubState = 1;
      Serial.println("# [TRAM] Dung xe. Cho 2s het rung lac...");
    }
    // Bước 1: Đã chờ xong 2s -> Quay sang Trái (P180)
    else if (taskSubState == 1) {
      if (now - taskStartTime >= 2000) {
        panServo.write(180);
        taskStartTime = now; // Lưu mốc thời gian mới
        taskSubState = 2;
        Serial.println("# [TRAM] Quay servo TRAI (180).");
      }
    }
    // Bước 2: Chờ 1.5s cho Servo quay tới nơi -> Bắn lệnh CHỤP
    else if (taskSubState == 2) {
      if (now - taskStartTime >= 1500) {
        DiseaseSerial.println("CHUP");
        Serial.println("# [TRAM] Gui lenh CHUP -> TRAI");
        isCameraDone = false;
        taskStartTime = now;
        taskSubState = 3;
      }
    }
    // Bước 3: Đợi AI xử lý xong luống Trái (báo DONE hoặc timeout 20s) -> Quay sang Phải (P0)
    else if (taskSubState == 3) {
      if (isCameraDone || (now - taskStartTime > 20000)) {
        if (isCameraDone) Serial.println("# [TRAM] Nhan DONE -> TRAI");
        else Serial.println("# [TRAM] TIMEOUT 20s -> TRAI");
        
        panServo.write(0); // Quét một vòng sang luống Phải
        taskStartTime = now;
        taskSubState = 4;
        Serial.println("# [TRAM] Quay servo PHAI (0).");
      }
    }
    // Bước 4: Chờ 1.5s cho Servo quét từ 180 về 0 -> Bắn lệnh CHỤP
    else if (taskSubState == 4) {
      if (now - taskStartTime >= 1500) {
        DiseaseSerial.println("CHUP");
        Serial.println("# [TRAM] Gui lenh CHUP -> PHAI");
        isCameraDone = false;
        taskStartTime = now;
        taskSubState = 5;
      }
    }
    // Bước 5: Đợi AI xử lý xong luống Phải -> Đưa cam về thẳng giữa (P100)
    else if (taskSubState == 5) {
      if (isCameraDone || (now - taskStartTime > 20000)) {
        if (isCameraDone) Serial.println("# [TRAM] Nhan DONE -> PHAI");
        else Serial.println("# [TRAM] TIMEOUT 20s -> PHAI");
        
        panServo.write(100);
        taskStartTime = now;
        taskSubState = 6;
        Serial.println("# [TRAM] Servo ve GIUA (100). Cho 2s de khoi hanh...");
      }
    }
    // Bước 6: Chờ 2s -> Chính thức nhả phanh cho xe chạy tiếp
    else if (taskSubState == 6) {
      if (now - taskStartTime >= 2000) {
        currentState    = STATE_RESUMING;
        resumeStartTime = now;
        lastLineSeenMs  = now;
        softBasePwm     = (float)MIN_BASE;
        isBlindCrawling = false;
        resumeSoftEndMs = now + 3000;  
        
        errFilt     = (float)bumperErrRx;
        previewFilt = (float)previewErrRx;
        errIntegral = 0.0f; prevErr = errFilt; dErrFilt = 0.0f;

        stallBoostPwm = 0;
        stallTimer    = now;
        
        taskSubState  = 0; // Đặt lại chu trình cho trạm tiếp theo
        isCameraDone  = false;
        Serial.println("# [TRAM] HOAN THANH! Xe chinh thuc khoi hanh tiep tuc.");
      }
    }
    
    recordTelemetry(2, bumperErrRx, 0, 0, 0, rpmL, rpmR);
    prevPwmTargetL = prevPwmTargetR = 0;
    return;
  }

  // ======================================================
  // LOGIC BÁM LINE BÊN DƯỚI GIỮ NGUYÊN BẢN 100%
  // ======================================================
  if (currentState == STATE_RUNNING) {
    static uint8_t irConfirm = 0;
    bool irL = (digitalRead(IR_LEFT_PIN)  == IR_DETECT_STATE);
    bool irR = (digitalRead(IR_RIGHT_PIN) == IR_DETECT_STATE);
    if (irL && irR) {
      if (++irConfirm >= 3) {
        irConfirm = 0;
        stopMotors(); rampedL = 0; rampedR = 0;
        currentState  = STATE_TASK;
        resetController();
        yawAngle = 0.0f; targetYaw = 0.0f; lastGoodYaw = 0.0f;
        recordTelemetry(2, bumperErrRx, 0, 0, 0, rpmL, rpmR);
        prevPwmTargetL = prevPwmTargetR = 0;
        Serial.println("# IR: Vach ngang xac nhan -> KICH HOAT CHU TRINH TRAM.");
        return;
      }
    } else { irConfirm = 0; }
  }

  if (currentState == STATE_RESUMING) {
    bool irL = (digitalRead(IR_LEFT_PIN)  == IR_DETECT_STATE);
    bool irR = (digitalRead(IR_RIGHT_PIN) == IR_DETECT_STATE);
    if (!irL && !irR && (now - resumeStartTime > 800)) {
      currentState    = STATE_RUNNING;
    }
  }

  bool seeingCross = (lastStopMarkerMs > 0 && (now - lastStopMarkerMs < 300)
                      && currentState == STATE_RUNNING);
  if (seeingCross && !isBlindCrawling) {
    isBlindCrawling = true;
    targetYaw = yawAngle;
    errFilt = 0; previewFilt = 0; errIntegral = 0; prevErr = 0; dErrFilt = 0;
  }
  
  if (!seeingCross && isBlindCrawling && currentState == STATE_RUNNING
      && lineDet && lineConf >= 3) {
    isBlindCrawling = false;
    errFilt     = (float)bumperErrRx;
    previewFilt = (float)previewErrRx;
    prevErr     = errFilt;
  }

  float curveSlowdown = 0.0f;

  if (isBlindCrawling) {
    logMode = 1; lastLineSeenMs = now;
    float yErr  = targetYaw - yawAngle;
    float gTgt  = constrain(yErr * 5.5f * GYRO_DIR_CORRECTION, -35.0f, 35.0f);
    steerCmd = steerRateLimit(steerCmd, gTgt, 4.0f, 8.0f);
  }
  else if (lineDet && lineConf >= 3) {
    logMode = 0; lastLineSeenMs = now;
    if (fabsf(steerCmd) < 8.0f) lastGoodYaw = 0.95f * lastGoodYaw + 0.05f * yawAngle;
    
    float bumperF = (fabsf(bumperErrRx - errFilt) > 18.0f) ? errFilt * 0.5f + bumperErrRx * 0.5f : (float)bumperErrRx;
    errFilt = 0.60f * errFilt + 0.40f * bumperF;

    float e = (fabsf(errFilt) < 1.5f) ? 0.0f : (errFilt > 0 ? errFilt - 1.5f : errFilt + 1.5f);
    if (fabsf(e) < 20.0f && fabsf(steerCmd) < 10.0f)
      errIntegral = constrain(errIntegral + e * dt_s, -6.0f, 6.0f);
    else
      errIntegral *= 0.88f;

    float dErr = (errFilt - prevErr) / dt_s;
    dErrFilt   = 0.75f * dErrFilt + 0.25f * dErr;
    prevErr    = errFilt;
    
    float Kp = min(0.10f + fabsf(e) * 0.06f, Kp_high);
    p_term = Kp * e;
    i_term = Ki * errIntegral;
    d_term = constrain(Kd * dErrFilt, -8.0f, 8.0f);

    float steerRaw = p_term + i_term + d_term;
    
    float gyroWeight = constrain(1.0f - fabsf(e) / 12.0f, 0.0f, 1.0f);
    float gyroHold   = gyroWeight * constrain((0.0f - yawAngle) * GYRO_KP * GYRO_DIR_CORRECTION, -6.0f, 6.0f);
    steerRaw += gyroHold;

    bool inSoftZone = (currentState == STATE_RESUMING) || (now < resumeSoftEndMs);
    float dynSteerMax = inSoftZone ? 12.0f : STEER_MAX;
    steerRaw  = constrain(steerRaw, -dynSteerMax, dynSteerMax);

    float adaptStep = (fabsf(e) < 8.0f) ? 6.0f : ((fabsf(e) > 15.0f) ? STEER_STEP_UP : (6.0f + (fabsf(e) - 8.0f) * 2.7f));
    float stepUp = inSoftZone ? 1.5f : adaptStep;
    steerCmd = steerRateLimit(steerCmd, steerRaw, stepUp, STEER_STEP_DOWN);
  }
  else {
    logMode = 3; steerCmd = steerRateLimit(steerCmd, 0.0f, 1.0f, 3.0f);
    if (now - lastLineSeenMs > LINE_STOP_TIMEOUT && currentState != STATE_RESUMING) {
      stopMotors(); rampedL = 0; rampedR = 0; resetController();
      recordTelemetry(3, bumperErrRx, steerCmd, 0, 0, rpmL, rpmR);
      prevPwmTargetL = prevPwmTargetR = 0;
      return;
    }
  }

  float nomRpm = TARGET_RPM;
  if      (currentRpmTarget < nomRpm) currentRpmTarget += 18.0f * dt_s;
  else if (currentRpmTarget > nomRpm) currentRpmTarget -= 30.0f * dt_s;
  currentRpmTarget = constrain(currentRpmTarget, 0.0f, TARGET_RPM);
  
  float errL = currentRpmTarget - rpmL;
  float errR = currentRpmTarget - rpmR;
  
  if (fabsf(steerCmd) < 8.0f) {
    speedErrIntL = constrain(speedErrIntL + errL * dt_s, -20.0f, 40.0f);
    speedErrIntR = constrain(speedErrIntR + errR * dt_s, -20.0f, 40.0f);
  } else {
    speedErrIntL *= 0.92f;
    speedErrIntR *= 0.92f;
  }

  int adjL = constrain((int)(SPEED_KP_W * errL + SPEED_KI_W * speedErrIntL), -30, 80);
  int adjR = constrain((int)(SPEED_KP_W * errR + SPEED_KI_W * speedErrIntR), -30, 80);
  
  bool isStall = (currentRpmTarget > 4.0f && rpmL < 1.5f && rpmR < 1.5f);
  if (isStall) {
    if (now - stallTimer > 300) stallBoostPwm = min(stallBoostPwm + 10.0f, 70.0f);
  } else { stallTimer = now; stallBoostPwm = max(stallBoostPwm - 12.0f, 0.0f); }

  int boostInt = (int)stallBoostPwm;
  int baseL = constrain(FAST_BASE + adjL + boostInt, MIN_BASE, MAX_BASE);
  int baseR = constrain(FAST_BASE + adjR + boostInt, MIN_BASE, MAX_BASE);
  
  if (baseL >= MAX_BASE - 5 && errL > 0) speedErrIntL -= errL * dt_s * 0.5f;
  if (baseR >= MAX_BASE - 5 && errR > 0) speedErrIntR -= errR * dt_s * 0.5f;
  
  int tL = baseL + (int)steerCmd;
  int tR = baseR - (int)steerCmd;
  
  int finalL = constrain(tL, MIN_WHEEL_PWM, MAX_BASE);
  int finalR = constrain(tR, MIN_WHEEL_PWM, MAX_BASE);

  applyRampedPWM(finalL, finalR);
  prevPwmTargetL = rampedL; prevPwmTargetR = rampedR;
  recordTelemetry(logMode, bumperErrRx, steerCmd, rampedL, rampedR, rpmL, rpmR);
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // KHOI TAO 2 SERVO TAI DAY
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);
  panServo.write(100);     // Góc giữa mặc định mới
  tiltServo.write(100);    // Góc ngẩng chuẩn mới

  setupMotors();
  stopMotors();
  setupMPU();

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncRA, CHANGE);

  pinMode(IR_LEFT_PIN,  INPUT_PULLDOWN);
  pinMode(IR_RIGHT_PIN, INPUT_PULLDOWN);

  setupCamUART();
  resetController();
  lastLineSeenMs = millis() + STARTUP_GRACE_MS;
  Serial.println("# AGV THESIS - SNAP STEER & PRO DASHBOARD ENABLED");
}

void loop() {
  static uint32_t lastControlUs = 0, lastStatusMs = 0;
  updateCamUART();

  uint32_t nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_INTERVAL_US) {
    float dt = (lastControlUs == 0) ? 0.020f : (nowUs - lastControlUs) / 1e6f;
    dt = constrain(dt, 0.010f, 0.050f);
    current_dt_ms = (int)(dt * 1000.0f);
    lastControlUs = nowUs;
    updateSpeedEstimate(dt);
    updateLineControl(dt);
  }

  uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = nowMs;
    if (Serial.available() == 0) {
      const char* stN[] = {"RUN", "TASK", "RSME"};
      float avgRpm = (rpmL + rpmR) * 0.5f;
      (void)avgRpm;
      
      Serial.printf("> %s | ERR:%d | CONF:%d | STR:%.1f | P:%.2f | I:%.2f | D:%.2f | TRPM:%.1f | rL:%.1f | rR:%.1f | adjL:%d | adjR:%d | PWML:%d | PWMR:%d | YAW:%.1f | DT:%d\n",
        stN[currentState], bumperErrRx, lineConf,
        steerCmd, p_term, i_term, d_term,
        currentRpmTarget, rpmL, rpmR,
        (int)(SPEED_KP_W*(currentRpmTarget-rpmL)),
        (int)(SPEED_KP_W*(currentRpmTarget-rpmR)),
        prevPwmTargetL, prevPwmTargetR, yawAngle, current_dt_ms);
    }
  }

  // ==============================================================
  // GÕ LỆNH TRÊN SERIAL MONITOR ĐỂ TEST GỬI THỦ CÔNG
  // ==============================================================
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim(); cmd.toUpperCase();
    if (cmd == "LOG") {
      Serial.println(F("=== LOG START ==="));
      Serial.println(F("ts,yaw,tYaw,bumperErr,steer,pwmL,pwmR,rpmL,rpmR,mode"));
      uint16_t start = (logCount < LOG_SIZE) ? 0 : logHead;
      for (uint16_t i = 0; i < logCount; i++) {
        Telemetry &e = logBuf[(start + i) % LOG_SIZE];
        Serial.printf("%lu,%.2f,%.2f,%d,%d,%u,%u,%.1f,%.1f,%u\n",
          e.ts, e.yaw, e.tYaw, e.bE, e.steer, e.pL, e.pR, e.rL, e.rR, e.mode);
        if (i % 20 == 0) delay(5);
      }
      Serial.println(F("=== LOG END ==="));
    }
    else if (cmd == "CLEAR") { logHead = 0; logCount = 0; Serial.println("# Log cleared"); }
    else if (cmd == "RESET") { resetController(); Serial.println("# Controller reset"); }
    else if (cmd == "STOP")  { stopMotors(); Serial.println("# Motors stopped"); }
    else if (cmd == "CHUP")  { 
      DiseaseSerial.println("CHUP"); 
      Serial.println("# [TEST] Da gui thu cong lenh CHUP toi ESP-S3-CAM"); 
    }
    else if (cmd == "HELP")  { Serial.println("# Lenh: LOG | CLEAR | RESET | STOP | CHUP | HELP"); }
  }
}