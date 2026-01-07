/* =========================================
 * PROJECT: SMART DOOR + ERa IOT
 * BOARD: ESP32
 * AUTHOR: FIX BY GEMINI - FINAL STABLE VERSION
 * ========================================= */

#define ERA_DEBUG
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"
#define ERA_AUTH_TOKEN "e6ffb657-9259-4cbd-8773-7b438f73db4f" // Token của bạn

#include <Arduino.h>
#include <ERa.hpp>
#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

/* ================= WIFI ================= */
const char ssid[] = "Phuong Nga_2.4G";
const char pass[] = "Aicungbiet@";

/* ================= PIN DEFINITIONS ================= */
#define SERVO_PIN   18
#define BUZZER_PIN  26
#define PIR_PIN     27   // Cảm biến chuyển động
#define TRIG_PIN    5    // Siêu âm Trig
#define ECHO_PIN    17   // Siêu âm Echo

/* ================= SERVO ================= */
Servo doorServo;
bool doorOpen = false;
unsigned long openTime = 0;
#define OPEN_ANGLE   90
#define CLOSE_ANGLE  0
#define AUTO_CLOSE_TIME 5000  // Tự đóng sau 5 giây

/* ================= STATS & SETTINGS ================= */
int openCount = 0; 
unsigned long lockDuration = 60000; // Mặc định khóa 1 phút (60000ms)

/* ================= LCD ================= */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ================= KEYPAD ================= */
const uint8_t ROWS = 4;
const uint8_t COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
uint8_t rowPins[ROWS] = {32, 33, 25, 23};
uint8_t colPins[COLS] = {19, 16, 4};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

/* ================= PASSWORD ================= */
String password = "1234";
String inputPass = "";

/* ================= SECURITY ================= */
int wrongCount = 0;
bool isLocked = false;
unsigned long lockTime = 0;

/* ================= TIMERS ================= */
unsigned long lastSensorRead = 0;
const long sensorInterval = 2000; 

/* ================= FUNCTIONS ================= */

void beep(int times = 1, int duration = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration);
  }
}

void resetLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ENTER PASSWORD");
  inputPass = "";
}

void openDoor() {
  if (!doorOpen) {
    doorServo.write(OPEN_ANGLE);
    doorOpen = true;
    openTime = millis();
    
    lcd.clear();
    lcd.print("DOOR OPENED");
    beep(1, 200);
    
    // Gửi dữ liệu lên ERa
    ERa.virtualWrite(V1, 1);
    ERa.virtualWrite(V5, "DA MO"); 
    openCount++;
    ERa.virtualWrite(V6, openCount);
    
    Serial.println("Cua da MO");
  }
}

void closeDoor() {
  if (doorOpen) {
    doorServo.write(CLOSE_ANGLE);
    doorOpen = false;
    
    lcd.clear();
    lcd.print("DOOR CLOSED");
    beep(1, 100);
    
    ERa.virtualWrite(V1, 0);
    ERa.virtualWrite(V5, "DA DONG");
    
    Serial.println("Cua da DONG");
    delay(1000);
    resetLCD();
  }
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (duration == 0) return -1;
  return duration * 0.034 / 2; 
}

/* ================= ERA HANDLERS ================= */

// Xử lý nút bấm V1 (Bật/Tắt)
ERA_WRITE(V1) {
  int value = param.getInt();
  if (value == 1) openDoor();
  else closeDoor();
}

// Xử lý nhập mật khẩu từ xa (V4)
ERA_WRITE(V4) {
  String remotePass = param.getString();
  if (remotePass == password) {
    ERa.virtualWrite(V4, "DUNG MK -> MO CUA");
    openDoor();
    delay(2000);
    ERa.virtualWrite(V4, ""); 
  } else {
    ERa.virtualWrite(V4, "SAI MAT KHAU!");
    beep(3, 100);
  }
}

// Đồng bộ số lần mở cửa (V6)
ERA_WRITE(V6) {
    int svVal = param.getInt();
    if(svVal > openCount) openCount = svVal;
}

// Cài đặt thời gian khóa báo động (V7)
ERA_WRITE(V7) {
    int minutes = param.getInt();
    if (minutes < 1) minutes = 1; // Tối thiểu 1 phút
    lockDuration = minutes * 60000; // Đổi phút ra mili giây
    
    Serial.print("Da cai dat thoi gian khoa: ");
    Serial.print(minutes);
    Serial.println(" phut");
}

ERA_CONNECTED() {
  ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("Connected!"));
  ERa.virtualWrite(V1, doorOpen ? 1 : 0);
  
  // Gửi thời gian hiện tại lên App (V7)
  ERa.virtualWrite(V7, lockDuration / 60000); 
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(PIR_PIN, INPUT);  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  doorServo.attach(SERVO_PIN, 500, 2400);
  doorServo.write(CLOSE_ANGLE);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.print("Connecting ERa...");

  ERa.begin(ssid, pass);
  resetLCD();
}

/* ================= LOOP ================= */
void loop() {
  ERa.run();
  unsigned long currentMillis = millis();

  /* ===== CẢM BIẾN ===== */
  if (currentMillis - lastSensorRead > sensorInterval) {
    lastSensorRead = currentMillis;
    long dist = getDistance();
    if (dist > 0) ERa.virtualWrite(V2, dist);

    bool motion = digitalRead(PIR_PIN);
    if (motion) ERa.virtualWrite(V3, "Motion Detected");
    else ERa.virtualWrite(V3, "Safe");
  }

  /* ===== TỰ ĐỘNG ĐÓNG ===== */
  if (doorOpen && currentMillis - openTime > AUTO_CLOSE_TIME) {
    closeDoor();
  }

  /* ===== KHÓA HỆ THỐNG (ĐÃ FIX LỖI LCD) ===== */
  if (isLocked) {
    // Kiểm tra đã hết thời gian khóa chưa
    if (currentMillis - lockTime >= lockDuration) {
      isLocked = false;
      wrongCount = 0;
      lcd.clear();
      lcd.print("UNLOCKED");
      delay(1000);
      resetLCD();
    } else {
      // Logic mới: Chỉ cập nhật màn hình mỗi 1 giây
      static unsigned long lastLCDUpdate = 0;
      if (currentMillis - lastLCDUpdate >= 1000) {
        lastLCDUpdate = currentMillis;
        
        lcd.setCursor(0,0);
        lcd.print("SYSTEM LOCKED!  ");
        lcd.setCursor(0,1);
        
        int remain = (lockDuration - (currentMillis - lockTime)) / 1000;
        lcd.print("Wait: "); 
        lcd.print(remain); 
        lcd.print("s       ");
      }
      return; // Chặn phím
    }
  }

  /* ===== KEYPAD ===== */
  char key = keypad.getKey();
  if (key) {
    beep(1, 50);
    if (key == '#') { 
      if (inputPass == password) {
        lcd.clear(); lcd.print("ACCESS GRANTED");
        delay(500); openDoor(); wrongCount = 0;
      } else {
        wrongCount++;
        lcd.clear(); lcd.print("WRONG PASSWORD");
        beep(2, 100); delay(1500);

        if (wrongCount >= 3) {
          isLocked = true;
          lockTime = millis();
          lcd.clear(); lcd.print("SYSTEM LOCKED!");
          beep(5, 200); 
        } else resetLCD();
      }
      inputPass = ""; 
    }
    else if (key == '*') { inputPass = ""; resetLCD(); }
    else { 
      if (inputPass.length() < 6) { 
        inputPass += key;
        lcd.setCursor(0, 1); lcd.print("                "); 
        lcd.setCursor(0, 1);
        for (size_t i = 0; i < inputPass.length(); i++) lcd.print("*");
      }
    }
  }
}