/* =========================================
 * PROJECT: SMART DOOR + ERa IOT
 * BOARD: ESP32
 * AUTHOR: FIX BY GEMINI
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
#define LOCK_DURATION 60000  // Khóa 1 phút nếu sai quá 3 lần

/* ================= TIMERS ================= */
unsigned long lastSensorRead = 0;
const long sensorInterval = 2000; // Đọc cảm biến mỗi 2 giây (tránh spam)

/* ================= FUNCTIONS ================= */

// Hàm kêu bíp
void beep(int times = 1, int duration = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration);
  }
}

// Cập nhật lại giao diện LCD mặc định
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
    
    // Đồng bộ trạng thái lên App (V1)
    ERa.virtualWrite(V1, 1);
    Serial.println("Cua da MO");
    
    // Sau 1s thì hiện lại màn hình nhập pass (nhưng cửa vẫn mở)
    // delay(1000); 
    // resetLCD(); // Tạm tắt dòng này để hiển thị chữ DOOR OPENED
  }
}

void closeDoor() {
  if (doorOpen) {
    doorServo.write(CLOSE_ANGLE);
    doorOpen = false;
    
    lcd.clear();
    lcd.print("DOOR CLOSED");
    beep(1, 100);
    
    // Đồng bộ trạng thái về App (V1)
    ERa.virtualWrite(V1, 0);
    Serial.println("Cua da DONG");
    
    delay(1000);
    resetLCD();
  }
}

// Đo khoảng cách
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

// [QUAN TRỌNG] Hàm này xử lý khi bạn nhấn nút trên App
ERA_WRITE(V1) {
  int value = param.getInt();// Lấy giá trị 0 hoặc 1 từ nút nhấn
  if (value == 1) {
    openDoor();
  } else {
    closeDoor();
  }
}

ERA_CONNECTED() {
  ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("Connected!"));
  ERa.virtualWrite(V1, doorOpen ? 1 : 0); // Đồng bộ trạng thái khi mới kết nối
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(PIR_PIN, INPUT);  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Cấu hình Servo
  doorServo.attach(SERVO_PIN);
  doorServo.write(CLOSE_ANGLE); // Đóng cửa khi khởi động

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

  /* ===== XỬ LÝ CẢM BIẾN (Mỗi 2 giây 1 lần) ===== */
  // Không in lên LCD để tránh đè chữ mật khẩu
  if (currentMillis - lastSensorRead > sensorInterval) {
    lastSensorRead = currentMillis;
    
    // 1. Đọc khoảng cách gửi lên V2 (bạn có thể tạo thêm widget Value trên App)
    long dist = getDistance();
    if (dist > 0) ERa.virtualWrite(V2, dist);

    // 2. Đọc chuyển động gửi lên V3 (Widget LED hoặc Label)
    bool motion = digitalRead(PIR_PIN);
    if (motion) {
        ERa.virtualWrite(V3, "Motion Detected");
        Serial.println("Phat hien chuyen dong!");
    } else {
        ERa.virtualWrite(V3, "Safe");
    }
  }

  /* ===== TỰ ĐỘNG ĐÓNG CỬA ===== */
  if (doorOpen && currentMillis - openTime > AUTO_CLOSE_TIME) {
    closeDoor();
  }

  /* ===== CHẾ ĐỘ KHÓA (BRUTE FORCE LOCK) ===== */
  if (isLocked) {
    if (currentMillis - lockTime >= LOCK_DURATION) {
      isLocked = false;
      wrongCount = 0;
      lcd.clear();
      lcd.print("UNLOCKED");
      delay(1000);
      resetLCD();
    } else {
      // Chỉ hiển thị thông báo khóa, chặn nhập phím
      lcd.setCursor(0,0);
      lcd.print("SYSTEM LOCKED!  ");
      lcd.setCursor(0,1);
      int remain = (LOCK_DURATION - (currentMillis - lockTime)) / 1000;
      lcd.print("Wait: "); lcd.print(remain); lcd.print("s   ");
      return; 
    }
  }

  /* ===== XỬ LÝ BÀN PHÍM ===== */
  char key = keypad.getKey();
  
  if (key) {
    beep(1, 50); // Bíp nhẹ khi nhấn phím
    
    if (key == '#') { // Nút ENTER
      if (inputPass == password) {
        lcd.clear();
        lcd.print("ACCESS GRANTED");
        delay(500);
        openDoor();
        wrongCount = 0;
      } else {
        wrongCount++;
        lcd.clear();
        lcd.print("WRONG PASSWORD");
        beep(2, 100);
        delay(1500);

        if (wrongCount >= 3) {
          isLocked = true;
          lockTime = millis();
          lcd.clear();
          lcd.print("SYSTEM LOCKED!");
          beep(5, 200); 
        } else {
          resetLCD();
        }
      }
      inputPass = ""; // Reset biến nhập
    }
    else if (key == '*') { // Nút XÓA (CLEAR)
      inputPass = "";
      resetLCD();
    }
    else { // Nhập số
      if (inputPass.length() < 6) { // Giới hạn độ dài pass hiển thị
        inputPass += key;
        lcd.setCursor(0, 1);
        lcd.print("                "); // Xóa dòng dưới
        lcd.setCursor(0, 1);
        for (size_t i = 0; i < inputPass.length(); i++) lcd.print("*");
      }
    }
  }
}