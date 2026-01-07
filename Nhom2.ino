/* =========================================
 * PROJECT: SMART DOOR + ERa IOT
 * BOARD: ESP32 (Ensure Keypad library is patched)
 * ========================================= */

#define ERA_DEBUG
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"
#define ERA_AUTH_TOKEN "e6ffb657-9259-4cbd-8773-7b438f73db4f"

#include <Arduino.h>
#include <ERa.hpp>
#include <Wire.h>
// IMPORTANT: You must edit Keypad.h and replace 'byte' with 'uint8_t' for ESP32 v3.0+
#include <Keypad.h> 
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// WiFi Credentials
const char ssid[] = "Phuong Nga_2.4G";
const char pass[] = "Aicungbiet@";

/* ================== PIN CONFIGURATION ================== */
#define SERVO_PIN 18
#define BUZZER_PIN 26
#define PIR_PIN 27
#define TRIG_PIN 5
#define ECHO_PIN 17

/* ================== OBJECTS & VARIABLES ================== */
// --- SERVO ---
Servo doorServo;
bool doorOpen = false;
unsigned long openTime = 0;
const int OPEN_ANGLE = 90;
const int CLOSE_ANGLE = 0;
const unsigned long AUTO_CLOSE_TIME = 5000;

// --- LCD ---
// Note: If screen is blank, try address 0x3F instead of 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- KEYPAD ---
const uint8_t ROWS = 4;
const uint8_t COLS = 3;

char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// Defined as uint8_t to match the patched Keypad library
uint8_t rowPins[ROWS] = {32, 33, 25, 23};
uint8_t colPins[COLS] = {19, 16, 4};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// --- PASSWORD ---
String password = "1234";
String inputPass = "";

/* ================== FUNCTIONS ================== */

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if(duration == 0) return -1;
  return duration * 0.034 / 2;
}

void openDoor() {
  if(!doorOpen){
    doorServo.write(OPEN_ANGLE);
    lcd.clear();
    lcd.print("Door: OPEN");
    doorOpen = true;
    openTime = millis();
    Serial.println("Door OPEN");
    ERa.virtualWrite(V1, 1); 
  }
}

void closeDoor() {
  if(doorOpen){
    doorServo.write(CLOSE_ANGLE);
    lcd.clear();
    lcd.print("Door: CLOSED");
    doorOpen = false;
    Serial.println("Door CLOSED");
    ERa.virtualWrite(V1, 0);
  }
}

void alarm() {
  for(int i=0; i<3; i++){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
}

/* ================== ERA CALLBACKS ================== */
ERA_CONNECTED() {
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa connected!"));
    ERa.virtualWrite(V1, doorOpen ? 1 : 0);
}

ERA_DISCONNECTED() {
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa disconnected!"));
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);

  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Setup Servo
  doorServo.attach(SERVO_PIN);
  doorServo.write(CLOSE_ANGLE); // Ensure door starts closed

  // Setup LCD
  Wire.begin(); // Default SDA 21, SCL 22
  lcd.init();
  lcd.backlight();
  lcd.print("Connecting ERa..");

  // Setup ERa
  ERa.begin(ssid, pass);
  
  lcd.clear();
  lcd.print("Enter Password");
  Serial.println("System ready.");
}

/* ================== LOOP ================== */
void loop() {
  ERa.run();

  // PIR Logic
  if(digitalRead(PIR_PIN) == HIGH){
     // Motion detected logic here
  }

  // Keypad Logic
  char key = keypad.getKey();
  if(key){
    if(key == '#'){ // Enter
      Serial.println("Input password: " + inputPass);
      if(inputPass == password){
        openDoor();
      } else {
        lcd.clear();
        lcd.print("Wrong Password");
        alarm();
        Serial.println("Password incorrect!");
        delay(1500); // Wait a bit so user sees the message
        lcd.clear();
        lcd.print("Enter Password");
      }
      inputPass = ""; 
    }
    else if(key == '*'){ // Clear
      inputPass = "";
      lcd.clear();
      lcd.print("Cleared");
      delay(1000);
      lcd.clear();
      lcd.print("Enter Password");
    }
    else{ // Number
      inputPass += key;
      lcd.setCursor(0, 0);
      lcd.print("Enter Password");
      lcd.setCursor(0, 1);
      // Mask password with *
      for(int i=0; i<inputPass.length(); i++) lcd.print("*");
    }
  }

  // Auto Close Logic
  if(doorOpen && millis() - openTime > AUTO_CLOSE_TIME){
    closeDoor();
    lcd.clear();
    lcd.print("Enter Password");
  }
}