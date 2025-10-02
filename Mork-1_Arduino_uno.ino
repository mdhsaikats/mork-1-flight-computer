#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include <ESP32Servo.h>

// Pins for ESP32
const int greenLedPin = 27;
const int redLedPin = 14;
const int buzzerPin = 12;
const int buttonPin = 13;
const int chipSelect = 5; // SD card CS pin

// Servo pins
const int servo1Pin = 25;
const int servo2Pin = 26;

// Objects
Adafruit_BMP085 bmp;
MPU6050 mpu;
Servo servo1;
Servo servo2;

// Variables
bool bmpOk = false;
bool mpuOk = false;
bool sdOk = false;
bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// MPU angles
float roll, pitch;

// Function to check all components
void checkComponents() {
  bmpOk = false;
  mpuOk = false;
  sdOk = false;

  // Check BMP180
  if (bmp.begin()) {
    bmpOk = true;
  }

  // Check MPU6050
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  mpu.initialize();
  if (mpu.testConnection()) {
    mpuOk = true;
  }

  // Check SD card
  if (SD.begin(chipSelect)) {
    sdOk = true;
  }

  // Indicate status
  if (bmpOk && mpuOk && sdOk) {
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
    noTone(buzzerPin);
  } else {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
    tone(buzzerPin, 1000); // 1kHz buzzer
  }
}

// Read MPU6050 angles (simplified)
void readMPU() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer data to roll/pitch angles in degrees
  roll  = atan2(ay, az) * 57.2958;  // roll
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958;  // pitch
}

// Map MPU tilt to servo angles (0-180)
void controlServos() {
  int servo1Angle = constrain(90 + roll, 0, 180);
  int servo2Angle = constrain(90 + pitch, 0, 180);

  servo1.write(servo1Angle);
  servo2.write(servo2Angle);
}

void setup() {
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Button with internal pull-up

  Serial.begin(115200);

  // Attach servos
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Initial check
  checkComponents();
}

void loop() {
  int reading = digitalRead(buttonPin);

  // Button debounce and full component test
  if (reading == LOW && !buttonPressed && (millis() - lastDebounceTime) > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = millis();

    Serial.println("Testing all components...");
    checkComponents();

    Serial.print("BMP180: "); Serial.println(bmpOk ? "OK" : "FAIL");
    Serial.print("MPU6050: "); Serial.println(mpuOk ? "OK" : "FAIL");
    Serial.print("SD Card: "); Serial.println(sdOk ? "OK" : "FAIL");

    // Servo test
    if (bmpOk && mpuOk && sdOk) {
      servo1.write(90);
      servo2.write(90);
      delay(500);
      servo1.write(0);
      servo2.write(0);
      delay(500);
      servo1.write(90);
      servo2.write(90);
    }
  }

  if (reading == HIGH) {
    buttonPressed = false;
  }

  // Continuous sensor readings and servo control
  if (bmpOk && mpuOk) {
    // Read MPU and control servos
    readMPU();
    controlServos();

    // Print sensor data
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);

    Serial.print(" | BMP180 Temp: "); Serial.print(bmp.readTemperature());
    Serial.print(" C | Pressure: "); Serial.println(bmp.readPressure());

    delay(200); // adjust as needed
  }
}
