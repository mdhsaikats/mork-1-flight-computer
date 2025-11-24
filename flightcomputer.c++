#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include "MPU6050.h"

// I2C pins
#define SDA_PIN 4
#define SCL_PIN 5

// Pins
#define BUTTON_PIN 3
#define RED_LED 8
#define GREEN_LED 9

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Sensors
Adafruit_BMP085 bmp;
MPU6050 mpu;

bool sensorsOk = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // LEDs and button
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
  } else {
    Serial.println("OLED OK");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("System Starting...");
    display.display();
  }

  // Initialize sensors
  if(!bmp.begin()) {
    Serial.println("BMP180 init failed!");
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
  } else {
    Serial.println("BMP180 OK");
  }

  mpu.initialize();
  if(!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
  } else {
    Serial.println("MPU6050 OK");
  }

  sensorsOk = bmp.begin() && mpu.testConnection();

  if(sensorsOk) {
    display.println("All sensors OK");
    display.display();
  }
}

void showData() {
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Serial output
  Serial.println("=== Sensor Data ===");
  Serial.print("BMP180 Temp: "); Serial.print(temp); Serial.println(" °C");
  Serial.print("BMP180 Pressure: "); Serial.print(pressure / 100); Serial.println(" hPa");
  Serial.println("MPU6050 Acceleration:");
  Serial.print("AX: "); Serial.println(ax);
  Serial.print("AY: "); Serial.println(ay);
  Serial.print("AZ: "); Serial.println(az);
  Serial.println("MPU6050 Gyroscope:");
  Serial.print("GX: "); Serial.println(gx);
  Serial.print("GY: "); Serial.println(gy);
  Serial.print("GZ: "); Serial.println(gz);
  Serial.println("===================");

  // OLED output
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.println("BMP180:");
  display.print("Temp: "); display.print(temp); display.println(" C");
  display.print("Pres: "); display.print(pressure/100); display.println(" hPa");

  display.println("\nMPU6050:");
  display.print("AX: "); display.println(ax);
  display.print("AY: "); display.println(ay);
  display.print("AZ: "); display.println(az);
  display.display();
}

void loop() {
  // Test on button press
  if(digitalRead(BUTTON_PIN) == LOW) {
    delay(200); // debounce

    bool ok = bmp.begin() && mpu.testConnection();
    if(ok) {
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      showData();
    } else {
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      Serial.println("Sensor test failed!");
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0,0);
      display.println("ERROR!");
      display.display();
    }

    while(digitalRead(BUTTON_PIN) == LOW); // wait until released
    delay(200);
  }

  // Continuous serial output every second
  showData();
  delay(1000);
}
