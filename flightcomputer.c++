#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050.h>
#include <Adafruit_BMP085.h>

// === Sensors ===
MPU6050 mpu;
Adafruit_BMP085 bmp;

// === Servos ===
Servo servoPitch;   // X-axis correction
Servo servoYaw;     // Y-axis correction

int servoPitchPin = 7;
int servoYawPin   = 10;

// === LEDs ===
const int RED_LED = 2;
const int GREEN_LED = 3;

// === PID parameters ===
float Kp = 1.0;
float Ki = 0.02;
float Kd = 0.15;

float errorX, errorY;
float prevX, prevY;
float intX = 0, intY = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(5, 6);   // SDA=5, SCL=6 (ESP32-C3)

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Attach servos
  servoPitch.attach(servoPitchPin, 500, 2500);
  servoYaw.attach(servoYawPin, 500, 2500);

  servoPitch.write(90);  
  servoYaw.write(90);

  // Initialize sensors
  if (!bmp.begin()) {
    Serial.println("BMP180 ERROR!");
    digitalWrite(RED_LED, HIGH);
    while (1);
  }

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 ERROR!");
    digitalWrite(RED_LED, HIGH);
    while (1);
  }

  Serial.println("SYSTEM OK");
  digitalWrite(GREEN_LED, HIGH);

  lastTime = millis();
}

void loop() {
  // === READ MPU6050 ===
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyro values to deg/sec
  float rateX = gx / 131.0;   // PITCH
  float rateY = gy / 131.0;   // YAW

  // === PID control ===
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Desired = 0 (rocket should be stable)
  errorX = 0 - rateX;
  errorY = 0 - rateY;

  intX += errorX * dt;
  intY += errorY * dt;

  float dX = (errorX - prevX) / dt;
  float dY = (errorY - prevY) / dt;

  prevX = errorX;
  prevY = errorY;

  float outX = (Kp * errorX) + (Ki * intX) + (Kd * dX);
  float outY = (Kp * errorY) + (Ki * intY) + (Kd * dY);

  // Convert to servo angles
  float pitchAngle = 90 + outX;  
  float yawAngle   = 90 + outY;

  // Limit angles
  pitchAngle = constrain(pitchAngle, 0, 180);
  yawAngle   = constrain(yawAngle, 0, 180);

  // Move servos
  servoPitch.write(pitchAngle);
  servoYaw .write(yawAngle);

  // === OPTIONAL: read altitude ===
  float altitude = bmp.readAltitude();

  // === DEBUG ===
  Serial.print("GX: "); Serial.print(rateX);
  Serial.print("  GY: "); Serial.print(rateY);
  Serial.print("  SPitch: "); Serial.print(pitchAngle);
  Serial.print("  SYaw: "); Serial.print(yawAngle);
  Serial.print("  Alt: "); Serial.println(altitude);

  // === LEDs ===
  if (abs(rateX) > 40 || abs(rateY) > 40) {
    digitalWrite(RED_LED, HIGH);
  } else {
    digitalWrite(RED_LED, LOW);
  }

  delay(10);
}
