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
float filteredRoll = 0, filteredPitch = 0;
const float alpha = 0.8; // Complementary filter constant

// Data logging variables
unsigned long lastLogTime = 0;
unsigned long logInterval = 100; // Log every 100ms
File dataFile;
String logFileName = "";

// Timing variables for non-blocking operations
unsigned long lastSensorRead = 0;
unsigned long sensorInterval = 50; // Read sensors every 50ms
unsigned long lastSerialPrint = 0;
unsigned long serialInterval = 200; // Print to serial every 200ms

// Error handling variables
unsigned long lastComponentCheck = 0;
unsigned long componentCheckInterval = 5000; // Check components every 5 seconds
int sensorFailCount = 0;
const int maxFailCount = 5;

// Altitude calculation variables
float baselinePressure = 1013.25; // Sea level pressure in hPa
float currentAltitude = 0;
float previousAltitude = 0;
float verticalSpeed = 0;
unsigned long lastAltitudeTime = 0;

// Flight state machine
enum FlightState {
  GROUND,
  ARMED,
  ASCENT,
  DESCENT,
  RECOVERY
};
FlightState flightState = GROUND;
float maxAltitude = 0;
float launchThreshold = 3.0; // meters above ground
float recoveryThreshold = 2.0; // m/s downward velocity

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

// Initialize data logging
void initializeLogging() {
  if (!sdOk) return;
  
  // Create filename with timestamp
  int fileNumber = 0;
  do {
    logFileName = "flight_" + String(fileNumber) + ".csv";
    fileNumber++;
  } while (SD.exists(logFileName) && fileNumber < 1000);
  
  // Create CSV header
  dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Timestamp,Roll,Pitch,Temperature,Pressure,Altitude,VerticalSpeed");
    dataFile.close();
    Serial.println("Log file created: " + logFileName);
  }
}

// Log sensor data to SD card
void logSensorData() {
  if (!sdOk || millis() - lastLogTime < logInterval) return;
  
  dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(roll);
    dataFile.print(",");
    dataFile.print(pitch);
    dataFile.print(",");
    dataFile.print(bmp.readTemperature());
    dataFile.print(",");
    dataFile.print(bmp.readPressure());
    dataFile.print(",");
    dataFile.print(currentAltitude);
    dataFile.print(",");
    dataFile.println(verticalSpeed);
    dataFile.close();
    lastLogTime = millis();
  }
}

// Calculate altitude from pressure
void calculateAltitude() {
  float pressure = bmp.readPressure() / 100.0; // Convert to hPa
  previousAltitude = currentAltitude;
  currentAltitude = 44330.0 * (1.0 - pow(pressure / baselinePressure, 0.1903));
  
  // Calculate vertical speed
  unsigned long currentTime = millis();
  if (lastAltitudeTime > 0) {
    float deltaTime = (currentTime - lastAltitudeTime) / 1000.0; // Convert to seconds
    verticalSpeed = (currentAltitude - previousAltitude) / deltaTime;
  }
  lastAltitudeTime = currentTime;
}

// Update flight state based on sensor data
void updateFlightState() {
  static unsigned long stateChangeTime = 0;
  
  switch (flightState) {
    case GROUND:
      if (currentAltitude > launchThreshold) {
        flightState = ASCENT;
        stateChangeTime = millis();
        // Flash green LED rapidly for launch detection
        for (int i = 0; i < 5; i++) {
          digitalWrite(greenLedPin, HIGH);
          delay(50);
          digitalWrite(greenLedPin, LOW);
          delay(50);
        }
      }
      break;
      
    case ASCENT:
      if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
      }
      if (verticalSpeed < -recoveryThreshold && millis() - stateChangeTime > 2000) {
        flightState = DESCENT;
        stateChangeTime = millis();
        tone(buzzerPin, 2000, 500); // High pitch beep for apogee
      }
      break;
      
    case DESCENT:
      if (currentAltitude < launchThreshold && verticalSpeed > -1.0) {
        flightState = RECOVERY;
        stateChangeTime = millis();
        // Continuous tone for recovery
        tone(buzzerPin, 800);
      }
      break;
      
    case RECOVERY:
      // Stay in recovery mode - beacon active
      break;
  }
  
  // Update LED status based on flight state
  switch (flightState) {
    case GROUND:
      // Green LED solid when all systems OK
      digitalWrite(greenLedPin, bmpOk && mpuOk && sdOk ? HIGH : LOW);
      digitalWrite(redLedPin, bmpOk && mpuOk && sdOk ? LOW : HIGH);
      break;
    case ASCENT:
      // Blink green during ascent
      digitalWrite(greenLedPin, (millis() / 250) % 2);
      digitalWrite(redLedPin, LOW);
      break;
    case DESCENT:
      // Blink red during descent
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, (millis() / 250) % 2);
      break;
    case RECOVERY:
      // Alternate red/green for recovery
      bool state = (millis() / 500) % 2;
      digitalWrite(greenLedPin, state);
      digitalWrite(redLedPin, !state);
      break;
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
  
  // Apply complementary filter for smoothing
  filteredRoll = alpha * filteredRoll + (1 - alpha) * roll;
  filteredPitch = alpha * filteredPitch + (1 - alpha) * pitch;
}

// Map MPU tilt to servo angles (0-180)
void controlServos() {
  int servo1Angle = constrain(90 + filteredRoll, 0, 180);
  int servo2Angle = constrain(90 + filteredPitch, 0, 180);

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
  
  // Initialize data logging if SD card is available
  if (sdOk) {
    initializeLogging();
  }
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

  // Periodic component health check
  if (millis() - lastComponentCheck >= componentCheckInterval) {
    checkComponents();
    lastComponentCheck = millis();
  }

  // Continuous sensor readings and servo control
  if (bmpOk && mpuOk && millis() - lastSensorRead >= sensorInterval) {
    // Read MPU and control servos
    readMPU();
    controlServos();
    
    // Calculate altitude and vertical speed
    calculateAltitude();
    
    // Update flight state
    updateFlightState();
    
    // Log data to SD card
    logSensorData();
    
    lastSensorRead = millis();
  }

  // Print sensor data at slower rate
  if (bmpOk && mpuOk && millis() - lastSerialPrint >= serialInterval) {
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Altitude: "); Serial.print(currentAltitude);
    Serial.print(" m | Vertical Speed: "); Serial.print(verticalSpeed);
    Serial.print(" m/s | BMP180 Temp: "); Serial.print(bmp.readTemperature());
    Serial.print(" C | Pressure: "); Serial.println(bmp.readPressure());
    
    lastSerialPrint = millis();
  }
}
