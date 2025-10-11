#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char* ssid = "RocketNet";       // ESP32 AP SSID
const char* password = "12345678";    // ESP32 AP password

WiFiUDP udp;
const unsigned int localPort = 4210;

void setup() {
  Serial.begin(115200);

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Waiting for data...");
  display.display();

  // Connect to ESP32 AP
  WiFi.begin(ssid, password);
  Serial.print("Connecting to AP");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.println("UDP listener started");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incoming[255];
    int len = udp.read(incoming, 255);
    if (len > 0) incoming[len] = 0;
    Serial.println(incoming); // Print to serial

    // Display on OLED
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Flight Data:");
    display.println(incoming); // Show received data
    display.display();
  }
}
