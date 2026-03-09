#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Pins
#define PIN_SW2_GRAB    4
#define PIN_SW3_RELS    13
#define PIN_CLAW_LED    18
#define PIN_6050_SDA    21
#define PIN_6050_SCL    22

// MAC Address for ESP32 on the arm
uint8_t armMACAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// ESP-NOW Data Structure
typedef struct struct_message {
  float pitch;         // Forward/backward tilt for arm link 1
  float roll;          // Side-to-side tilt for arm link 2 or base
  bool  clawGrab;      // True if SW2 is pressed
  bool  clawRelease;   // True if SW3 is pressed
} struct_message;

struct_message      armData;
esp_now_peer_info_t peerInfo;
Adafruit_MPU6050    mpuAccel;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  
  Serial.print("Last Packet Send Status: \t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Successful" : "Delivery Failed");
}

void setup() {
  Serial.begin(115200);

  // Initialize Pins
  pinMode(PIN_SW2_GRAB, INPUT_PULLUP);
  pinMode(PIN_SW3_RELS, INPUT_PULLUP);
  pinMode(PIN_CLAW_LED, OUTPUT);
  digitalWrite(PIN_CLAW_LED, LOW);

  // Initialize I2C for MPU6050
  Wire.begin(PIN_6050_SDA, PIN_6050_SCL);

  if (!mpuAccel.begin()) {
    Serial.println("MPU6050 Not Found. I2C Connection Failed.");
    while (1) { delay(10); } // Halt if sensor not found
  }

  mpuAccel.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpuAccel.setFilterBandwidth(MPU6050_BAND_21_HZ); // Smooths out the jitter from hand shakes

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA); // Set device as Wi-Fi Station

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialization Failed.");
    return;
  }

  // Register the Arm as receiver
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, armMACAddr, 6); 
  peerInfo.channel = 0; // Use current Wi-Fi channel
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Connection to Arm Failed. Peer Not Added.");
    return;
  }
}

void loop() {
  // Read SW2&3 (LOW = pressed for Input Pull-Up)
  bool isGrabbing  = !digitalRead(PIN_SW2_GRAB);
  bool isReleasing = !digitalRead(PIN_SW3_RELS);

  // Control LED
  if (isGrabbing || isReleasing) {
    digitalWrite(PIN_CLAW_LED, HIGH);
  } else {
    digitalWrite(PIN_CLAW_LED, LOW);
  }

  // Read MPU6050 Data
  sensors_event_t a, g, temp;
  mpuAccel.getEvent(&a, &g, &temp);

  // Process Data (Calculate Pitch and Roll in degrees)
  // Use X, Y, Z acceleration values to calculate angles
  float pitch = -(atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0) / PI;
  float roll = (atan2(a.acceleration.y, a.acceleration.z) * 180.0) / PI;

  // Populate Data Structure
  armData.pitch       = pitch;
  armData.roll        = roll;
  armData.clawGrab    = isGrabbing;
  armData.clawRelease = isReleasing;

  // Send Data
  esp_now_send(armMACAddr, (uint8_t *) &armData, sizeof(armData));

  delay(100); 
}