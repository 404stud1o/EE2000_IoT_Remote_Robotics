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
uint8_t armMACAddr[] = {0x44, 0x1D, 0x64, 0xF2, 0x1E, 0xA0}; 
esp_now_peer_info_t    armInfo;

// Set ESP-NOW Data Structure
typedef struct struct_message {
  float tiltX;
  float tiltY;
  float tiltZ;
  VectorInt16 posX;
  VectorInt16 posY;
  VectorInt16 posZ;
  bool  clawGrab;      // True if SW2 is pressed
  bool  clawRelease;   // True if SW3 is pressed
} struct_message;

struct_message  sendData;

// MPU6050 Setup
  // Calibration Variables
Adafruit_MPU6050  mpuModule;

struct Offsets {
  float       offsGyroX,  offsGyroY,  offsGyroZ;
  VectorInt16 offsAccelX, offsAccelY, offsAccelZ;
};

Offsets sensorOffsets = {0, 0, 0, 
                        0, 0, 0
};

  // Calibration Function
void calibrateMPU() {
  const int samples = 500;
  Serial.println("Calibrating MPU... Maintain still and level. ");
  
  float       rawGyroX = 0,  rawGyroY = 0,  rawGyroZ = 0;
  VectorInt16 rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;

  for (int i = 0; i < samples; i++) {
    sensors_event_t  gyr,  accel,  temp;
    mpuModule.getEvent(&gyr, &accel, &temp);
    
    rawGyroX  += gyr.gyro.x;
    rawGyroY  += gyr.gyro.y;
    rawGyroZ  += gyr.gyro.z;
    rawAccelX += accel.acceleration.x;
    rawAccelY += accel.acceleration.y;
    rawAccelZ += accel.acceleration.z;
    delay(5);
  }

  sensorOffsets.offsGyroX  = rawGyroX  / samples;
  sensorOffsets.offsGyroY  = rawGyroY  / samples;
  sensorOffsets.offsGyroZ  = rawGyroZ  / samples;
  sensorOffsets.offsAccelX = rawAccelX / samples;
  sensorOffsets.offsAccelY = rawAccelY / samples; 
  sensorOffsets.offsAccelZ = rawAccelZ / samples;

  // Serial display calibration results
  Serial.println("MPU Calibration Complete.");

  Serial.print("Gyroscope Offsets: ");
  Serial.print(sensorOffsets.offsGyroX); Serial.print(", ");
  Serial.print(sensorOffsets.offsGyroY); Serial.print(", ");
  Serial.println(sensorOffsets.offsGyroZ);

  Serial.print("Accelerometer Offsets: ");
  Serial.print(sensorOffsets.offsAccelX); Serial.print(", ");
  Serial.print(sensorOffsets.offsAccelY); Serial.print(", ");
  Serial.println(sensorOffsets.offsAccelZ);
}

// Data delivery status serial output
void DataDeliveryStat(const uint8_t *mac_addr, esp_now_send_status_t espNOWstatus) {
  Serial.print("Packet Send Status: \t");
  Serial.println(espNOWstatus == ESP_NOW_SEND_SUCCESS ? "Delivery Successful" : "Delivery Failed");
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

  if (!mpuModule.begin()) {
    Serial.println("MPU6050 Not Found. I2C Connection Failed.");
    while (1) { 
      delay(10);  // Pause until MPU is found
    } 
  }

  mpuModule.setAccelerometerRange(MPU6050_RANGE_2_G);  // Set acclerometer sensitivity to ±2g
  mpuModule.setGyroRange(MPU6050_RANGE_250_DEG);       // Set gyroscope sensitivity to ±250°/s
  mpuModule.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Jitter filtering from hand shake
  calibrateMPU();

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STN); // Set device as Wi-Fi Station

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialization Failed.");
    return;
  } 
  else if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Initialization Complete.");
    return;
  }

  // Serial display MAC address
  Serial.print("Controller Device MAC Address: ");
  Serial.print(WiFi.macAddress());
  Serial.print(" | Arm Device MAC Address: ");
  Serial.println(armMACAddr[6]);

  // Register Arm device as receiver
  esp_now_register_send_cb(DataDeliveryStat);
  memcpy(armInfo.peer_addr, armMACAddr, sizeof(armMACAddr)); 
  armInfo.channel = 1; // Use current Wi-Fi channel
  armInfo.encrypt = false;

  if (esp_now_add_peer(&armInfo) != ESP_OK) {
    Serial.println("Connection to Arm Device Failed. Device Not Added.");
    return;
  } 
  else if (esp_now_add_peer(&armInfo) == ESP_OK) {
    Serial.println("Device Connected via ESP-NOW.");
    return;
  }
}

void loop() {
  // Read SW2&3 (LOW = pressed, for Input Pull-Up)
  bool isGrabbing  = !digitalRead(PIN_SW2_GRAB);
  bool isReleasing = !digitalRead(PIN_SW3_RELS);

  // Control LED
  if (isGrabbing || isReleasing) {
    digitalWrite(PIN_CLAW_LED, HIGH);
  } 
  else {
    digitalWrite(PIN_CLAW_LED, LOW);
  }

  // Overheat Warning
  if (temp.temperature > 40.0) { 
    digitalWrite(PIN_CLAW_LED, HIGH);
    delay(500);
    digitalWrite(PIN_CLAW_LED, LOW);
    delay(500);
    Serial.println("Warning: High Temperature on MPU6050!");
  }
  
  // Read MPU6050 data
  sensors_event_t  gyr,  accel,  temp;
  mpuModule.getEvent(&gyr, &accel, &temp);

  // Process data w/ offsets
  float tiltX = gyr.gyro.x - sensorOffsets.offsGyroX;
  float tiltY = gyr.gyro.y - sensorOffsets.offsGyroY;
  float tiltZ = gyr.gyro.z - sensorOffsets.offsGyroZ;

  VectorInt16 accelX = accel.acceleration.x - sensorOffsets.offsAccelX;
  VectorInt16 accelY = accel.acceleration.y - sensorOffsets.offsAccelY;
  VectorInt16 accelZ = accel.acceleration.z - sensorOffsets.offsAccelZ;

  float dt = 0.01; // Assuming loop runs every 10ms

  VectorInt16 veloX += accelX * dt; 
  VectorInt16 veloY += accelY * dt; 
  VectorInt16 veloZ += accelZ * dt; 

	VectorInt16 posX  +=  veloX * dt;
	VectorInt16 posY  +=  veloY * dt;
	VectorInt16 posZ  +=  veloZ * dt;

  // Populate data structure
  sendData.tiltX       = tiltX;
  sendData.tiltY       = tiltY;
  sendData.tiltZ       = tiltZ;
  sendData.posX        = posX;
  sendData.posY        = posY;
  sendData.posZ        = posZ;
  sendData.clawGrab    = isGrabbing;
  sendData.clawRelease = isReleasing;

  Serial.println("Packet Data");
  Serial.print("Tilt: \t");
  Serial.print(sendData.tiltX); Serial.print(", ");
  Serial.print(sendData.tiltY); Serial.print(", ");
  Serial.println(sendData.tiltZ);
  Serial.print("Position: \t");
  Serial.print(sendData.posX); Serial.print(", ");
  Serial.print(sendData.posY); Serial.print(", "); 
  Serial.println(sendData.posZ);
  Serial.print("Claw: \t");
  if (isGrabbing) {
    Serial.println("Grabbing");
  } 
  else if (isReleasing) {
    Serial.println("Releasing");
  } 
  else {
    Serial.println("Idle");
  }

  // Send command
  esp_now_send(armMACAddr, (uint8_t *) &sendData, sizeof(sendData));
  delay(250);
}