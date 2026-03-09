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
esp_now_peer_info_t    armInfo;

// Set ESP-NOW Data Structure
typedef struct struct_message {
  // float pitch;         // Forward/backward tilt for arm link 1
  // float roll;          // Side-to-side tilt for arm link 2 or base
  float accelX;
  float accelY;
  float accelZ;
  bool  clawGrab;      // True if SW2 is pressed
  bool  clawRelease;   // True if SW3 is pressed
} struct_message;

struct_message  sendData;

// MPU6050 Setup
  // Calibration Variables
Adafruit_MPU6050  mpuUnit;

struct Offsets {
  float offsAccelX, offsAccelY, offsAccelZ;
  float offsGyroX,  offsGyroY,  offsGyroZ;
};
Offsets sensorOffsets = {0, 0, 0, 
                         0, 0, 0};

  // Calibration Function
void calibrateMPU() {
  const int samples = 500;
  Serial.println("Calibrating MPU... Maintain still and level. ");
  
  float rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;
  float rawGyroX = 0,  rawGyroY = 0,  rawGyroZ = 0;

  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyr, temp;
    mpuUnit.getEvent(&accel, &gyr, &temp);
    
    rawAccelX += accel.acceleration.x;
    rawAccelY += accel.acceleration.y;
    rawAccelZ += (accel.acceleration.z - 9.81); // 1g (9.81) on z at stationary
    rawGyroX  += gyr.gyro.x;
    rawGyroY  += gyr.gyro.y;
    rawGyroZ  += gyr.gyro.z;
    delay(5);
  }

  sensorOffsets.offsAccelX = rawAccelX / samples;
  sensorOffsets.offsAccelY = rawAccelY / samples;
  sensorOffsets.offsAccelZ = rawAccelZ / samples;
  sensorOffsets.offsGyroX  = rawGyroX / samples;
  sensorOffsets.offsGyroY  = rawGyroY / samples;
  sensorOffsets.offsGyroZ  = rawGyroZ / samples;

  Serial.println("MPU Calibration Complete.");

  Serial.print("Accelerometer Offsets: ");
  Serial.print(sensorOffsets.offsAccelX); Serial.print(", ");
  Serial.print(sensorOffsets.offsAccelY); Serial.print(", ");
  Serial.println(sensorOffsets.offsAccelZ);

  Serial.print("Gyroscope Offsets: ");
  Serial.print(sensorOffsets.offsGyroX); Serial.print(", ");
  Serial.print(sensorOffsets.offsGyroY); Serial.print(", ");
  Serial.println(sensorOffsets.offsGyroZ);
}

// Data delivery status serial output
void DataDeliveryStat(const uint8_t *mac_addr, esp_now_send_status_t espNOWstatus) {
  
  Serial.print("Last Packet Send Status: \t");
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

  if (!mpuUnit.begin()) {
    Serial.println("MPU6050 Not Found. I2C Connection Failed.");
    while (1) { 
      delay(10);  // Pause code if sensor not found
    } 
  }

  mpuUnit.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpuUnit.setFilterBandwidth(MPU6050_BAND_21_HZ); // Filter out jitter from hand shake
  calibrateMPU();

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA); // Set device as Wi-Fi Station

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialization Failed.");
    return;
  }

  // Register the Arm as receiver
  esp_now_register_send_cb(DataDeliveryStat);
  memcpy(armInfo.peer_addr, armMACAddr, 6); 
  armInfo.channel = 0; // Use current Wi-Fi channel
  armInfo.encrypt = false;
  
  if (esp_now_add_peer(&armInfo) != ESP_OK){
    Serial.println("Connection to Arm Device Failed. Peer Not Added.");
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

  // Read MPU6050 Data
  sensors_event_t accel, gyr, temp;
  mpuUnit.getEvent(&accel, &gyr, &temp);

  float accelX = accel.acceleration.x - sensorOffsets.offsAccelX;
  float accelY = accel.acceleration.y - sensorOffsets.offsAccelY;
  float accelZ = accel.acceleration.z - sensorOffsets.offsAccelZ;

  // Process Data (Calculate Pitch and Roll in degrees)
    // float pitch = -(atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0) / PI;
    // float roll  = (atan2(accelY, accelZ) * 180.0) / PI;
  
  // Prevent overheating
  if (temp.temperature > 40.0) { 
    digitalWrite(PIN_CLAW_LED, HIGH);
    delay(500);
    digitalWrite(PIN_CLAW_LED, LOW);
    delay(500);

    Serial.println("Warning: High Temperature on MPU6050!");
  }

  // Populate Data Structure
  sendData.accelX      = accelX;
  sendData.accelY      = accelY;
  sendData.accelZ      = accelZ;
  sendData.clawGrab    = isGrabbing;
  sendData.clawRelease = isReleasing;

  // Send Data
  esp_now_send(armMACAddr, (uint8_t *) &sendData, sizeof(sendData));
  delay(100);
}