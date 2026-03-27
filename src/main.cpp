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
  float veloX;
  float veloY;
  float veloZ;
  bool  clawGrab;
  bool  clawRelease;
} struct_message;

struct_message sendData;

// MPU6050 Setup
  // Calibration Variables
Adafruit_MPU6050  mpuModule;

struct Offsets {
  float offsGyroX,  offsGyroY,  offsGyroZ;
  float offsAccelX, offsAccelY, offsAccelZ;
};

Offsets sensorOffsets = {0, 0, 0, 
                        0, 0, 0
};

  // Calibration Function
void calibrateMPU() {
  const int samples = 500;
  Serial.println("Calibrating MPU... Maintain still and level. ");
  
  float rawGyroX = 0,  rawGyroY = 0,  rawGyroZ = 0;
  float rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;

  for (int i = 0; i < samples; i++) {
    sensors_event_t    gyr,  accel,  temp;
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

  Serial.print("Gyroscope Offsets:     ");
  Serial.print(sensorOffsets.offsGyroX); Serial.print(", ");
  Serial.print(sensorOffsets.offsGyroY); Serial.print(", ");
  Serial.println(sensorOffsets.offsGyroZ);

  Serial.print("Accelerometer Offsets: ");
  Serial.print(sensorOffsets.offsAccelX); Serial.print(", ");
  Serial.print(sensorOffsets.offsAccelY); Serial.print(", ");
  Serial.println(sensorOffsets.offsAccelZ);
}

// Data delivery status serial output
void DataDeliveryStat(const uint8_t * armMACAddr, esp_now_send_status_t espNOWstatus) {
  Serial.print("Packet Send Status: \t");
  Serial.println(espNOWstatus == ESP_NOW_SEND_SUCCESS ? "Delivery Successful" : "Delivery Failed");
}

// Assign time and velocity variables globally to prevent reset on loop iteration
unsigned long prevMicros = 0;
float veloX = 0, veloY = 0, veloZ = 0;

// Sensor data deadzones to prevent small hand movements from affecting  arm
const float accelDeadZone = 0.02f;
const float tiltDeadZone = 0.03f;

float addDeadZone(float val, float threshold) {
    return (fabs(val) < threshold) ? 0.0f : val;
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
  WiFi.mode(WIFI_STA); // Set device as Wi-Fi Station

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialization Failed.");
    return;
  } 
  else if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Initialization Complete.");
    digitalWrite(PIN_CLAW_LED, HIGH);
    delay(150);
    digitalWrite(PIN_CLAW_LED, LOW);
  }

  // Serial display MAC address
  Serial.print("Controller Device MAC Address: ");
  Serial.print(WiFi.macAddress());
  Serial.print(" | Arm Device MAC Address: ");
  Serial.println(armMACAddr[5]);

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
  }
}

void loop() {
  // dt calculation for velocity
  unsigned long currentMicros = micros();

  if (prevMicros == 0) {
    prevMicros = currentMicros;
    return;
  }

  float dt = (currentMicros - prevMicros) / 1000000.0; 
  prevMicros = currentMicros; 
  
  // Read MPU6050 data
  sensors_event_t    gyr,  accel,  temp;
  mpuModule.getEvent(&gyr, &accel, &temp);

  // Overheat Warning
  float tempdegC = temp.temperature;

  if (tempdegC > 37.5) { 
    digitalWrite(PIN_CLAW_LED, HIGH);
    delay(750);
    digitalWrite(PIN_CLAW_LED, LOW);
    delay(750);
    Serial.println("Warning: High Temperature on MPU6050!");
  }

  // Process data w/ offsets
  float tiltX = addDeadZone(gyr.gyro.x - sensorOffsets.offsGyroX, tiltDeadZone);
  float tiltY = addDeadZone(gyr.gyro.y - sensorOffsets.offsGyroY, tiltDeadZone);
  float tiltZ = addDeadZone(gyr.gyro.z - sensorOffsets.offsGyroZ, tiltDeadZone);

  float accelX = accel.acceleration.x - sensorOffsets.offsAccelX;
  float accelY = accel.acceleration.y - sensorOffsets.offsAccelY;
  float accelZ = accel.acceleration.z - sensorOffsets.offsAccelZ;

  // Velocity calculation
  veloX = accelX * dt; 
  veloY = accelY * dt; 
  veloZ = accelZ * dt; 

  // Read SW2&3 (LOW = pressed, for Input Pull-Up)
  bool isGrabbing  = !digitalRead(PIN_SW2_GRAB);
  bool isReleasing = !digitalRead(PIN_SW3_RELS);

  // Control Claw LED
  if (isGrabbing || isReleasing) {
    digitalWrite(PIN_CLAW_LED, HIGH);
  } 
  else {
    digitalWrite(PIN_CLAW_LED, LOW);
  }

  // Serial output all sent data
  Serial.print("Tilt: \t");
  Serial.print(tiltX); Serial.print(", ");
  Serial.print(tiltY); Serial.print(", ");
  Serial.println(tiltZ);
  Serial.print("Velo: \t");
  Serial.print(veloX); Serial.print(", ");
  Serial.print(veloY); Serial.print(", "); 
  Serial.println(veloZ);
  Serial.print("dt: \t"); Serial.println(dt);
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

  // Data structure
  sendData.tiltX       = tiltX;
  sendData.tiltY       = tiltY;
  sendData.tiltZ       = tiltZ;
  sendData.veloX       = veloX;
  sendData.veloY       = veloY;
  sendData.veloZ       = veloZ;
  sendData.clawGrab    = isGrabbing;
  sendData.clawRelease = isReleasing;

  // Send command
  esp_now_send(armMACAddr, (uint8_t *) &sendData, sizeof(sendData));
  delay(250);
}