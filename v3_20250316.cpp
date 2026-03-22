#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PIN_SW2_GRAB 4
#define PIN_SW3_RELS 13
#define PIN_CLAW_LED 18
#define PIN_6050_SDA 21
#define PIN_6050_SCL 22

uint8_t armMACAddr[] = {0x44, 0x1D, 0x64, 0xF2, 0x1E, 0xA0};
esp_now_peer_info_t armInfo;

typedef struct struct_message {
    float accelX;
    float accelY;
    float accelZ;
    bool clawGrab;
    bool clawRelease;
} struct_message;

struct_message sendData;

Adafruit_MPU6050 mpuModule;

struct Offsets {
    float offsAccelX, offsAccelY, offsAccelZ;
    float offsGyroX, offsGyroY, offsGyroZ;
};
Offsets sensorOffsets = {0, 0, 0,
                         0, 0, 0};

void calibrateMPU() {
    const int samples = 500;
    Serial.println("Calibrating MPU... Maintain still and level. ");

    float rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;
    float rawGyroX = 0, rawGyroY = 0, rawGyroZ = 0;

    for (int i = 0; i < samples; i++) {
        sensors_event_t accel, gyr, temp;
        mpuModule.getEvent(&accel, &gyr, &temp);

        rawAccelX += accel.acceleration.x;
        rawAccelY += accel.acceleration.y;
        rawAccelZ += accel.acceleration.z;
        rawGyroX += gyr.gyro.x;
        rawGyroY += gyr.gyro.y;
        rawGyroZ += gyr.gyro.z;
        delay(5);
    }

    sensorOffsets.offsAccelX = rawAccelX / samples;
    sensorOffsets.offsAccelY = rawAccelY / samples;
    sensorOffsets.offsAccelZ = rawAccelZ / samples;
    sensorOffsets.offsGyroX = rawGyroX / samples;
    sensorOffsets.offsGyroY = rawGyroY / samples;
    sensorOffsets.offsGyroZ = rawGyroZ / samples;

    Serial.println("MPU Calibration Complete.");

    Serial.print("Accelerometer Offsets: ");
    Serial.print(sensorOffsets.offsAccelX);
    Serial.print(", ");
    Serial.print(sensorOffsets.offsAccelY);
    Serial.print(", ");
    Serial.println(sensorOffsets.offsAccelZ);

    Serial.print("Gyroscope Offsets: ");
    Serial.print(sensorOffsets.offsGyroX);
    Serial.print(", ");
    Serial.print(sensorOffsets.offsGyroY);
    Serial.print(", ");
    Serial.println(sensorOffsets.offsGyroZ);
}

void DataDeliveryStat(const uint8_t *mac_addr, esp_now_send_status_t espNOWstatus)
{
    Serial.print("Last Packet Send Status: \t");
    Serial.println(espNOWstatus == ESP_NOW_SEND_SUCCESS ? "Delivery Successful" : "Delivery Failed");
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_SW2_GRAB, INPUT_PULLUP);
    pinMode(PIN_SW3_RELS, INPUT_PULLUP);
    pinMode(PIN_CLAW_LED, OUTPUT);
    digitalWrite(PIN_CLAW_LED, LOW);

    Wire.begin(PIN_6050_SDA, PIN_6050_SCL);

    if (!mpuModule.begin()) {
        Serial.println("MPU6050 Not Found. I2C Connection Failed.");
        while (1)
        {
            delay(10);
        }
    }
    calibrateMPU();

    WiFi.mode(WIFI_STN);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Initialization Failed.");
        return;
    }

    esp_now_register_send_cb(DataDeliveryStat);
    memcpy(armInfo.peer_addr, armMACAddr, sizeof(armMACAddr));
    armInfo.channel = 1;
    armInfo.encrypt = false;

    if (esp_now_add_peer(&armInfo) != ESP_OK) {
        Serial.println("Connection to Arm Device Failed. Arm Not Added.");
        return;
    }
}

void loop() {
    bool isGrabbing  = !digitalRead(PIN_SW2_GRAB);
    bool isReleasing = !digitalRead(PIN_SW3_RELS);

    if (isGrabbing || isReleasing) {
        digitalWrite(PIN_CLAW_LED, HIGH);
    }
    else {
        digitalWrite(PIN_CLAW_LED, LOW);
    }

    sensors_event_t accel, gyr, temp;
    mpuModule.getEvent(&accel, &gyr, &temp);

    float accelX = accel.acceleration.x - sensorOffsets.offsAccelX;
    float accelY = accel.acceleration.y - sensorOffsets.offsAccelY;
    float accelZ = accel.acceleration.z - sensorOffsets.offsAccelZ - 9.81;

    if (temp.temperature > 40.0) {
        digitalWrite(PIN_CLAW_LED, HIGH);
        delay(500);
        digitalWrite(PIN_CLAW_LED, LOW);
        delay(500);

        Serial.println("Warning: High Temperature on MPU6050!");
    }

    sendData.accelX = accelX;
    sendData.accelY = accelY;
    sendData.accelZ = accelZ;
    sendData.clawGrab = isGrabbing;
    sendData.clawRelease = isReleasing;

    esp_now_send(armMACAddr, (uint8_t *)&sendData, sizeof(sendData));
    delay(1000);
}