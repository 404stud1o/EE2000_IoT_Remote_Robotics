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

uint8_t armMACAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
    float accelX;
    float accelY;
    float accelZ;
    bool clawGrab;
    bool clawRelease;
} struct_message;

struct_message sendData;
esp_now_peer_info_t armInfo;
Adafruit_MPU6050 mpuModule;

void DataDeliveryStat(const uint8_t *mac_addr, esp_now_send_status_t espNOWstatus) {
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
        while (1) {
            delay(10);
        } 
    }

    WiFi.mode(WIFI_STN);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Initialization Failed.");
        return;
    }

    esp_now_register_send_cb(DataDeliveryStat);
    memcpy(armInfo.peer_addr, armMACAddr, 6);
    armInfo.channel = 0;
    armInfo.encrypt = false;

    if (esp_now_add_peer(&armInfo) != ESP_OK) {
        Serial.println("Connection to Arm Failed. Arm Not Added.");
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

    // Read MPU6050 Data
    sensors_event_t accel, gyro, temp;
    mpuModule.getEvent(&accel, &gyro, &temp);

    float accelX = accel.acceleration.x;
    float accelY = accel.acceleration.y;
    float accelZ = accel.acceleration.z;

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
    delay(100);
}