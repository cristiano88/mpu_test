#include <Arduino.h>
#include "mpu6050.h"

#define BUTTON_PIN 13

void setup() {
    Serial.begin(115200);
    MPU6050_setup();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50); // Debounce
        calibrateMPU6050();
        Serial.println("Sensor calibrado.");
        while (digitalRead(BUTTON_PIN) == LOW); // Aguarda soltar o botão
    }

    float calibratedAngleX, calibratedAngleY;
    getCalibratedAngles(calibratedAngleX, calibratedAngleY);

    Serial.print("Ângulo X: ");
    Serial.print(calibratedAngleX);
    Serial.print(" Ângulo Y: ");
    Serial.println(calibratedAngleY);

    delay(30);
}