#include "mpu6050.h"
#include <Arduino.h>

// Variáveis globais
float angleX = 0, angleY = 0;
float offsetAngleX = 0, offsetAngleY = 0;
float absAngleX = 0, absAngleY = 0;
unsigned long lastTime = 0;
const float alpha = 0.5;

void MPU6050_setup() {
    Wire.begin();
    Wire.beginTransmission(MPU6050_SENSOR_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

void calibrateMPU6050() {
    offsetAngleX = absAngleX;
    offsetAngleY = absAngleY;
}


void getCalibratedAngles(float &calibratedAngleX, float &calibratedAngleY) {
    int16_t ax, ay, az, gx, gy, gz;
    ax = readRawData(MPU6050_ACCEL_XOUT_H);
    ay = readRawData(MPU6050_ACCEL_XOUT_H + 2);
    az = readRawData(MPU6050_ACCEL_XOUT_H + 4);
    gx = readRawData(MPU6050_GYRO_XOUT_H);
    gy = readRawData(MPU6050_GYRO_XOUT_H + 2);
    gz = readRawData(MPU6050_GYRO_XOUT_H + 4);

    float accelAngleX = atan2(ay, az) * 180 / PI;
    float accelAngleY = atan2(ax, az) * 180 / PI;

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    angleX += gx * deltaTime / 131.0;
    angleY += gy * deltaTime / 131.0;

    angleX = alpha * angleX + (1 - alpha) * accelAngleX;
    angleY = alpha * angleY + (1 - alpha) * accelAngleY;

    absAngleX = abs(angleX);
    absAngleY = abs(angleY);

    calibratedAngleX = abs(absAngleX - offsetAngleX);
    calibratedAngleY = abs(absAngleY - offsetAngleY);
}

int16_t readRawData(int addr) {
    Wire.beginTransmission(MPU6050_SENSOR_ADDR);
    Wire.write(addr);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_SENSOR_ADDR, 2, 1);
    int16_t data = Wire.read() << 8 | Wire.read();
    return data;
}