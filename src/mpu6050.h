#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>

// Definições para o MPU-6050
#define MPU6050_SENSOR_ADDR 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43


void MPU6050_setup();
void calibrateMPU6050();
void getCalibratedAngles(float &calibratedAngleX, float &calibratedAngleY);
int16_t readRawData(int addr);
#endif