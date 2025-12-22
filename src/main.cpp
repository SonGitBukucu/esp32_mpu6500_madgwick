#include <Arduino.h>
#include <MadgwickAHRS.h>
#include "FastIMU.h"
#include <Wire.h>

// 1. Declare the objects and structs
MPU6500 IMU;
calData calib = { 0 };
AccelData accelData;
GyroData gyroData;
Madgwick filter;

unsigned long lastMicros;
float sampleFreq = 200.0; // We want to run at 200Hz (every 5ms)

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // Initialize IMU
  int err = IMU.init(calib, 0x68);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (1);
  }

  Serial.println("Keep IMU still for calibration...");
  IMU.calibrateAccelGyro(&calib);
  
  // 2. Initialize the filter with our frequency
  filter.begin(sampleFreq); 
  lastMicros = micros();
}

void loop() {
  // 3. Timing control: ensure we run exactly at 200Hz
  if (micros() - lastMicros >= (1000000 / sampleFreq)) {
    lastMicros = micros();

    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    // 4. Update the filter
    // Order: Gyro (deg/s) X, Y, Z then Accel (G) X, Y, Z
    filter.updateIMU(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, 
                     accelData.accelX, accelData.accelY, accelData.accelZ);

    // 5. Output for Serial Plotter
    Serial.print("Roll:");
    Serial.print(filter.getRoll());
    Serial.print(",");
    Serial.print("Pitch:");
    Serial.print(filter.getPitch());
    Serial.print(",");
    Serial.print("Yaw:");
    Serial.println(filter.getYaw());
  }
}