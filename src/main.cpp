#include "FastIMU.h"
#include <Wire.h>
#include <Arduino.h>

#define ADDR_FOREARM 0x68
#define ADDR_HAND    0x69

MPU6500 IMU_F;
MPU6500 IMU_H;

calData calF, calH;
AccelData accF, accH; 
GyroData gyroF, gyroH;

// Variables for Angle Tracking
float yawH = 0; 
unsigned long lastMicros;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // Init Sensors
  IMU_F.init(calF, ADDR_FOREARM);
  IMU_H.init(calH, ADDR_HAND);

  // Calibration (Keep sensors still!)
  Serial.println("Calibrating... DO NOT MOVE.");
  delay(2000);
  IMU_F.calibrateAccelGyro(&calF);
  IMU_H.calibrateAccelGyro(&calH);
  
  IMU_F.init(calF, ADDR_FOREARM);
  IMU_H.init(calH, ADDR_HAND);

  lastMicros = micros();
  Serial.println("System Ready!");
}

void loop() {
  // 1. Get Time Delta
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;

  // 2. Update Sensors
  IMU_F.update();
  IMU_H.update();
  IMU_F.getAccel(&accF);
  IMU_H.getAccel(&accH);
  IMU_H.getGyro(&gyroH);

  // 3. Pitch & Roll Math (using atan2 for stability)
  // Converting the gravity vector into degrees
  float pitchH = atan2(accH.accelX, sqrt(accH.accelY * accH.accelY + accH.accelZ * accH.accelZ)) * 180.0 / PI;
  float rollH  = atan2(accH.accelY, accH.accelZ) * 180.0 / PI;

  // 4. Yaw Math (Gyro Integration)
  if (abs(gyroH.gyroZ) > 0.8) { // Deadzone to reduce drift
    yawH += gyroH.gyroZ * dt;
  }

  // 5. Output for the Plotter
  Serial.print("Pitch:"); Serial.print(pitchH);
  Serial.print(",");
  Serial.print("Roll:");  Serial.print(rollH);
  Serial.print(",");
  Serial.print("Yaw:");   Serial.println(yawH);

  delay(10); // High-speed loop
}