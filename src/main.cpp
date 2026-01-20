#include <Arduino.h>
#include "FastIMU.h"
#include <Wire.h>

#define ADDR_FOREARM 0x68
#define ADDR_HAND    0x69

MPU6500 IMU_Forearm;
MPU6500 IMU_Hand;

calData calibForearm = { 0 };
calData calibHand = { 0 };

AccelData accF, accH; 
GyroData gyroF, gyroH;

// Angles
float rollHand, pitchHand, yawHand = 0;
float rollForearm, pitchForearm, yawForearm = 0;

unsigned long lastMicros;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  IMU_Forearm.init(calibForearm, ADDR_FOREARM);
  IMU_Hand.init(calibHand, ADDR_HAND);

  Serial.println("Calibrating... Keep IMUs level and still.");
  delay(2000);
  IMU_Forearm.calibrateAccelGyro(&calibForearm);
  IMU_Hand.calibrateAccelGyro(&calibHand);
  
  IMU_Forearm.init(calibForearm, ADDR_FOREARM);
  IMU_Hand.init(calibHand, ADDR_HAND);
  
  lastMicros = micros();
}

void loop() {
  IMU_Hand.update();
  IMU_Hand.getAccel(&accH);
  IMU_Hand.getGyro(&gyroH);

  IMU_Forearm.update();
  IMU_Forearm.getAccel(&accF);
  IMU_Forearm.getGyro(&gyroF);

  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  lastMicros = currentMicros;

  // --- 1. CALCULATE ROLL & PITCH (The "Intended" way using Gravity) ---
  // Hand
  rollHand  = atan2(accH.accelY, accH.accelZ) * 180.0 / PI;
  pitchHand = atan2(-accH.accelX, sqrt(accH.accelY * accH.accelY + accH.accelZ * accH.accelZ)) * 180.0 / PI;

  // Forearm
  rollForearm  = atan2(accF.accelY, accF.accelZ) * 180.0 / PI;
  pitchForearm = atan2(-accF.accelX, sqrt(accF.accelY * accF.accelY + accF.accelZ * accF.accelZ)) * 180.0 / PI;

  // --- 2. CALCULATE YAW (Integration way) ---
  if (abs(gyroH.gyroZ) > 0.5) yawHand += gyroH.gyroZ * dt;
  if (abs(gyroF.gyroZ) > 0.5) yawForearm += gyroF.gyroZ * dt;

  // --- 3. OUTPUT ---
  Serial.print("Hand_Pitch(Tilt):"); Serial.print(pitchHand);
  Serial.print(",Hand_Yaw(Pan):");   Serial.print(yawHand);
  Serial.print(",Fore_Pitch:");      Serial.print(pitchForearm);
  Serial.print(",Fore_Yaw:");        Serial.println(yawForearm);

  delay(10);
}

void angleCalc()