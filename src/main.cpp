#include <Arduino.h>
#include "FastIMU.h"
#include <Wire.h>

// Define the two addresses
#define ADDR_FOREARM 0x68
#define ADDR_HAND    0x69

// Two separate IMU objects
MPU6500 IMU_Forearm;
MPU6500 IMU_Hand;

// Calibration and Data storage for both
calData calibForearm = { 0 };
calData calibHand = { 0 };

AccelData accF, accH; 
GyroData gyroF, gyroH;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // Initialize Forearm Sensor
  Serial.println("Initializing Forearm IMU (0x68)...");
  int err1 = IMU_Forearm.init(calibForearm, ADDR_FOREARM);
  
  // Initialize Hand Sensor
  Serial.println("Initializing Hand IMU (0x69)...");
  int err2 = IMU_Hand.init(calibHand, ADDR_HAND);

  if (err1 != 0 || err2 != 0) {
    Serial.println("Error initializing one of the IMUs. Check wiring and AD0 pin!");
    while (true);
  }

  // Quick Calibration - KEEP STILL!
  Serial.println("Calibrating both... Keep sensors perfectly level and still.");
  delay(2000);
  IMU_Forearm.calibrateAccelGyro(&calibForearm);
  IMU_Hand.calibrateAccelGyro(&calibHand);
  Serial.println("Calibration Done!");
  
  // Re-init with calibration data
  IMU_Forearm.init(calibForearm, ADDR_FOREARM);
  IMU_Hand.init(calibHand, ADDR_HAND);
}

void loop() {
  // 1. Update both sensors
  IMU_Forearm.update();
  IMU_Hand.update();

  // 2. Get the data from the hardware
  IMU_Forearm.getAccel(&accF);
  IMU_Forearm.getGyro(&gyroF);
  
  IMU_Hand.getAccel(&accH);
  IMU_Hand.getGyro(&gyroH);

  // 3. Print values to Serial Plotter
  // Showing Accel Z (Gravity)
  Serial.print("Forearm_AccelZ:"); Serial.print(accF.accelZ);
  Serial.print(",");
  Serial.print("Hand_AccelZ:"); Serial.print(accH.accelZ);
  Serial.print(",");

  // Showing Gyro Z (The "Screwdriver" Spin)
  Serial.print("Forearm_SpinZ:"); Serial.print(gyroF.gyroZ);
  Serial.print(",");
  Serial.print("Hand_SpinZ:"); Serial.println(gyroH.gyroZ);

  delay(20); 
}