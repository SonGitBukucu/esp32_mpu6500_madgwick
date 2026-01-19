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

float yawHand = 0;        // The total angle we've turned
unsigned long lastMicros; // To track time

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
  IMU_Hand.update();
  IMU_Hand.getGyro(&gyroH);

  // 1. Calculate how much time has passed (Delta Time)
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0; // Convert to seconds
  lastMicros = currentMicros;

  // 2. Integration: New Angle = Old Angle + (Speed * Time)
  // We use a small "deadzone" (0.5) to stop tiny noise from drifting
  if (abs(gyroH.gyroZ) > 0.5) { 
    yawHand += gyroH.gyroZ * dt;
  }

  Serial.print("Hand_Yaw:"); 
  Serial.println(yawHand);

  delay(10);
}