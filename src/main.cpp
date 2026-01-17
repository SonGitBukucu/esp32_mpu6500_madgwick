/*
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
float yawF = 0; 
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

  float pitchF = atan2(accF.accelX, sqrt(accF.accelY * accF.accelY + accF.accelZ * accF.accelZ)) * 180.0 / PI;
  float rollF  = atan2(accF.accelY, accF.accelZ) * 180.0 / PI;

  // 4. Yaw Math (Gyro Integration)
  if (abs(gyroH.gyroZ) > 0.8) { // Deadzone to reduce drift
    yawH += gyroH.gyroZ * dt;
  }

  if (abs(gyroF.gyroZ) > 0.8) {
    yawF += gyroF.gyroZ * dt;
  }

  // 5. Output for the Plotter
  Serial.print("Pitch:"); Serial.print(pitchF);
  Serial.print(",");
  Serial.print("Roll:");  Serial.print(rollF);
  Serial.print(",");
  Serial.print("Yaw:");   Serial.println(yawF);

  delay(10); // High-speed loop
}
*/
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include <nRF24L01.h>
#include <RF24.h>

#define NRF24_CE   2
#define NRF24_CSN  5 //PLACEHOLDER
#define NRF24_SCK  18
#define NRF24_MISO 19
#define NRF24_MOSI 23

// HSPI (SD Kart için özel pinler)
// SCK: 13, MISO: 34, MOSI: 33, CS: 4
#define SD_SCK  13
#define SD_MISO 34
#define SD_MOSI 33
#define SD_CS   4

SPIClass hspi = SPIClass(HSPI);
SPIClass vspi = SPIClass(VSPI);
unsigned long counter = 0;

void writeTest();
void readTest();

RF24 radio(NRF24_CE,NRF24_CSN);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n--- SPI STRESS TEST START ---");

  // Init HSPI (SD)
  /*
  hspi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, hspi, 4000000)) {
    Serial.println("❌ SD init failed");
    while (1);
  }
  Serial.println("✅ SD init OK (HSPI)");

  // Init VSPI (unused)
  vspi.begin(NRF24_SCK, NRF24_MISO, NRF24_MOSI, NRF24_CSN);
  Serial.println("✅ VSPI init OK");

  SD.remove("/stress.txt");
  */

  Serial.begin(115200);
  delay(1000);

  vspi.begin(NRF24_SCK, NRF24_MISO, NRF24_MOSI, NRF24_CSN);
  radio.begin(&vspi);

  bool ok = radio.isChipConnected();

  Serial.print("NRF connected: ");
  Serial.println(ok ? "YES" : "NO");

  radio.setChannel(90);
  Serial.print("Channel readback: ");
  Serial.println(radio.getChannel());
}

void loop() {
  writeTest();
  readTest();

  counter++;
  if (counter % 10 == 0) {
    Serial.print("OK cycles: ");
    Serial.println(counter);
  }

  delay(20);
}

// --------------------------------

void writeTest() {
  File file = SD.open("/stress.txt", FILE_APPEND);
  if (!file) {
    Serial.println("❌ WRITE OPEN FAIL");
    while (1);
  }

  file.print("LINE ");
  file.println(counter);
  file.close();
}

// --------------------------------

void readTest() {
  File file = SD.open("/stress.txt");
  if (!file) {
    Serial.println("❌ READ OPEN FAIL");
    while (1);
  }

  String lastLine;
  while (file.available()) {
    lastLine = file.readStringUntil('\n');
  }
  file.close();

  if (!lastLine.startsWith("LINE")) {
    Serial.println("❌ DATA CORRUPTION");
    Serial.println(lastLine);
    while (1);
  }
}