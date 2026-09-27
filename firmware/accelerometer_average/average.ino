#include <Wire.h>

const int MPU_addr = 0x68;
const int N = 200;

int16_t AcX, AcY, AcZ;

void readAccelerometer() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);               // ACCEL_XOUT_H
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_addr, 6, true);

  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU-6500
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(100);

  Serial.println("Starting 200-sample accelerometer average...");
}

void loop() {

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for (int i = 0; i < N; i++) {

    readAccelerometer();

    sumX += AcX;
    sumY += AcY;
    sumZ += AcZ;

    delay(10);
  }

  float meanX = sumX / (float)N;
  float meanY = sumY / (float)N;
  float meanZ = sumZ / (float)N;

  Serial.println();
  Serial.println("---- Average ----");

  Serial.print("AcX = ");
  Serial.println(meanX);

  Serial.print("AcY = ");
  Serial.println(meanY);

  Serial.print("AcZ = ");
  Serial.println(meanZ);

  Serial.println("-----------------");
  Serial.println("Reposition sensor. Next measurement in 5 seconds.");

  delay(5000);
}