#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6500

// Preliminary calibration constants from the 6-position dataset (27 Sept 2026)
const float BIAS_X  =  263.74f;
const float SCALE_X = 16427.78f;

const float BIAS_Y  =  276.66f;
const float SCALE_Y = 16405.48f;

const float BIAS_Z  = -697.16f;
const float SCALE_Z = 16627.08f;

int16_t rawAcX, rawAcY, rawAcZ;

void readRawAccelerometer() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            // Starting register for accelerometer readings
  Wire.endTransmission(false); // Keep connection active for burst read
  
  Wire.requestFrom(MPU_ADDR, 6, true); // Request 6 bytes (X, Y, Z high and low bytes)
  if (Wire.available() >= 6) {
    rawAcX = (Wire.read() << 8) | Wire.read();
    rawAcY = (Wire.read() << 8) | Wire.read();
    rawAcZ = (Wire.read() << 8) | Wire.read();
  }
}

void setup() {
  Serial.begin(115200); // Faster speed for real-time telemetry
  Wire.begin();

  // Wake up MPU-6500 (write 0 to PWR_MGMT_1 register 0x6B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);
  Serial.println("ax_g, ay_g, az_g, magnitude_g, roll_deg");
}

void loop() {
  readRawAccelerometer();

  // 1. Convert raw counts to calibrated physical units (g)
  float ax = (rawAcX - BIAS_X) / SCALE_X;
  float ay = (rawAcY - BIAS_Y) / SCALE_Y;
  float az = (rawAcZ - BIAS_Z) / SCALE_Z;

  // 2. Calculate total gravity vector norm |a| = sqrt(ax^2 + ay^2 + az^2)
  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  // 3. Estimate static Roll tilt angle around the X-axis in degrees
  float roll_deg = atan2(ay, az) * 180.0f / PI;

  // 4. Print values for viewing and logging
  Serial.print("ax: ");   Serial.print(ax, 2);
  Serial.print(" g | ay: "); Serial.print(ay, 2);
  Serial.print(" g | az: "); Serial.print(az, 2);
  Serial.print(" g | Mag: "); Serial.print(magnitude, 2);
  Serial.print(" g | Roll Angle: "); Serial.print(roll_deg, 1);
  Serial.println(" deg");

  delay(200); // Update 5 times per second for comfortable reading
}