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
unsigned long readErrorCount = 0;

bool writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

bool readRegister(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU_ADDR, 1, true) != 1) {
    return false;
  }
  val = Wire.read();
  return true;
}

bool readRawAccelerometer(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            // Starting register for accelerometer readings
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  
  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6) {
    return false;
  }

  uint8_t xh = Wire.read();
  uint8_t xl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t yl = Wire.read();
  uint8_t zh = Wire.read();
  uint8_t zl = Wire.read();

  ax = (int16_t)((xh << 8) | xl);
  ay = (int16_t)((yh << 8) | yl);
  az = (int16_t)((zh << 8) | zl);
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU-6500 (PWR_MGMT_1 0x6B = 0x00) and explicitly set +/- 2g range (ACCEL_CONFIG 0x1C = 0x00)
  writeRegister(0x6B, 0x00);
  delay(50);
  writeRegister(0x1C, 0x00);

  uint8_t whoAmI = 0, accelCfg = 0xFF;
  readRegister(0x75, whoAmI);
  readRegister(0x1C, accelCfg);

  delay(100);
  Serial.println("--- MPU-6500 Calibrated Accelerometer & Static Roll Monitor ---");
}

void loop() {
  if (!readRawAccelerometer(rawAcX, rawAcY, rawAcZ)) {
    readErrorCount++;
    Serial.print("I2C read error (total: ");
    Serial.print(readErrorCount);
    Serial.println(")");
    delay(200);
    return;
  }

  // 1. Convert raw counts to calibrated physical units (g)
  float ax = (rawAcX - BIAS_X) / SCALE_X;
  float ay = (rawAcY - BIAS_Y) / SCALE_Y;
  float az = (rawAcZ - BIAS_Z) / SCALE_Z;

  // 2. Calculate total gravity vector norm |a| = sqrt(ax^2 + ay^2 + az^2)
  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  // 3. Estimate static Roll tilt angle around the X-axis in degrees
  float roll_deg = atan2(ay, az) * 180.0f / PI;

  // 4. Print human-readable values for bench verification
  Serial.print("ax: ");   Serial.print(ax, 2);
  Serial.print(" g | ay: "); Serial.print(ay, 2);
  Serial.print(" g | az: "); Serial.print(az, 2);
  Serial.print(" g | Mag: "); Serial.print(magnitude, 2);
  Serial.print(" g | Roll Angle: "); Serial.print(roll_deg, 1);
  Serial.println(" deg");

  delay(200);
}