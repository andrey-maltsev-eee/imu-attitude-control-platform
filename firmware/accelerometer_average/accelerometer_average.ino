#include <Wire.h>

const int MPU_ADDR = 0x68;
const int N = 200;

int16_t AcX, AcY, AcZ;
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

bool readAccelerometer(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
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
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU-6500 (PWR_MGMT_1 = 0x00) and explicitly set +/- 2g range (ACCEL_CONFIG 0x1C = 0x00)
  writeRegister(0x6B, 0x00);
  delay(50);
  writeRegister(0x1C, 0x00);

  uint8_t whoAmI = 0, accelCfg = 0xFF;
  readRegister(0x75, whoAmI);
  readRegister(0x1C, accelCfg);

  delay(500);
}

void loop() {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  int validSamples = 0;

  while (validSamples < N) {
    if (readAccelerometer(AcX, AcY, AcZ)) {
      sumX += AcX;
      sumY += AcY;
      sumZ += AcZ;
      validSamples++;
    } else {
      readErrorCount++;
    }
    delay(10);
  }

  float meanX = sumX / (float)N;
  float meanY = sumY / (float)N;
  float meanZ = sumZ / (float)N;

  Serial.println("---- Average ----");
  Serial.print("AcX = "); Serial.println(meanX);
  Serial.print("AcY = "); Serial.println(meanY);
  Serial.print("AcZ = "); Serial.println(meanZ);
  if (readErrorCount > 0) {
    Serial.print("I2C Read Errors = "); Serial.println(readErrorCount);
  }
  Serial.println("-----------------");
  Serial.println("Reposition sensor. Next measurement in 5 seconds.");
  Serial.println();

  delay(5000);
}