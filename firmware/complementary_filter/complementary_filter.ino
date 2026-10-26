#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of MPU-6500

// Accelerometer calibration parameters from 6-position test (27 Sept 2026)
const float BIAS_X  =  263.74f;
const float SCALE_X = 16427.78f;
const float BIAS_Y  =  276.66f;
const float SCALE_Y = 16405.48f;
const float BIAS_Z  = -697.16f;
const float SCALE_Z = 16627.08f;

// Gyroscope sensitivity for explicit +/- 250 deg/s full-scale range (GYRO_CONFIG = 0x00)
const float GYRO_SCALE = 131.0f; // counts per (deg/s)

// Complementary filter weight (98% gyro, 2% accelerometer)
const float ALPHA = 0.98f;

int16_t rawAcX, rawAcY, rawAcZ;
int16_t rawTmp;
int16_t rawGyX, rawGyY, rawGyZ;

float gyroBiasX = 0.0f;
float gyroRoll  = 0.0f;
float fusedRoll = 0.0f;

unsigned long prevMicros = 0;
unsigned long startMicros = 0;
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

// Read all 14 bytes (Accel + Temp + Gyro) in a single I2C transaction with error checking
bool readRawIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            // Start at ACCEL_XOUT_H register
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) {
    return false;
  }

  uint8_t axh = Wire.read(); uint8_t axl = Wire.read();
  uint8_t ayh = Wire.read(); uint8_t ayl = Wire.read();
  uint8_t azh = Wire.read(); uint8_t azl = Wire.read();
  uint8_t tmh = Wire.read(); uint8_t tml = Wire.read();
  uint8_t gxh = Wire.read(); uint8_t gxl = Wire.read();
  uint8_t gyh = Wire.read(); uint8_t gyl = Wire.read();
  uint8_t gzh = Wire.read(); uint8_t gzl = Wire.read();

  rawAcX = (int16_t)((axh << 8) | axl);
  rawAcY = (int16_t)((ayh << 8) | ayl);
  rawAcZ = (int16_t)((azh << 8) | azl);
  rawTmp = (int16_t)((tmh << 8) | tml);
  rawGyX = (int16_t)((gxh << 8) | gxl);
  rawGyY = (int16_t)((gyh << 8) | gyl);
  rawGyZ = (int16_t)((gzh << 8) | gzl);
  return true;
}

bool configureIMU() {
  // Wake sensor (PWR_MGMT_1 0x6B = 0x00)
  if (!writeRegister(0x6B, 0x00)) return false;
  delay(50);

  // Explicitly set Gyro full-scale to +/- 250 deg/s (GYRO_CONFIG 0x1B = 0x00)
  // and Accel full-scale to +/- 2g (ACCEL_CONFIG 0x1C = 0x00)
  if (!writeRegister(0x1B, 0x00)) return false;
  if (!writeRegister(0x1C, 0x00)) return false;

  // Read back WHO_AM_I and range configuration bits to verify
  uint8_t whoAmI = 0, gyroCfg = 0xFF, accelCfg = 0xFF;
  if (!readRegister(0x75, whoAmI)) return false;
  if (!readRegister(0x1B, gyroCfg)) return false;
  if (!readRegister(0x1C, accelCfg)) return false;

  // Verify WHO_AM_I == 0x70 (MPU-6500) and FS_SEL / AFS_SEL bits [4:3] are 00
  return (whoAmI == 0x70) && ((gyroCfg & 0x18) == 0x00) && ((accelCfg & 0x18) == 0x00);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  configureIMU();
  delay(150);

  // --- Step 1: Measure stationary gyroscope bias (500 valid samples) ---
  // Keep the sensor completely still on the table during startup!
  long sumGyX = 0;
  const int calSamples = 500;
  int validCal = 0;
  while (validCal < calSamples) {
    if (readRawIMU()) {
      sumGyX += rawGyX;
      validCal++;
    } else {
      readErrorCount++;
    }
    delay(3);
  }
  gyroBiasX = (float)sumGyX / (float)calSamples;

  // --- Step 2: Initialise starting angles from a valid accelerometer read ---
  while (!readRawIMU()) {
    readErrorCount++;
    delay(5);
  }
  float ay = (rawAcY - BIAS_Y) / SCALE_Y;
  float az = (rawAcZ - BIAS_Z) / SCALE_Z;
  float initialRoll = atan2(ay, az) * 180.0f / PI;

  gyroRoll  = initialRoll;
  fusedRoll = initialRoll;

  prevMicros  = micros();
  startMicros = prevMicros;

  // Print CSV header
  Serial.println("time_s,acc_roll_deg,gyro_roll_deg,fused_roll_deg");
}

void loop() {
  if (!readRawIMU()) {
    readErrorCount++;
    delay(5);
    return; // Skip integration on failed read so stale omegaX is not integrated
  }

  // Calculate exact time step dt in seconds only after a valid read
  unsigned long nowMicros = micros();
  float dt = (nowMicros - prevMicros) / 1000000.0f;
  prevMicros = nowMicros;
  float elapsed_s = (nowMicros - startMicros) / 1000000.0f;

  // 1. Calibrated accelerometer values (in g) and static tilt angle (in deg)
  float ay = (rawAcY - BIAS_Y) / SCALE_Y;
  float az = (rawAcZ - BIAS_Z) / SCALE_Z;
  float accRoll = atan2(ay, az) * 180.0f / PI;

  // 2. Calibrated gyroscope angular rate around X-axis (in deg/s)
  float omegaX = (rawGyX - gyroBiasX) / GYRO_SCALE;

  // 3. Pure gyroscope integration (accumulates drift over time)
  gyroRoll = gyroRoll + omegaX * dt;

  // 4. Complementary filter (combines fast gyro with drift-free accelerometer)
  fusedRoll = ALPHA * (fusedRoll + omegaX * dt) + (1.0f - ALPHA) * accRoll;

  // 5. Output CSV telemetry row
  Serial.print(elapsed_s, 3);
  Serial.print(",");
  Serial.print(accRoll, 2);
  Serial.print(",");
  Serial.print(gyroRoll, 2);
  Serial.print(",");
  Serial.println(fusedRoll, 2);

  delay(20); // ~41.9 Hz effective polling loop
}