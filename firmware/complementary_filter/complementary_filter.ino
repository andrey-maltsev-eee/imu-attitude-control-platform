#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of MPU-6500

// Accelerometer calibration parameters from 6-position test (27 Sept 2026)
const float BIAS_X  =  263.74f;
const float SCALE_X = 16427.78f;
const float BIAS_Y  =  276.66f;
const float SCALE_Y = 16405.48f;
const float BIAS_Z  = -697.16f;
const float SCALE_Z = 16627.08f;

// Gyroscope sensitivity for default +/- 250 deg/s full-scale range
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

// Read all 14 bytes (Accel + Temp + Gyro) in a single I2C transaction
void readRawIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            // Start at ACCEL_XOUT_H register
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_ADDR, 14, true);
  if (Wire.available() >= 14) {
    rawAcX = (Wire.read() << 8) | Wire.read();
    rawAcY = (Wire.read() << 8) | Wire.read();
    rawAcZ = (Wire.read() << 8) | Wire.read();
    rawTmp = (Wire.read() << 8) | Wire.read();
    rawGyX = (Wire.read() << 8) | Wire.read();
    rawGyY = (Wire.read() << 8) | Wire.read();
    rawGyZ = (Wire.read() << 8) | Wire.read();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU-6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(200);

  // --- Step 1: Measure stationary gyroscope bias (500 samples) ---
  // Keep the sensor completely still on the table during startup!
  long sumGyX = 0;
  const int calSamples = 500;
  for (int i = 0; i < calSamples; i++) {
    readRawIMU();
    sumGyX += rawGyX;
    delay(3);
  }
  gyroBiasX = (float)sumGyX / (float)calSamples;

  // --- Step 2: Initialise starting angles from accelerometer ---
  readRawIMU();
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
  // Calculate exact time step dt in seconds
  unsigned long nowMicros = micros();
  float dt = (nowMicros - prevMicros) / 1000000.0f;
  prevMicros = nowMicros;
  float elapsed_s = (nowMicros - startMicros) / 1000000.0f;

  readRawIMU();

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

  delay(20); // ~50 Hz sampling loop
}