#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Adafruit ICM-20948 default I2C address is 0x69.
// If you pull the AD0/ADDR pin to GND, change this to 0x68.
#define ICM_I2C_ADDR 0x69

const int CALIBRATION_MS = 3000;
const int PRINT_INTERVAL_MS = 20;

Adafruit_ICM20948 icm;

float magMinX = 1000000.0;
float magMinY = 1000000.0;
float magMinZ = 1000000.0;
float magMaxX = -1000000.0;
float magMaxY = -1000000.0;
float magMaxZ = -1000000.0;

float yawOffset = 0.0;
bool calibrated = false;
unsigned long calibrationStartMs = 0;
unsigned long lastPrintMs = 0;

float wrap360(float angle) {
  while (angle < 0.0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

void updateMagCalibration(float mx, float my, float mz) {
  magMinX = min(magMinX, mx);
  magMinY = min(magMinY, my);
  magMinZ = min(magMinZ, mz);
  magMaxX = max(magMaxX, mx);
  magMaxY = max(magMaxY, my);
  magMaxZ = max(magMaxZ, mz);
}

void applyMagCalibration(float &mx, float &my, float &mz) {
  mx -= (magMinX + magMaxX) * 0.5;
  my -= (magMinY + magMaxY) * 0.5;
  mz -= (magMinZ + magMaxZ) * 0.5;
}

float tiltCompensatedYaw(sensors_event_t accel, sensors_event_t mag) {
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  float mx = mag.magnetic.x;
  float my = mag.magnetic.y;
  float mz = mag.magnetic.z;

  applyMagCalibration(mx, my, mz);

  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt((ay * ay) + (az * az)));

  float magX = (mx * cos(pitch)) + (mz * sin(pitch));
  float magY = (mx * sin(roll) * sin(pitch)) + (my * cos(roll)) - (mz * sin(roll) * cos(pitch));

  return wrap360(atan2(-magY, magX) * 180.0 / PI);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();

  Serial.println(F("=== Adafruit ICM-20948 Head Tracker ==="));
  Serial.println(F("Wiring: VIN=5V, GND=GND, SDA=A4, SCL=A5 on Arduino Uno R3"));

  if (!icm.begin_I2C(ICM_I2C_ADDR)) {
    Serial.println(F("Failed to find ICM-20948."));
    Serial.println(F("Check wiring. If AD0/ADDR is tied to GND, set ICM_I2C_ADDR to 0x68."));
    while (true) delay(1000);
  }

  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  calibrationStartMs = millis();
  Serial.println(F("ICM-20948 found."));
  Serial.println(F("For 3 seconds, slowly rotate/tilt the IMU in a figure-eight to calibrate the magnetometer."));
  Serial.println(F("time_ms,calibrated,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,yaw_abs,yaw_relative"));
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  icm.getEvent(&accel, &gyro, &temp, &mag);

  updateMagCalibration(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  if (!calibrated && millis() - calibrationStartMs >= CALIBRATION_MS) {
    yawOffset = tiltCompensatedYaw(accel, mag);
    calibrated = true;
    Serial.print(F("Zero yaw set to: "));
    Serial.println(yawOffset, 1);
  }

  if (millis() - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }

  lastPrintMs = millis();
  float yaw = tiltCompensatedYaw(accel, mag);
  float relativeYaw = calibrated ? wrap360(yaw - yawOffset) : 0.0;

  Serial.print(millis() - calibrationStartMs);
  Serial.print(F(","));
  Serial.print(calibrated ? 1 : 0);
  Serial.print(F(","));
  Serial.print(accel.acceleration.x, 3);
  Serial.print(F(","));
  Serial.print(accel.acceleration.y, 3);
  Serial.print(F(","));
  Serial.print(accel.acceleration.z, 3);
  Serial.print(F(","));
  Serial.print(gyro.gyro.x, 3);
  Serial.print(F(","));
  Serial.print(gyro.gyro.y, 3);
  Serial.print(F(","));
  Serial.print(gyro.gyro.z, 3);
  Serial.print(F(","));
  Serial.print(mag.magnetic.x, 3);
  Serial.print(F(","));
  Serial.print(mag.magnetic.y, 3);
  Serial.print(F(","));
  Serial.print(mag.magnetic.z, 3);
  Serial.print(F(","));
  Serial.print(yaw, 1);
  Serial.print(F(","));
  Serial.println(relativeYaw, 1);
}
