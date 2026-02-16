#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

struct Accel {
  float x_g, y_g, z_g;
  void setRaw(int16_t x, int16_t y, int16_t z) {
    x_g = x / 16384.0f;
    y_g = y / 16384.0f;
    z_g = z / 16384.0f;
  }
};

struct Gyro {
  float x_dps, y_dps, z_dps;
  void setRaw(int16_t x, int16_t y, int16_t z) {
    x_dps = x / 131.0f;
    y_dps = y / 131.0f;
    z_dps = z / 131.0f;
  }
};

static void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

Accel accel;
Gyro gyro;
float temp_c = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();
  write8(REG_PWR_MGMT_1, 0x00);
  delay(100);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  int16_t t  = (Wire.read() << 8) | Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  accel.setRaw(ax, ay, az);
  gyro.setRaw(gx, gy, gz);
  temp_c = ((float)t + 521.0f) / 340.0f + 35.0f;

  Serial.print("A[g] ");
  Serial.print(accel.x_g, 3); Serial.print(" ");
  Serial.print(accel.y_g, 3); Serial.print(" ");
  Serial.print(accel.z_g, 3);

  Serial.print(" | G[dps] ");
  Serial.print(gyro.x_dps, 2); Serial.print(" ");
  Serial.print(gyro.y_dps, 2); Serial.print(" ");
  Serial.print(gyro.z_dps, 2);

  Serial.print(" | T[C] ");
  Serial.println(temp_c, 2);

  delay(2000);
}
