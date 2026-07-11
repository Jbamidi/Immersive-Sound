// BNO080 head-tracking IMU on the Arduino UNO Q (SKU ABX00162).
// The sketch runs on the Uno Q's STM32 microcontroller (3.3 V logic).
//
// Wiring (BNO080 breakout -> Uno Q):
//   3V3 -> +3V3   (power the sensor at 3.3 V, NOT +5V)
//   GND -> GND
//   SDA -> SDA    (digital pin D20, the pin labelled SDA next to AREF)
//   SCL -> SCL    (digital pin D21, the pin labelled SCL next to AREF)
//   INT -> D2     (optional, unused here; the loop polls instead)
//   RST -> D3     (optional)
// The pins labelled SDA/SCL are what the default Wire object targets.
// If you plug the sensor into the Uno Q QWIIC connector instead, that is a
// different bus (I2C4 on MCU pins PD13/PD12): uncomment the setSDA/setSCL
// lines in setup() to route the default Wire there.

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// BNO080 default I2C address is 0x4B. Close the ADR jumper for 0x4A.
const uint8_t BNO080_I2C_ADDR = 0x4B;

// Sensor report intervals in milliseconds. 20 ms == 50 Hz.
const uint16_t ACCEL_INTERVAL_MS    = 20;
const uint16_t GYRO_INTERVAL_MS     = 20;
const uint16_t ROTATION_INTERVAL_MS = 20;

const uint32_t PRINT_INTERVAL_MS = 100;

BNO080 imu;

struct Accel {
  float x_mps2, y_mps2, z_mps2;
};

struct Gyro {
  float x_rps, y_rps, z_rps;
};

struct Orientation {
  float qi, qj, qk, qr;
  float accuracy_rad;
  float roll_deg, pitch_deg, yaw_deg;
};

Accel accel;
Gyro gyro;
Orientation orient;

uint32_t lastPrintMs = 0;

static void quatToEuler(float qi, float qj, float qk, float qr,
                        float &roll, float &pitch, float &yaw) {
  float sinr_cosp = 2.0f * (qr * qi + qj * qk);
  float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (qr * qj - qk * qi);
  if (sinp >= 1.0f)       pitch =  PI / 2.0f;
  else if (sinp <= -1.0f) pitch = -PI / 2.0f;
  else                    pitch = asin(sinp);

  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  yaw = atan2(siny_cosp, cosy_cosp);

  roll  *= 180.0f / PI;
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI;
}

void setup() {
  Serial.begin(115200);
  // The Uno Q enumerates as native USB serial; wait briefly (but not forever)
  // for the monitor so the startup messages are not lost.
  uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000) { }

  // Uncomment to route the default Wire to the QWIIC connector (I2C4) instead
  // of the SDA/SCL header pins:
  //   Wire.setSDA(PD13);  // QWIIC I2C4_SDA
  //   Wire.setSCL(PD12);  // QWIIC I2C4_SCL
  Wire.begin();
  Wire.setClock(400000);

  if (imu.begin(BNO080_I2C_ADDR, Wire) == false) {
    Serial.println(F("BNO080 not detected on I2C. Check wiring and address."));
    while (true) delay(1000);
  }

  imu.enableAccelerometer(ACCEL_INTERVAL_MS);
  imu.enableGyro(GYRO_INTERVAL_MS);
  imu.enableRotationVector(ROTATION_INTERVAL_MS);

  Serial.println(F("BNO080 initialised."));
}

void loop() {
  if (imu.dataAvailable() == false) return;

  accel.x_mps2 = imu.getAccelX();
  accel.y_mps2 = imu.getAccelY();
  accel.z_mps2 = imu.getAccelZ();

  gyro.x_rps = imu.getGyroX();
  gyro.y_rps = imu.getGyroY();
  gyro.z_rps = imu.getGyroZ();

  orient.qi = imu.getQuatI();
  orient.qj = imu.getQuatJ();
  orient.qk = imu.getQuatK();
  orient.qr = imu.getQuatReal();
  orient.accuracy_rad = imu.getQuatRadianAccuracy();
  quatToEuler(orient.qi, orient.qj, orient.qk, orient.qr,
              orient.roll_deg, orient.pitch_deg, orient.yaw_deg);

  if (millis() - lastPrintMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = millis();

  Serial.print("A[m/s^2] ");
  Serial.print(accel.x_mps2, 3); Serial.print(" ");
  Serial.print(accel.y_mps2, 3); Serial.print(" ");
  Serial.print(accel.z_mps2, 3);

  Serial.print(" | G[rad/s] ");
  Serial.print(gyro.x_rps, 3); Serial.print(" ");
  Serial.print(gyro.y_rps, 3); Serial.print(" ");
  Serial.print(gyro.z_rps, 3);

  Serial.print(" | RPY[deg] ");
  Serial.print(orient.roll_deg,  2); Serial.print(" ");
  Serial.print(orient.pitch_deg, 2); Serial.print(" ");
  Serial.print(orient.yaw_deg,   2);

  Serial.print(" | acc[rad] ");
  Serial.println(orient.accuracy_rad, 3);
}