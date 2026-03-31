#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

float yawOffset = 0;
bool calibrated = false;
int sampleCount = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);
    while (!Serial);  // wait for serial on some boards
    
    Serial.println("=== MPU6050 Head Tracker Debug ===");
    
    Serial.print("Initializing MPU6050... ");
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "connected" : "FAILED");
    
    pinMode(2, INPUT);
    
    Serial.print("Initializing DMP... ");
    devStatus = mpu.dmpInitialize();
    Serial.print("status = ");
    Serial.println(devStatus);

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        Serial.println("Calibrating...");
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("DMP ready! Collecting 50 samples for zero reference...");
    } else {
        Serial.print("DMP init FAILED with code: ");
        Serial.println(devStatus);
        Serial.println("1 = initial memory load failed");
        Serial.println("2 = DMP config updates failed");
    }
}

void loop() {
    if (!dmpReady) return;
    if (!mpuInterrupt) return;
    mpuInterrupt = false;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float yawDeg = ypr[0] * 180.0 / M_PI;

        if (!calibrated) {
            sampleCount++;
            Serial.print("Calibrating: ");
            Serial.print(sampleCount);
            Serial.println("/50");
            if (sampleCount == 50) {
                yawOffset = yawDeg;
                calibrated = true;
                Serial.print("Zero set at: ");
                Serial.println(yawOffset);
                Serial.println("--- Streaming yaw ---");
            }
            return;
        }

        float relativeYaw = yawDeg - yawOffset;
        if (relativeYaw < 0) relativeYaw += 360;
        if (relativeYaw >= 360) relativeYaw -= 360;

        Serial.println(relativeYaw, 1);
    }
}
