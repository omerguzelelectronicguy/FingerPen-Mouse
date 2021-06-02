//This code is taken from github.com/berkaysaka for IMU calibration
//The link for the original code is: https://github.com/berkaysaka/ArduinoQuadcopterFlightController/blob/master/lessons/3_reading_imu/IMU_Calibration/IMU_Calibration.ino

#define INTERRUPT_PIN 2
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

void setup() {
  Serial.begin(115200);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
  }
}

void loop() {

}
