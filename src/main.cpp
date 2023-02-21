#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Pixy2I2C.h"

Pixy2I2C pixy;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;   // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
VectorInt16 aa;   // [x, y, z]            accel sensor measurements
VectorInt16 gy;   // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;   // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;   // [x, y, z]            gravity vector
float euler[3];   // [psi, theta, phi]    Euler angle container
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;   // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
      mpuInterrupt = true;
}

void pixy_get();
void imu_get();

int16_t yaw = 0;
uint8_t yaw_plus = 0, yaw_minus = 0;

int16_t yellow_angle = 0, blue_angle = 0, yellow_wide = 0, blue_wide = 0, tmp_yellow_wide = 0, tmp_blue_wide = 0, old_yellow_wide = 0, old_blue_wide = 0;

void setup() {
      Serial.begin(28800);

      // IMU
      //  join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);   // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
#endif

      mpu.initialize();
      if (mpu.testConnection() != true) {
            return;   // 接続失敗
      }

      devStatus = mpu.dmpInitialize();

      if (devStatus != 0) {
            return;   // 初期化失敗
      }

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(51);
      mpu.setYGyroOffset(8);
      mpu.setZGyroOffset(21);
      mpu.setXAccelOffset(1150);
      mpu.setYAccelOffset(-50);
      mpu.setZAccelOffset(1060);

      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();

      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      // cam
      pixy.init();

      TIMSK0 = 0;
}

void loop() {
      imu_get();
      pixy_get();

      Serial.write('a');
      Serial.write(yaw_plus);
      Serial.write(yaw_minus);
      Serial.flush();
}
void imu_get() {
      if (!dmpReady) return;
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {   // Get the Latest packet
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            yaw = ypr[0] * 180 / M_PI;
            yaw_plus = yaw > 0 ? yaw : 0;
            yaw_minus = yaw < 0 ? yaw * -1 : 0;
      }
}
void pixy_get() {
      pixy.ccc.getBlocks();
      if (pixy.ccc.numBlocks) {
            for (int count = 0; count < pixy.ccc.numBlocks; count++) {
                  if (pixy.ccc.blocks[count].m_signature == 1 && pixy.ccc.blocks[count].m_y > 100) {
                        old_yellow_wide = tmp_yellow_wide;
                        tmp_yellow_wide = pixy.ccc.blocks[count].m_width;
                        if (tmp_yellow_wide > old_yellow_wide) {
                              yellow_angle = pixy.ccc.blocks[count].m_x / 1.5;
                              yellow_wide = tmp_yellow_wide / 5;
                        }
                  }
                  if (pixy.ccc.blocks[count].m_signature == 2 && pixy.ccc.blocks[count].m_y > 100) {
                        old_blue_wide = tmp_blue_wide;
                        tmp_blue_wide = pixy.ccc.blocks[count].m_width;
                        if (tmp_blue_wide > old_blue_wide) {
                              blue_angle = pixy.ccc.blocks[count].m_x / 1.5;
                              blue_wide = tmp_blue_wide / 5;
                        }
                  }
            }
      }
}