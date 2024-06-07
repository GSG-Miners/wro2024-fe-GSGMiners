#include <sys/_stdint.h>
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

struct Gyroscope {
  void begin() {
    Wire.begin();

    bool status = mpu.begin();
    while (status != 0) {}
    mpu.setFilterGyroCoef(1.00);
  }

  int16_t readYawAngle() {
    mpu.update();
    return mpu.getAngleZ();
  }
};