// mpu test
#include "MPU9250.h"

//#define MPU_PIN_SDA  18    // I2C default SDA GPIO21
//#define MPU_PIN_SCL  19    // I2C default SCL GPIO22

MPU9250 mpu;      // create a object for the MPU communication


double orientation[3];  // array for the orientaion pitch / roll / yaw


void setup()
{
  Serial.println("setup started");

  // start the serial communication
  Serial.begin(115200);

  // start the I2C communiation for the mpu
  Wire.begin();
  mpu.setup(0x68);
  //calibrate mpu
  Serial.println("setup Gyro");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();
  Serial.println("setup Mag");
  delay(5000);
  mpu.calibrateMag();
  mpu.verbose(false); 
}

void loop()
{
  mpu.update();
  orientation[0] = mpu.getPitch();
  orientation[1] = mpu.getRoll();
  orientation[2] = mpu.getYaw();
  Serial.println(orientation[0]);
  Serial.println(orientation[1]);
  Serial.println(orientation[2]);
}
