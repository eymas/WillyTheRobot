/* MPU9250 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
  Modified by Brent Wilkins July 19, 2016

  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
  Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
  and the Teensy 3.1.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.

  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND
*/

#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS false         // Set to false for basic data read
const int kQuaternionMultFact = 100;

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

uint8_t message_count = 0;

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  while (!Serial) {};

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);


  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x73) // WHO_AM_I should always be 0x71, but the chip used could be a knockoff - TODO document issues.
  {
    // Start by performing self test
    myIMU.MPU9250SelfTest(myIMU.selfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);


    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if (d != 0x48)
    {
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
    
  } // if (c == 0x73)
  else
  {
    abort();
  }
}

void loop()
{
  int16_t accel_data[3];
  int16_t gyro_data[3];
  int16_t mag_data[3];
  for(uint8_t i = 0; i < 3; i++) {
    accel_data[i] = myIMU.accelCount[i];
    gyro_data[i] = myIMU.gyroCount[i];
    mag_data[i] = myIMU.magCount[i];
  }
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    
    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)accel_data[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)accel_data[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)accel_data[2] * myIMU.aRes; // - myIMU.accelBias[2];
    
    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)gyro_data[0] * myIMU.gRes;
    myIMU.gy = (float)gyro_data[1] * myIMU.gRes;
    myIMU.gz = (float)gyro_data[2] * myIMU.gRes;
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)mag_data[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)mag_data[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)mag_data[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //madgwick is more intensive, but more accurate. If the arduino cannot deal with the amount of calculations
  //switch to Mahony.
  MadgwickQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  //MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
  //                       myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
  //                       myIMU.mx, myIMU.mz, myIMU.deltat);

  myIMU.count = millis();
  digitalWrite(myLed, !digitalRead(myLed));  // toggle led - kept in to make it easier to check whether or not the arduino has crashed.

  Serial.print('$');
  Serial.print(0x03);
  // transmit quaternion bytes
  // rather than transmit floats, which take a lot of bytes (4 each) instead they are multiplied by 100 (ignoring everything more than 2 decimal places behind the 0
  // This allows for them to be transmitted as integers, without significant impact.
  // Kind of a workaround, there is probably a neater method out there.
  for(uint8_t i = 0; i < 4; i++) {
    int8_t quaternion_value = (*(getQ() + i)) * kQuaternionMultFact;
    Serial.print(quaternion_value);
  }
  // transmit accelerometer data
  /*
   * The data here is split into two bytes.
   * This is done because serial communication was previously done from a FIFO buffer which was just copied into another buffer
   * In order to minimize the amount of work needed on the receiving end, this format is expanded with the magnetometer values.
   * The Pi expects a high and low byte for every bit of data, so that is what it will receive.
   * All of the sensor readings are in reality 16 bit integers, split into two 8-bit integers for the sake of transmission
   * The bytes are labeled "xxx_high" and "xxx_low" for the sake of clarity.
   * these are then transmitted separately.
   */
  for(uint8_t i = 0; i < 3; i++) {
    uint8_t accel_high = (0xff & (accel_data[i] >> 8));     
    uint8_t accel_low = (0xff & accel_data[i]);
    Serial.print(accel_high);
    Serial.print(accel_low);
  }
  // transmit gyroscope data
  for(uint8_t i = 0; i < 3; i++) {
    uint8_t gyro_high = (0xff & (gyro_data[i] >> 8));
    uint8_t gyro_low = (0xff & gyro_data[i]);
    Serial.print(gyro_high);
    Serial.print(gyro_low);
  }
  // transmit magnetometer data
  for(uint8_t i = 0; i < 3; i++) {
    uint8_t mag_high = (0xff & (mag_data[i] >> 8));
    uint8_t mag_low = (0xff & mag_data[i]);
    Serial.print(mag_high);
    Serial.print(mag_low);
  }
  Serial.print(message_count);
  Serial.print("\r\n"); // terminating characters for the raspberry Pi
  if(message_count >= 255) { //message count loops on 255
    message_count = 0;
  } else {
    message_count++;
  }
  myIMU.count = millis();
  myIMU.sumCount = 0;
  myIMU.sum = 0;
  delay(5);
}
