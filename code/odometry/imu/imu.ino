#include <Arduino.h>
#include <Wire.h>
#include <TimerOne.h>

const uint8_t kMPU_9250Address       = 0x68;
const uint8_t kMAG_Address           = 0x0c;

const uint8_t kGyro_Full_Scale_250   = 0x00;
const uint8_t kGyro_Full_Scale_500   = 0x08;
const uint8_t kGyro_Full_Scale_750   = 0x10;
const uint8_t kGyro_Full_Scale_1000  = 0x18;

const uint8_t kAcc_Full_Scale_2_G    = 0x00;
const uint8_t kAcc_Full_Scale_4_G    = 0x08;
const uint8_t kAcc_Full_Scale_8_G    = 0x10;
const uint8_t kAcc_Full_Scale_16_G   = 0x18;

uint8_t data[100];
uint64_t ti; // time
volatile bool intFlag = false;

void I2CWriteByte(uint8_t address, uint8_t registers, uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(registers);
    Wire.write(data);
    Wire.endTransmission();
}

void I2Cread(uint8_t address, uint8_t registers, uint8_t n_bytes, uint8_t* data) {
    Wire.beginTransmission(address);
    Wire.write(registers);
    Wire.endTransmission();


    Wire.requestFrom(address, n_bytes);
    uint8_t index = 0;
    while(Wire.available()){
        data[index] = Wire.read();
        index++;
    }
}

void setup() {
    Wire.begin();
    Serial.begin(115200);

    //set accelerometers at low pass filter (5Hz)
    I2CWriteByte(kMPU_9250Address,29,0x06);
    // Set gyroscope low pass filter at 5Hz
    I2CWriteByte(kMPU_9250Address,26,0x06);

    I2CWriteByte(kMPU_9250Address,27,kGyro_Full_Scale_1000);

    I2CWriteByte(kMPU_9250Address, 28, kAcc_Full_Scale_4_G);

    I2CWriteByte(kMPU_9250Address,0x37,0x02);

    I2CWriteByte(kMAG_Address, 0x0A,0x16);

    pinMode(13, OUTPUT);
    Timer1.initialize(10000);
    Timer1.attachInterrupt(callback);

    ti = millis();
}

uint64_t cpt = 0;

void callback() {
    intFlag = true;
    digitalWrite(13, digitalRead(13)^1);
}

void loop() {
    while(!intFlag);
    intFlag = false;

    uint8_t buffer[14];
    I2Cread(kMPU_9250Address, 0x3B, 14, buffer);

    int16_t ax =- (buffer[0]<<8 | buffer[1]);
    int16_t ay =- (buffer[2]<<8 | buffer[3]);
    int16_t az =  (buffer[4]<<8 | buffer[5]);

    int16_t gx=-(buffer[8]<<8  | buffer[9]);
    int16_t gy=-(buffer[10]<<8 | buffer[11]);
    int16_t gz= (buffer[12]<<8 | buffer[13]);

    Serial.print (ax,DEC);
    Serial.print ("\t");
    Serial.print (ay,DEC);
    Serial.print ("\t");
    Serial.print (az,DEC);
    Serial.print ("\t");

    // Gyroscope
    Serial.print (gx,DEC);
    Serial.print ("\t");
    Serial.print (gy,DEC);
    Serial.print ("\t");
    Serial.print (gz,DEC);
    Serial.print ("\t");

    uint8_t ST1;
    do
    {
        I2Cread(kMAG_Address,0x02,1,&ST1);
    }
    while (!(ST1&0x01));

    // Read magnetometer data
    uint8_t Mag[7];
    I2Cread(kMAG_Address,0x03,7,Mag);


    // Create 16 bits values from 8 bits data

    // Magnetometer
    int16_t mx=-(Mag[3]<<8 | Mag[2]);
    int16_t my=-(Mag[1]<<8 | Mag[0]);
    int16_t mz=-(Mag[5]<<8 | Mag[4]);


    // Magnetometer
    Serial.print (mx+200,DEC);
    Serial.print ("\t");
    Serial.print (my-70,DEC);
    Serial.print ("\t");
    Serial.print (mz-700,DEC);
    Serial.print ("\t");



    // End of line
    Serial.println("");
  delay(100);
}
