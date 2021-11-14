

#include <I2Cdev.h>
#include <L298NX2.h>
#include <ky-040.h>
#include "Adafruit_LIS3DH.h"
#include <Wire.h>


//Change name below
//<p>File myFile;</p><p>// class default I2C address is 0x68
//// specific I2C addresses may be passed as a parameter here
//// AD0 low = 0x68 (default for InvenSense evaluation board)
//// AD0 high = 0x69
//
////MPU6050 accelgyro(0x69); // <-- use for AD0 high</p><p>int pinCS = 10; //CS pin for SD card 

//Insert lis3dh library for accel
//ACCELEROMETER
double scale = 16384; //divide values by MPU6050 sensitivity scale to get readings in g's (m/s^2 / 9.8)
Adafruit_LIS3DH accOne =  Adafruit_LIS3DH();
Adafruit_LIS3DH accTwo =  Adafruit_LIS3DH();

struct accel{
  const Adafruit_LIS3DH a;
};
accel acc []= {
  {accOne},
  {accTwo}
};

#define r1                   10
#define interruptClkPin         3      // This pin must have a minimum 0.47 uF capacitor
//#define ENCODER_DT1         A0      // data pin
#define switchPin         A9      // switch pin (active LOW)
#define maxRot        1      // this example defines two rotaries for this encoder


ky040 encOne = ky040(interruptClkPin, A0, switchPin,maxRot);
ky040 encTwo = ky040(interruptClkPin, A1, switchPin,maxRot);
ky040 encThree = ky040(interruptClkPin, A2, switchPin,maxRot);
ky040 encFour = ky040(interruptClkPin, A3, switchPin,maxRot);


struct encoders{
    const ky040 e;
  };
  encoders enc [] = {
    {encOne},
    {encTwo},
    {encThree},
    {encFour}
  };


const unsigned int IN1 = 5;
const unsigned int IN2 = 6;
const unsigned int EN1 = 4;
const unsigned int IN3 = 8;
const unsigned int IN4 = 9;
const unsigned int EN2 = 7;
L298NX2 motorOne(EN1,IN1,IN2,EN2,IN3,IN4);

const unsigned int IN5 = 11;
const unsigned int IN6 = 12;
const unsigned int EN3 = 10;
const unsigned int IN7 = 25;
const unsigned int IN8 = 28;
const unsigned int EN4 = 24;
L298NX2 motorTwo(EN3,IN5,IN6,EN4,IN7,IN8);

struct mot{
  const L298NX2 m;
};
mot motors [] = {
  {motorOne},
  {motorTwo}
};

void setup() {
  // put your setup code here, to run once:
  delay(1000);

  for(int i = 0; i < 4; i++){
    enc[i].e.AddRotaryCounter(r1,-20,40,.01,0,false);
    enc[i].e.SetRotary(r1);
  }

  Serial.println("LIS3DH test");
  if (! accOne.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start1");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  Serial.println("LIS3DH test");
  if (! accTwo.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start2");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");
  motors[0].m.setSpeed(250);
  motors[1].m.setSpeed(250);

}
void loop() {
  for(int i = 0; i < 2;i++){
    acc[i].a.read();
    Serial.println(acc[i].a.z / scale);
  }
  

  

  
}
