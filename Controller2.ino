

#include <I2Cdev.h>
#include <L298NX2.h>
#include <ky-040.h>
#include "Adafruit_LIS3DH.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>

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
//Adafruit_LIS3DH accOne =  Adafruit_LIS3DH();
//Adafruit_LIS3DH accTwo =  Adafruit_LIS3DH();

Adafruit_MPU6050 accLeft = Adafruit_MPU6050();
Adafruit_MPU6050 accRight = Adafruit_MPU6050();

//struct accel{
//  const Adafruit_LIS3DH a;
//};
//accel acc []= {
//  {accOne},
//  {accTwo}
//};

#define r1                   10
#define interruptClkPin         3      // This pin must have a minimum 0.47 uF capacitor
//#define ENCODER_DT1         A0      // data pin
#define switchPin         A9      // switch pin (active LOW)
#define maxRot        1      // this example defines two rotaries for this encoder
//
#define LEDRED 32
#define LEDGREEN 33
#define LEDBOOL 34
#define loose 41
#define butt 40
bool lsFor = true;
double prevValL = 0;
double prevValR = 0;
bool posL = false;
bool posR = false;
bool first = false;
//int i = 0;
//
ky040 encOne = ky040(interruptClkPin, 38, switchPin,maxRot);
ky040 encTwo = ky040(interruptClkPin, 37, switchPin,maxRot);
ky040 encThree = ky040(interruptClkPin, 36, switchPin,maxRot);
ky040 encFour = ky040(interruptClkPin, 35, switchPin,maxRot);


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
L298NX2 lMotor(EN1,IN1,IN2,EN2,IN3,IN4);

const unsigned int IN5 = 11;
const unsigned int IN6 = 12;
const unsigned int EN3 = 10;
const unsigned int IN7 = 28;
const unsigned int IN8 = 29;
const unsigned int EN4 = 24 ;
L298NX2 rMotor(EN3,IN5,IN6,EN4,IN7,IN8);
//
//struct mot{
//  const L298NX2 m;
//};
//mot motors [] = {
//  {motorOne},
//  {motorTwo}
//};
double b[] = {.9912,-1.9823,.9912};
double a[] = {1,-1.9822,.9824};
double x[3];
double y[3];
double xr[3];
double yr[3];
double startTime = 0;
double sumL = 0;
double sumR = 0;
double initD = 10;
double alpha = 1/ ((2*3.14*((initD/1000)) *.25) + 1);
bool enter = true;

void setup() {
  // put your setup code here, to run once:
  delay(1000);
//  pinMode(LEDRED, OUTPUT);
//  pinMode(LEDGREEN,OUTPUT);
//  pinMode(LEDBOOL, OUTPUT);
  pinMode(butt, INPUT_PULLUP);
   pinMode(loose, INPUT_PULLUP);
   pinMode(4,OUTPUT);
   pinMode(5,OUTPUT);
   pinMode(6,OUTPUT);
   pinMode(7,OUTPUT);
   pinMode(8,OUTPUT);
   pinMode(9,OUTPUT);
   
  if(!accLeft.begin(0x69)){
    Serial.println("Couldnt start Left");
    while (1) yield();
  }


  if(!accRight.begin(0x68)){
    Serial.println("Couldnt start Right");
    while (1) yield();
  }

  
//  Serial.println("found");

  accLeft.setFilterBandwidth(MPU6050_BAND_5_HZ);
  accRight.setFilterBandwidth(MPU6050_BAND_5_HZ);

  startTime = millis();
  for(int i = 0; i < 4; i++){
    enc[i].e.AddRotaryCounter(r1,0,-20,35,1.65,false);
    enc[i].e.SetRotary(r1);
  }

  lMotor.setSpeed(120);
  rMotor.setSpeed(120);
}

int i = 100;
void loop() {
  delay(initD);
//  accOne.read();
while(!digitalRead(loose)){
    Serial.println("LOOSE");
          digitalWrite(4,HIGH);
          digitalWrite(5,HIGH);
          digitalWrite(6,HIGH);
          
          digitalWrite(7,HIGH);
          digitalWrite(8,HIGH);
          digitalWrite(9,HIGH);

          digitalWrite(10,HIGH);
          digitalWrite(11,HIGH);
          digitalWrite(12,HIGH);

          digitalWrite(24,HIGH);
          digitalWrite(28,HIGH);
          digitalWrite(29,HIGH);
          first = false;
}
  sensors_event_t al, gl, templ;
  accLeft.getEvent(&al, &gl, &templ);
if(!digitalRead(butt) || first){
  

//          digitalWrite(4,HIGH);
//          digitalWrite(5,HIGH);
if(i % 400 > 200 && enter){
          lMotor.setSpeedB(120);
          lMotor.setSpeedA(120);
          lMotor.forward();
          enter = false;
}
  if(y[0] > -10) {
//    lMotor.setSpeedB(90);
//          lMotor.s etSpeedA(120);
          lMotor.forward();
  }
  if(y[0] < 15){
//    lMotor.setSpeedA(90);
//  lMotor.setSpeedB(120);
  lMotor.backward();
  }
//          digitalWrite(6,LOW);
//  if(i % 400 > 200){
//          lMotor.setSpeedB(90);
//          lMotor.setSpeedA(120);
//          lMotor.forward();
//          
//          
////          lMotor.reset();
//} else
//{
//  lMotor.setSpeedA(90);
//  lMotor.setSpeedB(120);
//  lMotor.backward();
//
//}
//          digitalWrite(7,HIGH);
//          digitalWrite(8,HIGH);
//          digitalWrite(9,HIGH);

//          digitalWrite(10,HIGH);
//          digitalWrite(11,HIGH);
//          digitalWrite(12,HIGH);
//
//          digitalWrite(24,HIGH);
//          digitalWrite(28,HIGH);
//          digitalWrite(29,LOW);
          
          
//          delay(1000);
//          digitalWrite(4,HIGH);
//          digitalWrite(5,HIGH);
//          digitalWrite(6,HIGH);
//          
//          digitalWrite(7,HIGH);
//          digitalWrite(8,LOW);
//          digitalWrite(9,HIGH);
//          lMotor.backwardFor(1000);
//          lMotor.reset();
//          if (gl.gyro.z < .1 && gl.gyro.z > -.1) {
//    gl.gyro.z = 0;
//  }
    
          sumL += gl.gyro.z;
            Serial.print(gl.gyro.z / 3.14 * 180);
          Serial.print('\t');

  x[0] = sumL;
//  y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2];
y[0] = alpha*y[1] + alpha*(x[0] - x[1]);
//  x[2] = x[1];
  x[1] = x[0];
//  y[2] = y[1];
  y[1] = y[0];

  Serial.println(y[0]);
          i++;
//          digitalWrite(10,HIGH);
//          digitalWrite(11,LOW);
//          digitalWrite(12,HIGH);
//
//          digitalWrite(24,HIGH);
//          digitalWrite(28,HIGH);
//          digitalWrite(29,HIGH);
//          delay(1000);
          first = true;
  }else{

//
//  sensors_event_t a, g, temp;
//  sensors_event_t al, gl, templ;
//  accLeft.getEvent(&al, &gl, &templ);
//  sensors_event_t ar, gr, tempr;
//  accRight.getEvent(&ar, &gr, &tempr);  
//
////  if(lsFor){
//    a = al;
//    g = gl;
////    temp = templ
////  } else {
////    a = ar;
////    g = gr; 
////  }
//  double changeL;
//  double changeR;
////  acc[0].a.read();
////  double xO;
//
//
///*
// * 
// * ASSUMING LEFT LEG FORWARD
// * 
// * When Yellow is HIGH
// *  - Move forward left motor
// *  - Move right back motor
// *  - When red triggers, stop
// *  - Wait for blue
// *  
// *  
//*  When Blue is HIGH
//*   - Move back left motor
//*   - Move front right motor
//*   - When red triggers, stop
//*   - Wait for blue
//*   
//*   
//*   Let out?
//*   After yellow, until blue high spin opposite?
//*/
// 
//
////  if (lsFor){
//
//
//  if (g.gyro.z < .1 && g.gyro.z > -.1) {
//    g.gyro.z = 0;
//  }
//  sumL += g.gyro.z;
//
//  x[0] = sumL;
//  y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2];
//  x[2] = x[1];
//  x[1] = x[0];
//  y[2] = y[1];
//  y[1] = y[0];
//
////  Serial.print(y[0]);
////  Serial.print("\t");
//  
//  if(prevValL == 0){
//      prevValL = g.gyro.z;
//      lMotor.stop();
//  } else {
//    changeL = g.gyro.z - prevValL;
//      if (g.gyro.z > 0) {
//        if (changeL > 0){
//          //YELLOW HIGH 
//          //BACKWARD        
//          lMotor.forward();
////          Serial.print("LBACK");
////          Serial.print('\t');
////          lMotor.backward();
////          rMotor.backward();
////          rMotor.backward(); 
//        } else {
//          //YELLOW LOW
////          Serial.print("LSTOP");
////          Serial.print('\t');
//          digitalWrite(5,HIGH);
//          digitalWrite(6,HIGH);
//          digitalWrite(8,HIGH);
//          digitalWrite(9,HIGH);
////          lMotor.stop();
////          rMotor.stop();
//        }
//          //BLUE LOW
////          lMotor.stop();
////          rMotor.stop();
//      } else if (g.gyro.z < 0){
//        if (changeL < 0){
//          //BLUE HIGH
////          lMotor.backwardA();
////          Serial.print("LFORWARD");
////          Serial.print('\t');
//          lMotor.backward();
////          rMotor.forward();
////          rMotor.backwardA();
//        } else {
//          //BLUE YELLOW LOW
////          Serial.print("LSTOP");
////          Serial.print('\t');
//          digitalWrite(5,HIGH);
//          digitalWrite(6,HIGH);
//          digitalWrite(8,HIGH);
//          digitalWrite(9,HIGH);
////          lMotor.stop();
////          rMotor.stop();
//        }
//      }
//
//    
//    
//    if (changeL > .05) {
//      if(posL == false){
//        //RED HIGH
//          digitalWrite(5,HIGH);
//          digitalWrite(6,HIGH);
//          digitalWrite(8,HIGH);
//          digitalWrite(9,HIGH);
////        lMotor.stop();
////        rMotor.stop();
//      }
//      posL = true;
//    } else if (changeL < -.05){
//      if(posL == true){
//        //RED HIGH
//        lMotor.stop();
////        rMotor.stop();
//      }
//        posL = false;
//      } else {
//          digitalWrite(5,HIGH);
//          digitalWrite(6,HIGH);
//          digitalWrite(8,HIGH);
//          digitalWrite(9,HIGH);
////        lMotor.stop();
//      }
//      prevValL = g.gyro.z;
//    }
//
////  Serial.print(lMotor.isMovingA());
////  Serial.print("\t");
//  
/////////////////////////////////////////////////////
////
////  rMotor.forwardFor(1000);
//  
//  g = gr;
//  a = ar;
//  
//  if (g.gyro.z < .1 && g.gyro.z > -.1) {
//    g.gyro.z = 0;
//  } else {
//    g.gyro.z = - g.gyro.z;
//  }
//  sumR += g.gyro.z;
//
//  xr[0] = sumR;
//  yr[0] = b[0]*xr[0] + b[1]*xr[1] + b[2]*xr[2];
//  xr[2] = xr[1];
//  xr[1] = xr[0];
//  yr[2] = yr[1];
//  yr[1] = yr[0];
//
////  Serial.println(yr[0]);
//  
//  if(prevValR == 0){
//      prevValR = g.gyro.z;
//  } else {
//    changeR = g.gyro.z - prevValR;
////    Serial.print(g.gyro.z);
////    Serial.print("\t");
////    Serial.println(changeR);
//      if (g.gyro.z > 0) {
//        if (changeR > 0){
//          //YELLOW HIGH         
////          lMotor.backward();
////          lMotor.backward();
//          rMotor.backward();
////          rMotor.backward(); 
//        } else {
//          //YELLOW LOW
////          lMotor.stop();
//          digitalWrite(11,HIGH);
//          digitalWrite(12,HIGH);
//          digitalWrite(28,HIGH);
//          digitalWrite(29,HIGH);
////          rMotor.stop();
//          }
//          //BLUE LOW
////          lMotor.stop();
////          rMotor.stop();
//      } else {
//        if (changeR < 0){
//          //BLUE HIGH
////          lMotor.backwardA();
////          lMotor.forward();
//          rMotor.forward();
////          rMotor.backwardA();
//        } else {
//          //BLUE YELLOW LOW
////          lMotor.stop();
//          digitalWrite(11,HIGH);
//          digitalWrite(12,HIGH);
//          digitalWrite(28,HIGH);
//          digitalWrite(29,HIGH);
////          rMotor.stop();
//        }
//      } 
//
//    
//    
//    if (changeR > .05) {
//      if(posR == false){
//        //RED HIGH
////        lMotor.stop();
// digitalWrite(11,HIGH);
//          digitalWrite(12,HIGH);
//          digitalWrite(28,HIGH);
//          digitalWrite(29,HIGH);
////        rMotor.stop();
//      }
//      posR = true;
//    } else if (changeR < -.05){
//      if(posR == true){
//        //RED HIGH
////        lMotor.stop();
//        rMotor.stop();
//      }
//        posR = false;
//      } else {
//         digitalWrite(11,HIGH);
//          digitalWrite(12,HIGH);
//          digitalWrite(28,HIGH);
//          digitalWrite(29,HIGH);
////        rMotor.stop();
//      }
//      prevValR = g.gyro.z;
//    }
//    
// 
//Serial.println(g.gyro.z);
//


//}
//  Serial.print(millis() - startTime);
//  Serial.print('\t');
//  Serial.print(g.gyro.z);
//  Serial.print("\t");
//  Serial.println(g.gyro.y);



//  }
  

  }

  
}
