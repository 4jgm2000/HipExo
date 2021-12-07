#include <L298NX2.h>
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
const unsigned int EN4 = 24;
L298NX2 rMotor(EN3,IN5,IN6,EN4,IN7,IN8);
#define loose 41
#define butt 40



void setup() {
  // put your setup code here, to run once:
  lMotor.setSpeed(250);
  rMotor.setSpeed(250);
    pinMode(butt, INPUT_PULLUP);
   pinMode(loose, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!digitalRead(butt)){
    lMotor.forwardA();
  }
  while(!digitalRead(loose)){
    lMotor.backwardB();
  } 
  lMotor.stop();

    
  }
