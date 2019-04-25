// Source: https://github.com/NicolasU-N/raptor/blob/master/raptor_nl_pid.ino
// Arduino Pro Micro, TB6612 driver, Pololu QTR-8 sensor
// Adaptado por: @NicolasU-N https://github.com/NicolasU-N ...

// TB6612 driver pinout
//const int STBY = 15; // standby
const int PWMA = 10; // speed and direction control motor A (left)
const int AIN1 = 5;
const int AIN2 = 4;
const int PWMB = 9; // speed and direction control motor B (right)
const int BIN1 = 7;
const int BIN2 = 8;

// Pololu QTR-8A analog array readout
#include <QTRSensors.h>
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2,1,0}, 8,4,13);
unsigned int IR[8];

// parameters and variables for non linear PID
const int vmin=70; //Estaba 80
const int vmax=80; // 150
const float kp=.015;
const float ki=0.0003;
const float kd=0.2;
const float kv=0.07;
int p,d,u,vbase;
long i=0;
int p_old=0;

void setup() {
    pinMode(AIN1,OUTPUT);
    pinMode(AIN2,OUTPUT);
    pinMode(PWMA,OUTPUT);
    pinMode(PWMB,OUTPUT);
    pinMode(BIN1,OUTPUT);
    pinMode(BIN2,OUTPUT);
}

void loop()
{
  //digitalWrite(STBY, HIGH);
  
  qtra.read(IR); // read raw sensor values
  
  p = -7*IR[0]-5*IR[1]-3*IR[2]-IR[3]+IR[4]+3*IR[5]+5*IR[6]+7*IR[7];
  i=i+p;
  d=p-p_old;
  p_old=p;
  if ((p*i)<0) i=0;  // integral windup

  u=kp*p+ki*i+kd*d;
  vbase=vmin+(vmax-vmin)*exp(-kv*abs(kp*p));
  drive(vbase+u,vbase-u);
 
}

void drive(int L, int R) // speed for wheels Left and Right, positive is forward
{
  L=constrain(L,-255,255); // avoid PWM overflow
  R=constrain(R,-255,255);
  
  digitalWrite(AIN1, L>0); // Estaba <       switch < and >= if left wheel doesnt spin as expected
  digitalWrite(AIN2, L<=0);// >=
  analogWrite(PWMA, abs(L));
  
  digitalWrite(BIN1, R<0); // Estaba <       switch < and >= if left wheel doesnt spin as expected
  digitalWrite(BIN2, R>=0);// >=
  analogWrite(PWMB, abs(R));
}
