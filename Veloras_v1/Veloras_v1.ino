// Source: https://github.com/NicolasU-N/raptor/blob/master/raptor_nl_pid.ino
// Arduino Pro Micro, TB6612 driver, Pololu QTR-8 sensor
// Adaptado por: @NicolasU-N     https://github.com/NicolasU-N 
//               @OrlandoMurcia1 https://github.com/OrlandoMurcia1

//#include <PIDfromBT.h> // Libreria para PID por Bluetooth

// TB6612 driver pinout
//const int STBY = 15; // standby
const int PWMA = 10; // speed and direction control motor A (left)
const int AIN1 = 5;
const int AIN2 = 4;
const int PWMB = 9; // speed and direction control motor B (right)
const int BIN1 = 7;
const int BIN2 = 8;
const int boton_1 = 12;  //Pin para boton
const int led1 = 11; //Led Amarillo
const int led2 = 6;  // Led Rojo

// Pololu QTR-8A analog array readout
#include <QTRSensors.h>
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2,1,0}, 8,4,13);
unsigned int IR[8];

// parameters and variables for non linear PID
const int vmax=120; // origina-->150,, prueba1 --> 100,  220  vmax
const int vmin=100; //Estaba prueba 1-->80, 150
const float kp=0.2;   //Estaba kp=.015 original
const float ki=0.00001; //0.0003, 0.0007
const float kd= 1;

const float kv=0.07; //0.07
int p,d,u,vbase;
long i=0;
int p_old=0;



void setup() {
    
    pinMode(led1, OUTPUT); //led1
    pinMode(led2, OUTPUT); //led2
    pinMode(AIN1,OUTPUT);
    pinMode(AIN2,OUTPUT);
    pinMode(PWMA,OUTPUT);
    pinMode(PWMB,OUTPUT);
    pinMode(BIN1,OUTPUT);
    pinMode(BIN2,OUTPUT);
    
    pinMode(boton_1, INPUT); //boton 1 como pull up
    
    
    digitalWrite(led2,HIGH); //encender led 2 para indicar la
                                 // espera  de pulsacion de boton

    while(true)
        {
          int x=digitalRead(boton_1); //leemos y guardamos el valor
                                      // del boton en variable x
                                      delay(100);
          if(x==0) //si se presiona boton 
          {
             digitalWrite(led2,LOW); //indicamos que se presiono boton
             digitalWrite(led1,HIGH); //encendiendo led 1
             delay(100);
             break; //saltamos hacia el bucle principal
          }
        }
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
  
  digitalWrite(AIN1, L<0); //        switch < and >= if left wheel doesnt spin as expected
  digitalWrite(AIN2, L>=0);// 
  analogWrite(PWMA, abs(L));
  
  digitalWrite(BIN1, R<0); //        switch < and >= if left wheel doesnt spin as expected
  digitalWrite(BIN2, R>=0);// 
  analogWrite(PWMB, abs(R));
}
