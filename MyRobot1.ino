//#include <MQ135.h>    //Библиотека датчика воздуха

#include <Servo.h>          //Библиотека сервомотора. Есть по стандарту
#include <NewPing.h>        //Библиотека Ультразвукового датчика (нужно установить)
#include <OLED_I2C.h>

//#include <MQUnifiedsensor.h>

#define SDA 11//A4
#define SCL 10//A5

OLED  myOLED(SDA, SCL, 8);
extern uint8_t SmallFont[];
//MQ135 gasSensor = MQ135(A3); //GAS Sensor PIN AO


//Пины контроллера L298N
const int LeftMotorForward = 6;
const int LeftMotorBackward = 9;
const int RightMotorForward = 5;
const int RightMotorBackward = 3;


#define pinSensorL A2 //Аналоговый вход 3
#define pinSensorC A3 //Аналоговый вход 3
#define pinSensorR A4 //Аналоговый вход 3

//Пины ультразвукового датчика
#define trig_pin A0 //Аналоговый вход 1
#define echo_pin A1 //Аналоговый вход 2

#define maximum_distance 200

int distance = 100;
int spd = 0;

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name


void setup(){
  myOLED.begin();
  myOLED.setFont(SmallFont);
 
  //  float rzero = gasSensor.getRZero();
  
Serial.begin (9600); // Задаем скорость обмена com-порта 9600
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  

  
  servo_motor.attach(8); //Пин подключения сервомотора

  servo_motor.write(115);
  delay(1000);
  distance = readPing();
/*  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();*/
  delay(100);
}

void loop(){


  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 15){
    moveStop();
    delay(100);
    moveBackward();
    delay(300);
    moveStop();
    delay(100);
    distanceRight = lookRight();

    delay(100);
        if (distance >= distanceRight){
           distanceLeft = lookLeft();

           delay(100);
            if (distance >= distanceLeft){
                moveBackward();

                delay(300);
                moveStop();
            }else{
               turnLeft();
                moveStop();
              }
        }else{
               
               turnRight();
               moveStop();
              }
 
  }
  else {
    moveForward(distance*2+20); 
  }
   distance = readPing();

//  float ppm = gasSensor.getPPM();
//  float ppsc = gasSensor.getCorrectedPPM(22, 44);

  float pinSensorLv=analogRead(pinSensorL);
  float pinSensorCv=analogRead(pinSensorC);
  float pinSensorRv=analogRead(pinSensorR);

  
  myOLED.clrScr();
  myOLED.print("Speed:" + String(spd) , LEFT, 0);
  myOLED.print("Dist:" + String(distance), CENTER, 16);
  myOLED.print(String(distanceLeft), LEFT, 16);
  myOLED.print(String(distanceRight), RIGHT, 16);
  myOLED.print(String(pinSensorLv), LEFT, 32);
  myOLED.print(String(pinSensorCv), CENTER, 32);
  myOLED.print(String(pinSensorRv), RIGHT, 32);

  
//  myOLED.print("ppm "+String(ppm), LEFT, 32);
//  myOLED.print("ppm "+String(ppsc), LEFT, 48);
  myOLED.update();
  
  Serial.println ("Speed:" + String(spd) + ", Dist:" + String(distance) + ", DistL:" + String(distanceLeft) + ", DistR:" + String(distanceRight) ); // Пишем в консоль spd

}

int lookRight(){  
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft(){
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void moveStop(){
  spd=0;
  analogWrite(RightMotorForward, spd);
  analogWrite(LeftMotorForward, spd);
  analogWrite(RightMotorBackward, spd);
  analogWrite(LeftMotorBackward, spd);
}

void moveForward(int k){
  if (k>250){k=250;}
  if (k<80){k=80;}
  spd=spd+30;
  if (spd>k){spd=k;}
    
    analogWrite(LeftMotorBackward, 0);
    analogWrite(RightMotorBackward, 0);
    analogWrite(LeftMotorForward, spd);
    analogWrite(RightMotorForward, spd);
 }



void moveBackward(){
spd=150;
  
  analogWrite(LeftMotorForward, 0);
  analogWrite(RightMotorForward, 0);
  analogWrite(LeftMotorBackward, spd);
  analogWrite(RightMotorBackward, spd);
  delay(500);
 
}

void turnRight(){
spd=200;
  analogWrite(LeftMotorBackward, 0);
  analogWrite(RightMotorForward, 0);
  analogWrite(LeftMotorForward, spd);
  analogWrite(RightMotorBackward, spd);

  delay(300);
 spd=0;
}

void turnLeft(){
spd=200;
  analogWrite(LeftMotorForward, 0);
  analogWrite(RightMotorBackward, 0);
  analogWrite(LeftMotorBackward, spd);
  analogWrite(RightMotorForward, spd);

 delay(300);
  spd=0;
 
}
