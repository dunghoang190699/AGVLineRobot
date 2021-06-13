#ifndef CONFIG_H // include guard
#define CONFIG_H

#include <Arduino.h>
#include "SPI.h"     // SPI library
#include "MFRC522.h" // RFID library (https://github.com/miguelbalboa/rfid)

const int pinRST = 5;

const int pinSDA = 53;

MFRC522 mfrc522(pinSDA, pinRST); // Set up mfrc522 on the Arduino
int k[4] = {0, 0, 0, 0};
int k1[4] = {0, 0, 0, 0};
int count = 0;

unsigned long uidDec, uidDecTemp; // hien thi so UID dang thap phan
byte bCounter, readBit;
unsigned long ticketNumber;

int flag, flag_rfid, flag_tracking, flag_reading;
unsigned long count_rfid;

//if your car speed too fast or too slow or not moving, please adjust MID_SPEED,HIGH_SPEED,LOW_SPEED value in line 13,14,15,
// these values control the current power of your car motors
#define MID_SPEED 135
#define HIGH_SPEED 140 //160
#define LOW_SPEED 130
#define LONG_DELAY_TIME 20
#define DELAY_TIME 10
#define SHORT_DELAY_TIME 10

#define speedPinR A13        //  Rear Wheel PWM pin connect Left MODEL-X ENA
#define RightMotorDirPin1 32 //Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
#define RightMotorDirPin2 34 //Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1)
#define LeftMotorDirPin1 33  //Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
#define LeftMotorDirPin2 35  //Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
#define speedPinL A15        //  Rear Wheel PWM pin connect Left MODEL-X ENB

#define speedPinRB A12        //  Front Wheel PWM pin connect Right MODEL-X ENA
#define RightMotorDirPin1B 38 //Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define RightMotorDirPin2B 36 //Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)
#define LeftMotorDirPin1B 39  //Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2B 37  //Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)
#define speedPinLB A14        //  Front Wheel PWM pin connect Right MODEL-X ENB

#define sensor1 A2 // Left   sensor
#define sensor2 A1 // center  sensor
#define sensor3 A0 // right sensor

int car_position[2] = {0, 0};
int car_position1[2] = {0, 0};
int car_position2[2] = {0, 0};
int no_object = 1;
int mode = 1;      // mode = 1: from 0 0 to 2 1 take package, mode = 0, from 2 1 to 0 0 return package
int turn_mode = 1; //turn_mode = 1: turn left, turn_mode = 2: turn right

int period = 5000;
unsigned long time_now = 0;

unsigned long previousMillis1 = 0; // will store last time LED was updated
long OnTime1 = 10;                 // milliseconds of on-time
long OffTime1 = 3000;              // milliseconds of off-time

unsigned long previousMillis2 = 0; // will store last time LED was updated
long OnTime2 = 300;                // milliseconds of on-time
long OffTime2 = 6000;              // milliseconds of off-time

unsigned long previousMillis3 = 0; // will store last time LED was updated
long OnTime3 = 3000;               // milliseconds of on-time
long OffTime3 = 750;               // milliseconds of off-time

unsigned long previousMillis4 = 0; // will store last time LED was updated
long OnTime4 = 6000;               // milliseconds of on-time

int count_delay = 0;
char buffer[16];
int flag_test = 0;

#endif