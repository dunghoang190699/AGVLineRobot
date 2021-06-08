/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____
   / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
  | |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
   \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
                    (____/
   Arduino Mecanum Omni Direction Wheel Robot Car Lesson 3 Line Tracking
   Tutorial URL https://osoyoo.com/?p=30022
   CopyRight www.osoyoo.com

*/
//if your car speed too fast or too slow or not moving, please adjust MID_SPEED,HIGH_SPEED,LOW_SPEED value in line 13,14,15,
#include "SPI.h"     // SPI library
#include "MFRC522.h" // RFID library (https://github.com/miguelbalboa/rfid)
#include <TimerFive.h>

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

/*motor control*/
void FR_fwd(int speed) //front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  analogWrite(speedPinR, speed);
}
void FR_bck(int speed) // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  analogWrite(speedPinR, speed);
}
void FL_fwd(int speed) // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, speed);
}
void FL_bck(int speed) // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, speed);
}

void RR_fwd(int speed) //rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  analogWrite(speedPinRB, speed);
}
void RR_bck(int speed) //rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  analogWrite(speedPinRB, speed);
}
void RL_fwd(int speed) //rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
  analogWrite(speedPinLB, speed);
}
void RL_bck(int speed) //rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
  analogWrite(speedPinLB, speed);
}
void forward(int speed_left, int speed_right)
{
  RL_fwd(speed_left);
  RR_fwd(speed_right);
  FR_fwd(speed_right);
  FL_fwd(speed_left);
}
void reverse_left(int speed)
{
  RL_bck(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(speed);
}
void reverse_right(int speed)
{
  RL_fwd(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(speed);
}
void spin(int speed)
{
  RL_bck(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_bck(speed);
}
void right_shift(int speed_fl_fwd, int speed_rl_bck, int speed_rr_fwd, int speed_fr_bck)
{
  FL_fwd(speed_fl_fwd);
  RL_bck(speed_rl_bck);
  RR_fwd(speed_rr_fwd);
  FR_bck(speed_fr_bck);
}
void left_shift(int speed_fl_bck, int speed_rl_fwd, int speed_rr_bck, int speed_fr_fwd)
{
  FL_bck(speed_fl_bck);
  RL_fwd(speed_rl_fwd);
  RR_bck(speed_rr_bck);
  FR_fwd(speed_fr_fwd);
}

void left_turn(int speed)
{
  RL_bck(0);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(0);
}
void right(int speed)
{
  RL_fwd(speed);
  RR_bck(0);
  FR_bck(0);
  FL_fwd(speed);
}
void left(int speed)
{
  RL_fwd(0);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(0);
}
void right_back(int speed)
{
  RL_bck(speed);
  RR_fwd(0);
  FR_fwd(0);
  FL_bck(speed);
}
void sharpRightTurn(int speed_left, int speed_right)
{
  RL_fwd(speed_left);
  RR_bck(speed_right);
  FR_bck(speed_right);
  FL_fwd(speed_left);
}
void sharpLeftTurn(int speed_left, int speed_right)
{
  RL_bck(speed_left);
  RR_fwd(speed_right);
  FR_fwd(speed_right);
  FL_bck(speed_left);
}

void stop_bot() //Stop
{
  analogWrite(speedPinLB, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, LOW);
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, LOW);
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
  delay(60);
}

void changeMode()
{
  if (mode == 1)
  {
    if (car_position2[0] == 0 and car_position2[1] == 1)
    {
      turn_mode = 2; // right
    }
    else if (car_position2[0] == 1 and car_position2[1] == 1)
    {
      turn_mode = 0; // right
    }
    else if (car_position2[0] == 2 and car_position2[1] == 1)
    {
      turn_mode = 1; // left
      mode = 0;
    }
  }
  else if (mode == 0)
  {
    if (car_position2[0] == 1 and car_position2[1] == 1)
    {
      turn_mode = 0; // right
    }
    else if (car_position2[0] == 0 and car_position2[1] == 1)
    {
      turn_mode = 1; // right
    }
    else if (car_position2[0] == 0 and car_position2[1] == 0)
    {
      turn_mode = 2; // right
    }
  }
  //  Serial.print("mode = ");
  //  Serial.println(mode);
  //  Serial.print("turn mode = ");
  //  Serial.println(turn_mode);
}

void timer4_init()
{
  cli(); //stop interrupts

  //set timer4 interrupt at 1Hz
  TCCR4A = 0; // set entire TCCR1A register to 0
  TCCR4B = 0; // same for TCCR1B
  TCNT4 = 0;  //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR4A = 15624 / 2; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR4B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);

  sei(); //allow interrupts
}

void timer3_init()
{
  cli(); //stop interrupts

  //set timer4 interrupt at 1Hz
  TCCR3A = 0; // set entire TCCR1A register to 0
  TCCR3B = 0; // same for TCCR1B
  TCNT3 = 0;  //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR3A = 15624; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR3B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR3B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);

  sei(); //allow interrupts
}

void timer2_init()
{
  cli(); //stop interrupts

  //set timer4 interrupt at 1Hz
  TCCR2A = 0; // set entire TCCR1A register to 0
  TCCR2B = 0; // same for TCCR1B
  TCNT2 = 0;  //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR2A = 15624 / 2; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR2B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR2B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A); //TOIE1: overflow, OCIE2A: compare

  sei(); //allow interrupts
}

//Pins initialize
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);

  stop_bot();
}

void setup()
{
  SPI.begin();        // open SPI connection
  mfrc522.PCD_Init(); // Initialize Proximity Coupling Device (PCD)
  init_GPIO();
  Serial.begin(9600);
  flag_tracking = 1;
  flag_reading = 1;
  //  timer5_init();
  timer4_init();
  timer3_init();
  //    Timer5.initialize(65536 *2);
  //    Timer5.attachInterrupt(readRFID); // blinkLED to run every 0.15 seconds
}

int count_delay = 0;

char buffer[16];
void FSM()
{

  // if we get a command, turn the LED on or off:
  if (Serial.available() > 0)
  {
    int size = Serial.readBytesUntil('\n', buffer, 12);
    if (buffer[0] == 'S')
    {
      if (buffer[1] == '1')
      {
        no_object = 0;
        flag_tracking = 0;
        Serial.println("Stop motor 1");
      }
    }
    else if (buffer[0] == 'F')
    {
      if (buffer[1] == '1')
      {
        no_object = 1;
        flag_tracking = 1;
        Serial.println("Run motor 1 forward");
      }
    }
    else if (buffer[0] == 'B')
    {
      if (buffer[1] == '1')
      {
        Serial.println("Run motor 1 backward");
      }
    }
  }
  changeMode();
}

int flag_test = 0;
void readRFID()
{
  count = 0;
  if (!mfrc522.PICC_IsNewCardPresent())
  {
    return;
  }
  // Select one of the cards
  if (!mfrc522.PICC_ReadCardSerial())
  {
    return;
  }
  for (byte i = 0; i < mfrc522.uid.size; ++i)
  { // read id (in parts)
    k[count] = mfrc522.uid.uidByte[i];
    //        Serial.println(k[count]);
    //        Serial.print(" thdhdjgcghgd");
    count++;
  }
  mfrc522.PICC_HaltA();      // halt PICC
  mfrc522.PCD_StopCrypto1(); // stop encryption on PCD
  flag_tracking = 0;
  flag_reading = 0;

  if (k[0] == 198 && k[1] == 201 && k[2] == 180 && k[3] == 43)
  {
    car_position[0] = 0;
    car_position[1] = 1;
    Serial.print(car_position[0]);
    Serial.println(car_position[1]);
  }
  else if (k[0] == 233 && k[1] == 28 && k[2] == 149 && k[3] == 153)
  {
    car_position[0] = 2;
    car_position[1] = 1;
    Serial.println(car_position[0]);
    Serial.println(car_position[1]);
  }
  else if (k[0] == 123 && k[1] == 228 && k[2] == 132 && k[3] == 34)
  {
    car_position[0] = 1;
    car_position[1] = 0;
    Serial.println(car_position[0]);
    Serial.println(car_position[1]);
  }
  else if (k[0] == 224 && k[1] == 234 && k[2] == 214 && k[3] == 16)
  {
    car_position[0] = 1;
    car_position[1] = 1;
    Serial.println(car_position[0]);
    Serial.println(car_position[1]);
  }
  else if (k[0] == 53 && k[1] == 93 && k[2] == 73 && k[3] == 45)
  {
    car_position[0] = 0;
    car_position[1] = 0;
    Serial.println(car_position[0]);
    Serial.println(car_position[1]);
    //    if (car_position1 != car_position)
    //    {
    //      car_position1[0] = car_position[0];
    //      car_position1[1] = car_position[1];
    //    }
  }

  //  car_position[0] = 0;
  //  car_position[1] = 0;

  k[0] = 0;
  k[1] = 0;
  k[2] = 0;
  k[3] = 0;
}

void tracking()
{
  String senstr = "";
  int s0 = digitalRead(sensor1);
  int s1 = digitalRead(sensor2);
  int s2 = digitalRead(sensor3);

  int sensorvalue = 8;
  sensorvalue += s0 * 4 + s1 * 2 + s2;
  senstr = String(sensorvalue, BIN);
  senstr = senstr.substring(1, 4);

  //  Serial.println(senstr);
  if (senstr == "000")
  {
    //    Serial.println("Stop");
    if (turn_mode = 1)
    {
      reverse_left(LOW_SPEED);
      delay(DELAY_TIME);
      stop_bot();
    }
    else
    {
      reverse_right(LOW_SPEED);
      delay(DELAY_TIME);
      stop_bot();
    }
  }
  else if (senstr == "100")
  {
    //    Serial.println(" Shift Left");
    sharpLeftTurn(LOW_SPEED, MID_SPEED);
    //  left_shift(HIGH_SPEED,HIGH_SPEED,HIGH_SPEED,HIGH_SPEED);
    delay(DELAY_TIME);
    stop_bot();
  }

  else if (senstr == "110")
  {
    //    Serial.println("Slight Shift Left");
    forward(LOW_SPEED, MID_SPEED);
    delay(DELAY_TIME);
    stop_bot();
  }

  else if (senstr == "010" || senstr == "101")
  {
    //    Serial.println("Forward");
    forward(LOW_SPEED, LOW_SPEED);
    delay(DELAY_TIME);
    stop_bot();
  }

  else if (senstr == "011")
  {
    //    Serial.println("Slight Shift to Right ");
    //    forward(HIGH_SPEED, 0);
    forward(MID_SPEED, LOW_SPEED);
    delay(DELAY_TIME);
    stop_bot();
  }
  else if (senstr == "001")
  {
    //    Serial.println("Shift to Right");
    sharpRightTurn(MID_SPEED, LOW_SPEED);
    delay(DELAY_TIME);
    stop_bot();
  }
  else if (senstr == "111")
  {
    //    Serial.println("sharpRight");
    forward(LOW_SPEED, LOW_SPEED);
    delay(DELAY_TIME);
    stop_bot();
  }
}

ISR(TIMER4_COMPA_vect)
{
  FSM();
}

ISR(TIMER3_COMPA_vect)
{
  //  readRFID();
  //  Serial.print("1");
}

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
void loop()
{
  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis1 >= OnTime1)
  {
    previousMillis1 = currentMillis; // Remember the time
    if (flag_tracking == 1 and no_object == 1)
    {
      tracking();
    }
  }
  if (currentMillis - previousMillis3 >= OffTime1)
  {
    previousMillis3 = currentMillis; // Remember the time
    if (flag_tracking == 0)
    {
      flag_tracking = 1;
    }
  }

  if (currentMillis - previousMillis2 >= OnTime2)
  {
    previousMillis2 = currentMillis; // Remember the time
    //    if (flag_reading == 1) {
    //      Serial.println(flag_reading);
    readRFID();
    //    }
  }
  //    if (currentMillis - previousMillis4 >= OffTime2)
  //    {
  //      previousMillis4 = currentMillis;  // Remember the time
  //      if (flag_reading == 0) {
  //        flag_reading = 1;
  //      }
  //    }
}

void loop123()
{
  //  Serial.println(flag_tracking);
  // Serial.print(flag_tracking);
  if (flag_tracking == 1)
  {
    tracking();
  }
  else
  {
    if (millis() > time_now + period)
    {
      time_now = millis();
      Serial.print("1");
      flag_tracking = 1;
    }
  }
  //    reverse_left(LOW_SPEED);
  //    delay(100);
  //    stop_bot();
  //    count_delay = count_delay + 1;
  //    if (count_delay >= 1000 and no_object == 1) {
  //      Serial.println("read");
  //      delay(1000);
  //      count_delay = 0;
  //      flag_tracking = 1;
  //    }
  //}
}
