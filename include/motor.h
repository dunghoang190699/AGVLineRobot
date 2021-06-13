#ifndef MOTOR_H // include guard
#define MOTOR_H

#include "config.h"

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

#endif
