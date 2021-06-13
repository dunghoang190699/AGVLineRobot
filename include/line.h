#ifndef LINE_H // include guard
#define LINE_H

#include "config.h"
#include "motor.h"

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

#endif