#ifndef RFID_H // include guard
#define RFID_H

#include "config.h"

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

#endif
