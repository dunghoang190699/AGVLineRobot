#include "./config.h"
#include "./line.h"
#include "./rfid.h"
#include "./motor.h"
#include "./timer.h"

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
	SPI.begin();		// open SPI connection
	mfrc522.PCD_Init(); // Initialize Proximity Coupling Device (PCD)
	init_GPIO();
	Serial.begin(9600);
	flag_tracking = 1;
	flag_reading = 1;
	timer4_init();
	timer3_init();
}

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

ISR(TIMER4_COMPA_vect)
{
	FSM();
}


// TODO: 
// var currentPosition;
// var hasItem = 0;
// var moveBack = 0;
void loop()
{
	// check to see if it's time to change the state of the LED
	unsigned long currentMillis = millis();

	// TODO:
	// if (currentPosition == initPosition)
	// {
	// 	// readItemRFID will return the boolean value True or False
	// 	if (readItemRFID() ) { // set goal
	// 		hasItem = 1;
	// 	}
	// }
	// else if (currentPosition == endPosition) // endPostion ==  goal 
	// {
	// 	moveBack = 1;
	// }

	if (currentMillis - previousMillis1 >= OnTime1)
	{
		previousMillis1 = currentMillis; // Remember the time
		if (flag_tracking == 1 and no_object == 1) // TODO: add has_item  if (flag_tracking == 1 and no_object == 1 and has_item == 1)
		{
			tracking();
		}
		// TODO: move back
		// else if (moveBack == 1){
		// 	tracking();
		// }
	}

	// TODO: return back hasItem when reach the destination
	

	// ENDTODO: return back hasItem when reach the destination 

	// TODO: return back moveBack when reach the initPositon

	// ENDTODO: return back moveBack when reach the initPositon

	if (currentMillis - previousMillis3 >= OffTime1)
	{
		previousMillis3 = currentMillis; // Remember the time
		if (flag_tracking == 0)
		{
			flag_tracking = 1;
		}
	}


	// always read the robot position
	if (currentMillis - previousMillis2 >= OnTime2)
	{
		previousMillis2 = currentMillis; // Remember the time
		readRFID();
	}

}
