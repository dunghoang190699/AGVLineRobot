#ifndef TIMER_H // include guard
#define TIMER_H

#include "config.h"

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

#endif