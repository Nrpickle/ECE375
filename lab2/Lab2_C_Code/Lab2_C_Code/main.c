/*
 * Lab2_C_Code.c
 *
 * Created: 10/7/2015 6:10:21 PM
 * Author : Nick McComb
 
 PORT MAP
 Port B, Pin 4 -> Output -> Right Motor Enable
 Port B, Pin 5 -> Output -> Right Motor Direction
 Port B, Pin 7 -> Output -> Left Motor Enable
 Port B, Pin 6 -> Output -> Left Motor Direction
 Port D, Pin 1 -> Input -> Left Whisker
 Port D, Pin 0 -> Input -> Right Whisker
 
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

/* These are the macros that are used to help ensure readability of code. They do not reduce execution time, because they are compiled into the code, not called as functions are. */

#define MOVE_FORWARD(void) (PORTB = 0b01100000)
#define MOVE_REVERSE(void) (PORTB = 0b00000000)
#define ROTATE_LEFT(void) (PORTB = 0b00100000)
#define ROTATE_RIGHT(void) (PORTB = 0b01000000)

#define CHECK_LEFT_WHISKER(void) (! (PIND & (1<<PD1)) )
#define CHECK_RIGHT_WHISKER(void) (! (PIND & (1<<PD0)) )
#define CHECK_BOTH_WHISKERS(void) (! (PIND & (1<<PD2)) )


int main(void)
{

//Initialize registers 
DDRB =  0b11110000;  //Set the upper nibble to outputs to drive the motors
DDRD =  0b00000000;  //Set bits 0, 1, and 2 to inputs
PORTB = 0b11110000;  //Set the default state of the motors

    while (1) 
    {
		MOVE_FORWARD();
		
		if (CHECK_BOTH_WHISKERS()) {  //Check for "Both" whiskers, which is really just a button press
			_delay_ms(300);
			MOVE_REVERSE();
			_delay_ms(300);
		}
		else if (CHECK_LEFT_WHISKER()){  //Check for left whisker
			_delay_ms(300);
			MOVE_REVERSE();
			_delay_ms(300);
			ROTATE_LEFT();
			_delay_ms(500);
		}
		else if (CHECK_RIGHT_WHISKER()){  //Check for right whisker
			_delay_ms(300);
			MOVE_REVERSE();
			_delay_ms(300);
			ROTATE_RIGHT();
			_delay_ms(500);
		}
		
    }
	_delay_ms(50);
}
