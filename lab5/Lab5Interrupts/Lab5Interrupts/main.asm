;***********************************************************
;*
;*	Lab 5: Interrupts
;*
;*	This file uses the hardware interrupts on the 
;*  ATmega chip to replicate the previous Tekbot bumping code.
;*
;*	This is the skeleton file Lab 5 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Nick McComb
;*	   Date: 11/4/2015
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multipurpose register 
.def	waitcnt = r17				; Wait Loop Counter
.def	ilcnt = r18				; Inner Loop Counter
.def	olcnt = r19				; Outer Loop Counter

.equ	WTime = 100				; Time to wait in wait loop

.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit


; Using the constants from above, create the movement 
; commands, Forwards, Backwards, Stop, Turn Left, and Turn Right

.equ	MovFwd = (1<<EngDirR|1<<EngDirL)	; Move Forwards Command
.equ	MovBck = $00				; Move Backwards Command
.equ	TurnR = (1<<EngDirL)			; Turn Right Command
.equ	TurnL = (1<<EngDirR)			; Turn Left Command
.equ	Halt = (1<<EngEnR|1<<EngEnL)		; Halt Command

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

; Set up the interrupt vectors for the interrupts, .i.e
;.org	$002E					; Analog Comparator IV
;		rcall	HandleAC		; Function to handle Interupt request
;		reti					; Return from interrupt

.org	$0002  ;Configure interrupt 0
		rcall  HitLeft
		reti

.org    $0004  ;Configure interrupt
		rcall  HitRight
    reti
		

.org	$0046					; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:							; The initialization routine
		; Initialize Stack Pointer
		LDI		mpr, LOW(RAMEND)  ;Low Byte of End SRAM Address
		OUT		SPL, mpr		  ;Write byte to SPL
		LDI		mpr, HIGH(RAMEND) ;High Byte of End SRAM Address
		OUT		SPH, mpr		  ;Write byte to SPH

		; Initialize Port B for output
		LDI		mpr, 0xF0    ; We need to set the upper nibble here to drive the motors
		OUT		DDRB, mpr	 ; "

		;PIC = 0 for output
		;Atmel = 1 for output

		; Initialize Port E for input
		LDI		mpr, 0x00    ;Set all PORTE pins to be input (specifically 0, 1, and 2)
		OUT		DDRD, mpr    ; "

		;Initial condition, TekBot moving forward
		ldi		mpr, MovFwd
		OUT		PORTB, mpr

		; Initialize external interrupts
		; Set the Interrupt Sense Control to Rising Edge detection
		LDI		mpr, 0xFF   ;We want to set all interrupts to output rising edge detection
		STS		EICRB, mpr
		STS		EICRA, mpr
		; NOTE: must initialize both EICRA and EICRB

		; Set the External Interrupt Mask
		LDI		mpr, 0b00000011
		OUT		EIMSK, mpr

		; Turn on interrupts
		; NOTE: This must be the last thing to do in the INIT function
		SEI

;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:							; The Main program

		; Send command to Move Robot Forward
		;LDI		mpr, 0xFF
		;STS		PORTB, mpr

		;ldi		waitcnt, WTime	; Wait for 1 second
		;rcall	Wait			; Call wait function

		;rcall HitLeft

		NOP
		NOP

		rjmp	MAIN			; Create an infinite while loop to signify the 
								; end of the program.


;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;----------------------------------------------------------------
; Sub:	HitRight
; Desc:	Handles functionality of the TekBot when the right whisker
;		is triggered.
;----------------------------------------------------------------
HitRight:
		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backwards command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Turn left for a second
		ldi		mpr, TurnL	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forwards command
		out		PORTB, mpr	; Send command to port

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

;----------------------------------------------------------------
; Sub:	HitLeft
; Desc:	Handles functionality of the TekBot when the left whisker
;		is triggered.
;----------------------------------------------------------------
HitLeft:
		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backwards command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function


		; Turn right for a second
		ldi		mpr, TurnR	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	Wait			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forwards command
		out		PORTB, mpr	; Send command to port

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

;----------------------------------------------------------------
; Sub:	Wait
; Desc:	A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;		waitcnt*10ms.  Just initialize wait for the specific amount 
;		of time in 10ms intervals. Here is the general eqaution
;		for the number of clock cycles in the wait loop:
;			((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;----------------------------------------------------------------
Wait:
		push	waitcnt			; Save wait register
		push	ilcnt			; Save ilcnt register
		push	olcnt			; Save olcnt register

Loop:	ldi		olcnt, 224		; load olcnt register
OLoop:	ldi		ilcnt, 237		; load ilcnt register
ILoop:	dec		ilcnt			; decrement ilcnt
		brne	ILoop			; Continue Inner Loop
		dec		olcnt		; decrement olcnt
		brne	OLoop			; Continue Outer Loop
		dec		waitcnt		; Decrement wait 
		brne	Loop			; Continue Wait loop	

		pop		olcnt		; Restore olcnt register
		pop		ilcnt		; Restore ilcnt register
		pop		waitcnt		; Restore wait register
		ret				; Return from subroutine

;-----------------------------------------------------------
; You will probably need several functions, one to handle the 
; left whisker interrupt, one to handle the right whisker 
; interrupt, and maybe a wait function
;------------------------------------------------------------

;-----------------------------------------------------------
; Func: Template function header
; Desc: Cut and paste this and fill in the info at the 
;		beginning of your functions
;-----------------------------------------------------------
FUNC:							; Begin a function with a label
		; Save variable by pushing them to the stack

		; Execute the function here
		
		; Restore variable by popping them from the stack in reverse order
		ret						; End a function with RET

;***********************************************************
;*	Stored Program Data
;***********************************************************

; Enter any stored data you might need here

;***********************************************************
;*	Additional Program Includes
;***********************************************************
; There are no additional file includes for this program
