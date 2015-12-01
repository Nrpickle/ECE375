;***********************************************************
;*
;*	Lab 7 Timer/Counter
;*
;*	Implements a remote control feature for the TekBot!
;*
;* PORT MAP
;* Port B, Pin 4 -> Output -> Right Motor Enable
;* Port B, Pin 5 -> Output -> Right Motor Direction
;* Port B, Pin 7 -> Output -> Left Motor Enable
;* Port B, Pin 6 -> Output -> Left Motor Direction
;*
;***********************************************************
;*
;*	 Author: Nick McComb
;*	   Date: 11/29/2015
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multipurpose register
.def	speedCnt = r17
.def	waitcnt = r18
.def	ilcnt = r19				; Inner Loop Counter
.def	olcnt = r20				; Outer Loop Counter

.equ	waitTime = 20
.equ	EngEnR = 4				; right Engine Enable Bit
.equ	EngEnL = 7				; left Engine Enable Bit
.equ	EngDirR = 5				; right Engine Direction Bit
.equ	EngDirL = 6				; left Engine Direction Bit

.equ	MovFwd = (1<<EngDirR|1<<EngDirL)	; Move Forwards Command
.equ	MovBck = $00				; Move Backwards Command
.equ	TurnR = (1<<EngDirL)			; Turn Right Command
.equ	TurnL = (1<<EngDirR)			; Turn Left Command
.equ	Halt = (1<<EngEnR|1<<EngEnL)		; Halt Command

; Device ID specification
;                    - Defined in spec to be 0
;                    |- Set for remote control device
;                    ||- Set for interface robot
;                    |||- Set for recieve robot
;                    ||||
.equ	DeviceID = 0b00100111

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000
		rjmp	INIT			; reset interrupt

.org	$003C
		jmp		ISR_RX_COMPLETE
		; place instructions in interrupt vectors here, if needed

.org	$0046					; end of interrupt vectors

;***********************************************************
;*	Program Initialization
;***********************************************************
INIT:
		; Initialize Stack Pointer
		LDI		mpr, LOW(RAMEND)  ;Low Byte of End SRAM Address
		OUT		SPL, mpr		  ;Write byte to SPL
		LDI		mpr, HIGH(RAMEND) ;High Byte of End SRAM Address
		OUT		SPH, mpr		  ;Write byte to SPH

		;Configre Data Direction Registers (no pullups required)
		LDI		mpr, $FF			;Sets  PortB to be outputs (output display/motor control)
		OUT		DDRB, mpr

		LDI		mpr, (1<<PD1)			;Sets PortD to be inputs (all buttons), except for pin 2 for the TX
		OUT		DDRD, mpr

		;Configure Interrups/Timers

		
		;Configure USART
		LDI		mpr, (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1) ;0b11011000
		STS		UCSR1B, mpr     ;Enable RX, TX interrupt, Reciever, Enable

		;LDI		mpr, 0b00000000
		;STS		UCSR1B, mpr     ;Configure UCSR1B to be Asynchronous

		LDI		mpr, (1<<UPM11) | (1<<UCSZ11) | (1<<UCSZ10) | (1<<USBS1)
				;0b00101110	;8 databits, 2 stop bits, even parity
		STS		UCSR1C, mpr

		;Load 2400 baud ($01A0)
		LDI		mpr, $A0	;LSB of baud rate selection
		STS		UBRR1L, mpr

		LDI		mpr, $01	;MSB of baud rate selection
		STS		UBRR1H, mpr

		;Enable Interrupts
		SEI

		RJMP	MAIN

MAIN:
	;Determine where in execution code to send robot
	LDI		mpr, DeviceID
	SBRC	mpr, 6  ;Check bit 6 to see if device is a remote
	rjmp	MAIN_REMOTE
	SBRC	mpr, 5  ;Check bit 5 to see if device is an interface robot
	jmp		MAIN_INTERFACE
	SBRC	mpr, 4  ;Check bit 4 to see if device is a reciever robot
	jmp		MAIN_RECIEVER

MAIN_REMOTE:
	NOP

	rjmp	MAIN_REMOTE
MAIN_INTERFACE:
	NOP

	;LDI		mpr, $FF
	;OUT		PORTB, mpr
	;LDI		waitcnt, waitTime 
	;CALL	Wait			;Wait 20ms 

	;LDI		mpr, $00
	;OUT		PORTB, mpr
	;LDI		waitcnt, waitTime 
	;CALL	Wait			;Wait 20ms 

	rjmp	MAIN_INTERFACE
MAIN_RECIEVER:
	NOP

	rjmp	MAIN_RECIEVER

    jmp MAIN



ISR_RX_COMPLETE:
	PUSH	mpr

	LDS mpr, UDR1	;Load the RX'd data into mpr
	;LDI		mpr, $FF
	OUT		PORTB, mpr	;Output the RX'd data onto the PORTB

	POP		mpr

	RETI

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





