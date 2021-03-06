;***********************************************************
;*
;*	Lab 6 Timer/Counter
;*
;*	Provides different speed levels for the TekBot platform.
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
;*	   Date: 11/16/2015
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
.def	speedDial = r21

.equ	debounceTime = 20
.equ	EngEnR = 4				; right Engine Enable Bit
.equ	EngEnL = 7				; left Engine Enable Bit
.equ	EngDirR = 5				; right Engine Direction Bit
.equ	EngDirL = 6				; left Engine Direction Bit

.equ	MovFwd = (1<<EngDirR|1<<EngDirL)	; Move Forwards Command
.equ	MovBck = $00				; Move Backwards Command
.equ	TurnR = (1<<EngDirL)			; Turn Right Command
.equ	TurnL = (1<<EngDirR)			; Turn Left Command
.equ	Halt = (1<<EngEnR|1<<EngEnL)		; Halt Command


;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000
		rjmp	INIT			; reset interrupt

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

		; Configure I/O ports
		LDI		mpr, $FF	;Set portB to be outputs (display number and PWM)
		OUT		DDRB, mpr

		LDI		mpr, $00	;Set lower 2 bits of Port D to be inputs (as well as the rest)
		OUT		DDRD, mpr


		; Configure External Interrupts, if needed

		; Configure 8-bit Timer/Counters
		LDI		mpr, 0b01111001 ;clk/0, Fast PWM, Set on compare, clear at top
		OUT		TCCR0, mpr
		OUT		TCCR2, mpr

		LDI		mpr, 255
		OUT		OCR0, mpr
		OUT		OCR2, mpr

		; Set TekBot to Move Forward (1<<EngDirR|1<<EngDirL)
		LDI		mpr, MovFwd
		OUT		PORTB, mpr


		; Set initial duty cycle ("speed"), display on Port B
		LDI		speedCnt, 0
		LDI		speedDial, 0
		OUT		OCR0, speedDial
		OUT		OCR2, speedDial

		IN		mpr, PORTB		;Read in the current state of PORTB
		ANDI	mpr, $F0
		OR		mpr, speedCnt	;OR PortB with our count (only possible lower nibble values)
		OUT		PORTB, mpr		;Set the mpr to the output

		; Enable global interrupts (if any are used)

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
		;BUTTON 1 - INCREASE SPEED
		IN		mpr, PIND
		SBRC	mpr, 0		;Check the first button, if pressed, skip jump
		RJMP	TESTBUTTONTWO
		;Process first button
		CPI		speedCnt, $0F	;If the count is 15, do nothing (skip inc)
		BREQ	ENDTESTBUTTONONE
		LDI		waitcnt, debounceTime 
		CALL	Wait			;Wait 20ms (debouncing)

		INC		speedCnt		;Increment the speedCnt
		LDI		mpr, 17			
		ADD		speedDial, mpr	;Increment the speed by 17 (255/15)
ENDTESTBUTTONONE:
		OUT		OCR0, speedDial
		OUT		OCR2, speedDial


		;BUTTON 2 - DECREASE SPEED
TESTBUTTONTWO:
		IN		mpr, PIND
		SBRC	mpr, 1
		RJMP	DONETESTING
		;Process 2nd button
		CPI		speedCnt, $00   ;If the count is 0, skip DEC
		BREQ	ENDTESTBUTTONTWO
		LDI		waitcnt, debounceTime	
		CALL	Wait			;Wait 20ms (debouncing)

		DEC		speedCnt		;Increment the speedCnt

		SUBI	speedDial, 17	;Increment the speed by 17 (255/15)
ENDTESTBUTTONTWO:
		OUT		OCR0, speedDial
		OUT		OCR2, speedDial


DONETESTING:
		;Output to LEDs
		IN		mpr, PORTB		;Read in the current state of PORTB
		ANDI	mpr, $F0
		OR		mpr, speedCnt	;OR PortB with our count (only possible lower nibble values)
		OUT		PORTB, mpr		;Set the mpr to the output

		NOP
END:
		rjmp	MAIN			; return to top of MAIN


;***********************************************************
;*	Functions and Subroutines
;***********************************************************

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



