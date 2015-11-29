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

; Device ID specification
;                    - Defined in spec to be 0
;                    |- Set for remote control device
;                    ||- Set for interface robot
;                    |||- Set for recieve robot
;                    ||||
.equ	DeviceID = 0b01000111

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

		;Configre Data Direction Registers (no pullups required)

		;Configure Interrups/Times

		;Configure USART




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
MAIN_INTERFACE:
	NOP
MAIN_RECIEVER:
	NOP
    rjmp MAIN
