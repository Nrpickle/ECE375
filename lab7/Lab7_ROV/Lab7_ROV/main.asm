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
.def	prep_to_send = r21

.def	rx_deviceid = r5
.def	rx_actioncode = r6
.def	previousMotorCommand = r7
.def	previousActionCode = r8

.equ	remoteWaitTime = 20
.equ	robotWaitTime = 20
.equ	freezeTime = 250
.equ	EngEnR = 4				; right Engine Enable Bit
.equ	EngEnL = 7				; left Engine Enable Bit
.equ	EngDirR = 5				; right Engine Direction Bit
.equ	EngDirL = 6				; left Engine Direction Bit

.equ	cmd_forward = 0b10110000
.equ	cmd_backward = 0b10000000
.equ	cmd_turnRight = 0b10100000
.equ	cmd_turnLeft = 0b10010000
.equ	cmd_halt = 0b11001000
.equ	cmd_freeze = 0b11111000

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

;Interface robot: $27

.equ	RemoteTargetId = $27 ;0b00100111

;01010101

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
		rjmp		ISR_RX_COMPLETE
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

		LDI		mpr, $08 			;Sets PortD to be inputs (all buttons), except for pin 2 for the TX
		OUT		DDRD, mpr

		;Configure Interrups/Timers

		
		;Configure USART
		LDI		mpr, (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1) ;0b11011000
		STS		UCSR1B, mpr     ;Enable RX, TX interrupt, Reciever, Enable


		LDI		mpr, (1<<UPM11) | (1<<UCSZ11) | (1<<UCSZ10) | (1<<USBS1)
				;0b00101110	;8 databits, 2 stop bits, even parity
		STS		UCSR1C, mpr

		;Load 2400 baud ($01A0)
		LDI		mpr, $A0	;LSB of baud rate selection
		STS		UBRR1L, mpr

		LDI		mpr, $01	;MSB of baud rate selection
		STS		UBRR1H, mpr

		;Initialize the rx registers with a valid move forward command
		LDI		mpr, DeviceId
		MOV		rx_deviceid, mpr
		LDI		mpr, cmd_forward
		MOV		rx_actioncode, mpr

		;Specially set output for robot, because the remote will override this
		;with it's own output values anyway
		LDI		mpr, MovFwd
		;OUT		PORTB, mpr
		MOV		previousMotorCommand, mpr  ;Store the default previous motor command
		

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
	LDI		mpr, $00   ;test
	OUT		PORTB, mpr ;test

	;Check if any buttons are pressed
	IN		mpr, PIND
	COM		mpr
	ANDI	mpr, 0b11110011
	;CPI		mpr, $00
	BREQ	MAIN_REMOTE

	clr		prep_to_send

	IN		mpr, PIND
	SBRS	mpr, 7
	LDI		prep_to_send, cmd_turnLeft	;Left
	SBRS	mpr, 6
	LDI		prep_to_send, cmd_turnRight	;Right
	SBRS	mpr, 5
	LDI		prep_to_send, cmd_backward	;Backward
	SBRS	mpr, 4
	LDI		prep_to_send, cmd_forward	;Forward
	SBRS	mpr, 3
	NOP		;NOTHING
	SBRS	mpr, 2
	NOP		;NOTHING
	SBRS	mpr, 1
	LDI		prep_to_send, cmd_halt		;Halt
	SBRS	mpr, 0
	LDI		prep_to_send, cmd_freeze	;Freeze


MAIN_REMOTE_LOOP1:
	LDS		mpr, UCSR1A
	SBRS	mpr, UDRE1
	rjmp	MAIN_REMOTE_LOOP1

	;Actually send command
	LDI		mpr, RemoteTargetID
	STS		UDR1, mpr

MAIN_REMOTE_LOOP2:
	LDS		mpr, UCSR1A
	SBRS	mpr, UDRE1
	rjmp	MAIN_REMOTE_LOOP2

	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP

	;Actually send command
	STS		UDR1, prep_to_send

	;Output "SEND" flash
	LDI		mpr, $FF   ;test
	OUT		PORTB, mpr ;test
	LDI		waitCnt, remoteWaitTime	;test
	CALL	Wait					;test

	rjmp	MAIN_REMOTE


; ----------------------------
; BEGIN OF MAIN_INTERFACE
; ----------------------------

MAIN_INTERFACE:
	NOP

	;MOV		mpr, rx_deviceid
	;OUT		PORTB, mpr
	;Need to add bumper parsing

	;Check if a "freeze" command has been recieved
	LDI		mpr, $03
	CP		rx_deviceid, mpr
	BREQ	MAIN_INTERFACE_RECEIVE_FREEZE
	
	;LDI		mpr, cmd_freeze
	;CP		mpr , rx_actioncode
	;BREQ	MAIN_INTERFACE_RECEIVE_FREEZE

	;Check if "my" id has been recieved
	LDI		mpr, DeviceID
	CP		rx_deviceid, mpr	;Compare the recieved device ID to the current id
								;if it doesn't match, then restart this part of the
	BRNE	MAIN_INTERFACE		;program

	;If we've reached this point, we can assume that we are working with a valid command
	MOV		mpr, rx_actioncode
	;We need to find out if the command was to send a freeze command
	CPI		mpr, cmd_freeze				;If the command recieved was the freeze command,
	BREQ	MAIN_INTERFACE_SEND_FREEZE  ;branch to the send freeze command branch
	;Else, we need to output to the motors

	LSL		mpr						;Rotate left to get an actual motor command
	OUT		PORTB, mpr				;Output actual motor command to the motors
	MOV		previousMotorCommand, mpr	;Store this value into our storage register
	MOV		previousActionCode, rx_actioncode

	;LDI		mpr, $FF
	;OUT		PORTB, mpr

	;LDI		mpr, remoteWaitTime
	;MOV		waitCnt, mpr
	;CALL	Wait

	;MOV		mpr, previousMotorCommand
	;OUT		PORTB, mpr

	rjmp	MAIN_INTERFACE ;Return to top of program

MAIN_INTERFACE_RECEIVE_FREEZE:
	CLI		;We don't want anything else going on

	LDI		mpr, Halt  ;stop the robot
	OUT		PORTB, mpr

	;Wait for 5 seconds... [TODO: Replace with timers!]
	LDI		waitCnt, freezeTime
	CALL	Wait
	LDI		waitCnt, freezeTime
	CALL	Wait

	;Output previously valid motor command to the freeze
	LDI		mpr, $55	;DEBUG
	OUT		PORTB, mpr	;DEBUG
	;OUT		PORTB, previousMotorCommand
	MOV		rx_actioncode, previousActionCode
	LDI		mpr, DeviceID
	MOV		rx_deviceid, mpr

	SEI		;Return interrupts to normal operation
	rjmp	MAIN_INTERFACE

MAIN_INTERFACE_SEND_FREEZE:  ;process the freeze command
	;Gotsta do the freeze stuff!
	CLI			;We don't want to recieve our own freeze command
	;Configure USART to not receive
	LDI		mpr, (1<<TXEN1) 
	STS		UCSR1B, mpr     ;Enable TX ONLY (disable RX)

MAIN_INTERFACE_SEND_FREEZE_LOOP:
	LDS		mpr, UCSR1A
	SBRS	mpr, UDRE1
	rjmp	MAIN_INTERFACE_SEND_FREEZE_LOOP

	;Actually send command
	LDI		mpr, $03		;By convention, all zeros is an "all robot"
							;freeze command
	STS		UDR1, mpr

	;We don't want to send freeze forever, so load the previous command and proceess
	;it the next iteration through MAIN_INTERFACE
	MOV		rx_actioncode, previousActionCode
	LDI		mpr, DeviceID
	MOV		rx_deviceid, mpr
	
	LDI		mpr, robotWaitTime
	MOV		waitCnt, mpr
	CALL	Wait

	NOP  ;Couplea NOPs for good measure
	NOP
	
	;Renable RX USART
	LDI		mpr, (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1) ;0b11011000
	STS		UCSR1B, mpr     ;Enable RX, TX, Reciever, Transmitter

	SEI			;Reenable freezing
	rjmp	MAIN_INTERFACE

; ----------------------------
; END OF MAIN_INTERFACE
; ----------------------------

MAIN_RECIEVER:
	NOP

	rjmp	MAIN_RECIEVER

    jmp MAIN



ISR_RX_COMPLETE:
	PUSH	mpr

	LDS		mpr, UDR1	;Load the RX'd data into mpr
	SBRS	mpr, 7		;Check if "Action Code"
	rjmp	DeviceIDLoad
	rjmp	ActionCodeLoad

DeviceIDLoad:
	mov		rx_deviceid, mpr
	rjmp	ISR_RX_END
ActionCodeLoad:
	mov		rx_actioncode, mpr
	rjmp	ISR_RX_END
	

ISR_RX_END:
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





