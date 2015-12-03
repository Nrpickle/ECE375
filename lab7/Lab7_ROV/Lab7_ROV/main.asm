;***********************************************************
;*
;*	Lab 7 Remotely Operated Vehicle
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

.def	rx_deviceid = r5			;Stores the actionable device id
.def	rx_actioncode = r6			;Stores the actionable action code
.def	previousMotorCommand = r7	;Stores the previous valid motor command
.def	previousActionCode = r8		;Stores previous valid action code
.def	rx_deviceid_temp = r9		;Stores the latest recieved device id
.def	rx_actioncode_temp = r10	;Stores the latest recieved action code
.def	lifeCounter = r11			;Number of labs

.equ	remoteWaitTime = 20		;How long the remote keeps the lights on, and waits to send another
.equ	robotWaitTime = 20		;This is the time that the robot waits to turn on Rx after sending a freeze
.equ	freezeTime = 250		;1/2 of the total freeze time
.equ	WTime = 100				; Bumper wait time
.equ	EngEnR = 4				; right Engine Enable Bit
.equ	EngEnL = 7				; left Engine Enable Bit
.equ	EngDirR = 5				; right Engine Direction Bit
.equ	EngDirL = 6				; left Engine Direction Bit

;Defines for all of the commands to be sent/recieved over IR
.equ	cmd_forward = 0b10110000
.equ	cmd_backward = 0b10000000
.equ	cmd_turnRight = 0b10100000
.equ	cmd_turnLeft = 0b10010000
.equ	cmd_halt = 0b11001000
.equ	cmd_freeze = 0b11111000

;Defines commands to be sent to the motors
.equ	MovFwd = (1<<EngDirR|1<<EngDirL)	; Move Forwards Command
.equ	MovBck = $00				; Move Backwards Command
.equ	TurnR = (1<<EngDirL)			; Turn Right Command
.equ	TurnL = (1<<EngDirR)			; Turn Left Command
.equ	Halt = (1<<EngEnR|1<<EngEnL)		; Halt Command

; Device ID specification
;                    - Defined in spec to be 0
;                    |- Set for remote control device
;                    ||- Set for interface robot
;                    |||- Set for recieve robot (depreciated)
;                    ||||
;.equ	DeviceID = 0b01000111 

;Note: All three configurations require all 3 IDs to be defined, for reasons.

;Configuration for Remote
;.equ	RemoteTargetId = $27 ;0b00100111
;.equ	DeviceID = $47
;.equ	targetRobotID = $23

;Configuration for Robot1 (doesn't matter, just what I used)
;.equ	RemoteTargetId = $27 ;0b00100111
;.equ	DeviceID = $27
;.equ	targetRobotId = $23

;Configuration for Robot2
.equ	RemoteTargetId = $27 ;0b00100111
.equ	DeviceID = $23
.equ	targetRobotId = $22


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
		;;None used;;
		
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
		MOV		rx_deviceid_temp, mpr
		LDI		mpr, cmd_forward
		MOV		rx_actioncode_temp, mpr

		;Specially set output for robot, because the remote will override this
		;with it's own output values anyway
		LDI		mpr, MovFwd
		;OUT		PORTB, mpr
		MOV		previousMotorCommand, mpr  ;Store the default previous motor command
		LDI		mpr, cmd_forward
		MOV		previousActionCode, mpr		;Store the forward command in the previous 
											;action code
		;Load 3 lives into 'da computer
		LDI		mpr, 3
		MOV		lifeCounter, mpr

		;Enable Interrupts
		SEI

		RJMP	MAIN

MAIN:
	;Determine where in execution code to send robot (this allows all of the code to use the same file
	LDI		mpr, DeviceID
	SBRC	mpr, 6  ;Check bit 6 to see if device is a remote
	rjmp	MAIN_REMOTE
	SBRC	mpr, 5  ;Check bit 5 to see if device is an interface robot
	jmp		MAIN_INTERFACE
	SBRC	mpr, 4  ;Check bit 4 to see if device is a reciever robot
	jmp		MAIN_RECIEVER ;DEPRECIATED


; ----------------------------
; BEGIN OF MAIN_REMOTE
; ----------------------------

MAIN_REMOTE:
	;By default, we want to output all zeros to the screen
	LDI		mpr, $00   
	OUT		PORTB, mpr

	;Check if any buttons are pressed
	IN		mpr, PIND
	COM		mpr
	ANDI	mpr, 0b11110011
	;CPI		mpr, $00
	BREQ	MAIN_REMOTE

	clr		prep_to_send	;Clear the register that will be sent out over IR

	;Parsing whether or not the pins are pressed, then adds the appropriate
	;command to the prep register
	;This can work for multiple buttons too
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

	;Wait until we are ready to send, really a formality here
MAIN_REMOTE_LOOP1:
	LDS		mpr, UCSR1A
	SBRS	mpr, UDRE1
	rjmp	MAIN_REMOTE_LOOP1

	;Actually send command
	LDI		mpr, RemoteTargetID
	STS		UDR1, mpr
	
	;Wait until ready to send again
MAIN_REMOTE_LOOP2:
	LDS		mpr, UCSR1A
	SBRS	mpr, UDRE1
	rjmp	MAIN_REMOTE_LOOP2

	;Actually send command
	STS		UDR1, prep_to_send

	;Output "SEND" flash (just user feedback)
	LDI		mpr, $FF   
	OUT		PORTB, mpr 
	LDI		waitCnt, remoteWaitTime	
	CALL	Wait					

	rjmp	MAIN_REMOTE		;Restart the remote "main"


; ----------------------------
; BEGIN OF MAIN_INTERFACE
; ----------------------------

MAIN_INTERFACE:
	NOP


	;First, we need to check for bumpers. If the buttons are pressed, then
	;we want to run the according command, then parse parse the most recent instruction recieved
	IN		mpr, PIND
	SBRS	mpr, 1			;Button 1 press
	CALL	HitLeft
	SBRS	mpr, 0			;Button 2 press
	CALL	HitRight

	;Now we want to put the temp stuff actually into the parsing register
	;This prevents any funkiness with interrupts overriding values that
	;we are currently using
	MOV		rx_deviceid, rx_deviceid_temp
	MOV		rx_actioncode, rx_actioncode_temp

	;LDI		mpr, cmd_freeze
	;CP		mpr , rx_actioncode
	;BREQ	MAIN_INTERFACE_RECEIVE_FREEZE

	;Check if "my" id has been recieved
	LDI		mpr, DeviceID
	CP		rx_deviceid, mpr	;Compare the recieved device ID to the current id
								;if it doesn't match, then restart this part of the
	BRNE	MAIN_INTERFACE		;program

	;If we've reached this point, we can assume that we are working with a valid command
	
	;Check if an "all robot freeze" command has been recieved
	LDI		mpr, $FF
	CPSE	mpr, rx_actioncode
	RJMP	MAIN_SKIP_FREEZE
	RJMP	MAIN_INTERFACE_RECEIVE_FREEZE


; ----------------------------
; BEGIN OF MAIN_SKIP_FREEZE
; ----------------------------

;This handles the freeze commands. 

MAIN_SKIP_FREEZE:

	MOV		mpr, rx_actioncode
	;We need to find out if the command was to send a freeze command
	CPI		mpr, cmd_freeze				;If the command recieved was the freeze command,
	BREQ	MAIN_INTERFACE_SEND_FREEZE  ;branch to the send freeze command branch
	;Else, we need to output to the motors
	
	LSL		mpr						;Rotate left to get an actual motor command
	OUT		PORTB, mpr				;Output actual motor command to the motors
	MOV		previousMotorCommand, mpr	;Store this value into our storage register
	MOV		previousActionCode, rx_actioncode

	rjmp	MAIN_INTERFACE ;Return to top of program

; -------------------------------
; BEGIN OF MAIN_RECIEVE_FREEZE
; -------------------------------

;This handles the robot recieving a freeze command
;which is a very special case compared to the others

MAIN_INTERFACE_RECEIVE_FREEZE:
	CLI		;We don't want anything else going on

	LDI		mpr, Halt  ;stop the robot
	OUT		PORTB, mpr

	;Wait for 5 seconds... [TODO: Replace with timers!]
	LDI		waitCnt, freezeTime
	CALL	Wait
	LDI		waitCnt, freezeTime
	CALL	Wait

	;Code to only last 3 lives
	DEC		lifeCounter
	BREQ	ULTIMATE_DEATH


	;Output previously valid motor command to the freeze
	MOV		rx_actioncode_temp, previousActionCode
	LDI		mpr, DeviceID
	MOV		rx_deviceid_temp, mpr

	SEI		;Return interrupts to normal operation
	rjmp	MAIN_INTERFACE

; -------------------------------
; BEGIN OF ULTIMATE_DEATH
; -------------------------------

;This is where execution goes to die

;ULTIMATE DEATH graveyard
ULTIMATE_DEATH:
	RJMP	ULTIMATE_DEATH

; -------------------------------
; BEGIN OF MAIN_RECIEVE_FREEZE
; -------------------------------

;This handles the robot needing to send a freeze command
;which is a very special case compared to the others

MAIN_INTERFACE_SEND_FREEZE:  ;process the freeze command
	;Gotsta do the freeze stuff!
	CLI			;We don't want to recieve our own freeze command
	;Configure USART to not receive
	LDI		mpr, (1<<TXEN1) 
	STS		UCSR1B, mpr     ;Enable TX ONLY (disable RX)

MAIN_INTERFACE_SEND_FREEZE_LOOP2:
	LDS		mpr, UCSR1A
	SBRS	mpr, UDRE1
	rjmp	MAIN_INTERFACE_SEND_FREEZE_LOOP2

	;Actually send command
	LDI		mpr, $FF		;By convention, all zeros is an "all robot"
							;freeze command
	STS		UDR1, mpr

MAIN_INTERFACE_SEND_FREEZE_LOOP:
	LDS		mpr, UCSR1A
	SBRS	mpr, UDRE1
	rjmp	MAIN_INTERFACE_SEND_FREEZE_LOOP

	;Send the targetRobot ID
	LDI		mpr, targetRobotID
	STS		UDR1, mpr

	;We don't want to send freeze forever, so load the previous command and proceess
	;it the next iteration through MAIN_INTERFACE
	MOV		rx_actioncode_temp, previousActionCode
	LDI		mpr, DeviceID
	MOV		rx_deviceid_temp, mpr
	
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


;USART Recieved Interrupt

ISR_RX_COMPLETE:
	PUSH	mpr

	LDS		mpr, UDR1	;Load the RX'd data into mpr
	SBRS	mpr, 7		;Check if "Action Code"
	rjmp	DeviceIDLoad
	rjmp	ActionCodeLoad

DeviceIDLoad:
	mov		rx_deviceid_temp, mpr
	rjmp	ISR_RX_END
ActionCodeLoad:
	mov		rx_actioncode_temp, mpr
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

