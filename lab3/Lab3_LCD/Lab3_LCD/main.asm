;***********************************************************
;*
;*	Enter Name of file here
;*
;*	Enter the description of the program here
;*
;*	This is the skeleton file Lab 3 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Enter your name
;*	   Date: Enter Date
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multipurpose register required for LCD Driver
.def	mpr2 = r23              ; 2nd multipurpose reg


;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp INIT				; Reset interrupt

.org	$0046					; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:							; The initialization routine
		; Initialize Stack Pointer
								; Init the 2 stack pointer registers
		LDI		mpr, LOW(RAMEND)  ;Low Byte of End SRAM Address
		OUT		SPL, mpr		  ;Write byte to SPL
		LDI		mpr, HIGH(RAMEND) ;High Byte of End SRAM Address
		OUT		SPH, mpr		  ;Write byte to SPH

		; Initialize LCD Display
		RCALL	LCDInit			  ;Call LCD  Init subroutine
								; An RCALL statement

		; Move strings from Program Memory to Data Memory
		LDI		ZL, LOW(STRING_BEG<<1)  ;Load Z Pointer Address
		LDI		ZH, HIGH(STRING_BEG<<1) ;

		LDI		YL, LOW(LCDLn1Addr)
		LDI		YH, HIGH(LCDLn1Addr)

		LDI		mpr2, 15    ;Put 15 into the mpr2
string_lp:
		LPM		mpr, Z+     ;Get the character from memory, post inc
		st		Y+, mpr     ;Store the character to the lcd prep space
		DEC		mpr2        ;dec counter var
		BRNE	string_lp   ;if counter is not equal to 0, then goto string_lp

		;Load the second string (2nd line) into program memory
		LDI		ZL, LOW(ROBOT_STRING<<1)  ;Load Z Pointer Address (2nd string)
		LDI		ZH, HIGH(ROBOT_STRING<<1) ;
		LDI		YL, LOW(LCDLn2Addr)       ;Load Y pointer (LCD Line 2)
		LDI		YH, HIGH(LCDLn2Addr)

		LDI		mpr2, 15    ;Put 15 into the mpr2
string_lp_2:
		LPM		mpr, Z+     ;Get the character from memory, post inc
		st		Y+, mpr     ;Store the character to the lcd prep space
		DEC		mpr2        ;dec counter var
		BRNE	string_lp_2   ;if counter is not equal to 0, then goto string_lp


;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:							; The Main program
		; Display the strings on the LCD Display
		RCALL LCDWrLn1
		RCALL LCDWrLn2
								; An RCALL statement
		
		rjmp	MAIN			; jump back to main and create an infinite
								; while loop.  Generally, every main program is an
								; infinite while loop, never let the main program
								; just run off

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: Template function header
; Desc: Cut and paste this and fill in the info at the 
;		beginning of your functions
;-----------------------------------------------------------
FUNC:							; Begin a function with a label
		; Save variable by pushing them to the stack

		; Execute the function here
		
		; Restore variable by popping them from the stack in reverse order\
		ret						; End a function with RET


;***********************************************************
;*	Stored Program Data
;***********************************************************

;----------------------------------------------------------
; An example of storing a string, note the preceeding and
; appending labels, these help to access the data
;----------------------------------------------------------
STRING_BEG:
.DB		"Nick's String   "		; Storing the string in Program Memory
STRING_END:

ROBOT_STRING:
.DB		"ROBOTS RULE  :) "
ROBOT_END:

;***********************************************************
;*	Additional Program Includes
;***********************************************************
.include "LCDDriver.asm"		; Include the LCD Driver
