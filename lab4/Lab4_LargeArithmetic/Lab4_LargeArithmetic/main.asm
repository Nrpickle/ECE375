;***********************************************************
;*
;*	main.asm
;*
;*	This program contains functions that preform multi-digit arithmetic
;*
;*	This is the skeleton file Lab 4 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Nick McComb
;*	   Date: 10/21/2015
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multipurpose register
.def	mpr2 = r22
.def	rlo = r0				; Low byte of MUL result
.def	rhi = r1				; High byte of MUL result
.def	zero = r2				; Zero register, set to zero in INIT, useful for calculations
.def	A = r3					; An operand
.def	B = r4					; Another operand

.def	oloop = r17				; Outer Loop Counter
.def	iloop = r18				; Inner Loop Counter

.equ	addrA = $0100			; Beginning Address of Operand A data
.equ	addrB = $0103			; Beginning Address of Operand B data
.equ	LAddrP = $0106			; Beginning Address of Product Result
.equ	HAddrP = $0111			; End Address of Product Result

.equ	addr_add_A = $0112
.equ    addr_add_B = $0114
.equ	addr_add_RESULT = $0116


;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

.org	$0046					; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:							; The initialization routine
		; Initialize Stack Pointer
		; TODO					; Init the 2 stack pointer registers
		LDI		mpr, LOW(RAMEND)  ;Low Byte of End SRAM Address
		OUT		SPL, mpr		  ;Write byte to SPL
		LDI		mpr, HIGH(RAMEND) ;High Byte of End SRAM Address
		OUT		SPH, mpr		  ;Write byte to SPH

		clr		zero			; Set the zero register to zero, maintain
								; these semantics, meaning, don't load anything
								; to it.


;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------
MAIN:							; The Main program
		; Setup the add funtion
		; Add the two 16-bit numbers
		LDI		YL, LOW(  addr_add_A)
		LDI		YH, HIGH( addr_add_A)
		LDI		mpr, 0xFF		;LSByte
		ST		Y+, mpr
		LDI		mpr, 0xFF       ;MSByte
		ST		Y, mpr

		LDI		YL, LOW(  addr_add_B)
		LDI		YH, HIGH( addr_add_B)
		LDI		mpr, 0xFF       ;LSByte
		ST		Y+, mpr	
		LDI		mpr, 0xFF       ;MSByte
 		ST		Y, mpr

		rcall	ADD16			; Call the add function
		; The result is now in 0114:0116

		; First operand (result from addition)
		LDI		YL, LOW(  addrA)
		LDI		YH, HIGH( addrA)
		LDI		ZL, LOW(  addr_add_RESULT)
		LDI		ZH, HIGH( addr_add_RESULT)
		
		LD		mpr, Z+
		ST		Y+, mpr
		LD		mpr, Z+
		ST		Y+, mpr
		LD		mpr, Z
		ST		Y, mpr

		; Second operand (same as first operand)
		LDI		YL, LOW(  addrB)
		LDI		YH, HIGH( addrB)
		LDI		ZL, LOW(  addr_add_RESULT)
		LDI		ZH, HIGH( addr_add_RESULT)

		LD		mpr, Z+
		ST		Y+, mpr
		LD		mpr, Z+
		ST		Y+, mpr
		LD		mpr, Z
		ST		Y, mpr

		; Setup the multiply function

		; Multiply two 24-bit numbers
		RCALL	MUL24			; Call the multiply function

		NOP
DONE:	rjmp	DONE			; Create an infinite while loop to signify the 
								; end of the program.

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: ADD16
; Desc: Adds two 16-bit numbers and generates a 24-bit number
;		where the high byte of the result contains the carry
;		out bit.
;-----------------------------------------------------------
ADD16:
		; Save variables by pushing them to the stack
		PUSH 	A				; Save A register
		PUSH	B				; Save B register
		PUSH	rhi				; Save rhi register
		PUSH	rlo				; Save rlo register
		PUSH	zero			; Save zero register
		PUSH	XH				; Save X-ptr
		PUSH	XL
		PUSH	YH				; Save Y-ptr
		PUSH	YL				
		PUSH	ZH				; Save Z-ptr
		PUSH	ZL
		PUSH	oloop			; Save counters
		PUSH	iloop				

		;     AH, AL
		;     BH, BL
		; RC, RH, RL

		; FIRST BYTE
		LDI		ZL, LOW(addr_add_A)
		LDI		ZH, HIGH(addr_add_A)
		LD		mpr, Z  ;Put AL into mpr

		LDI		ZL, LOW(addr_add_B)
		LDI		ZH, HIGH(addr_add_B)
		LD		mpr2, Z+ ;Put BL into mpr2

		ADD		mpr, mpr2  ;Add BL+AL into mpr

		LDI		YL, LOW(addr_add_RESULT)    ;Load the result location into the Y reg
		LDI		YH, HIGH(addr_add_RESULT)   ; 

		ST		Y+, mpr ;Store (BL+AL) into RL, Y now points to RH

		; SECOND BYTE
		LD		mpr2, Z ;Put BH into mpr2

		LDI		ZL, LOW(addr_add_A)   ;Load the first operand
		LDI		ZH, HIGH(addr_add_A)  ;
		INC		ZL                    ;Increment to get AH
		LD		mpr, Z ;Put AH into mpr

		ADC		mpr, mpr2 ;Add AH, BH, and carry -> mpr

		ST		Y+, mpr   ;Store RH into the result, Y now points to RC
		
		; CARRY BIT
		BRCS	if_carry
		;If not carry
		clr		mpr
		rjmp	func_end
if_carry: ;if carry
		LDI		mpr, 0x01
func_end: 
		ST		Y, mpr ;Store mpr into RC


		; Restore variable by popping them from the stack in reverse order
		POP		iloop			; Restore all registers in reverse order
		POP		oloop
		POP		ZL				
		POP		ZH
		POP		YL
		POP		YH
		POP		XL
		POP		XH
		POP		zero
		POP		rlo
		POP		rhi
		POP		B
		POP		A

		ret						; End a function with RET

;-----------------------------------------------------------
; Func: MUL24
; Desc: Multiplies two 24-bit numbers and generates a 48-bit 
;		result.
;-----------------------------------------------------------
MUL24:
		push 	A				; Save A register
		push	B				; Save B register
		push	rhi				; Save rhi register
		push	rlo				; Save rlo register
		push	zero			; Save zero register
		push	XH				; Save X-ptr
		push	XL
		push	YH				; Save Y-ptr
		push	YL				
		push	ZH				; Save Z-ptr
		push	ZL
		push	oloop			; Save counters
		push	iloop				

		clr		zero			; Maintain zero semantics

		; Set Y to beginning address of B
		ldi		YL, low(addrB)	; Load low byte
		ldi		YH, high(addrB)	; Load high byte

		; Set Z to begginning address of resulting Product
		ldi		ZL, low(LAddrP)	; Load low byte
		ldi		ZH, high(LAddrP); Load high byte

		; Begin outer for loop
		ldi		oloop, 3		; Load counter
MUL24_OLOOP:
		; Set X to beginning address of A
		ldi		XL, low(addrA)	; Load low byte
		ldi		XH, high(addrA)	; Load high byte

		; Begin inner for loop
		ldi		iloop, 3		; Load counter
MUL24_ILOOP:
		ld		A, X+			; Get byte of A operand
		ld		B, Y			; Get byte of B operand
		mul		A,B				; Multiply A and B
		ld		A, Z+			; Get a result byte from memory
		ld		B, Z+			; Get the next result byte from memory
		add		rlo, A			; rlo <= rlo + A
		adc		rhi, B			; rhi <= rhi + B + carry
		ld		A, Z			; Get a third byte from the result
		adc		A, zero			; Add carry to A
		st		Z, A			; Store third byte to memory
		st		-Z, rhi			; Store second byte to memory
		st		-Z, rlo			; Store third byte to memory
		adiw	ZH:ZL, 1		; Z <= Z + 1			
		dec		iloop			; Decrement counter
		brne	MUL24_ILOOP		; Loop if iLoop != 0
		; End inner for loop

		sbiw	ZH:ZL, 2		; Z <= Z - 1
		adiw	YH:YL, 1		; Y <= Y + 1
		dec		oloop			; Decrement counter
		brne	MUL24_OLOOP		; Loop if oLoop != 0
		; End outer for loop
		 		
		pop		iloop			; Restore all registers in reverves order
		pop		oloop
		pop		ZL				
		pop		ZH
		pop		YL
		pop		YH
		pop		XL
		pop		XH
		pop		zero
		pop		rlo
		pop		rhi
		pop		B
		pop		A

		ret						; End a function with RET

;-----------------------------------------------------------
; Func: MUL16
; Desc: An example function that multiplies two 16-bit numbers
;			A - Operand A is gathered from address $0101:$0100
;			B - Operand B is gathered from address $0103:$0102
;			Res - Result is stored in address 
;					$0107:$0106:$0105:$0104
;		You will need to make sure that Res is cleared before
;		calling this function.
;-----------------------------------------------------------
MUL16:
		push 	A				; Save A register
		push	B				; Save B register
		push	rhi				; Save rhi register
		push	rlo				; Save rlo register
		push	zero			; Save zero register
		push	XH				; Save X-ptr
		push	XL
		push	YH				; Save Y-ptr
		push	YL				
		push	ZH				; Save Z-ptr
		push	ZL
		push	oloop			; Save counters
		push	iloop				

		clr		zero			; Maintain zero semantics

		; Set Y to beginning address of B
		ldi		YL, low(addrB)	; Load low byte
		ldi		YH, high(addrB)	; Load high byte

		; Set Z to begginning address of resulting Product
		ldi		ZL, low(LAddrP)	; Load low byte
		ldi		ZH, high(LAddrP); Load high byte

		; Begin outer for loop
		ldi		oloop, 2		; Load counter
MUL16_OLOOP:
		; Set X to beginning address of A
		ldi		XL, low(addrA)	; Load low byte
		ldi		XH, high(addrA)	; Load high byte

		; Begin inner for loop
		ldi		iloop, 2		; Load counter
MUL16_ILOOP:
		ld		A, X+			; Get byte of A operand
		ld		B, Y			; Get byte of B operand
		mul		A,B				; Multiply A and B
		ld		A, Z+			; Get a result byte from memory
		ld		B, Z+			; Get the next result byte from memory
		add		rlo, A			; rlo <= rlo + A
		adc		rhi, B			; rhi <= rhi + B + carry
		ld		A, Z			; Get a third byte from the result
		adc		A, zero			; Add carry to A
		st		Z, A			; Store third byte to memory
		st		-Z, rhi			; Store second byte to memory
		st		-Z, rlo			; Store third byte to memory
		adiw	ZH:ZL, 1		; Z <= Z + 1			
		dec		iloop			; Decrement counter
		brne	MUL16_ILOOP		; Loop if iLoop != 0
		; End inner for loop

		sbiw	ZH:ZL, 1		; Z <= Z - 1
		adiw	YH:YL, 1		; Y <= Y + 1
		dec		oloop			; Decrement counter
		brne	MUL16_OLOOP		; Loop if oLoop != 0
		; End outer for loop
		 		
		pop		iloop			; Restore all registers in reverves order
		pop		oloop
		pop		ZL				
		pop		ZH
		pop		YL
		pop		YH
		pop		XL
		pop		XH
		pop		zero
		pop		rlo
		pop		rhi
		pop		B
		pop		A
		ret						; End a function with RET

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

; Enter any stored data you might need here

;***********************************************************
;*	Additional Program Includes
;***********************************************************
; There are no additional file includes for this program
