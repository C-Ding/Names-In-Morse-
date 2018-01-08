;*----------------------------------------------------------------------------
;* Name:    Lab_2_program.s 
;* Purpose: This code template is for Lab 2 on the Tiva Board
;* Author: Eric Praetzel
;*----------------------------------------------------------------------------*/

; The setup code is based upon InputOutput.s from the book:
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
;

; One can also define an address, or constant, using the EQUate directive

RED       EQU 0x02				; bit positions for the Tiva board LEDs and switches all on port F
BLUE      EQU 0x04
GREEN     EQU 0x08
SW1       EQU 0x10                 ; switch on the left side of the Launchpad board
SW2       EQU 0x01                 ; switch on the right side of the Launchpad board


        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start

SOMEDELAY             EQU 5000000      ; approximately 1s delay at ~16 MHz clock

Start
;
; Turn off all LEDs 
    BL  Port_Init                  ; initialize input and output pins of the Ports used in ECE 222

		MOV R1, #2					; pass in a 2 to get a high pitched beep
		BL	SpeakerBeep
		
		MOV R1, #500				; delay for a while between beeps
		BL ShortDelay
		
		MOV R1, #30					; pass in a 30 to get a lower pitched beep
		BL	SpeakerBeep

ResetLUT
		LDR         R5, =InputLUT            ; assign R5 to the address at label LUT

NextChar
		
        	LDRB        R0, [R5]		; Read a character to convert to Morse
        	ADD         R5, #1              ; point to next value for number of delays, jump by 1 byte
		TEQ         R0, #0              ; If we hit 0 (null at end of the string) then reset to the start of lookup table
		BNE		ProcessChar	; If we have a character process it

		MOV		R0, #4		; delay 4 extra spaces (7 total) between words
		BL		DELAY
		BEQ         ResetLUT

ProcessChar	BL		CHAR2MORSE	; convert ASCII to Morse pattern in R1		


;	Alternate Method #3
; All of the above methods do not use the shift operation properly.
; In the shift operation the bit which is being lost, or pushed off of the register,
; "falls" into the C flag - then one can BCC (Branch Carry Clear) or BCS (Branch Carry Set)
; This method works very well when coupled with an instruction which counts the number 
;  of leading zeros (CLZ) and a shift left operation to remove those leading zeros.

	CLZ R2, R1			; store the number of leading zeros in R2
leadingzeros
	LSL R1, R1, #1		; left shift R1 by 1
	SUBS R2, R2, #1		; decrement the counter

	BNE leadingzeros	; R1 contains the hex value of the character in the top bits
	
getbit
	CBZ R1, CHAR_DONE	; if R1 = 0 (no more bits to shift + display) then go to char_done
	LSLS R1, R1, #1		; shift left, first bit falls into carry flag
	BCS TURN_ON			; carry = 1: turn on
	BCC TURN_OFF		; carry = 0: turn off
TURN_ON
	BL LED_ON
	B getbit
TURN_OFF
	BL LED_OFF
	B getbit
	
CHAR_DONE
	BL LED_OFF			; turn off the LED (includes 1 delay)
	MOV R0, #2			; delay 2 more times (total of 3 delay cycles)
	BL DELAY
	B NextChar			; ready to process next character


		ALIGN				; make sure things fall on word addresses
;
; Data used in the Morse Code program
;
; DCB is Define Constant Byte size
; DCW is Define Constant Word (16-bit) size
; EQU is EQUate or assign a value.  This uses no memory but instead of typing the same address in many places one can just use an EQU to then use a label
;

; One way to provide a data to convert to Morse code is to use a string in memory.
; Simply read bytes of the string until the NULL or "0" is hit.  This makes it very easy to loop until done.
;
InputLUT	DCB		"SOS", 0	; strings must be stored, and read, as BYTES

		ALIGN				; make sure things fall on word addresses
MorseLUT 
		DCW 	0x17, 0x1D5, 0x75D, 0x75 	; A, B, C, D
		DCW 	0x1, 0x15D, 0x1DD, 0x55 	; E, F, G, H
		DCW 	0x5, 0x1777, 0x1D7, 0x175 	; I, J, K, L
		DCW 	0x77, 0x1D, 0x777, 0x5DD 	; M, N, O, P
		DCW 	0x1DD7, 0x5D, 0x15, 0x7 	; Q, R, S, T
		DCW 	0x57, 0x157, 0x177, 0x757 	; U, V, W, X
		DCW 	0x1D77, 0x775 			; Y, Z

		ALIGN				; make sure things fall on word addresses


; Subroutines
;
;			convert ASCII character to Morse pattern
;			pass ASCII character in R0, output in R1
;			index into MorseLuT must be by steps of 2 bytes
;
;	INPUT:  R0 ASCII character as a byte
;	OUTPUT:	R1 Morse Code pattern in the 16 lowest bits with the upper 16 bits cleared

CHAR2MORSE	STMFD		R13!,{R14}	; push Link Register (return address) on stack
		
		;R0 has the character in ASCII -> we want to convert it to hex. 
		
		
		SUB R0, R0, #0x41 ; R0 is the offset and we can lookup the hex value in MorseLUT
		LDR R2, =MorseLUT ; R2 has the address of MorseLUT
		ADD R0, R0, R0		; multiply the offset by 2 since the letters in MorseLUT are 2 bytes apart
		ADD R2, R2, R0 ; Add R0 to R2 so R2 now contains MorseLUT+offset -> address of a hex character
		LDR R1, [R2] ;R1 has the hex value of the character now
		MOVT R1, #0000000000000000
			
		;
		;... add code here to convert the ASCII code to an index (subtract 41) and lookup the Morse patter in the Lookup Table
		;
		
		LDMFD		R13!,{R15}	; restore LR to R15 the Program Counter to return


; Turn the LED on, but deal with the stack in a simpler way
; NOTE: This method of returning from subroutine (BX  LR) does NOT work if subroutines are nested!!
;
;	INPUT:  none
;	OUTPUT:  none
;
LED_ON 	   	STMFD		R13!,{R3, R4, R14}		; preserve R3 and R4 on the R13 stack
;		... insert your code here

	MOV R2, #RED			; load in the value to turn the RED led ON
	LDR R4, =GPIO_PORTF_DATA_R ; pointer to Port F data register
    STR R2, [R4]               ; write data to Port F to turn lights on and off
	MOV R0, #1	;get ready to delay 1*500ms
	BL DELAY
		
		LDMFD		R13!,{R3, R4, R15}

; Turn the LED off, but deal with the stack in the proper way
; the Link register gets pushed onto the stack so that subroutines can be nested
;
;	INPUT:  none
;	OUTPUT:  none
;
LED_OFF	   	STMFD		R13!,{R3, R14}	; push R3 and Link Register (return address) on stack
;		... insert your code here

	MOV R2, #0				; load in the value to turn the RED led ON
	LDR R4, =GPIO_PORTF_DATA_R ; pointer to Port F data register
    STR R2, [R4]               ; write data to Port F to turn lights on and off
	MOV R0, #1	;get ready to delay 1*500ms
	BL DELAY

		LDMFD		R13!,{R3, R15}	; restore R3 and LR to R15 the Program Counter to return

;	Delay 500ms * R0 times
;	Use a modified delay loop from Lab-1
;
;	INPUT:  R0 - how many times is the basic time loop run? 0 means none and just exit without any delay
;	OUTPUT:  none
;
DELAY			STMFD		R13!,{R2, R14}
	LDR R2, =SOMEDELAY			; load 5000000 into R2
	MUL R0, R0, R2				; multiply # of delay cycles * delay time
delay1
	SUBS R0, R0, #1             ; decrement until 0
	BNE delay1
	
				LDMFD R13!,{R2, R15}
	
MultipleDelay		TEQ		R0, #0		; test R0 to see if it's 0 and if so exit - this sets the Zero flag so you can use BEQ, BNE
;			... insert your code here
exitDelay		LDMFD		R13!,{R2, R14}

; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
;   DO NOT EDIT CODE BELOW THIS LINE
; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
; Tons of initialization to be done in order to use the I/O ports as they're off by default.
;
; Define the addresses and provide functions to initialize everything.

GPIO_PORTF_DIR_R   EQU 0x40025400		; Port F Data Direction Register setting pins as input or output
GPIO_PORTF_DATA_R  EQU 0x400253FC		; address for reading button inputs and writing to LEDs
GPIO_PORTF_AFSEL_R EQU 0x40025420
GPIO_PORTF_PUR_R   EQU 0x40025510
GPIO_PORTF_DEN_R   EQU 0x4002551C
GPIO_PORTF_LOCK_R  EQU 0x40025520
GPIO_PORTF_CR_R    EQU 0x40025524
GPIO_PORTF_AMSEL_R EQU 0x40025528
GPIO_PORTF_PCTL_R  EQU 0x4002552C

;Section 3.1.2 Nested Vector Interrupt Controller

;The Cortex-M4F processor supports interrupts and system exceptions. The processor and the
;Nested Vectored Interrupt Controller (NVIC) prioritize and handle all exceptions. An exception
;changes the normal flow of software control. The processor uses Handler mode to handle all
;exceptions except for reset. See “Exception Entry and Return” on page 108 for more information.
;The NVIC registers control interrupt handling. See “Nested Vectored Interrupt Controller
;(NVIC)” on page 124 for more information.

;Table 3-8 on page 134 details interrupt Set / Clear 
; they allow one to enable individual interrupts and DIS? lets one disable individual interrupt numbers

; Table 2-9 Interrupts on page 104 details interrupt number / bit assignments
; Port F - Bit 30
; Timer 0A Bit 19
; Timer 0B Bit 20
 
;For edge-triggered interrupts, software must clear the interrupt to enable any further interrupts.

; NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
; the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
; and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
; is written to the Port F GPIO Lock Register.  After Port F is
; unlocked, bit 0 of the Port F GPIO Commit Register must be set to
; allow access to PF0's control registers.  On the LM4F120, the other
; bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
; that the rest of Port F can always be freely re-configured at any
; time.  Requiring this procedure makes it unlikely to accidentally
; re-configure the JTAG and NMI pins as GPIO, which can lock the
; debugger out of the processor and make it permanently unable to be
; debugged or re-programmed.
	
; These are the configuration registers which should not be touched
; Port Base addresses for the legacy (not high-performance) interface to I/O ports
GPIO_PORTA			EQU 0x40004000
GPIO_PORTB			EQU 0x40005000
GPIO_PORTC			EQU 0x40006000
GPIO_PORTD			EQU 0x40007000
GPIO_PORTE			EQU 0x40024000
GPIO_PORTF			EQU 0x40025000

; These are the masks for pins which are outputs
PORT_A_MASK			EQU 0xfc	;0xE0		; PA7,6,5 are outputs for LEDs
PORT_B_MASK			EQU 0xff	;3f	; exclude B2:3 0xff	;33		; PB5,4,1,0 are outputs %0011 0011 
PORT_C_MASK			EQU 0x30	; this hangs the CPU 0xf0	
PORT_D_MASK			EQU 0xcc	;exclude d7 0xcf	Disable D0, D1 due to short with B6, B7
PORT_E_MASK			EQU 0x3f	;0x30		; PE5,4 are outputs %0011 0000
PORT_F_MASK			EQU 0x0e		; PF has LEDs on PF 1,2,3 and buttons PF0, PF4 (don't enable buttons as outputs)
	
; Offsets are from table 10-6 on page 660
GPIO_DATA_OFFSET	EQU 0x000		; Data address is the base address - YOU HAVE TO ADD AN ADDRESS MASK TOO to read or write this!!
GPIO_DIR_OFFSET		EQU 0x400		; Direction register
GPIO_AFSEL_OFFSET EQU 0x420			; Alternate Function SELection
GPIO_PUR_OFFSET   EQU 0x510			; Pull Up Resistors
GPIO_DEN_OFFSET   EQU 0x51C			; Digital ENable
GPIO_LOCK_OFFSET  EQU 0x520
GPIO_CR_OFFSET    EQU 0x524
GPIO_AMSEL_OFFSET EQU 0x528			; Analog Mode SELect
GPIO_PCTL_OFFSET  EQU 0x52C

SYSCTL_HBCTL  EQU   0x400FE06C		; high performance bus control for ports A to F

GPIO_LOCK_KEY      EQU 0x4C4F434B  ; Unlocks the GPIO_CR register
SYSCTL_RCGCGPIO_R  EQU   0x400FE608		; Register to enable clocks to the I/O port hardware

;------------Port_Init------------
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2, R3
Port_Init
	STMFD		R13!,{R14}		; push the LR or return address

; First enable the clock to the I/O ports, by default the clocks are off to save power
; If a clock is not enabled to a port and you access it - then the processor hard faults
	LDR R1, =SYSCTL_RCGCGPIO_R      ; activate clock for Ports (see page 340)
    LDR R0, [R1]                 
    ORR R0, R0, #0x3F               ; turn on clock to all 6 ports (A to F, bits 0 to 5)
    STR R0, [R1]                  
    NOP
    NOP                             ; allow time for clock to finish
	
; Set all ports to APB bus instead of AHB - this should be unnecessary
;	LDR R1, =SYSCTL_HBCTL
;	LDR R0, [R1]
;	AND R0, #0xFFFFFFE0		; set Ports A thru F to APB (0) and leave the rest at their default
;	STR R0, [R1]

; Page 650, Table 10-1 GPIO Pins with Special Considerations.
; These pins must be left as configured after reset:
;  PA[5:0] (UART0 and SSIO), PB[3:2] (I2C), PC[3:0] (JTAG)

; Initialize the I/O ports A, B, E, F via a common subroutine Port_Init_Individual
; Call Port_Init_Individual with the following paramaters passed:
; R1 is the base port address
; R2 is the output pin mask (which bits are outputs)
; R3 is the input pin mask  (which bits get configured as inputs)

	MOV R3, #0x00				; Select no pins as input (unless it's changed as for port F)
	
; Init Port A, B, E are by default GPIO - set all output pins used to a 1 to enable them
;   and leave all of the other pins as previously configured!
    LDR R1, =GPIO_PORTA
    MOV R2, #PORT_A_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port B
    LDR R1, =GPIO_PORTB
    MOV R2, #PORT_B_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port C
    LDR R1, =GPIO_PORTC
    MOV R2, #PORT_C_MASK
	;BL Port_Init_Individual

; Init Port D
    LDR R1, =GPIO_PORTD
    MOV R2, #PORT_D_MASK
	BL Port_Init_Individual

; Init Port E
	LDR R1, =GPIO_PORTE
    MOV R2, #PORT_E_MASK			; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port F
	LDR R1, =GPIO_PORTF
    MOV R2, #PORT_F_MASK		; enable commit for Port, 1 means allow access
	MOV R3, #0x11				; enable weak pull-up on PF0 and PF4 (buttons)
	BL Port_Init_Individual

	LDMFD		R13!,{R15}		; pull the LR or return address from the stack and return


;------------Port_Init_Individual------------
; Initialize one GPIO Port with select bits as inputs and outputs
; Output: none
; Input: R1, R2, R3
; R1 has to be the port address
; R2 has to hold the mask for output pins
; R3 has to be the mask for input pins
; Modifies: R0

Port_Init_Individual
	STMFD		R13!,{R14}		; push the LR or return address
    LDR R0, =0x4C4F434B             ; unlock GPIO Port F Commit Register
    STR R0, [R1, #GPIO_LOCK_OFFSET]	; 2) unlock the lock register
	ORR R0, R2, R3					; all access to inputs and outputs as masked in R2 and R3
    STR R0, [R1, #GPIO_CR_OFFSET]	; enable commit for Port F
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1, #GPIO_AMSEL_OFFSET]	; 3) disable analog functionality
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1, #GPIO_PCTL_OFFSET]	; 4) configure as GPIO
    LDR R0, [R1, #GPIO_DIR_OFFSET]	; 5) read default direction register configuration
    ORR R0, R2						; ORR in only the bits we want as outputs
    STR R0, [R1, #GPIO_DIR_OFFSET]	; 5) set direction register
    MOV R0, #0                      ; 0 means disable alternate function 
    STR R0, [R1, #GPIO_AFSEL_OFFSET]	; 6) regular port function
    STR R3, [R1, #GPIO_PUR_OFFSET]	; pull-up resistors for PF4,PF0
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1, #GPIO_DEN_OFFSET]
	LDMFD		R13!,{R15}		; pull the LR or return address and return


; Beep the speaker on the ECE Shield using port E4 and E5
; The speaker is conencted to two pins - toggle each end for more volume than a singled ended drive
; Ensure that each beep is about the same length - 0x300 loops of delay loop
;
; Input: R1 sets the tone - 2 is a high pitch
; Output: none

SpeakerBeep
	STMFD		R13!,{R1-R3, R11, R14}		; push the LR or return address

	MOV R3, #0x300
	UDIV R3, R1		; loop the tone R1 / R3 times to ensure a total of 0x100 delays for all tones
	
	LDR R2, =GPIO_PORTE + (PORT_E_MASK << 2)
	LDR R11, [R2, #GPIO_DATA_OFFSET]		; get the initial value - read-modify-write to only change 2 bits
	AND R11, #0xcf							; clear two bits that the speaker is on
	ORR R11, #0x10		; initial speaker output (one side high 0x10, the other low 0x20)
buzz_loop
	BL ShortDelay			; delay
	EOR R11, #0x30
	STR R11, [R2, #GPIO_DATA_OFFSET]
	SUBS R3, #1
	BNE buzz_loop
	LDMFD		R13!,{R1-R3, R11, R15}		; pull the LR or return address and return
	
; ShortDelay subroutine - delay for a fixed amount of time
;
; This is a crap piece of code - only use it as an example for what never to do!!
;
; Input: R1 - how many times do we multiply the fixed delay
; Output: none

SHORTDELAYCOUNT             EQU 400      ; faction of a second delay

ShortDelay
	STMFD		R13!,{R0, R1, R14}		; push the LR or return address

delay_outer_loop
	TEQ R1, #0
	BNE not_done_delay
	B	done_delay
not_done_delay
	SUB R1, #1
	LDR R0, =SHORTDELAYCOUNT   	       ; R0 = a value to get about a second delay
delay_loop
    SUBS R0, R0, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
				; Note: For SUBs the "s" suffix means to set the status bits, without this the loops would not exit
	CBNZ R0, delay_back			; compare if not zero then keep counting down
	B delay_outer_loop
delay_back
	B delay_loop
done_delay
	LDMFD		R13!,{R0, R1, R15}		; pull the LR or return address and return

	ALIGN

Port_Table
	DCD	GPIO_PORTA + (PORT_A_MASK << 2)		; DCD - Define Constant Double Word (32-bits)
	DCD	GPIO_PORTB + (PORT_B_MASK << 2), GPIO_PORTC + (PORT_C_MASK << 2)
	DCD	GPIO_PORTD + (PORT_D_MASK << 2), GPIO_PORTE + (PORT_E_MASK << 2)
	DCD	GPIO_PORTF + (PORT_F_MASK << 2), 0

    ALIGN                           ; make sure the end of this section is aligned
    END                             ; end of file - nothing after this is assembled