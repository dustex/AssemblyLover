;
; ***************************************************************************
; * uC Seminar - I2C Display and PCF8591 AD/DA       					   	*
; * Version 0.1.1                            								*
; *																			*
; * (C)2013 by Christian Feichtinger         								*
; ***************************************************************************
;
.NOLIST                		; disable Listing
.INCLUDE "include\m8515def.inc" 	; Headerfile for ATMega8515
.LIST                  		; enable Listing
;
; ============================================
;	Hardware Information
; ============================================
;
; ATMega8515 with some additional stuff
;
; ============================================
;	Ports and Pins
; ============================================
;
.EQU ResA		= PORTA			; Port A not used yet

.EQU pAC 		= PORTB			; Output Port for Analog Comparator
.EQU pinCharge	= PINB0			; Analog Comparator RC charge/discharge output
.EQU pinACref	= PINB2			; Analog Comparator reference voltage input (RC)
.EQU pinACin	= PINB3			; Analog Comparator measure input

.EQU pKey		= PORTC			; Port for Keys
.EQU iKey		= PINC			; Input Register for Keys
.EQU bUp		= PINC0			; Button up
.EQU bLeft		= PINC1			; Button left
.EQU bRight		= PINC2			; Button right
.EQU bDown		= PINC3			; Button down
.EQU KeyMask	= 0b00001111	; Button mask (PINC0...PINC3)

.EQU pLED		= PORTD			; Output Port for LEDs, UART and DEBUG Signals
.EQU pinUartRX	= PIND0			; UART receive Pin
.EQU pinUartTX	= PIND1			; UART transmit Pin
.EQU pinXXXDEBUG1 	= PIND2		; Pin 1 for Debugging
.EQU pinXXXDEBUG2	= PIND3		; Pin 2 for Debugging
.EQU pinLedR	= PIND4			; red LED on Pin D4
.EQU pinLedY	= PIND5			; yellow LED on Pin D5
.EQU pinLedG	= PIND6			; green LED on Pin D6
.EQU pinXXXDEBUG3	= PIND7		; Pin 3 for Debugging

.EQU pI2C		= PORTE			; I2C Interface Pins on Port E
.EQU iI2C		= PINE			; Input Register for I2C Port
.EQU ddrI2C		= DDRE			; I2C Datadirection register
.EQU pinSCL		= PINE0			; I2C SCL Pin
.EQU pinSDA		= PINE1			; I2C SDA Pin
;
; =======================================================
;  Constants and Definitions
; =======================================================
;
.EQU F_CPU 	= 4000000		; Oscillator Freq. in Hz
.EQU F_ISR	= 400			; Timer Interrupt Freq. in Hz [400Hz - 2,5ms]
;.EQU F_ISR	= 800			; Timer Interrupt Freq. in Hz [800Hz - 1,25ms]
							; Timer Interrupt Freq. cannot be lower than 1,25ms because UART_TX@9600 takes 1,14ms for one char
.EQU BAUD  	= 9600			; Baudrate
 
; UART Bauderate Calculations
.EQU UBRR_VAL   = ((F_CPU+BAUD*8)/(BAUD*16)-1)	; Rounding
.EQU BAUD_REAL  = (F_CPU/(16*(UBRR_VAL+1)))     ; Real Baudrate
.EQU BAUD_ERROR = ((BAUD_REAL*1000)/BAUD-1000)	; Error in per mill
 
.IF ((BAUD_ERROR>10) || (BAUD_ERROR<-10))       ; max. +/-10 per mill Error
  .ERROR "Bauderate error to high!"
.ENDIF

; Timer Interrupt Interval Calculations
.EQU INTERVAL	= F_CPU/F_ISR		; Deadline between two Timer Interrupts in Cycles
.EQU VT10MS		= F_ISR/100     	; Prescaler for 100Hz (10ms)

; Analog Comparator: Timer 0 Preset Value
.EQU T0PRESET	= 192		; T/C0 Preset Constant (265-64)

; I2C Bus definitions...
.EQU b_I2Cdir	= 0			; I2C transfer direction bit in address byte
.EQU b_I2Crd	= 1			; I2C direction read
.EQU b_I2Cwr	= 0			; I2C direction write
.EQU t_I2C_Ack	= 50		; I2C timeout for acknowledge in ms

; Character definitions
.EQU ccr	= 0x0D			; \r Carriage return character
.EQU clf	= 0x0A			; \n Line feed character
.EQU cnull	= 0x00			; \0 NULL character
.EQU cesc	= 0x1B			;    ESCAPE character
.EQU cbel	= 0x07			; \a Bell character
.EQU cbs	= 0x08			; \b Back space

; Menu definitions
.EQU NOOFMENUS	= 7			; number of menu screens
.EQU STARTMENU	= 3			; which menu should be shown first

; Flag definitions for the rFlag register
.EQU b_JOB_ADC			= 7	; Job Analog Comparator scheduled
.EQU b_JOB_PCF8591_RD	= 6	; Job PCF8591 ADC: read all 4 input channels
.EQU b_JOB_PCF8591_WR	= 5	; Job PCF8591 ADC: set output Voltage
.EQU b_JOB_UART_TX		= 4	; Job UART transmit: one byte scheduled
.EQU b_UPDATE_SCREEN	= 3	; if set, Display has to be updated
.EQU b_I2C_WAIT_STATE	= 2	; I2C engine is in state I2C_get_ack_wait (waiting for SCL pin HIGH)
.EQU b_SEC				= 1	; one second elapsed
.EQU b_MAIN_TICK		= 0	; 400Hz (2,5ms elapsed): used for executing main loop every 2,5 ms

; Flag definitions for the rFlag2 register
.EQU b_res6				= 7	; 
.EQU b_res5				= 6	; 
.EQU b_res4				= 5	; 
.EQU b_res3				= 4	; 
.EQU b_res2				= 3	; 
.EQU b_res1				= 2	; 
.EQU b_PCF8591_WR_EN	= 1	; PCF8591 ADC: write enabled by user
.EQU b_PCF8591_RD_EN	= 0	; PCF8591 ADC: read enabled by user
;

;
; ============================================
;   REGISTER Definitions
; ============================================
;
.DEF rMl		= R0		; 16Bit Multiplication Result Low Byte / LPM Instructions
.DEF rMh		= R1		; 16Bit Multiplication Result High Byte
.DEF rNULL		= R2		; always 0x00
.DEF rFF		= R3		; always 0xFF
.DEF rStampL	= R4		; ms Timestamp Low Byte
.DEF rStampH	= R5		; ms Timestamp High Byte
;.DEF rStampToL	= R6		; ms Timestamp Low Byte for Timeout calculations  - not used at the moment
;.DEF rStampToH	= R7		; ms Timestamp High Byte for Timeout calculations - not used at the moment
;.DEF rRes8		= R8		; Reserve
;.DEF rRes9		= R9		; Reserve
;.DEF rRes10	= R10		; Reserve
;.DEF rRes11	= R11		; Reserve
;.DEF rRes12	= R12		; Reserve
;.DEF rRes13	= R13		; Reserve
;.DEF rRes14	= R14		; Reserve
;.DEF rRes15	= R15		; Reserve
.DEF rwl		= R16 		; general purpose working register low
.DEF rwh		= R17		; general purpose working register high
.DEF rtemp3		= R18		; general purpose working register 3
.DEF rFlag		= R19		; Flag Register (Jobs and timing)
.DEF rFlag2		= R20		; Flag Register 2 (user enable/disable stuff)
.DEF rI2Cstat	= R21		; I2C bus status register
.DEF rKeyPressed= R22		; Button flags - Bit set to 1 if Button was pressed (Active H)
.DEF rCounter	= R23		; Counter for ADC measurements
.DEF rMp		= R24		; Pointer for menu handling
.DEF rPrescaler	= R25		; Prescaler used in Timer 1 Interrupt
;	 xl			= R26		; Data Pointer X L		
;	 xh			= R27		; Data Pointer X H	
;	 yl			= R28		; Data Pointer Y L	
;	 yh			= R29		; Data Pointer Y H	
;	 zl			= R30		; Data Pointer Z L	
;	 zh			= R31		; Data Pointer Z H	
;

;
; ============================================
;	SRAM Definitions
; ============================================
;
.DSEG
.ORG	SRAM_START
; Format: Label: .BYTE N 	; reserve N Bytes for ...

sKeyState:		.byte 3		; Byte 0: Button Status, Byte 1 + Byte 2: Debouncing Counter (2 bit counter for each input)
sRdptr:			.byte 1		; Read pointer for UART buffer
sWrptr:			.byte 1		; Write pointer for UART buffer
;sI2CTimer:		.byte 2		; Timer I2C waiting for acknowledge
sJobTimer:		.byte 2		; Timer for low frequent jobs in main loop
sTimeS:			.byte 1		; System time stamp Seconds
sTimeM:			.byte 1		; System time stamp Minutes
sTimeH:			.byte 1		; System time stamp Hours
sAdcRes:		.byte 1		; Result of internal Analog Comparator
sAdcI2cAIN0:	.byte 1		; Result of external I2C-ADC PCF8591 analog input AIN0
sAdcI2cAIN1:	.byte 1		; Result of external I2C-ADC PCF8591 analog input AIN1
sAdcI2cAIN2:	.byte 1		; Result of external I2C-ADC PCF8591 analog input AIN2
sAdcI2cAIN3:	.byte 1		; Result of external I2C-ADC PCF8591 analog input AIN3
; Total: 		19 Bytes 	End Adress: 0x0060+(17x8Bit)=0x

.ORG	0x0100
sUartBuffer:	.byte 256	; 256 Byte as ringbuffer for UART TX (whole Page 1)
;

;
; ==============================================
;   RESET and INTERRUPT Vectors
; ==============================================
;
.CSEG
.ORG	0x0000
			rjmp 	main 			; Reset-Vector
			rjmp 	deadbeef		; Ext Int 0
			rjmp 	deadbeef		; Ext Int 1
			rjmp 	deadbeef		; Timer/Counter1 Capture Event
			rjmp 	ISR_TC1			; Timer/Counter1 Compare Match A
			rjmp 	deadbeef		; Timer/Counter1 Compare Match B
			rjmp 	deadbeef		; Timer/Counter1 Overflow
			rjmp 	deadbeef		; Timer/Counter0 Overflow
			rjmp 	deadbeef		; Serial Transfer Complete
			rjmp 	deadbeef		; UART Rx Complete
			rjmp 	deadbeef		; UART Data register empty
			rjmp 	deadbeef		; UART Tx Complete
			rjmp 	ISR_ANACmp		; Analog Comparator
			rjmp 	deadbeef		; External Interrupt Request 2
			rjmp 	ISR_ANACmp		; Timer 0 Compare Match
			rjmp 	deadbeef		; EEPROM Ready
			rjmp 	deadbeef		; Store Program Memory Ready
;
; ==========================================
;    User Includes
; ==========================================
;
.CSEG
.ORG	INT_VECTORS_SIZE

.INCLUDE	"tools.inc"			; usefull subroutines
.INCLUDE	"macros.inc"		; Macros definitions
.INCLUDE	"print.inc"			; output stream driver routines
;.INCLUDE	"iic_driver.inc"	; IIC first level driver
.INCLUDE	"iic_driver_orig.inc"	; IIC first level driver - original ATMEL driver
.INCLUDE	"lcd_2x16a.inc"		; LCD driver
;
;
; ==========================================
;    DeadLoop for wrong ISR call
; ==========================================
;
deadbeef:	TOGGLE_PIN	PORTD, pinLedY	; toggle yello led to show error	
			rjmp	deadbeef			; let programm end here in case of error
			reti
;
; ============================================
;    Main Program Init
; ============================================
;
main:
; Initialize Stack
			ldi		rwl, HIGH(RAMEND)	; Initialize MSB Stack
			out		SPH, rwl
			ldi		rwl, LOW(RAMEND)	; Initialize LSB Stack
			out		SPL, rwl

; Initialize Special registers and pointers
			clr		rNULL			; always 0x00			
			clr		rFF
			dec		rFF				; always 0xFF

			sts		sRdptr, rNULL	; UART Read pointer to initial value
			sts		sWrptr, rNULL	; UART Write pointer to initial value

; DDRx Data Direction Input/Output - 1 is Output, 0 is Input
; PINx Read to get Input, if Pin is set as Input
; PORTx if Pin is Input "1" activates PullUp, if Pin is Output write here to set/reset Output

; Initialize Port A
			ldi 	rwl, 0 			; all Pins as input
			ldi 	rwh, 0xFF
			out 	DDRA, rwl	
			out 	PORTA, rwh		; enable all pull ups

; Initialize Port B
			ldi 	rwl, 1<<pinCharge	; set pin charge as outout
			ldi 	rwh, 0x00		; reset all outputs
			out 	DDRB, rwl
			out 	PORTB, rwh		; enable all pull ups

; Initialize Port C
			ldi		rwl, 0			; all pins as inputs
			;ldi 	rwh, (1<<bUp)|(1<<bDown)|(1<<bLeft)|(1<<bRight) 		; enable internal pull ups for Keys
			ldi 	rwh, 0xFF		; enable all internal pull ups
			out 	DDRC,rwl
			out 	PORTC, rwh

; Initialize Port D
			ldi 	rwl, 0xFF		; all pins as output
			clr		rwh
			ldi 	rwh, (1<<pinLedR)|(1<<pinLedG)|(1<<pinLedY)	; reset LED outputs (active L)
			out 	DDRD, rwl
			out 	PORTD, rwh

; Initialize UART com - 9600,8N1
			ldi 	rwl, HIGH(UBRR_VAL)	; set Baudrate 9600
			out 	UBRRH, rwl
			ldi 	rwl, LOW(UBRR_VAL)
			out 	UBRRL, rwl
	
			ldi 	rwl, (1<<RXEN)|(1<<TXEN)	; Enable TX and RX
			out 	UCSRB, rwl

			ldi 	rwl, (1<<URSEL)|(0<<USBS)|(3<<UCSZ0)	; URSEL=1 to write to UCSRC register, USBS=0: 1 Stoppbit, UCSZ0=3: 8 Databit
			out 	UCSRC, rwl								; otherwise it would write the baud reg

; Initialize Button Flags and Debouncing Stuff
			ldi 	rwl, 0x00		; only registers can write directly to SRAM
			ldi 	rwh, 0xFF			
			sts 	sKeyState, rwl	; SRAM Byte 0: reset button status
			sts 	sKeyState+1, rwh; SRAM Byte 1 debounce counter: 0xFF
			sts 	sKeyState+2, rwl; SRAM Byte 2 debounce counter: 0x00
			clr 	rKeyPressed		; reset button flags

; Initialize Flag Registers
			clr		rFlag
			clr		rFlag2
			sbr		rFlag, (1<<b_UPDATE_SCREEN)		; update screen for first time

; Initialize Sleep mode
			ldi 	rwl, 1<<SE 	; Enable sleep
			out 	MCUCR, rwl

; Start with defined menu screen
			ldi		rMp, STARTMENU	

; Set Timestamps to -1
			mov		rStampL, rFF
			mov		rStampH, rFF

; Clear Time and other SRAM stuff
			sts		sTimeS, rNULL
			sts		sTimeM, rNULL
			sts		sTimeH, rNULL
			sts		sAdcRes, rNULL
			sts		sAdcI2cAIN0, rNULL
			sts		sAdcI2cAIN1, rNULL
			sts		sAdcI2cAIN2, rNULL
			sts		sAdcI2cAIN3, rNULL

; Initialize Timer/Counter 1
			ldi 	rwl, 1<<CS10	; Prescaler = 1
			ldi 	rwh, 0x00		; Timer 1 normal counting mode, free running
			out 	TCCR1B, rwl		; switch on Timer 1
			out 	TCCR1A, rwh		

; Initialize Timer/Counter 0			
			ldi 	rwl, (1<<OCIE0)|(1<<OCIE1A) ; enable Timer0-Interrupt and Timer1-Interrupt
			out 	TIMSK,rwl

; Initialize Analog Comparator
			ldi		rwl, (1<<ACIE)|(3<<ACIS0)	; enable Comparator-Interrupt, Mode 3: interrupt on rising edge
			out		ACSR, rwl

; XXX only for test Prepare timeout for LED blinking in main loop
			PREPARE_TO sJobTimer		; save current timestamp for timeout to sram

; Enable Interrupts
			IRQ_ENABLE

			rjmp 	mainLoop
;
; ============================================
;    Main Program Loop
; ============================================
;
MenuHandle:
keyUp:		JMP_IFCLR_R rKeyPressed, bUP, keyDown	; if not key "up" pressed: test next key
			sbr		rFlag, (1<<b_UPDATE_SCREEN)		; update screen
			inc		rMp								; else ("up" pressed) rMp++
			cpi		rMp, NOOFMENUS					;	if not last menu
			brne	MenuJump						;		display menu
			ldi		rMp, 0							;	else show first menu (wrap arround)

keyDown:	JMP_IFCLR_R rKeyPressed, bDown, MenuJump; if not key "up" pressed: go on with current menu handling
			sbr		rFlag, (1<<b_UPDATE_SCREEN)		; update screen
			dec		rMp								; else ("up" pressed) rMp--	
			cpi		rMp, 0xFF						;  	if not first menu
			brne	MenuJump						;  		display menu
			ldi		rMp, NOOFMENUS-1				;  	else show last menu (wrap arround)
			;brne	MenuJump						;  		display menu			
			
MenuJump:	; jump to correct menu page
			ldi		zl, LOW(menutab)				; get Base Pointer to menu jump table
			ldi		zh, HIGH(menutab)
			add 	zl, rMp							; adjust pointer to chosen menu
			adc		zh, rNULL
			ijmp									; call menu (jumps to correct line in menutab and from there to correct menu label mp_00...mp_06)

menutab:	; Menu jump table
 			rjmp 	mp_00
 			rjmp 	mp_01
 			rjmp 	mp_02
 			rjmp 	mp_03
			rjmp 	mp_04
 			rjmp 	mp_05
 			rjmp 	mp_06

mp_00:		; menu screen 00 - Toggle red LED with right button, toggle green LED with left button
			JMP_IFCLR_R rKeyPressed, bRight, mp_00_left	; if not key "right" pressed: test if left key pressed
			TOGGLE_PIN	PORTD, pinLedR					; else ("right" pressed) toggle red LED		
			rjmp	clearKeys							;	and finished
mp_00_left:	JMP_IFCLR_R rKeyPressed, bLeft, clearKeys	; if not key "left" pressed: nothing to do - finished
			TOGGLE_PIN	PORTD, pinLedG					; else ("left" pressed) toggle green LED	
			rjmp	clearKeys							;	and finished

mp_01:		; menu screen 01 - reset program with right button, send I2C Byte with left button
			JMP_IFCLR_R rKeyPressed, bRight, mp_01_left	; if not key "right" pressed: test if left key pressed
			rjmp 	main								; else ("right" pressed) reset program
mp_01_left:	JMP_IFCLR_R rKeyPressed, bLeft, clearKeys	; if not key "left" pressed: nothing to do - finished
			PRINTF_I2C 0xA0, StrI2Ctest					; else ("left" pressed) send Flash String over I2C
			rjmp	clearKeys							;	and finished

mp_02:		; menu screen 02 - right: PCF8591 read enable/disable by user
			JMP_IFCLR_R rKeyPressed, bRight, mp_02_left	; if not key "right" pressed: test if left key pressed
			;PRINTF_I2C 0xA0, StrI2CtestL				; else ("right" pressed) send Flash String over I2C
			TOGGLE_PIN	PORTD, pinLedR					; else ("right" pressed) toggle red LED to show read enabled/disabled
			;TOGGLE_REG_BIT, rFlag2, b_PCF8591_RD_EN		;	toggle enable bit in register
			push	rwl
			ldi 	rwl, (1<<b_PCF8591_RD_EN)			; set Bit mask
			eor 	rFlag2, rwl							; write Register
			pop		rwl
					
mp_02_left:	JMP_IFCLR_R rKeyPressed, bLeft, clearKeys	; if not key "left" pressed: nothing to do - finished
			;PRINTF_I2C 0xA0, StrI2CtestN				; else ("left" pressed) send Flash String over I2C
			TOGGLE_PIN	PORTD, pinLedG					; else ("right" pressed) toggle green LED to show write enabled/disabled
			;TOGGLE_REG_BIT, rFlag2, b_PCF8591_WR_EN		;	toggle enable bit in register
			push	rwl
			ldi 	rwl, (1<<b_PCF8591_WR_EN)			; set Bit mask
			eor 	rFlag2, rwl							; write Register
			pop		rwl
			rjmp	clearKeys							;	and finished

mp_03:		; menu point 03 - I2C LCD init
			JMP_IFCLR_R rKeyPressed, bRight, mp_03_left	; if not key "right" pressed: test if left key pressed
			rcall	I2C_LCD_WRITE					; else ("right" pressed) send  over I2C
			rjmp	clearKeys							;	and finished

mp_03_left:	JMP_IFCLR_R rKeyPressed, bLeft, clearKeys	; if not key "left" pressed: nothing to do - finished
			IRQ_DISABLE
			push	rwl
			push	rwh
			rcall	I2C_init			

			ldi		rwh, 0b01110100+b_I2Cwr		; Set device address and write
			rcall	I2C_start				; Send start condition and address
			
			ldi		rwl, 0x40				; Controlbyte Co=0 RS=1 Data
			rcall	I2C_do_transfer			; Execute transfer
			ldi		rwl, 'a'				; send "a"
			rcall	I2C_do_transfer			; Execute transfer

			rcall	I2C_stop			; Send stop condition
			pop		rwh
			pop		rwl
			IRQ_ENABLE
			rjmp	clearKeys

mp_04:		; menu point 04 - print ADC result from PCF8591 channel AIN3
			PRINTF	StrL2C10								; set cursor to Line=2, Column=10
			JMP_IFCLR_R rFlag2, b_PCF8591_RD_EN, mp_04_off	; if read disabled print OFF string
			lds		rwh, sAdcI2cAIN3
			PRINT16 rNULL, rwh, 2, sig
			rjmp	mp_04_end 
mp_04_off:	PRINTF	StrOff
mp_04_end:	rjmp	clearKeys

mp_05:		; menu point 05 - ;print ADC result from PCF8591 channel AIN2
			PRINTF	StrL2C10					; set cursor to Line=2, Column=10
			JMP_IFCLR_R rFlag2, b_PCF8591_RD_EN, mp_04_off	; if read disabled print OFF string
			lds		rwh, sAdcI2cAIN2
			PRINT16 rNULL, rwh, 2, sig
			rjmp	mp_05_end 
mp_05_off:	PRINTF	StrOff
mp_05_end:	rjmp	clearKeys 

mp_06:		; menu point 06 - print ADC result from internal comparator
			PRINTF	StrL2C10					; set cursor to Line=2, Column=10
			lds		rwl, sAdcRes
			PRINT16 rNULL, rwl, 2, sig 
			rjmp	clearKeys


clearKeys:	cbr 	rKeyPressed, KeyMask		; empty Key buffer	
			sbrc	rFlag, b_UPDATE_SCREEN		; skip next line if bit not set
			rcall	UPDATE_MENUFRAME					; if Flag set - update menu
			cbr		rFlag, (1<<b_UPDATE_SCREEN)	; stop updating screen 
			ret									;	else return

;******************************************************************************
						
mainLoop:
			JMP_IFCLR_R rFlag, b_MAIN_TICK, mainLoop_sleep		; only execute main loop every tick (2,5ms)
																; only necesary if sleep mode not activated
			cbr		rFlag, (1<<b_MAIN_TICK)						; clear main tick

			sbi 	PORTD, pinXXXDEBUG3	; only for debugging to measure time for whole main loop

			tst		rKeyPressed		; if no button pressed
			breq	main_do_jobs	; 	go on with other stuff
			rcall	MenuHandle		; else (button pressed) do the button stuff	

main_do_jobs:
			;CALL_IFSET_R rFlag, b_I2C_WAIT_STATE, I2C_do_transfer; handle error, if I2C is waiting for acknowledge

			CALL_IFTO sJobTimer, 50, TIMED_JOBS					; if timeout elapsed: do low frequent jobs

			CALL_IFSET_R rFlag, b_JOB_UART_TX, UART_TX_handle	; if scheduled: do UART transmit

			lds		rtemp3, sAdcRes								; set parameter for PCF8591_WRITE
			CALL_IFSET_R rFlag, b_JOB_PCF8591_WR, PCF8591_WRITE	; if scheduled: do PCF8591 write
			
			CALL_IFSET_R rFlag, b_JOB_PCF8591_RD, PCF8591_READ	; if scheduled: do PCF8591 read

			CALL_IFSET UCSRA, RXC, UART_RX_handle 				; if uart received byte: do UART receive 
			
			; Analog Comparator has to be done as last job
			; otherwise the quick interrupt would disturb other tasks
			CALL_IFSET_R rFlag, b_JOB_ADC, AD_convert			; do AD comparator if scheduled

mainLoop_sleep:
			cbi 	PORTD, pinXXXDEBUG3	; only for debugging to measure time for whole main loop
			
			sleep						; go to sleep till next interrupt
			rjmp 	mainLoop

;***************************************************************************
;
; ============================================
;     Subroutines
; ============================================
;
;***************************************************************************

I2C_LCD_WRITE:
			IRQ_DISABLE
			push	rwl
			push	rwh

			rcall	I2C_init			
			
			ldi		rwh, 0x74+b_I2Cwr		; Set device address and read
			rcall	I2C_start				; Send start condition and address
			rcall	I2C_hp_delay
			rcall	I2C_hp_delay

			ldi		rwl, 0x00				; Control Byte (control mode on)
			rcall	I2C_do_transfer			; Execute transfer
			rcall	I2C_hp_delay
			rcall	I2C_hp_delay

			ldi		rwl, 0b00101110			; 0x2E - Function set: 		0011 1110 DL=0 N=1 M=1 G=1 
			rcall	I2C_do_transfer			; Execute transfer
			rcall	I2C_hp_delay
			rcall	I2C_hp_delay

			ldi		rwl, 0b00001111			; 0x0F - Display Control:	0000 1111 Disp=0 Cur=1 Blink=1
			rcall	I2C_do_transfer			; Execute transfer
			rcall	I2C_hp_delay
			rcall	I2C_hp_delay

			ldi		rwl, 0x06				; 0x06 - Entry Mode Set:	0000 0110 I/D=1 S=0
			rcall	I2C_do_transfer			; Execute transfer
			rcall	I2C_hp_delay
			rcall	I2C_hp_delay

			ldi		rwl, 0b00001111			; 0x0F - Display Control:	0000 1111 Disp=1
			rcall	I2C_do_transfer			; Execute transfer

			rcall	I2C_stop

			pop		rwh
			pop		rwl
			IRQ_ENABLE
			ret

;***************************************************************************
;*
;* FUNCTION
;*	TIMED_JOBS
;*
;* DESCRIPTION
;*	this subroutine is called from main whenever the preset timeout occures
;*	(every 500ms).
;*	Put all jobs here which have to be done on a low frequent base,
;*
;* USAGE
;*	no Parameters
;*
;* RETURN
;*	None
;*
;***************************************************************************
TIMED_JOBS:		
			PREPARE_TO sJobTimer		; set timeout for next cycle
			TOGGLE_PIN	PORTD, pinLedY	; toggle yellow led
			rcall	MenuHandle			; do the menu thing (in case it needs to be updated but no button was pressed)
			ret


;***************************************************************************
;*
;* FUNCTION
;*	PCF8591_READ
;*
;* DESCRIPTION
;*	reads the 4 analog inputs and stores the results to SRAM
;*
;* USAGE
;*	no Parameters
;*
;* RETURN
;*	corresponding values in SRAM
;*
;***************************************************************************
PCF8591_READ:
			IRQ_DISABLE
			push	rwl
			push	rwh

			cbr		rFlag, (1<<b_JOB_PCF8591_RD); release scheduled job
			JMP_IFCLR_R rFlag2, b_PCF8591_RD_EN, PCF8591_READ_END	; user enabled? if Flag not set jump to end 

			; all channels are shifted by 1
			; that is why select channel AIN0 is sent
			; and channel AIN3 is received

			; read channels AINx---------------------------------------------
			; prepare adc for read
			rcall	I2C_init			
			
			; get value from AIN3
			ldi		rwh, 0b10010000+b_I2Crd	; Set device address and read
			rcall	I2C_start				; Send start condition and address

			;clc								; clear carry flag to force uC to send aknowledge
			;rcall	I2C_do_transfer			; read value - rwl contains received data
			;sts		sAdcI2cAIN0, rwl		; store received value in SRAM

			sec								; set carry flag to force uC to send "not aknowledge" because last byte
			rcall	I2C_do_transfer			; read value - rwl contains received data
			sts		sAdcI2cAIN3, rwl		; store received value in SRAM
	
			;------------DEBUG--------------------
			;CALL_IFTO sI2cTimer, 200, PRINT_RWL	; if timeout elapsed: print for debug
			;-------------------------------------


			rcall	I2C_stop
			;---------------------------------------------------------------

PCF8591_READ_END:
			pop		rwh
			pop		rwl
			IRQ_ENABLE
			ret

;-----------------for debugging
;PRINT_RWL:
;			PREPARE_TO sI2cTimer	; set timeout for next call
;			push	xl
;			lds		xl, sI2cResult
;			rcall	lcd_printb
;			pop		xl
;			ret

;***************************************************************************
;*
;* FUNCTION
;*	PCF8591_WRITE
;*
;* DESCRIPTION
;*	writes the analog output
;*
;* USAGE
;*	rtemp3 - contains the 8-bit value to which the analoug output should be set
;*
;* RETURN
;*	corresponding values in SRAM
;*
;***************************************************************************
PCF8591_WRITE:
			IRQ_DISABLE
			push	rwl
			push	rwh

			cbr		rFlag, (1<<b_JOB_PCF8591_WR); release scheduled job
			JMP_IFCLR_R rFlag2, b_PCF8591_WR_EN, PCF8591_WRITE_END	; user enabled? if Flag not set jump to end 

			rcall	I2C_init			

			ldi		rwh, 0b10010000+b_I2Cwr	; Set device address and write
			rcall	I2C_start				; Send start condition and address
			
			ldi		rwl, 0b01000011			; Mode: set voltage on AOUT pin
			rcall	I2C_do_transfer			; Execute transfer

			;lds		rwl, sAdcRes
			mov		rwl, rtemp3				; copy parameter to I2C send register
			rcall	I2C_do_transfer			; Execute transfer

			rcall	I2C_stop				; Send stop condition

PCF8591_WRITE_END:
			pop		rwh
			pop		rwl
			IRQ_ENABLE
			ret

;***************************************************************************
;*
;* FUNCTION
;*	UPDATE_MENUFRAME
;*
;* DESCRIPTION
;*	updates the display
;*
;* USAGE
;*	no Parameters
;*
;* RETURN
;*	None
;*
;***************************************************************************
UPDATE_MENUFRAME:
			PRINTT	StrMenuFrame, rMp	; display current menu screen
									; the macro takes the label to the string table 
									; and the menu pointer
			ret


;***************************************************************************
;*
;* FUNCTION
;*	UART_TX_handle
;*
;* DESCRIPTION
;*	checks if there is data in the ringbuffer to send and
;*	sends one byte over uart
;*
;* USAGE
;*	no Parameters
;*
;* RETURN
;*	None
;*
;***************************************************************************
UART_TX_handle:		; UART Send one Byte from ringbuffer
			cbr		rFlag, (1<<b_JOB_UART_TX)	; 	release scheduled job			
			push	xl
			push	xh
			push	rwl

			lds		xl, sRdptr			; load read pointer
			lds		xh, sWrptr			; load write pointer only for comparison
			cp		xl, xh				; if buffer empty (sRdptr==sWrptr)
			breq	UART_TX_End			;	nothing to do - end routine
			ldi		xh, 1				; else set SRAM pointer offset to page 1 (0x0100) (rwl contains the low counting byte for the address)
			ld		rwl, x+				; 	get next Byte from buffer
			sts		sRdptr, xl			;	save unmodified readpointer in SRAM
			out		UDR, rwl			; 	put Byte to send in UART transmit register
UART_TX_End:
			pop		rwl
			pop		xh
			pop		xl
			ret


;***************************************************************************
;*
;* FUNCTION
;*	UART_RX_handle
;*
;* DESCRIPTION
;*	checks if byte over uart received and handles received byte
;*
;* USAGE
;*	no Parameters
;*
;* RETURN
;*	None
;*
;***************************************************************************
UART_RX_handle:
			push	rwl	

			;JMP_IFCLR UCSRA, RXC, uartFinished	; if no char received: go to end	
			in		rwl, UDR		; else get char
			cpi		rwl, '+'		;	if "+" received
			breq	uartRXu			;		handle "up"
			cpi		rwl, '-'		;	if	"-" received
			breq	uartRXd			;		handle "down"
			cpi		rwl, ','		;	if	"," received
			breq	uartRXl			;		handle "left"
			cpi		rwl, '.'		;	if	"." received
			breq	uartRXr			;		handle "right"
			cpi		rwl, 'r'		;	if	"r" received
			breq	uartRXrd		;		handle "read"  PCF8591
			cpi		rwl, 'w'		;	if	"w" received
			breq	uartRXwr		;		handle "write" PCF8591
			rjmp	uartFinished	;	else ignore char

uartRXu:	ldi		rwl, 1<<bUp		; simulate key up pressed
			or		rKeyPressed, rwl
			rjmp	uartFinished	

uartRXd:	ldi		rwl, 1<<bDown	; simulate key down pressed
			or		rKeyPressed, rwl
			rjmp	uartFinished	

uartRXl:	ldi		rwl, 1<<bLeft	; simulate key down pressed
			or		rKeyPressed, rwl
			rjmp	uartFinished	
		
uartRXr:	ldi		rwl, 1<<bRight	; simulate key down pressed
			or		rKeyPressed, rwl
			rjmp	uartFinished	

uartRXrd:	; enable PCF8591 I2C read
			TOGGLE_PIN	PORTD, pinLedR	; toggle red LED to show PCF8591 read enabled/disabled
			ldi 	rwl, (1<<b_PCF8591_RD_EN)	; set Bit mask
			eor 	rFlag2, rwl					; write Register
			rjmp	uartFinished	

uartRXwr:	; enable PCF8591 I2C write
			TOGGLE_PIN	PORTD, pinLedG	; toggle green LED to show PCF8591 write enabled/disabled
			ldi 	rwl, (1<<b_PCF8591_WR_EN)	; set Bit mask
			eor 	rFlag2, rwl					; write Register
			rjmp	uartFinished	
						
uartFinished:
			pop		rwl
			ret


;**************************************************************************  
;*  
;* ISR_ANACmp - Analog comparator interrupt routine  
;*  
;*  
;* DESCRIPTION  
;* This routine is executed when one of two events occur:  
;* 1. Timer/counter0 overflow interrupt  
;* 2. Analog Comparator interrupt  
;* Both events signals the end of a conversion. Timer overflow if the signal  
;* is out of range, and analog comparator if it is in range.  
;* The offset will be corrected, and the t'flag will be set.  
;* Due to the cycles needed for interruption handling, it is necessary  
;* to subtract 1 more than was added previously.  
;*   
;*  
;* Total numbers of words		: 7+1+1  =9  
;* Total number of cycles		: 10+2+2 =11  
;* Low register usage			: 0  
;* High register usage			: 2 (result,rwl)  
;* Status flags					: 1 (t flag)  
;*  
;**************************************************************************  
ISR_ANACmp:
			;IRQ_DISABLE		; disable all interrupst XXX TODO das funzt no net so richtig bei 0V
				
			in		rwl, TCNT0				; Load timer value  
			out		TCCR0, rNULL			; Stop timer0            

			subi	rwl,T0PRESET+1		 	; Rescale A/D output  
  
  			sts		sAdcRes, rwl			; store result to SRAM
			cbi		pAC, pinCharge			; Start discharge  
			;set							; Set conversion complete flag  
		  
		  	;IRQ_ENABLE						; enable interrupts again
			reti                    		; Return from interrupt  


;**************************************************************************  
;*  
;* AD_convert - Subroutine to start an A/D conversion  
;*  
;* DESCRIPTION  
;* This routine starts the conversion. It loads the offset value into the  
;* timer0 and starts the timer. It also starts the charging of the   
;* capacitor.  
;*  
;*  
;* Total number of words		: 7  
;* Total number of cycles		: 10  
;* Low register usage			: 0  
;* High register usage			: 1 (rwl)  
;* Status flag usage			: 0
;*  
;**************************************************************************  
AD_convert: 
			push	rwl

			cbr		rFlag, (1<<b_JOB_ADC)	; Reset Flag for AD Job
			out     TCNT0, rNULL			; and load offset value  
  
			ldi		rwl, $02				;Start timer0 with prescaling f/8  
			out     TCCR0, rwl      
			sbi     pAC, pinCharge			;Start charging of capacitor
			  
			pop		rwl
			ret								;Return from subroutine  
  

;***************************************************************************
;*
;* FUNCTION
;*	ISR_TC1
;*
;* DESCRIPTION
;*	Interrupt is executed every 2,5ms (if F_ISR=400Hz)
;*	- set deadline for next interrupt
;*  - handle the 16Bit timeout counter
;*  - handle the 16Bit ms secounds counter
;*  - do the button debouncing stuff
;*	- set flags for jobs which should be executed in main loop
;*  - no subroutines are called here
;*
;* USAGE
;*	no Parameters
;*
;* RETURN
;*	None
;*
;***************************************************************************
ISR_TC1:	; inactive: Timer 1 Interrupt every 1,25ms (with interval set to 800Hz)
			; active: Timer 1 Interrupt every 2,5ms (with interval set to 400Hz)

			sbi 	PORTD, pinXXXDEBUG1	; 	only for debugging to measure time
						
			push	rwl					; store registers on stack
			push	rwh
			push	xl
			push	xh

ISR_TC1_fast_jobs:	; inactive: jobs to do every 1,25ms - set by F_ISR (800Hz)
					; active: jobs to do every 2,5ms - set by F_ISR (400Hz)
			
			; set deadline for next Interrupt (interrupts every "interval")
			in		rwl, OCR1AL			; get current values
			in		rwh, OCR1AH	
			subi	rwl, LOW(-INTERVAL)	; add Intervall till next interrupt (subtract constant from register)
			sbci	rwh, HIGH(-INTERVAL); - AVR does no ADD with constants	(subtract constant from register with carry)
			out 	OCR1AH, rwh			; and set as deadline for next interrupt
			out 	OCR1AL, rwl

ISR_TC1_fast_job_1:
			; count ms
			; counter resets after 1000ms and sets the second flag
			; one tick is 2,5ms - counter must count to 1000/2,5 = 400
			ldi		rwl, LOW(F_ISR)		; 	compare with 400 (1000ms elapsed)
			ldi		rwh, HIGH(F_ISR)

			inc		rStampL				; increment low byte
			brne	ISR_TC1_fast_job_1_1; if no overrun: check if high byte has already counted up
			inc		rStampH				; else also increment high byte

ISR_TC1_fast_job_1_1:
			cp		rStampH, rwh
			brne	ISR_TC1_fast_job_2	;	if not 1000ms elapsed: go on with next job		
			cp		rStampL, rwl
			brne	ISR_TC1_fast_job_2	;	if not 1000ms elapsed: go on with next job		
			clr		rStampH				;	else (1000ms elapsed): clear ms counter
			clr		rStampL				;
			sbr		rFlag, (1<<b_SEC)	; 		set flag for seconds (will be handled in main loop)	

ISR_TC1_fast_job_2:
			; schedule high frequent jobs do be executed in main loop
			sbr		rFlag, (1<<b_MAIN_TICK)		; 	used to start main loop
			sbr		rFlag, (1<<b_JOB_UART_TX)	; 	Schedule UART TX Job to be executed in mainloop			

ISR_TC1_10ms_prepare:
			; handle prescaler for 10ms jobs
			ldi		rwl, VT10MS
			dec		rPrescaler			; decrement prescaler (starts with 0x00-1=0xFF=255)
			brne	ISR_TC1_End			; if prescaler!=0 finished
			mov		rPrescaler, rwl		; else (10ms elapsed): reset prescaler to 10
			rjmp	ISR_TC1_10ms_jobs	;	and do 10ms jobs


ISR_TC1_10ms_jobs:	; jobs to do every 10ms - set by VT10MS			
			sbi 	PORTD, pinXXXDEBUG2	; 	only for debugging to measure time
			
			; schedule jobs do be executed in main loop
			sbr		rFlag, (1<<b_JOB_ADC)		; Schedule ADC Job
			sbr		rFlag, (1<<b_JOB_PCF8591_RD); Schedule Job PCF8591 ADC read
			sbr		rFlag, (1<<b_JOB_PCF8591_WR); Schedule Job PCF8591 ADC write

			;------------------------------------------------------------------------------
			; Debouncing Algorithm from Peter Dannegger (mikrocontroller.net)
			; can handle up to 8 buttons, at the moment only 4 buttons are used
			lds		rwh, sKeyState		; load last button Status from SRAM
			lds		xl, sKeyState+1		; load Debounce Counter Byte 1 from SRAM
			lds		xh, sKeyState+2		; load Debounce Counter Byte 2 from SRAM
			
			in		rwl, iKey			; read buttons from port (active LOW)
			com		rwl					; invert (active HIGH)
			andi	rwl, KeyMask		; just use the bits for connected buttons
			eor		rwl, rwh			; only changes get HIGH (only edges)
			and 	xl, rwl      		; reset debouncing counter of unchanged Keys (Byte 1)
 			and 	xh, rwl      		; reset debouncing counter of unchanged Keys (Byte 2)
 			com 	xl         			; L-Bit counting 0,2,->1, 1,3,->0 (com: one's complement)
			eor 	xh, xl      		; H-Bit counting 0,2,->tz1 toggeln
 			and 	rwl, xl      		; only keep changes if both bits
 			and 	rwl, xh      		; are set in debouncig counter (counter = 3 = 0b11)
 			eor 	rwh, rwl    		; erhaltene Änderungen toggeln alten (valid) KeyMaskstatus
 			and 	rwl, rwh    		; only (new) pressed buttons are kept
 			or 		rKeyPressed, rwl  	; and set corresponding button bits in register (has to be cleared after button handling)
 			
			sts 	sKeyState, rwh  	; save new key state to SRAM,
 			sts 	sKeyState+1, xl 	; save Debounce Counter Byte 1 to SRAM
 			sts 	sKeyState+2, xh 	; save Debounce Counter Byte 2 to SRAM
			;------------------------------------------------------------------------------

ISR_TC1_End:			
			pop		xh					; Restore registers
			pop		xl
			pop		rwh
			pop		rwl
			
			cbi 	PORTD, pinXXXDEBUG2	; 	only for debugging to measure time for 10ms part
			cbi 	PORTD, pinXXXDEBUG1	; 	only for debugging to measure time for whole interrupt part

			reti


;***************************************************************************
;
; ============================================
;     Strings in Flash
; ============================================
;
;***************************************************************************

;___________________________________________________________________________
; ============================================
;     Data for VT100 Terminal
; ============================================
;
; VT100 escape sequences:
; 	cesc,"[2J" 		--clear terminal screen
;	cesc,"[H"		--set cursor to upper left (1:1)
;	cesc,"[2;1H"	--set cursor to line 2, column 1
;___________________________________________________________________________

StrInit:	.DB "Hallo Tina !    "
			.DB ccr, clf
			.DB "und Hallo Welt  "
			.DB ccr, clf, cnull, cnull


StrMenuFrame:.DW StrMp00, StrMp01, StrMp02, StrMp03, StrMp04, StrMp05, StrMp06		;jump table for menu headline

StrMp00:	.DB cesc,"[2J",cesc,"[H",	"0: Toggle LEDs   "
			.DB cesc,"[2;1H",			"[L] grn [R] red  ", cnull

StrMp01:	.DB cesc,"[2J",cesc,"[H",	"1: Reset and I2C "
			.DB cesc,"[2;1H",			"[L] I2C [R] rst  ", cnull

StrMp02:	.DB cesc,"[2J",cesc,"[H",	"2: PCF8591 ON/OFF"
			.DB cesc,"[2;1H",			"[L]Wr   [R]Rd    ", cnull

StrMp03:	.DB cesc,"[2J",cesc,"[H",	"3: I2C LCD test  "
			.DB cesc,"[2;1H",			"[L]char [R]init  ", cnull

StrMp04:	.DB cesc,"[2J",cesc,"[H",	"4: I2C AD AIN3   "
			.DB cesc,"[2;1H", "Voltage: ", cnull

StrMp05:	.DB cesc,"[2J",cesc,"[H",	"5: I2C AD AIN2   "
			.DB cesc,"[2;1H", "Voltage: ", cnull

StrMp06:	.DB cesc,"[2J",cesc,"[H",	"6: internal ADC  "
			.DB cesc,"[2;1H", "Voltage: ", cnull

; set cursor to Line 2 Column 10
StrL2C10:	.DB cesc,"[2;10H", cnull

StrOff:		.DB	"OFF    ", cnull

StrI2Ctest:	.DB "I2C_Test", cnull, cnull
StrI2CtestN:.DB "xx1234567890123456", cnull, cnull
StrI2CtestL:.DB "I2C_Test This is a very long teststring to check behavior for ms timing", cnull

StrLCDinit:	.DB 0x00, cnull
StrLCDinit2:.DB 0x3E, 0x08, 0x06, 0x0C, cnull, cnull
			
;
; End of Codesegment


IRQ_DISABLE
			push	rwl
			push	rwh
			rcall	I2C_init			

			ldi		rwh, 0x75+b_I2Cwr		; Set device address and write
			;ldi		rwh, 0b01110100+b_I2Cwr	; Set device address and write
			rcall	I2C_start				; Send start condition and address
			
			ldi		rwl, 0x00				; Control Byte (control mode on)
			rcall	I2C_do_transfer			; Execute transfer
			ldi		rwl, 0b00101110			; Function set: 	0011 1110 DL=0 N=1 M=1 G=1 
			rcall	I2C_do_transfer			; Execute transfer
			ldi		rwl, 0b00001111			; Display Control:	0000 1111 Disp=0 Cur=1 Blink=1
			rcall	I2C_do_transfer			; Execute transfer
			ldi		rwl, 0x06				; Entry Mode Set:	0000 0110 I/D=1 S=0
			rcall	I2C_do_transfer			; Execute transfer
			;ldi	rwl, 0b0000 1111		; Display Control:	0000 1111 Disp=1
			;rcall	I2C_do_transfer			; Execute transfer

			rcall	I2C_stop			; Send stop condition
			pop		rwh
			pop		rwl
			IRQ_ENABLE
