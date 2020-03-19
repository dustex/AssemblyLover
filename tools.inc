;***************************************************************************
;*
;* FUNCTION
;*	Delay1ms, Delay50ms 
;*
;* DESCRIPTION
;*	Generates 16-Bit Delay with variable time. Oscillator Freq. is taken
;*	into account (default F_CPU=4MHz)
;*
;* R26 used for LSB
;* R27 used for MSB
;*
;***************************************************************************
.EQU FCK = F_CPU/1000 					; Freq. in kHz
.EQU C1MS = (100*FCK)/4000 - 6	 		; constant for 1 ms
.EQU C50MS = (50*FCK)/4 - 6 			; constant for 50 ms
;
; Delayroutine
;
Delay1ms: 	; 1 ms Routine
			push	R26					; 2
			push	R27					; 2
			ldi 	R27, HIGH(C1MS)
			ldi 	R26, LOW(C1MS)
			rjmp 	DelayLoop

Delay50ms: ; 50 ms Routine
			push	R26					; 2
			push	R27					; 2
			ldi 	R27, HIGH(C50MS)
			ldi 	R26, LOW(C50MS)
			rjmp 	DelayLoop

DelayLoop:	; expects 16Bit value in R27:R26
			sbiw 	R26, 1 				; loop
			brne 	DelayLoop 			; while != 0
			nop 						; additional delay
			nop 						; additional delay
			pop		R27					; 2
			pop		R26					; 2
			ret
