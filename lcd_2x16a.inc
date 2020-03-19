

;***************************************************************************
;*
;* DESCRIPTION
;*	Routines for handling an LC-Display
;*
;* USAGE
;*	rwl and rwh Registers must be declared in main program
;*	ringbuffer from $0100...$01FF must be reserved in SRAM
;*
;*
;*
;***************************************************************************


;***************************************************************************
;*
;* FUNCTION
;*	lcd_data
;*
;* DESCRIPTION
;*	Takes one Byte (in Register wl) and puts it in the ringbuffer
;*
;* USAGE
;*	wl contains the byte to be put in the buffer
;*
;* RETURN
;*	sWrptr is modified and stored back to SRAM
;*
;* COMMENT
;*
;*
;***************************************************************************
lcd_data:	; puts one Byte in the ringbuffer
			push	yh
 			push 	yl
 			lds 	yl, sWrptr		; load writepointer
lcd_data1:
 			lds 	yh, sRdptr		; load readpointer only for comparison
 			dec 	yh				; 1 less because for comparison
 			cp 		yl, yh			; if buffer full (sRdptr == sWrptr)
 			breq 	lcd_data1		;	wait till interrupt has flushed the buffer
 			ldi 	yh, 1			; else Block $0100...$01FF as Ringbuffer
 			st 		y+, rwl			;	write Byte (store indirect and post increment)
 			sts 	sWrptr, yl		;	save writepointer back to SRAM
 			pop 	yl
 			pop 	yh
 			ret
