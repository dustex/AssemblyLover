; I2C low level routines
;
; Used Registers:
; rwl (data)
; rwh (address)
; rtemp3  (delay)
;
;***************************************************************************
;*
;* FUNCTION
;*	I2C_tx_test
;*
;* DESCRIPTION
;*	tests I2C TX 
;*
;* USAGE
;*	no Parameters
;*
;* RETURN
;*	None
;*
;* COMMENT
;*	I2C Write data => Adr(00) = 0x55
;*
;***************************************************************************
;i2c_tx_test:
;			rcall	I2C_init
;
;			ldi		rI2Cadr, $A0+b_I2Cwr; Set device address and write
;			rcall	I2C_start			; Send start condition and address
;
;			ldi		rI2Cdata, 'a'		; Write a
;			rcall	I2C_do_transfer		; Execute transfer
;
;			ldi		rI2Cdata, 'U'		; Write U (01010101b)
;			rcall	I2C_do_transfer		; Execute transfer
;
;			rcall	I2C_stop			; Send stop condition
;			ret


; ============================================
;   Macro I2C_SEND_BYTE
; ============================================
;
; Parameter 1: Address @0
; Parameter 2: Data	@1
;
; rwl used for data
; rwh used for address
; 
; usage:  I2C_SEND_BYTE 0x05, 'U'
;
.MACRO I2C_SEND_BYTE ;address, data
		
			IRQ_DISABLE					; no interrupts durring transfer
			push	rwl
			push	rwh
			push	rtemp3
  		
			rcall	I2C_init			

			ldi		rwh, @0+b_I2Cwr		; Set device address and write
			rcall	I2C_start			; Send start condition and address

			ldi		rwl, @1				; Write byte
			rcall	I2C_do_transfer		; Execute transfer

			rcall	I2C_stop			; Send stop condition
			
			pop		rtemp3
			pop		rwh
			pop		rwl
			IRQ_ENABLE
.ENDMACRO


;***************************************************************************
;*
;* FUNCTION
;*	I2C_hp_delay
;*	I2C_qp_delay
;*
;* DESCRIPTION
;*	hp - half I2C clock period delay (normal: 5.0us / fast: 1.3us)
;*	qp - quarter I2C clock period delay (normal: 2.5us / fast: 0.6us)
;*
;*	SEE DOCUMENTATION !!!
;*
;* USAGE
;*	no parameters
;*
;* RETURN
;*	none
;*
;***************************************************************************

I2C_hp_delay:
			ldi		rtemp3, 10
I2C_hp_delay_loop:
			dec		rtemp3
			brne	I2C_hp_delay_loop
			ret

I2C_qp_delay:
			ldi		rtemp3, 2	
I2C_qp_delay_loop:
			dec		rtemp3
			brne	I2C_qp_delay_loop
			ret


;***************************************************************************
;*
;* FUNCTION
;*	I2C_rep_start
;*
;* DESCRIPTION
;*	Sends repeated start condition and slave address.
;*
;* USAGE
;*	rwh - Contains the slave address and transfer direction.
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to the address.
;*
;***************************************************************************

I2C_rep_start:
			sbi		ddrI2C, pinSCL	; force SCL low
			cbi		ddrI2C, pinSDA	; release SDA
			rcall	I2C_hp_delay	; half period delay
			cbi		ddrI2C, pinSCL	; release SCL
			rcall	I2C_qp_delay	; quarter period delay
									; go on with start condition


;***************************************************************************
;*
;* FUNCTION
;*	I2C_start
;*
;* DESCRIPTION
;*	Generates start condition and sends slave address.
;*
;* USAGE
;*	rwh - Contains the slave address and transfer direction.
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to the address.
;*
;***************************************************************************
		
I2C_start:	mov		rwl, rwh		; copy address to transmitt register
			sbi		ddrI2C, pinSDA	; force SDA low
			rcall	I2C_qp_delay	; quarter period delay
									; go on with write data


;***************************************************************************
;*
;* FUNCTION
;*	I2C_write
;*
;* DESCRIPTION
;*	Writes data (one byte) to the I2C bus. Also used for sending
;*	the address.
;*
;* USAGE
;*	rwl - Contains data to be transmitted.
;*
;* RETURN
;*	Carry flag - Set if the slave respond transfer.
;*
;***************************************************************************


I2C_write:	sec						; set carry flag
			rol		rwl				; shift in carry and out bit one
			rjmp	I2C_write_first
I2C_write_bit:
			lsl		rwl				; if transmit register empty
I2C_write_first:
			breq	I2C_get_ack		;	goto get acknowledge
			sbi		ddrI2C, pinSCL	; force SCL low

			brcc	I2C_write_low	; if bit high
			nop						;	(equalize number of cycles)
			cbi		ddrI2C, pinSDA	;	release SDA
			rjmp	I2C_write_high
I2C_write_low:						; else
			sbi		ddrI2C, pinSDA	;	force SDA low
			rjmp	I2C_write_high	;	(equalize number of cycles)
I2C_write_high:
			rcall	I2C_hp_delay	; half period delay
			cbi		ddrI2C, pinSCL	; release SCL
			rcall	I2C_hp_delay	; half period delay

			rjmp	I2C_write_bit


;***************************************************************************
;*
;* FUNCTION
;*	I2C_get_ack
;*
;* DESCRIPTION
;*	Get slave acknowledge response.
;*
;* USAGE
;*	(used only by I2C_write in this version)
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to a request.
;*
;***************************************************************************

I2C_get_ack:
			;JMP_IFSET_R rFlag, b_I2C_WAIT_STATE, I2C_get_ack_wait	; if wait state is active: jump directly to get_ack_wait
			sbi		ddrI2C, pinSCL	; else	force SCL low
			cbi		ddrI2C, pinSDA	; 	release SDA
			rcall	I2C_hp_delay	; 	half period delay
			;PREPARE_TO sI2CTimer	; 	set Timer for SCL HIGH timeout
			cbi		ddrI2C, pinSCL	; 	release SCL

I2C_get_ack_wait:
			sbis	iI2C, pinSCL	; wait SCL high
			;rjmp	I2C_get_ack_timeout	; in case SCL is not HIGH, wait for HIGH with timeout ;(In case wait states are inserted)
			rjmp	I2C_get_ack_wait	; in case SCL is not HIGH, (wait states are inserted)
									
			clc						; clear carry flag
			sbic	iI2C, pinSDA	; if SDA is high
			sec						;	set carry flag
			rcall	I2C_hp_delay	; half period delay
I2C_get_ack_end:
			ret

;I2C_get_ack_timeout:
;			sbr		rFlag, (1<<b_I2C_WAIT_STATE)	; activate wait state
;			JMP_IFTO sI2CTimer, 398, I2C_get_ack_timeout_elapsed, I2C_get_ack_end ;2Byte-SRAM-Reg, timeout, tag-timeout-elapsed, tag-no-timeout 

;I2C_get_ack_timeout_elapsed:
;			; code which should be executed if timeout is elapsed(error counter...)
;			cbr		rFlag, (1<<b_I2C_WAIT_STATE)	; clear wait state
;			TOGGLE_PIN pLED, pinLEDr		
;			rjmp	I2C_get_ack_end					; and end

			


;***************************************************************************
;*
;* FUNCTION
;*	I2C_do_transfer
;*
;* DESCRIPTION
;*	Executes a transfer on bus. This is only a combination of I2C_read
;*	and I2C_write for convenience.
;*	This funtion must be directly followed by I2C_read.
;*
;* USAGE
;*	rwh - Must have the same direction as when I2C_start was called.
;*	see I2C_read and I2C_write for more information.
;*
;* RETURN
;*	depends on type of transfer, read or write
;*  in case of read: rwl - Contains received data.
;*
;***************************************************************************

I2C_do_transfer:
			;JMP_IFSET_R rFlag, b_I2C_WAIT_STATE, I2C_get_ack	; if wait state is active: jump directly to get_ack and do no transfer
			sbrs	rwh, b_I2Cdir	; if data direction = write
			rjmp	I2C_write		;	goto write data
									; else continue with read data


;***************************************************************************
;*
;* FUNCTION
;*	I2C_read
;*
;* DESCRIPTION
;*	Reads data (one byte) from the I2C bus.
;*
;* USAGE
;*	Carry flag - 	If set no acknowledge is given to the slave
;*			indicating last read operation before a STOP.
;*			If cleared acknowledge is given to the slave
;*			indicating more data.
;*
;* RETURN
;*	rwl - Contains received data.
;*
;***************************************************************************

I2C_read:	rol		rI2Cstat		; store acknowledge (if carry flag set register looks like this: 100000000
									; (used by I2C_put_ack)
			ldi		rwl, 0x01		; data = 0x01
I2C_read_bit:						; do
			sbi		ddrI2C, pinSCL	; 	force SCL low
			rcall	I2C_hp_delay	;	half period delay

			cbi		ddrI2C, pinSCL	;	release SCL
			rcall	I2C_hp_delay	;	half period delay

			clc						;	clear carry flag
			sbic	iI2C, pinSDA	;	if SDA is high
			sec						;		set carry flag

			rol		rwl			; 	store data bit
			brcc	I2C_read_bit	; while receive register not full
									; go on with acknowledge

			

;***************************************************************************
;*
;* FUNCTION
;*	I2C_put_ack
;*
;* DESCRIPTION
;*	Put acknowledge.
;*
;* USAGE
;*	(used only by I2C_read in this version)
;*
;* RETURN
;*	none
;*
;***************************************************************************

I2C_put_ack:
			sbi		ddrI2C, pinSCL	; force SCL low
			ror		rI2Cstat		; get status bit
			brcc	I2C_put_ack_low	; if bit low goto assert low
			cbi		ddrI2C, pinSDA	;	release SDA
			rjmp	I2C_put_ack_high;	
I2C_put_ack_low:					; else
			sbi		ddrI2C, pinSDA	; 	force SDA low
I2C_put_ack_high:
			rcall	I2C_hp_delay	; half period delay
			cbi		ddrI2C, pinSCL	; release SCL
I2C_put_ack_wait:
			sbis	iI2C, pinSCL	; wait SCL high
									; wait states inserted here XXX chris
			rjmp	I2C_put_ack_wait
			rcall	I2C_hp_delay	; half period delay
			cbi		ddrI2C, pinSDA	; release SDA 
			ret


;***************************************************************************
;*
;* FUNCTION
;*	I2C_stop
;*
;* DESCRIPTION
;*	Assert stop condition.
;*
;* USAGE
;*	No parameters.
;*
;* RETURN
;*	None.
;*
;***************************************************************************

I2C_stop:	sbi		ddrI2C, pinSCL	; force SCL low
			sbi		ddrI2C, pinSDA	; force SDA low
			rcall	I2C_hp_delay	; half period delay
			cbi		ddrI2C, pinSCL	; release SCL
			rcall	I2C_qp_delay	; quarter period delay
			cbi		ddrI2C, pinSDA	; release SDA
			rcall	I2C_hp_delay	; half period delay
			ret


;***************************************************************************
;*
;* FUNCTION
;*	I2C_init
;*
;* DESCRIPTION
;*	Initialization of the I2C bus interface.
;*
;* USAGE
;*	Call this function once to initialize the I2C bus. No parameters
;*	are required.
;*
;* RETURN
;*	None
;*
;* NOTE
;*	pI2C and ddrI2C pins not used by the I2C bus interface will be
;*	set to Hi-Z (!).
;*
;* COMMENT
;*	This function can be combined with other pI2C initializations.
;*
;***************************************************************************

I2C_init:	clr		rI2Cstat			; clear I2C status register (used
										; as a temporary register)
			out		pI2C, rI2Cstat		; set I2C pins to open colector
			out		ddrI2C, rI2Cstat
			ret
