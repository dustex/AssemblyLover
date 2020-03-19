;***************************************************************************
;*
;* DESCRIPTION
;*	Macro Definitions
;*
;***************************************************************************



; ============================================
;   IRQ_DISABLE
; ============================================
;
; just disables the global interrupt
;
; usage:  IRQ_DISABLE
;
.MACRO IRQ_DISABLE		
			cli
.ENDMACRO

; ============================================
;   IRQ_ENABLE
; ============================================
;
; just disables the global interrupt
;
; usage:  IRQ_ENABLE
;
.MACRO IRQ_ENABLE		
			sei
.ENDMACRO


;============================================
;  PREPARE_TO (16 Bit)
;============================================
;
; stores the current timestamp in SRAM so that it can be used
; to calculate the timeout later
;
;
; Parameter 1: 16 Bit-SRAM-Reg @0
;
; usage:  SET_TO 2Byte-SRAM-Reg, 2Byte-timeout

.MACRO PREPARE_TO ;2Byte-SRAM-Reg
			sts		@0, rStampL
			sts		@0+1, rStampH
.ENDMACRO

;============================================
;  JMP_IFTO (16 Bit)
;============================================
;
;checks if timeout elapsed and jumps to given tags
;needs a 2Byte SRAM mem to store the old timestamp
;and the timeout which should elapse
;
;Maximum timeout is: (0xFFFF=65536) 400*(1/F_ISR)
;with F_ISR=400 -> 2,5ms: maximum timeout is 1000ms = 1s in 2,5ms steps
;
; Parameter 1: 16 Bit-SRAM-Reg @0
; Parameter 2: 16 Bit timeout  @1
; Parameter 3: where to jump if timeout elapsed	@2
; Parameter 4: where to jump if not timeout		@3
;
; usage:  JMP_IFTO 2Byte-SRAM-Reg, 2Byte-timeout, tag-timeout-elapsed, tag-no-timeout

.MACRO JMP_IFTO ;2Byte-SRAM-Reg, timeout, tag-timeout-elapsed, tag-no-timeout 
			.IF (@1>398)       ; max. timeout is 398 -> 1000ms
  				.ERROR "Maximum Timeout is 398 (1000ms)"
			.ENDIF	
			push	rwl
			push	rwh
			push	xl
			push	xh
			push	yl
			push	yh
			
			; rStampTo counts from 0x0000 to 0xFFFF, wraps arround and starts again at 0x0000
			; rStamp counts from 0x0000 to 0x0190, wraps arround and starts again at 0x0000
			mov		xl, rStampL		; get copy of current time
			mov		xh, rStampH
			lds		yl, @0			; get old timestamp
			lds		yh, @0+1

			; y: old timestamp
			; x: current timestamp
			cp		xl, yl			; if [(xl-yl)<0  -> (xl<yl) -> current time < old timestamp]
			cpc		xh, yh
			brlo	IF_TO_adjust	;	yes it is lower
			sub		xl, yl			; else (no not lower) - calculate the time difference
			sbc		xh, yh
			rjmp	IF_TO_compare	;	check if timeout occured		

IF_TO_adjust:
			; new timestamp is lower than old one
			; so it needs to becorrected here
			; y=0xFF00 old timestamp
			; x=0x0000 new timestamp (old timstamp + timeout)
			ldi		rwl, LOW(F_ISR)		; rwl= 0x90			-> calc (400-old stamp) + new stamp
			ldi		rwh, HIGH(F_ISR)	; rwh= 0x01			->
			sub		rwl, yl				; rwl= rwl-yl		-> 0x90-0x00=0x90	
			sbc		rwh, yh				; rwh= rwh-yh-c		-> 0x01-0xFF=0x00
			add		xl, rwl				; xl = xl+rwl		-> 
			adc		xh, rwh				; xh = xh+rwh+c		-> x=0x00FF

IF_TO_compare:
			ldi		rwl, LOW(@1)		; difference > timeout?
			ldi		rwh, HIGH(@1)
			cp		xl, rwl
			cpc		xh, rwh
			
			pop		yh
			pop		yl
			pop		xh
			pop		xl
			pop		rwh
			pop		rwl	
			
			brlo	@3					; jump to this tag if no timeout
			breq	@3					; jump to this tag if no timeout
			rjmp	@2					; jump to this tag if timeout occured
.ENDMACRO

;============================================
;  CALL_IFTO (16 Bit)
;============================================
;
;checks if timeout elapsed and calls given subroutine
;needs a 2Byte SRAM mem to store the old timestamp
;and the timeout which should elapse
;
;Maximum timeout is: (0xFFFF=65536) 400*(1/F_ISR)
;with F_ISR=400 -> 2,5ms: maximum timeout is 1000ms = 1s in 2,5ms steps
;
; Parameter 1: 16 Bit-SRAM-Reg @0
; Parameter 2: 16 Bit timeout  @1
; Parameter 3: what to call if timeout		@2
;
; usage:  CALL_IFTO 2Byte-SRAM-Reg, 2Byte-timeout, call-if-timeout-elapsed

.MACRO CALL_IFTO ;2Byte-SRAM-Reg, timeout,  
			.IF (@1>398)       ; max. timeout is 398 -> 1000ms
  				.ERROR "Maximum Timeout is 398 (1000ms)"
			.ENDIF	
			push	rwl
			push	rwh
			push	xl
			push	xh
			push	yl
			push	yh
			
			; rStampTo counts from 0x0000 to 0xFFFF, wraps arround and starts again at 0x0000
			; rStamp counts from 0x0000 to 0x0190, wraps arround and starts again at 0x0000
			mov		xl, rStampL		; get copy of current time
			mov		xh, rStampH
			lds		yl, @0			; get old timestamp
			lds		yh, @0+1

			; y: old timestamp
			; x: current timestamp
			cp		xl, yl			; if [(xl-yl)<0  -> (xl<yl) -> current time < old timestamp]
			cpc		xh, yh
			brlo	IF_TO_adjust	;	yes it is lower
			sub		xl, yl			; else (no not lower) - calculate the time difference
			sbc		xh, yh
			rjmp	IF_TO_compare	;	check if timeout occured		

IF_TO_adjust:
			; new timestamp is lower than old one
			; so it needs to becorrected here
			; y=0xFF00 old timestamp
			; x=0x0000 new timestamp (old timstamp + timeout)
			ldi		rwl, LOW(F_ISR)		; rwl= 0x90			-> calc (400-old stamp) + new stamp
			ldi		rwh, HIGH(F_ISR)	; rwh= 0x01			->
			sub		rwl, yl				; rwl= rwl-yl		-> 0x90-0x00=0x90	
			sbc		rwh, yh				; rwh= rwh-yh-c		-> 0x01-0xFF=0x00
			add		xl, rwl				; xl = xl+rwl		-> 
			adc		xh, rwh				; xh = xh+rwh+c		-> x=0x00FF

IF_TO_compare:
			ldi		rwl, LOW(@1)		; difference > timeout?
			ldi		rwh, HIGH(@1)
			cp		xl, rwl
			cpc		xh, rwh
			
			pop		yh
			pop		yl
			pop		xh
			pop		xl
			pop		rwh
			pop		rwl	
			
			brlo	IF_TO_end			; jump to this tag if no timeout
			breq	IF_TO_end			; jump to this tag if no timeout
			rcall	@2					; call subroutine if timeout occured
IF_TO_end:
.ENDMACRO

; ============================================
;   Toggle_RegBit
; ============================================
;
; toggles Level at given Port and Pin
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; usage:  TOGGLE_PIN PORTD ledR
.MACRO TOGGLE_REG_BIT ;register, bit		
			push 	rwl
 			push	rwh
 			
			ldi 	rwl, (1<<@1)	; set Bit mask
			eor 	@0, rwl			; write Register
 			
			pop		rwh
			pop		rwl
.ENDMACRO

; ============================================
;   Toggle Pin
; ============================================
;
; toggles Level at given Port and Pin
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; usage:  TOGGLE_PIN PORTD ledR
.MACRO TOGGLE_PIN ;port, bit		
			push 	rwl
 			push	rwh
 			
			ldi 	rwl, (1<<@1)	; set Bit for Led Pin
			in 		rwh, @0			; read Port status
			eor 	rwh, rwl		; toggle Led Pin
			out 	@0, rwh			; write Port
 			
			pop		rwh
			pop		rwl
.ENDMACRO

; ============================================
;   Jump if Bit in Register is Cleared
; ============================================
;
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; Parameter 3: Target @2
;
; usage:  JMP_IFCLR_R R18 1 LABEL
;
.MACRO JMP_IFCLR_R ;register, bit, target
 			sbrs	@0, @1
			rjmp	@2
.ENDMACRO

; ============================================
;   Jump if Bit in Register is Set
; ============================================
;
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; Parameter 3: Target @2
;
; usage:  JMP_IFSET_R R18 1 LABEL
;
.MACRO JMP_IFSET_R ;register, bit, target
 			sbrc	@0, @1
			rjmp	@2
.ENDMACRO

; ============================================
;   Call Subroutine if Bit in Register is Cleared
; ============================================
;
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; Parameter 3: Target @2
;
; usage:  CALL_IFCLR_R R18 1 LABEL
;
.MACRO CALL_IFCLR_R ;register, bit, target
 			sbrs	@0, @1
			rcall	@2
.ENDMACRO

; ============================================
;   Call Subroutine if Bit in Register is Set
; ============================================
;
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; Parameter 3: Target @2
;
; usage:  CALL_IFSET_R R18 1 LABEL
;
.MACRO CALL_IFSET_R ;register, bit, target
 			sbrc	@0, @1
			rcall	@2
.ENDMACRO

; ============================================
;   Jump if Bit in I/O-Register is Cleared
; ============================================
;
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; Parameter 3: Target @2
; usage:  JMP_IFCLR PORTB 1 LABEL
.MACRO JMP_IFCLR ;port, bit, target
		.IF 	@0 < 0x20
 			sbis	@0, @1
			rjmp	@2
  		.ELIF 	@0 < 0x40
			push	zl
			in		zl, @0
			sbrs	zl, @1
			pop		zl
			rjmp	@2
  		.ELSE
			push	zl
			lds		zl, @0
			sbrs	zl, @1
			pop		zl
			rjmp	@2
  		.ENDIF
.ENDMACRO


; ============================================
;   Jump if Bit in I/O-Register is Set
; ============================================
;
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; Parameter 3: Target @2
; usage:  JMP_IFSET PORTB 1 LABEL
.MACRO JMP_IFSET ;port, bit, target
  		.IF 	@0 < 0x20		;Ports and UART stuff
 			sbic	@0, @1
			rjmp	@2
  		.ELIF 	@0 < 0x40		;Ports and UART stuff
			push	zl
			in		zl, @0
			sbrc	zl, @1
			pop		zl
			rjmp	@2
  		.ELSE
			push	zl
			lds		zl, @0
			sbrc	zl, @1
			pop		zl
			rjmp	@2
  		.ENDIF
.ENDMACRO


; ============================================
;   Call Subroutine if Bit in I/O-Register is Cleared
; ============================================
;
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; Parameter 3: Routine @2
; usage:  JMP_IFCLR PORTB 1 ROUTINE
.MACRO CALL_IFCLR ;port, bit, routine
  		.IF 	@0 < 0x20
 			sbis	@0, @1
			rcall	@2
  		.ELIF 	@0 < 0x40
			push	zl
			in		zl, @0
			sbrs	zl, @1
			pop		zl
			rcall	@2
  		.ELSE
			push	zl
			lds		zl, @0
			sbrs	zl, @1
			pop		zl
			rcall	@2
		.ENDIF
.ENDMACRO


; ============================================
;   Call Subroutine if Bit in I/O-Register is Set
; ============================================
;
; Parameter 1: Port @0
; Parameter 2: Pin	@1
; Parameter 3: Routine @2
; usage:  JMP_IFSET PORTB 1 ROUTINE
.MACRO CALL_IFSET ;port, bit, routine
  		.IF 	@0 < 0x20
 			sbic	@0, @1
			rcall	@2
  		.ELIF 	@0 < 0x40
			push	zl
			in		zl, @0
			sbrc	zl, @1
			pop		zl
			rcall	@2
  		.ELSE
			push	zl
			lds		zl, @0
			sbrc	zl, @1
			pop		zl
			rcall	@2
  		.ENDIF
.ENDMACRO
