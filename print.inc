.EQU using=0b00100000   ;Flag für formatierte Ausgabe (Print Using)
.EQU sig = 0b01000000   ;Flag für Vorzeichen-Zahl (signed Integer)
.EQU fnul= 0b10000000   ;Flag für Führungsnullen

;-----------------------------------------------------------------------------
;  PRINT8 Reg     		;Gibt Zahl im Register 2- oder 3-stellig am LCD aus.
                        ;Parameter bestimmt das Register, in dem die Zahl liegt.
                        ;Dieses Register wird nicht verändert.
                        ;Kopie der Zahl temporär in xl.
;-----------------------------------------------------------------------------
.MACRO PRINT8 ;Registername
			push 	xl
 			
			mov 	xl,@0		;Kopie vom Quellregister
 			rcall 	lcd_printb	;Aufruf...
 			
			pop 	xl
.ENDMACRO


;-----------------------------------------------------------------------------
;  PRINT16 ;H-Reg, L-Reg, Kommastellen, [using]+[sig]+[fnul]
                        ;Gibt 16-Bit-Zahl als Ziffern aus
                        ;H-Reg,L-Reg enthält die Zahl
                        ;Komma ist die Anzahl der Nachkommastellen
                        ;using erzwingt formatierte Ausgabe (Print Using)
                        ;sig erzwingt Vorzeichen
                        ;fnul erzwingt Führungsnullen
;-----------------------------------------------------------------------------
.MACRO PRINT16 ;H-Reg, L-Reg, Kommastellen, using+sign+fnul
 			push 	xh				;verwendete Register
 			push 	xl				;sichern
 			push 	rwh
 			push 	rwl
 			
			mov 	xl, @1			;Low-Byte
 			mov 	xh, @0			;High-Byte
 			ldi 	rwh, (@3 & $e0)|16|(6-(@2 & 15))	;Mode-Flags und Komma-Position
 			rcall 	lcd_print16		;Aufruf
 			
			pop 	rwl
 			pop 	rwh
 			pop 	xl				;verwendete Register
 			pop 	xh				;wiederherstellen
.ENDMACRO


;-----------------------------------------------------------------------------
;  PRINTT adr16, Reg	;Gibt einen Text aus, dessen Adresse in einer
                        ;Indextabelle steht.
                        ;adr16 ist die Basisadresse der Indextabelle,
                        ;Reg ist ein Register, das den den Eintrag in
                        ;der Tabelle auswählt.
                        ;Dient zum Ausgeben von indizierten Texten wie
                        ;Menüpunkte, Wochentage usw.
;-----------------------------------------------------------------------------
.MACRO PRINTT	;Adresse der Tabelle, Index auf Tabelle
			push	zh				;verwendete Register
 			push 	zl				;sichern
 			push 	rwl
 			ldi 	zh, HIGH(@0*2)	; get Base adress
 			ldi 	zl, LOW(@0*2)	;der Indextabelle
 			mov 	rwl,	@1			;Index holen
 			rcall 	lcd_printt		;Aufruf
 			pop 	rwl				;verwendete
 			pop 	zl				;Register
 			pop 	zh				;wiederherstellen
.ENDMACRO


;-----------------------------------------------------------------------------
;  PRINTF adr16         ;Gibt den 0-terminierten Text an Adr.16 im Flash aus
;-----------------------------------------------------------------------------
.MACRO PRINTF	;Startadresse des Strings im Flash
				;Gibt einen String aus dem Flash an LCD aus
				;Ein Parameter beschreibt die Startadresse, das Ende ist $00
 			push	zh				;verwendete Register
 			push 	zl				;sichern
 			ldi 	zh, HIGH(2*@0)	;Pointer
 			ldi 	zl, LOW(2*@0)	;setzen
 			rcall 	lcd_printf		;Aufruf...
 			pop 	zl				;verwendete Register
 			pop 	zh				;wiederherstellen
.ENDMACRO

;-----------------------------------------------------------------------------
;  PRINTF_I2C adr16        ;Gibt den 0-terminierten Text an Adr.16 im Flash über I2C aus
;
; Parameter 1: I2C address @0
; Parameter 2: Stringpointer @1
;
; uses rwl, rwh and zl, zh Registers
;
; usage: PRINTF_I2C address, stringpointer
;-----------------------------------------------------------------------------
.MACRO PRINTF_I2C	;I2C device address ;Startadresse des Strings im Flash
					;Gibt einen String aus dem Flash an LCD aus
					;Ein Parameter beschreibt die Startadresse, das Ende ist $00
 			
			IRQ_DISABLE				; no interrupts durring transfer
			push	zh				
 			push 	zl				
			push	rwl
			push 	rwh

			ldi		rwh, @0+b_I2Cwr	; Set device address and write
 			ldi 	zh, HIGH(2*@1)	;Pointer
 			ldi 	zl, LOW(2*@1)	;setzen
 			rcall 	i2c_printf		;Aufruf...
 			
			pop		rwh
			pop		rwl
			pop 	zl				
 			pop 	zh
			IRQ_ENABLE				
.ENDMACRO

;-----------------------------------------------------------------------------
lcd_printt:	;Wird von Makro aufgerufen. Gibt indizierten Text aus
			push 	rwh
			lsl 	rwl				;Index mal 2 (2 Bytes pro Eintrag)
			add 	zl, rwl			;Index addieren
			adc 	zh, rNull		;Übertrag auch
			lpm 	rwl, z+			;Low-Byte holen
			lpm 	rwh, z+			;High-Byte holen
			mov 	zl, rwl			;in Z-Pointer
			mov 	zh, rwh			;kopieren
			lsl 	zl				;Index auf Text
			rol 	zh				;mal 2 (Byteweise Adressierung)
			rcall 	lcd_printf		;Text ausgeben...
			pop 	rwh
			ret

;-----------------------------------------------------------------------------
i2c_printf:	;Wird vom Makro aufgerufen. Gibt Flash-String über I2C aus.
			; in rwl steht zu sendendes zeichen 

			push	rtemp3
			rcall	I2C_init
			rcall	I2C_start			; Send start condition and address

			lpm 	rwl, z+				;Zeichen holen
 			tst 	rwl					;Ende-Kennung? 
 			breq 	pc+3				;ja...			
			rcall	I2C_do_transfer		; nein- Execute transfer
 			rjmp 	pc-4				;nochmal
			
			rcall	I2C_stop			; finished - Send stop condition
			pop		rtemp3
 			ret

;-----------------------------------------------------------------------------
lcd_printf:	;Wird vom Makro aufgerufen. Gibt Flash-String an LCD aus.
			; in rwl steht zu sendendes zeichen 
 			push 	rwl
 			lpm 	rwl, z+			; Zeichen holen
 			tst 	rwl				; Ende-Kennung? 
 			breq 	pc+3			; ja...
 			rcall 	lcd_data		; nein, ausgeben
 			rjmp 	pc-4			; nochmal
 			pop		rwl
 			ret

;-----------------------------------------------------------------------------
lcd_printb: 	;Wird vom Makro aufgerufen. Gibt Byte 2 oder 3 stellig als ASCII Zahl aus.
			push	rwl

			ldi 	rwl,-1+'0'		; Hunderter-Stelle als ASCII-Zeichen, Zahl ist positiv
			inc 	rwl				; Hunderter hoch und
			subi 	xl,100			; 100 subtrahieren bis negativ wird
			brsh 	pc-2			; negativ? nein, 2 Zeilen hoch
			; cpi 	rwl,'0'			; ja, ist Ziffer = "0"?
			; breq 	pc+2			; ja, nicht ausgeben...
			rcall 	lcd_data		; Hunderter ausgeben...
			ldi 	rwl,10+'0'		; Zehner-Stelle als ASCII-Zeichen, Zahl ist negativ
			dec 	rwl				; Zehner runter und
			subi 	xl,-10			; 10 addieren bis positiv wird
			brlo 	pc-2			; positiv? nein, 2 Zeilen hoch...
			rcall 	lcd_data		; ja, Zehner Stelle ausgeben...
			ldi 	rwl,'0'			; ASCII-0
			add 	rwl,xl			; Einer addieren (Rest war ja positiv)
			rcall 	lcd_data		; Einer ausgeben...

			pop 	rwl
			ret

;-----------------------------------------------------------------------------
lcd_print16:	;Wird von Macro aufgerufen. Gibt Word als Zahl in mehreren Modi aus
 			sbrs 	rwh, 6			;Signum-Flag gesetzt? ja...
 			rjmp 	lcd_printui		;nein, als unsigned-Int behandeln...
 			sbrs 	xh, 7			;Wert negativ? ja...
 			rjmp 	lcd_printsig	;nein, positiv...
 			ldi 	rwl, '-'		;Vorzeichen
 			rcall 	lcd_data		;ausgeben
 			neg 	xl				;Zahl positiv
 			com 	xh				;machen

lcd_printui:	;Wird von Makro aufgerufen. Gibt unsigned-Integer als 5 Ziffern an LCD aus
 			ldi 	rwl, -1+'0'		;Ziffer für Zehntausender
 			inc 	rwl				;Zehntausender Stelle hoch und
 			subi 	xl, low(10000)	;von Zahl 10000 subtrahieren
 			sbci 	xh, high(10000)	;bis Zahl negativ wird
 			brcc 	pc-3                  ;schon negativ? nein...
lcd_print16_a:	;Einsprung aus 24-Bit-Routine
 			rcall 	lcd_format		;ja, Ziffer ausgeben (Zehntausender)
 			ldi 	rwl, 10+'0'		;Ziffer für Tausender, Zahl ist jetzt negativ
 			dec 	rwl				;Tausender Stelle verkleinern und
 			subi 	xl, low(-1000)	;1000 zur (negativen) Zahl addieren,
 			sbci 	xh, high(-1000)	;bis Zahl wieder positiv wird
 			brcs 	pc-3			;noch negativ? ja...
 			rcall 	lcd_format		;nein, Ziffer ausgeben (Tausender)
 			ldi 	rwl, -1+'0'		;Ziffer für Hunderter (Zahl ist jetzt positiv)
 			inc 	rwl				;Hunderter Stelle hoch und
 			subi 	xl, low(100)		;von Zahl 100 subtrahieren
 			sbci 	xh, high(100)	;bis Zahl negativ wird
 			brcc 	pc-3			;schon negativ? nein...
 			rcall 	lcd_format		;ja, Ziffer ausgeben (Hunderter)
 			ldi 	rwl, 10+'0'		;Zehner-Stelle als ASCII-Zeichen, Zahl ist negativ
 			dec 	rwl				;Zehner runter und
 			subi 	xl, -10			;10 addieren bis positiv wird
 			brlo 	pc-2			;positiv? nein, 2 Zeilen hoch...
 			rcall 	lcd_format		;ja, Zehner Stelle ausgeben...
 			ldi 	rwl, '0'		;ASCII-0
 			add 	rwl, xl			;Einer addieren (Rest war ja positiv)
 			rcall 	lcd_format		;Einer ausgeben
 			andi 	rwh, using		;Führungsnullen oder Using aktiv?
 			brne 	pc+3			;ja, also konstante Textlänge...
 			ldi 	rwl, ' '		;nein, variable Textlänge, deshalb
 			rjmp 	lcd_data		;Leerzeichen anhängen um alte Ziffer zu löschen
 			ret						;zurück

lcd_printsig:
 			ldi 	rwl, '+'		;Vorzeichen
 			rcall 	lcd_data		;ausgeben
 			rjmp 	lcd_printui		;Zahl ausgeben
lcd_format:
 			dec 	rwh				;Kommazähler runter
 			push 	rwh				;Mode-Flags sichern
 			andi 	rwh, $0f		;Kommazähler isolieren
 			cpi 	rwh, 1			;Letzte Stelle vor dem Komma?
 			pop 	rwh				;(Mode-Flags wegen Veränderung holen)
 			brne 	pc+2			;nein...
 			sbr 	rwh, fnul		;Nullausgabe aktivieren
 			push 	rwh				;geänderte Mode-Flags nochmal sichern
 			andi 	rwh, $0f			;Kommazähler isolieren (wegen Zero-Flag)
 			pop 	rwh				;Mode-Flags wiederherstellen
 			brne 	lcd_keinkomma	;Kommazähler nicht Null...
 			push 	rwl				;Ziffer sichern
 			ldi 	rwl, ','		;Komma
 			rcall 	lcd_data		;ausgeben
 			pop 	rwl				;Ziffer wiederherstellen
lcd_keinkomma:
 			cpi 	rwl, '0'		;Ziffer = "0"?
 			breq 	lcd_format0		;ja...
 			sbr 	rwh, fnul		;nein, künftige Nullen nicht mehr unterdrücken
 			rjmp 	lcd_data		;Ziffer ausgeben und zurück...
lcd_format0:
 			sbrs 	rwh, 7			;Nullen ausgeben? ja...
 			ldi 	rwl, ' '		;nein, Null durch Leerzeichen ersetzen
 			sbrc 	rwh, 7			;Nullen ausgeben? nein...
 			rjmp 	lcd_data		;ja, Ziffer "0" ausgeben und zurück...
 			sbrc 	rwh, 5			;Print Using? nein...
 			rjmp 	lcd_data		;ja, Leerzeichen ausgeben und zurück...
 			ret						;nein, ohne Ausgabe zurück...
