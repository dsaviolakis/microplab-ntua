.include "m328PBdef.inc"

.equ PD0=0
.equ PD1=1
.equ PD2=2
.equ PD3=3
.equ PD4=4
.equ PD5=5
.equ PD6=6
.equ PD7=7

.def temp = r16
.def ADC_L = r21
.def ADC_H = r22

.org 0x00
	rjmp reset

.org 0x1A
	rjmp ISR_TIMER1_OVF

.org 0x2A			;ADC Conversion Complete Interrupt
	rjmp measure

reset:
	;Initialize stack
	ldi temp, low(RAMEND)
	out SPL, temp
	ldi temp, high(RAMEND)
	out SPH, temp

	;Set PORTD as output for LED screen
	ldi temp, 0xFF
	out DDRD, temp

	ldi temp, 0x00
	out DDRC, temp


	
	; REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0001 => select ADC1(pin PC1),
	; ADLAR=1 => Left adjust the ADC result
	ldi temp, 0b01000001
	sts ADMUX, temp

	; ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
	; ADIE=1 => enable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
	ldi temp, 0b10001111
	sts ADCSRA, temp

	;Enable interrupts
	sei

	; Setup TCCR1B flags
	ldi temp, (1<<CS12) | (0<<CS11) | (1<<CS10)	;CK/1024
	sts TCCR1B, temp

	ldi temp, HIGH(49910) ; Initialize TCNT1
	sts TCNT1H, temp
	ldi temp, LOW(49910)  ; for overflow after 1 sec
	sts TCNT1L, temp

	ldi temp, (1<<TOIE1)   ; Allow overflow interrupt for TCNT1
	sts TIMSK1, temp       ;

	;Initialize lcd screen
	rcall lcd_init
	ldi r24 ,low(50)	;
	ldi r25 ,high(50)	; Wait 200 mSec
	rcall wait_msec

main_loop:
	rjmp main_loop

conv_adc_to_volt:

	;Vin = (ADC/1024)*Vref = 5(ADC/1024)
	mov r24, ADC_L
	mov r25, ADC_H

	; Multiply low byte by 5
	ldi r22, 5         ; Load 5 into r22
	mul r24, r22       ; Multiply r24 (low byte) by 5
	mov r18, r0        ; Store low byte of result in r18
	mov r19, r1        ; Store high byte of result in r19

	; Multiply high byte by 5
	mul r25, r22       ; Multiply r25 (high byte) by 5
	add r19, r0        ; Add low byte of this result to r19 (carry-aware)
	adc r25, r1        ; Add high byte with carry to r25

	; Final result in r24:r25 (low:high bytes)
	mov r26, r18       ; Move final low byte to r24
	mov r27, r19       ; Move final high byte to r25

	mov r19,r27
	lsr r19
	lsr r19
	ldi r20, 0b00110000
	add r19,r20
	rcall lcd_clear_display
	mov r24,r19
	out PORTD ,r24
	rcall lcd_data
	ldi r20, 0b00101110
	mov r24, r20
	out PORTD ,r24
	rcall lcd_data
	// calc first decimal point 
	mov r24, r19
	clr r25
	subi r24,0b00110000
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	mov r19, r27
	mov r18, r26
	sub r19, r25
	sub r18, r24
	mov r24, r18           ; Copy lower byte to temporary register
	mov r25, r19           ; Copy upper byte to temporary register
	lsl r18
	rol r19
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	add r18,r24
	add r19,r25
	mov r24,r19
	lsr r24
	lsr r24
	ldi r20,0b00110000
	add r24, r20
	out PORTD ,r24
	rcall lcd_data
	// calc second decimal
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	mov r19, r27
	mov r18, r26
	sub r19, r25
	sub r18, r24
	mov r24, r18           ; Copy lower byte to temporary register
	mov r25, r19           ; Copy upper byte to temporary register
	lsl r18
	rol r19
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	add r18,r24
	add r19,r25
	mov r24,r19
	lsr r24
	lsr r24
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	sub r19, r25
	sub r18, r24
	mov r24, r18           ; Copy lower byte to temporary register
	mov r25, r19           ; Copy upper byte to temporary register
	lsl r18
	rol r19
	lsl r24
	rol r25
	lsl r24
	rol r25
	lsl r24
	rol r25
	add r18,r24
	add r19,r25
	mov r24,r19
	lsr r24
	lsr r24
	ldi r20,0b00110000
	add r24, r20
	out PORTD ,r24
	rcall lcd_data
	
	reti

ISR_TIMER1_OVF:
	clr temp
	lds temp, ADCSRA	;
	ori temp, (1<<ADSC)	; Set ADSC flag of ADCSRA
	sts ADCSRA, temp	;
	ldi temp, HIGH(49910) ; Initialize TCNT1
	sts TCNT1H, temp
	ldi temp, LOW(49910)  ; for overflow after 1 sec
	sts TCNT1L, temp
	reti

measure:
    	lds ADC_L,ADCL		;
	lds ADC_H,ADCH		; Read ADC result(Left adjusted)
	rcall conv_adc_to_volt
	reti

lcd_init:
	ldi r24 ,low(200)	;
	ldi r25 ,high(200)	; Wait 200 mSec
	rcall wait_msec		;

	ldi r24 ,0x30		; Command to switch to 8 bit mode
	out PORTD ,r24		;
	sbi PORTD ,PD3		; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi r24 ,250		;
	ldi r25 ,0		; Wait 250uSec
	rcall wait_usec		;

	ldi r24 ,0x30		;command to switch to 8 bit mode
	out PORTD ,r24		;
	sbi PORTD ,PD3		; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi r24 ,250		;
	ldi r25 ,0		; Wait 250uSec
	rcall wait_usec		;

	ldi r24 ,0x30		; command to switch to 8 bit mode
	out PORTD ,r24		;
	sbi PORTD ,PD3		; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi r24 ,250		;
	ldi r25 ,0		; Wait 250uSec
	rcall wait_usec

	ldi r24 ,0x20		; command to switch to 4 bit mode
	out PORTD ,r24
	sbi PORTD ,PD3		; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi r24 ,250		;
	ldi r25 ,0		; Wait 250uSec
	rcall wait_usec

	ldi r24 ,0x28		; 5x8 dots, 2 lines
	rcall lcd_command

	ldi r24 ,0x0c		; display on, cursor off
	rcall lcd_command

	rcall lcd_clear_display

	ldi r24 ,0x06		; Increase address, no display shift
	rcall lcd_command	;
	ret

lcd_data:
	sbi PORTD ,PD2		;LCD_RS=1(PD2=1), Data
	rcall write_2_nibbles	;send data
	ldi r24 ,250		;
	ldi r25 ,0		; Wait 250uSec
	rcall wait_usec
	ret

lcd_command:
	cbi PORTD ,PD2		; LCD_RS=0(PD2=0), Instruction
	rcall write_2_nibbles	; send Instruction
	ldi r24 ,250		;
	ldi r25 ,0		; Wait 250uSec
	rcall wait_usec
	ret

lcd_clear_display:
	ldi r24 ,0x01		; Clear display command
	rcall lcd_command

	ldi r24 ,low(5)		;
	ldi r25 ,high(5)	; Wait 5 mSec
	rcall wait_msec		;

	ret
	
write_2_nibbles:
    push r24
    in r25 ,PIND
    andi r25 ,0x0f
    andi r24 ,0xf0
    add r24 ,r25
    out PORTD ,r24
    sbi PORTD ,PD3
    nop
    nop
    cbi PORTD ,PD3
    pop r24
    swap r24
    andi r24 ,0xf0
    add r24 ,r25
    out PORTD ,r24
    sbi PORTD ,PD3
    nop
    nop
    cbi PORTD ,PD3
    ret

wait_msec:
    push r24
    push r25
    ldi r24 , low(999)
    ldi r25 , high(999)
    rcall wait_usec
    pop r25
    pop r24
    nop
    nop
    sbiw r24 , 1
    brne wait_msec
    ret

wait_usec:
    sbiw r24 ,1
    call delay_8cycles
    brne wait_usec
    ret

delay_8cycles:
    nop
    nop
    nop
    ret
