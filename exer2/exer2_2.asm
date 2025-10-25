.include "m328PBdef.inc"          ; ATmega328P microcontroller definitions

.equ FOSC_MHZ=16                  ; Microcontroller operating frequency in MHz
.equ DEL_mS=2000                  ; Delay in mS (valid number from 1 to 4095)
.equ DEL_NU=FOSC_MHZ*DEL_mS       ; delay_mS routine: (1000*DEL_NU+6) cycles

.org 0x0                          ; Reset vector
    rjmp reset

.org 0x2                          ; INT0 vector (external interrupt)
    rjmp isr0
    
reset:
    ; Init Stack Pointer
    ldi r24, LOW(RAMEND)
    out SPL, r24
    ldi r24, HIGH(RAMEND)
    out SPH, r24

    ; Init PORTB as input for buttons and PORTC as output
    ldi r24, 0x00                 ; Set PORTB as input (0x00)
    out DDRB, r24                 ; Set DDRB to input
    ldi r24, 0xFF                 ; Set PORTC as output (0xFF)
    out DDRC, r24                 ; Set DDRC to output
    clr r22

    ; Enable INT0 interrupt
    ldi r24, (1<<ISC01) | (1<<ISC00) ; Set INT0 to trigger on rising edge
    sts EICRA, r24                ; Store configuration in EICRA
    ldi r24, (1<<INT0)            ; Enable INT0 interrupt
    out EIMSK, r24                ; Store in EIMSK to enable the interrupt

    sei                            ; Enable global interrupts

loop1:
    clr r26                       ; Clear r26

loop2:
    out PORTC, r26                ; Output to PORTC
    ldi r24, low(DEL_NU)          ; Set delay (number of cycles)
    ldi r25, high(DEL_NU)
    rcall delay_mS                ; Call delay subroutine
    inc r26                       ; Increment r26
    cpi r26, 32                   ; Compare r26 with 32
    breq loop1                    ; If equal, restart loop1
    rjmp loop2                    ; Repeat loop2

; Delay of 1000*F1+6 cycles (almost equal to 1000*F1 cycles)
delay_mS:
    ldi r23, 249                  ; (1 cycle)
loop_inn:
    dec r23                       ; 1 cycle
    nop                           ; 1 cycle
    brne loop_inn                 ; 1 or 2 cycles
    sbiw r24, 1                   ; 2 cycles
    brne delay_mS                 ; 1 or 2 cycles
    ret                            ; 4 cycles

; ISR for INT0
isr0:
    in r20, PINB                  ; Read the state of PORTB into r20
    clr r22                       ; Clear r22 to use it as a counter (number of pressed buttons)

    ; Count the number of set bits in r20
    sbrs r20, 0                  ; Skip if bit 0 is clear (not pressed)
    inc r22                       ; Increment counter if bit 0 is set (pressed)

    sbrs r20, 1                  ; Skip if bit 1 is clear
    inc r22                       ; Increment if bit 1 is set

    sbrs r20, 2                  ; Skip if bit 2 is clear
    inc r22                       ; Increment if bit 2 is set

    sbrs r20, 3                  ; Skip if bit 3 is clear
    inc r22                       ; Increment if bit 3 is set
    
    cpi r22, 0x02
    breq two_leds
    
    cpi r22, 0x03
    breq three_leds
    
    cpi r22, 0x04
    breq four_leds

display:
    out PORTC, r22               ; Output the count to PORTB
    ldi r24, low(16*1000)           ; Set delay (number of cycles)
    ldi r25, high(16*1000)
    rcall delay_mS                ; Call delay routine
    out PORTC, r26
    reti                          ; Return from interrupt
    
two_leds:
    ldi r22, 0x03
    rjmp display
    
three_leds:
    ldi r22, 0x07
    rjmp display
    
four_leds:
    ldi r22, 15
    rjmp display
