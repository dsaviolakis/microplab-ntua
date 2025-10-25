.include "m328PBdef.inc"         ; Include ATmega328PB definitions
.equ F_CPU=16000000
    
.org 0x0                         ; Reset vector
    rjmp reset

.org 0x4                         ; INT1 vector (external interrupt)
    rjmp isr1

reset:
; Init Stack Pointer
    ldi r19, LOW(RAMEND)
    out SPL, r19
    ldi r19, HIGH(RAMEND)
    out SPH, r19
    clr r19
    ldi r24,low(100) 
    ldi r25,high(100) 

    ser r19                       ; Set r26 to all 1's
    out DDRB, r19                 ; Set all PORTB pins as output
    
; Enable INT1 (external interrupt)
    ldi r19, (1<<ISC11) | (1<<ISC10) ; Set INT1 to trigger on rising edge
    sts EICRA, r19                   ; Store configuration in EICRA
    ldi r19, (1<<INT1)               ; Enable INT1 interrupt
    out EIMSK, r19                   ; Store in EIMSK to enable the interrupt
    sei  

main:
    cpi r20,0
    breq main_not_lit
    rjmp main_lit
    
main_not_lit:
    ldi r19,0x00
    out PORTB,r19
    rjmp main
    
main_lit:
    ldi r19,0x01
    out PORTB, r19
    dec r20
    ldi r24,low(100) 
    ldi r25,high(100) 
    rcall wait_x_msec
    rjmp main
    
isr1:
    ldi r22, (1 << INTF1)
    out EIFR, r22
    ldi r22, 5
    rcall wait_x_msec
    in r22, EIFR
    sbrc r22, INTF1
    rjmp isr1
    cpi r20, 0
    breq not_lit
    rjmp lit
    
lit:
    ldi r20,49
    ldi r21, 0xFF
    out PORTB,r21
    ldi r24,low(500) 
    ldi r25,high(500) 
    rcall wait_x_msec
    reti 
    
not_lit:    
    ldi r20,49
    reti 
    
wait_x_msec:
    push r26
    push r27
    push r28
    sbiw r24,1
    breq wait_loop_one
wait_loop_not_one:
    rcall delay_loop
    sbiw r24,1
    brne wait_loop_not_one
    nop
wait_loop_one:
    rcall delay_last
    ldi r24,low(100) 
    ldi r25,high(100) 
    pop r28
    pop r27
    pop r26
    ret
    
delay_loop:
    ldi r26,low(3197) 
    ldi r27,high(3197)
delay_loop_inner:
    nop
    sbiw r26,1
    brne delay_loop_inner
    nop
    nop
    nop
    ret
    
delay_last:
    ldi r26,low(3193) 
    ldi r27,high(3193)
delay_last_inner:
    nop
    sbiw r26,1
    brne delay_last_inner
    nop
    nop
    nop
    ret

  
