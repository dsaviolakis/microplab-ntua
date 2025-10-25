.include "m328PBdef.inc"

.equ F_CPU=16000000
.def temp=r16
.def leds=r17

reset:
    ;Initialize stack
    ldi temp,low(RAMEND)
    out SPL,temp
    ldi temp,high(RAMEND)
    out SPH,temp
    ldi r24,low(1000) ;insert x here 
    ldi r25,high(1000) 

start:
    ;Initialize leds and T flag
    ldi leds,0x80
    clt         ;T flag zero if moving left
    
    ;Initialize PORTD as output
    ldi temp, 0xFF
    out DDRD, temp
    out PORTD, leds
    out DDRC, temp
    ldi r22, 0xFF
    bld r22,0
    com r22
    out PORTC,r22
 
loop_left:
    rcall wait_x_msec
    mov temp, leds
    ror temp           ;Logical shift left
    brcs swift_right   ;If C=1swift right
    mov leds, temp
    out PORTD, leds  ;Save changes to leds
    rjmp loop_left   ;Repeat

swift_right:
    ldi r22, 0xFF
    bld r22,0
    com r22
    out PORTC,r22
    rcall wait_x_msec
    rol leds     ;Logical shift right
    out PORTD, leds  ;Save changes to leds
    set         ;T flag zero if moving left
    rjmp loop_right  ;Start moving right

loop_right:
    rcall wait_x_msec
    mov temp, leds
    rol temp
    brcs swift_left
    mov leds, temp
    out PORTD, leds
    rjmp loop_right

swift_left:
    ldi r22, 0xFF
    bld r22,0
    com r22
    out PORTC,r22
    rcall wait_x_msec
    ror leds
    out PORTD, leds
    clt         ;T flag zero if moving left
    rjmp loop_left

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
    ldi r24,low(1000) 
    ldi r25,high(1000) 
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
