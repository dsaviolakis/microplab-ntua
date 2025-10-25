.include "m328PBdef.inc"

.equ F_CPU=16000000
.def temp=r16

reset:
    ;Initialize stack
    ldi temp,low(RAMEND)
    out SPL,temp
    ldi temp,high(RAMEND)
    out SPH,temp
    ldi r24,low(6420) ;insert x here 
    ldi r25,high(6420) 

main:

    rcall wait_x_msec
    rjmp main

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
    ldi r24,low(6420) 
    ldi r25,high(6420) 
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
    
