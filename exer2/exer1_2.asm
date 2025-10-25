.include "m328PBdef.inc"         ; Include ATmega328PB definitions
.equ FOSC_MHZ=16                  ; Microcontroller operating frequency in MHz
.equ DEL_mS=500                   ; Delay in mS (valid number from 1 to 4095)
.equ DEL_NU=FOSC_MHZ*DEL_mS       ; delay_mS routine: (1000*DEL_NU+6) cycles
.def int1_count = r20             ; Define register r20 as the INT1 counter

.org 0x0                         ; Reset vector
    rjmp reset

.org 0x4                         ; INT1 vector (external interrupt)
    rjmp isr1

reset:
; Init Stack Pointer
    ldi r24, LOW(RAMEND)
    out SPL, r24
    ldi r24, HIGH(RAMEND)
    out SPH, r24
    clr int1_count                ; Clear the INT1 counter

; Init PORTB as output
    ser r26                       ; Set r26 to all 1's
    out DDRB, r26                 ; Set all PORTB pins as output
    ser r26                       ; Set r26 to all 1's
    out DDRC, r26                 ; Set all PORTB pins as output


; Enable INT1 (external interrupt)
    ldi r24, (1<<ISC11) | (1<<ISC10) ; Set INT1 to trigger on rising edge
    sts EICRA, r24                   ; Store configuration in EICRA
    ldi r24, (1<<INT1)               ; Enable INT1 interrupt
    out EIMSK, r24                   ; Store in EIMSK to enable the interrupt
    sei                              ; Enable global interrupts

loop1:
    clr r26                          ; Clear r26 to start counting from 0

loop2:
    out PORTB, r26                   ; Output the value of r26 to PORTB
    ldi r24, low(DEL_NU)             ; Load low byte of DEL_NU into r24
    ldi r25, high(DEL_NU)            ; Load high byte of DEL_NU into r25
    rcall delay_mS                   ; Call delay subroutine
    inc r26                          ; Increment r26
    cpi r26, 16                      ; Compare r26 with 16
    breq loop1                       ; If equal to 16, jump to loop1
    rjmp loop2                       ; Otherwise, repeat loop2

; Delay subroutine (1000 * F1 + 6 cycles)
delay_mS:
    ldi r23, 249                     ; Initialize r23 with 249 (1 cycle)
loop_inn:
    dec r23                          ; Decrement r23 (1 cycle)
    nop                              ; No operation (1 cycle)
    brne loop_inn                    ; If r23 != 0, repeat (1 or 2 cycles)
    sbiw r24, 1                      ; Subtract 1 from r24:r25 (2 cycles)
    brne delay_mS                    ; If r24:r25 != 0, repeat (1 or 2 cycles)
    ret                              ; Return from subroutine (4 cycles)

; External Interrupt 1 Service Routine (ISR)
isr1:
    ldi r22, (1 << INTF1)
    out EIFR, r22
    ldi r22, 5
    rcall wait_x_msec
    in r22, EIFR
    sbrc r22, INTF1
    rjmp isr1
 
    in r21, PIND          ; Read the state of PORTD into r24
    sbrc r21, 5           ; Skip next instruction if bit 4 (5th bit) is clear (button is pressed)
    inc int1_count                   ; Increment INT1 counter
    cpi int1_count, 64               ; Compare counter with 16
    breq make_int1_count_zero        ; If equal to 16, reset counter

return_for_int1:
    out PORTC, int1_count            ; Output INT1 counter value to PORTB
    reti                             ; Return from interrupt

; Subroutine to reset INT1 counter
make_int1_count_zero:
    clr int1_count                   ; Clear the INT1 counter
    rjmp return_for_int1             ; Jump back to return for INT1
    
wait_x_msec:
    push r26                         ; Save register r26 on stack
    push r27                         ; Save register r27 on stack
    push r28                         ; Save register r28 on stack
    dec r22                       ; Subtract 1 from r22
    breq wait_loop_one               ; If zero, jump to wait_loop_one
wait_loop_not_one:
    rcall delay_loop                 ; Call delay loop
    dec r22                       ; Subtract 1 from r22
    brne wait_loop_not_one           ; Repeat if not zero
    nop
wait_loop_one:
    rcall delay_last                 ; Call final delay
    pop r28                          ; Restore r28
    pop r27                          ; Restore r27
    pop r26                          ; Restore r26
    ret

delay_loop:
    ldi r26,low(3197)                ; Load lower byte of delay (for larger delay)
    ldi r27,high(3197)               ; Load higher byte of delay
delay_loop_inner:
    nop
    sbiw r26,1                       ; Subtract 1 from r26
    brne delay_loop_inner            ; Repeat if not zero
    nop
    nop
    nop
    ret

delay_last:
    ldi r26,low(3193)                ; Load lower byte of final delay
    ldi r27,high(3193)               ; Load higher byte of final delay
delay_last_inner:
    nop
    sbiw r26,1                       ; Subtract 1 from r26
    brne delay_last_inner            ; Repeat if not zero
    nop
    nop
    nop
    ret

