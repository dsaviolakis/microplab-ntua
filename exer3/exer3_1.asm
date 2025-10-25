.include "m328PBdef.inc"

.def temp = r16
.def DC_VALUE = r17
.def uflag = r18
.def dflag = r19

.org 0x00
    rjmp reset

reset:
    ; Initialize stack
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp
    ldi r24, 5

    ; Load table starting address
    ldi ZL, low(table*2)      ; Load low byte of table address into ZL
    ldi ZH, high(table*2)     ; Load high byte of table address into ZH
    adiw ZL, 12              ; Move index to the middle of the table (index 6)

    ; Setup input for buttons (PD3 and PD4)
    clr temp
    out DDRD, temp          ; Set PORTD as input (buttons PD3, PD4)

    ; Setup output for PWM (PB1/OC1A)
    ser temp
    out DDRB, temp          ; Set all PORTB pins as output

    ; Fast PWM, 8-bit configuration
    ldi temp, (1 << WGM10) | (1 << COM1A1) ; Fast PWM, non-inverted on OC1A
    sts TCCR1A, temp
    ldi temp, (1 << WGM12) | (1 << CS10)   ; No prescaler, start the timer
    sts TCCR1B, temp

    ; Initialize duty cycle
    lpm temp, Z             ; Load initial brightness from table (index 6)
    sts OCR1AL, temp
    clr temp                ; Set high byte to 0 (8-bit PWM)
    sts OCR1AH, temp

    ; Initialize flags
    ldi uflag, 0
    ldi dflag, 0

main_loop:
    rcall wait_x_msec       ; Call debounce delay

    ; Read buttons PD3 (increase) and PD4 (decrease)
    in temp, PIND
    sbrc temp, 3            ; Check if PD3 is released
    ldi uflag, 0            ; Reset uflag if button is released
    sbrc temp, 4            ; Check if PD4 is released
    ldi dflag, 0            ; Reset dflag if button is released

    ; Handle increase/decrease logic
    sbrs temp, 3            ; If PD3 (increase button) is pressed
    rcall inc_dc
    sbrs temp, 4            ; If PD4 (decrease button) is pressed
    rcall dec_dc

    rjmp main_loop          ; Repeat the main loop

return:
    ret

inc_dc:
    cpi uflag, 1            ; Check if uflag is set
    breq return             ; If already pressed, return
    cpi DC_VALUE, 250       ; Check if at max value (250)
    breq return             ; If at max, return

    adiw ZL, 2              ; Move table pointer to the next value
    lpm DC_VALUE, Z         ; Load the new duty cycle from table
    sts OCR1AL, DC_VALUE    ; Update PWM duty cycle (low byte)
    ldi uflag, 1            ; Set uflag to indicate button press
    ret

dec_dc:
    cpi dflag, 1            ; Check if dflag is set
    breq return             ; If already pressed, return
    cpi DC_VALUE, 5         ; Check if at min value (5)
    breq return             ; If at min, return

    sbiw ZL, 2              ; Move table pointer to the previous value
    lpm DC_VALUE, Z         ; Load the new duty cycle from table
    sts OCR1AL, DC_VALUE    ; Update PWM duty cycle (low byte)
    ldi dflag, 1            ; Set dflag to indicate button press
    ret

wait_x_msec:
    ; Delay function for button debounce
    push r26                ; Save used registers
    push r27
    push r28
    subi r24, 1
    breq wait_loop_one

wait_loop_not_one:
    rcall delay_loop
    subi r24, 1
    brne wait_loop_not_one
    nop

wait_loop_one:
    rcall delay_last
    ldi r24, 5              ; Reset delay counter
    pop r28                 ; Restore registers
    pop r27
    pop r26
    ret

delay_loop:
    ldi r26, low(3197)      ; Load delay counter
    ldi r27, high(3197)
delay_loop_inner:
    nop
    sbiw r26, 1
    brne delay_loop_inner
    nop
    nop
    nop
    ret

delay_last:
    ldi r26, low(3193)      ; Load final delay counter
    ldi r27, high(3193)
delay_last_inner:
    nop
    sbiw r26, 1
    brne delay_last_inner
    nop
    nop
    nop
    ret

; Lookup table for duty cycle values (5 to 250)
table:
    .dw 5, 25, 46, 66, 86, 107, 128, 148, 168, 189, 209, 230, 250
