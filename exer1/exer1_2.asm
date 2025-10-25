.include "m328PBdef.inc"

.org 0x0000
rjmp main


main:
        ldi r16, 0x51	;r16 = A
        ldi r17, 0x41   ;r17 = B
        ldi r18, 0x21   ;r18 = C
        ldi r19, 0x01	;r19 = D

	ldi r23, 0x06

loop:

        mov r21, r17	;r21 = B
	com r21		;r21 = B'
	mov r22, r21	;r22 = B'
        and r21, r16    ;r21 = A * B'
	and r22, r19	;r22 = B' * D
	or r22, r21	;r22 = (B' * D + B' * A)
	com r22		;r22 = F0


	mov r20, r19	;r20 = D
	com r20		;r20 = D'
	or r20, r17	;r20 = D' + B
	mov r24, r18	;r24 = C
	com r24		;r24 = C'
	or r24,r16	;r24 = A + C'
	and r24, r20	;r24 = F1
	nop
        inc r16         ;A = A + 1
        ldi r27, 0x02
        add r17, r27	;B = B + 2

        ldi r27, 0x03
        add r18, r27	;C = C + 3

        ldi r27, 0x04
        add r19, r27	;D = D + 4

        dec r23
        brne loop


