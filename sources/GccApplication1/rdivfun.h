/*
Russian peasant division
Implemented by Jose Gama, June 2015
*/

void rdiv8u(uint8_t *result, uint8_t *remainder, uint8_t dividend, uint8_t divisor)
{
asm volatile ( \
"	ser %A0  \n\t" \
"	tst %A3  \n\t" /* if divisor==0*/ \
"	breq 5f  \n\t" /* then return 255*/ \
"	cp %A2, %A3  \n\t" /* if dividend<divisor*/ \
"	brcc 0f  \n\t" \
"	clr %A0  \n\t" \
"	rjmp 5f \n\t" /* then return 0*/ \
"0:	clr r0  \n\t" \
"1:	inc r0  \n\t" \
"	bst %A3, 7  \n\t" /* to avoid an overflow*/ \
"	brts 2f  \n\t" /* Skip if bit 7 in divisor==1*/ \
"	lsl %A3  \n\t" \
"	cp %A2, %A3  \n\t" \
"	brcc 1b  \n\t" \
"	lsr %A3  \n\t" /* divisor=divisor/2*/ \
"2: clr %A0  \n\t" \
"3:	lsl %A0  \n\t" /* quotient=quotient*2*/ \
"	cp %A2, %A3  \n\t" \
"	brcs 4f  \n\t" \
"	inc %A0  \n\t" \
"	sub %A2, %A3  \n\t" /* dividend -= divisor */ \
"4:	lsr %A3  \n\t" /* divisor=divisor/2*/ \
"	dec r0  \n\t" \
"	brne 3b  \n\t" \
"5:	mov %A1, %A2 \n\t" /* last subtraction == remainder */ \
: "+a" (*result), "+a" (*remainder) \
: "a" (dividend),  "a" (divisor) \
);
}

void rdiv16u(uint16_t *result, uint16_t *remainder, uint16_t dividend, uint16_t divisor)
{
asm volatile ( \
"	ser %B0  \n\t" \
"	ser %A0  \n\t" \
"	cp %A3, r1  \n\t" \
"	cpc %B3, r1   \n\t" /* if divisor==0*/ \
"	breq 6f  \n\t" /* then return 0xFFFF*/ \
"	cp %A2, %A3  \n\t" /* if dividend<divisor*/ \
"	cpc %B2, %B3  \n\t" /* if dividend<divisor*/ \
"	brcc 0f	  \n\t" \
"	clr %B0  \n\t" \
"	clr %A0  \n\t" \
"	rjmp 6f \n\t" /* then return 0*/ \
"0: clr r0  \n\t" \
"1:	inc r0  \n\t" \
"	bst %B3, 7  \n\t" /* to avoid an overflow*/ \
"	brts 2f  \n\t" /* Skip if bit 15 in divisor==1*/ \
"	lsl %A3  \n\t" /* divisor=divisor*2*/ \
"	rol %B3  \n\t" \
"	cp %A2, %A3  \n\t" \
"	cpc %B2, %B3  \n\t" \
"	brcc 1b  \n\t" \
"	lsr %B3  \n\t" /* divisor=divisor/2*/ \
"	ror %A3	  \n\t" \
"2: clr %B0  \n\t" \
"   clr %A0  \n\t" \
"3:	lsl %A0  \n\t" /* quotient=quotient*2*/ \
"	rol %B0  \n\t" \
"	cp %A2, %A3  \n\t" \
"	cpc %B2, %B3  \n\t" \
"	brcs 5f  \n\t" \
"	inc %A0  \n\t" \
"	brne 4f  \n\t" \
"	inc %B0  \n\t" \
"4:	sub %A2, %A3  \n\t" /* dividend -= divisor*/ \
"	sbc %B2, %B3  \n\t" \
"5: lsr %B3  \n\t" /* divisor=divisor/2*/ \
"	ror %A3  \n\t" \
"	dec r0  \n\t" \
"	brne 3b  \n\t" \
"6: movw %A1, %A2  \n\t" /*  remainder = last subtraction*/ \
: "+a" (*result), "+a" (*remainder) \
: "a" (dividend),  "a" (divisor) \
);
}

void rdiv32u(uint32_t *result, uint32_t *remainder, uint32_t dividend, uint32_t divisor)
{
asm volatile ( \
"	and %D0, r1 \n\t" \
"	and %C0, r1  \n\t" \
"	and %B0, r1  \n\t" \
"	and %A0, r1  \n\t" \
"	cp %A3, r1  \n\t" \
"	cpc %B3, r1  \n\t" \
"	cpc %C3, r1  \n\t" \
"	cpc %D3, r1   \n\t" /* if divisor==0*/ \
"	breq 6f  \n\t" /* then return 0xFFFF*/ \
"	cp %A2, %A3  \n\t" /* if dividend<divisor*/ \
"	cpc %B2, %B3  \n\t" \
"	cpc %C2, %C3  \n\t" \
"	cpc %D2, %D3  \n\t" /* if dividend<divisor*/ \
"	brcc 0f	  \n\t" \
"	clr %A0  \n\t" \
"	clr %B0  \n\t" \
"	clr %C0  \n\t" \
"	clr %D0  \n\t" \
"	rjmp 6f \n\t" /* then return 0*/ \
"0: clr r0  \n\t" \
"1:	inc r0  \n\t" \
"	bst %D3, 7  \n\t" /* to avoid an overflow*/ \
"	brts 2f  \n\t" /* Skip if bit 15 in divisor==1*/ \
"	lsl %A3  \n\t" /* divisor=divisor*2*/ \
"	rol %B3  \n\t" \
"	rol %C3  \n\t" \
"	rol %D3  \n\t" \
"	cp %A2, %A3  \n\t" \
"	cpc %B2, %B3  \n\t" \
"	cpc %C2, %C3  \n\t" \
"	cpc %D2, %D3  \n\t" \
"	brcc 1b  \n\t" \
"	lsr %D3  \n\t" /* divisor=divisor/2*/ \
"	ror %C3	  \n\t" \
"	ror %B3	  \n\t" \
"	ror %A3	  \n\t" \
"2: clr %A0  \n\t" \
"   clr %B0  \n\t" \
"   clr %C0  \n\t" \
"   clr %D0  \n\t" \
"3:	lsl %A0  \n\t" /* quotient=quotient*2*/ \
"	rol %B0  \n\t" \
"	rol %C0  \n\t" \
"	rol %D0  \n\t" \
"	cp %A2, %A3  \n\t" \
"	cpc %B2, %B3  \n\t" \
"	cpc %C2, %C3  \n\t" \
"	cpc %D2, %D3  \n\t" \
"	brcs 5f  \n\t" \
"	inc %A0  \n\t" \
"	brne 4f  \n\t" \
"	inc %B0  \n\t" \
"	brne 4f  \n\t" \
"	inc %C0  \n\t" \
"	brne 4f  \n\t" \
"	inc %D0  \n\t" \
"4:	sub %A2, %A3  \n\t" /* dividend -= divisor*/ \
"	sbc %B2, %B3  \n\t" \
"	sbc %C2, %C3  \n\t" \
"	sbc %D2, %D3  \n\t" \
"5: lsr %D3  \n\t" /* divisor=divisor/2*/ \
"	ror %C3  \n\t" \
"	ror %B3  \n\t" \
"	ror %A3  \n\t" \
"	dec r0  \n\t" \
"	brne 3b  \n\t" \
"6: movw %A1, %A2  \n\t" /* remainder = last subtraction*/ \
"   movw %C1, %C2  \n\t" \
: "+l" (*result), "+l" (*remainder) \
: "r" (dividend),  "r" (divisor) \
);
}
