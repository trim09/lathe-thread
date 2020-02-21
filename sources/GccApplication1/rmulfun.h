/*
Russian peasant multiplication
Implemented by Jose Gama, June 2015
*/

uint32_t rmul16u(uint16_t multiplicand, uint16_t multiplier) \
{
uint32_t result, multiplier2;
multiplier2 = multiplier;  
asm volatile ( \
"	clr %D0 \n\t" \
"	clr %C0 \n\t" \
"	clr %B0 \n\t" \
"	clr %A0 \n\t" \
"	cp %A1, r1  \n\t" /* if multiplicand==0*/ \
"	cpc %B1, r1 \n\t" \
"	breq 2f  \n\t" /* then return 0	*/ \
"	cp %A2, r1  \n\t" /* if multiplier==0*/ \
"	cpc %B2, r1 \n\t" \
"	breq 2f  \n\t" /* then return 0*/ \
"	bst %A1, 0 \n\t" \
"	brtc 0f  \n\t" /* if the multiplicand is odd then add the multiplier*/ \
"	add %A0, %A2 \n\t" \
"	adc %B0, %B2	 \n\t" \
"	adc %C0, %C2	 \n\t" \
"	adc %D0, %D2	 \n\t" \
"0:	 \n\t" \
"	lsl %A2   \n\t" /* multiplier=multiplier*2*/ \
"	rol %B2 \n\t" \
"	rol %C2 \n\t" \
"	rol %D2 \n\t" \
"	lsr %B1  \n\t" /* multiplicand=multiplicand/2*/ \
"	ror %A1 \n\t" \
"	bst %A1, 0 \n\t" \
"	brtc 1f  \n\t" /* if the multiplicand is odd then add the multiplier*/ \
"	add %A0, %A2 \n\t" \
"	adc %B0, %B2	 \n\t" \
"	adc %C0, %C2	 \n\t" \
"	adc %D0, %D2 \n\t" \
"1:	cpi %A1, 1 \n\t" \
"	cpc %B1, r1 \n\t" \
"	breq 2f  \n\t" /*  stop when multiplicand == 1*/ \
"	rjmp 0b \n\t" \
"2: \n\t" \
: "=&r" (result) \
: "r" (multiplicand),  "r" (multiplier2) \
);
return result;
}

uint16_t rmul8u(uint8_t multiplicand, uint8_t multiplier) \
{
uint16_t result, multiplier2;
multiplier2 = multiplier;   
asm volatile ( \
"	clr %B0  \n\t" \
"	clr %A0  \n\t" \
"	cp %A1, r1  \n\t" /* if multiplicand==0*/ \
"	breq 2f  \n\t" /* then return 0*/ \
"	cp %A2, r1  \n\t" /* if multiplier==0*/ \
"	breq 2f  \n\t" /* then return 0*/ \
"	bst %A1, 0 \n\t" \
"	brtc 0f  \n\t" /* if the multiplicand is odd then add the multiplier*/ \
"	add %A0, %A2 \n\t" \
"	adc %B0, %B2 \n\t" \
"0:	 \n\t" \
"	lsl %A2   \n\t" /* multiplier=multiplier*2*/ \
"	rol %B2 \n\t" \
"	lsr %A1  \n\t" /* multiplicand=multiplicand/2*/ \
"	bst %A1, 0 \n\t" \
"	brtc 1f  \n\t" /* if the multiplicand is odd then add the multiplier*/ \
"	add %A0, %A2 \n\t" \
"	adc %B0, %B2 \n\t" \
"1:	cpi %A1, 2 \n\t" \
"	brlo 2f  \n\t" /*  stop when multiplicand <= 1*/ \
"	rjmp 0b \n\t" \
"2: \n\t" \
: "=&a" (result) \
: "a" (multiplicand),  "a" (multiplier2) \
);
return result;
}
