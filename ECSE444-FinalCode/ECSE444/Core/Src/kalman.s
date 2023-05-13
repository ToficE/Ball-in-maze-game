//.section .data
.syntax unified //as.pdf : p141
.align 16 //as.pdf :p71
.section .text, "x" //as.pdf :p96
//.rodata
.global kalman //as.pdf : p254

// Kalman subroutine
// R0 - base address of struct (q, r, x, p, k)
// S0 - measurement value
kalman: //label
	// push local variables used on stack

	//enable FPU

	PUSH {R1-R3,LR} // fixed-point push

	LDR   R2, =0xE000ED88      	  // load CPACR
	LDR     R3, [R2]              // retrieve CPACR value
	ORR     R3, R3, #(0xF << 20)  // Set bits 20-23 to enable CP10 and CP11 coprocessors for FP operations
	STR     R3, [R2]              // Write back to the CPACR
	DSB                           // data sync barrier (wait for all memory accesses)
	ISB                           // instruction sync barrier (wait for all instructions)

	VPUSH {S1-S6} // floating-point push

	MOV R1, #1
	VMSR FPSCR, R1 // clear FPSCR by writing to it

	// load q, r, x, p, and k from memory
	VLDR.32 S1, [R0] // q
	VLDR.32 S2, [R0, #4] // r
	VLDR.32 S3, [R0, #8] // x
	VLDR.32 S4, [R0, #12] // p
	//VLDR.32 S5, [R0, #16] // k (no need to load?)

	// arithmic
	VADD.F32 S4, S4, S1 // p = p + q
	VADD.F32 S6, S4, S2 // den = p + r
	VDIV.F32 S5, S4, S6 // k = p/den (need to store k)
	VSUB.F32 S0, S0, S3 // mult = measurement - x
	VMUL.F32 S6, S5, S0 // addr = k*mult
	VADD.F32 S3, S6, S3 // x = addr + x

	// p = (1-k)*p
	// p = p - k*p
	VMUL.F32 S6, S5, S4 // mul = k*p
	VSUB.F32 S4, S4, S6 // p = p - mul

	// store variables
	VSTR S3, [R0, #8] // store x
	VSTR S4, [R0, #12] // store p
	VSTR S5, [R0, #16] // store k

	// check division by zero, overflow and underflow
	VMRS R1, FPSCR 	//Copy FPSCR to R1
	TST R1, #0x2 // test the DZC (Division by zero) bit
	BNE ERROR // bracnh to error if division by zero has happened
	TST  R1, #0x4  	// test the OFC (overflow cumulative) bit in FPSCR
	BNE ERROR 	// branch to error if overflow has happened
	TST R1, #0x8 // test the UFC (undeflow cumulative) bit in FPSCR
	BNE ERROR // branch to error if underflow has happened

	// We can reduce the above instruction to:
	// This TST instruction checks if any of the flag bits are set to 1.
	TST R1, #14 // bin(0...01110) = dec(14)
	BNE ERROR

	// These two instructions are reach iff no division by zero,
	// underflow or overflow occured.
	MOV R0, #0 // If the function returns 0 in the C code,
			   // then no exception has occured.
	B EXIT

	ERROR:
	MOV R0, #-1 // return -1. If the function returns -1 in the C code,
				// then an exception must have occured.

	EXIT:
	// retrieve local variables from stack used in subroutine
	VPOP {S1-S6} // floating-point pop
	POP {R1-R3,LR} // fixed-point pop
	BX LR // exit subroutine





