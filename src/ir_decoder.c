#include <stdint.h>
#include "xprintf.h"

#include "mik32_hal_scr1_timer.h"

#define	NEC_BIT_START	(45 * OSC_SYSTEM_VALUE / 10000)	// 4500 us - Start-bit duration 
#define	NEC_BIT_ZERO	( 6 * OSC_SYSTEM_VALUE / 10000)	//  600 us - Zero duration 
#define	NEC_BIT_ONE	(15 * OSC_SYSTEM_VALUE / 10000)	// 1500 us - One duration 
#define	NEC_BIT_TOL	( 2 * OSC_SYSTEM_VALUE / 10000)	//  200 us - Tolerance 

volatile uint32_t ir_decoder_last_ts = 0;
volatile uint32_t ir_decoder_bit_count = 0;
volatile uint32_t ir_decoder_command = 0;
volatile uint32_t ir_decoder_command_ready = 0;

int ir_decoder_irq(int ir_pin_state)
{

	if(ir_pin_state == 1) { // Rising (0->1)

		ir_decoder_last_ts = SCR1_TIMER->MTIME;

	} else { // Falling (1->0)
	
		volatile uint32_t ts = SCR1_TIMER->MTIME;
		uint32_t pulse = (ts > ir_decoder_last_ts) ? (ts - ir_decoder_last_ts) : 0xffffffff;

		ir_decoder_last_ts = ts;

		//xprintf("\r\nIRQ_IR: pulse = %u\r\n", pulse);

		if(pulse > NEC_BIT_START - NEC_BIT_TOL && pulse < NEC_BIT_START + NEC_BIT_TOL) {

			ir_decoder_bit_count = 0;	
			ir_decoder_command_ready = 0;	
			ir_decoder_command = 0;	

		} else if(pulse > NEC_BIT_ZERO - NEC_BIT_TOL && pulse < NEC_BIT_ZERO + NEC_BIT_TOL) {

			ir_decoder_command = (ir_decoder_command >> 1) | 0; // shift in Zero bit
			ir_decoder_bit_count++;	

		} else if(pulse > NEC_BIT_ONE - NEC_BIT_TOL && pulse < NEC_BIT_ONE + NEC_BIT_TOL) {

			ir_decoder_command = (ir_decoder_command >> 1) | 0x80000000; // shift in One bit
			ir_decoder_bit_count++;	
		}

		if(ir_decoder_bit_count == 32) {
			ir_decoder_command_ready = 1;	
			xprintf("\r\nnIR_DECODER_IRQ: Ready, cmd = 0x%08X\r\n", ir_decoder_command);
		}
	}

	return 0;
}
