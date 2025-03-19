#ifndef __IRQ_DECODER_H__
#define __IRQ_DECODER_H__

extern volatile uint32_t ir_decoder_last_ts;
extern volatile uint32_t ir_decoder_bit_count;
extern volatile uint32_t ir_decoder_command;
extern volatile uint32_t ir_decoder_command_ready;

int ir_decoder_irq(int ir_pin_state);

#endif // __IRQ_DECODER_H__
