
#include "mss_sdr.h"

#ifdef GPIO_DRIVER_EN
void MSS_GPIO_config(MSS_GPIO_Type port_id, unsigned int config){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO)
		MSS_GPIO->cfg[gpio_idx] = config;
}

void MSS_GPIO_set_output(MSS_GPIO_Type port_id, unsigned int value){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO)
		MSS_GPIO_BIT_BAND->gpio_out[gpio_idx] = value;
}

inline void MSS_GPIO_set_outputs(unsigned int value){
	MSS_GPIO->out = value;
}

inline unsigned int MSS_GPIO_get_input(MSS_GPIO_Type port_id){
	unsigned int gpio_idx = (unsigned int)port_id;
	if (gpio_idx < NB_OF_GPIO)
		return MSS_GPIO_BIT_BAND->gpio_in[gpio_idx];
	else
		return 0;
}

inline unsigned int MSS_GPIO_get_inputs(void){
	return MSS_GPIO->in;
}

inline unsigned int MSS_GPIO_get_outputs(void){
	return MSS_GPIO->out;
}
#endif

#ifdef TIMER2_DRIVER_EN
inline void MSS_TIMER2_start(void){
	MSS_TIMER2_BIT_BAND->tim2enable = 1;
}

inline void MSS_TIMER2_stop(void){
	MSS_TIMER2_BIT_BAND->tim2enable = 0;
}

inline unsigned int MSS_TIMER2_get_current_value(void){
	return MSS_TIMER2->tim2_val;
}

inline void MSS_TIMER2_load_immediate(unsigned int value){
	MSS_TIMER2->tim2_loadval = value;
}

inline void MSS_TIMER2_load_background(unsigned int value){
	MSS_TIMER2->tim2_bgloadval = value;
}
#endif

#ifdef PDMA_DRIVER_EN
const unsigned int src_dest_to_ctrl_reg_lut[] ={
	CHANNEL_N_CTRL_PDMA_MASK,                                                                               /* PDMA_FROM_UART_0 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)1 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_UART_0 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)2 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_UART_1 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)3 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_UART_1 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)4 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_SPI_0 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)5 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_SPI_0 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)6 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_SPI_1 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)7 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_SPI_1 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)8 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_FPGA_1 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)8 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_FPGA_1 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)9 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_FPGA_0 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)9 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_FPGA_0 */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)10 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK, /* PDMA_TO_ACE */
	CHANNEL_N_CTRL_PDMA_MASK | ( (unsigned int)11 << CHANNEL_N_PERIPH_SELECT_SHIFT)                             /* PDMA_FROM_ACE */
};

void PDMA_configure(PDMA_CHANNEL_ID_Type channel_id, PDMA_SRC_DEST_Type src_dest, unsigned int channel_cfg, unsigned char write_adjust){
	/* Reset the channel. */
	MSS_PDMA->CHANNEL[channel_id].CRTL |= CHANNEL_RESET_MASK;
	MSS_PDMA->CHANNEL[channel_id].CRTL &= ~CHANNEL_RESET_MASK;

	/* Configure PDMA channel's data source and destination. */
	if ( src_dest != PDMA_MEM_TO_MEM )
		MSS_PDMA->CHANNEL[channel_id].CRTL |= src_dest_to_ctrl_reg_lut[src_dest];

	/* Configure PDMA channel trnasfer size, priority, source and destination address increment. */
	MSS_PDMA->CHANNEL[channel_id].CRTL |= channel_cfg;
	/* Posted write adjust. */
	MSS_PDMA->CHANNEL[channel_id].CRTL |= ((unsigned int)write_adjust << CHANNEL_N_POSTED_WRITE_ADJUST_SHIFT);
}


unsigned char g_pdma_next_channel[NB_OF_PDMA_CHANNELS];
unsigned char g_pdma_started_a[NB_OF_PDMA_CHANNELS];
unsigned char g_pdma_started_b[NB_OF_PDMA_CHANNELS];


void PDMA_init(void){
	// part of initialization is done in kernal space
	int i;
	for (i=0; i<NB_OF_PDMA_CHANNELS; ++i){
		g_pdma_next_channel[i] = NEXT_CHANNEL_A;
		g_pdma_started_a[i] = CHANNEL_STOPPED;
		g_pdma_started_b[i] = CHANNEL_STOPPED;
		//g_pdma_isr_table[i] = 0;
	}
}

void PDMA_start(PDMA_CHANNEL_ID_Type channel_id, unsigned int src_addr, unsigned int dest_addr, unsigned short transfer_count){
	/* Pause transfer. */
	MSS_PDMA->CHANNEL[channel_id].CRTL |= PAUSE_MASK;

	/* Clear complete transfers. */
	if (MSS_PDMA->CHANNEL[channel_id].STATUS & PORT_A_COMPLETE_MASK){
		MSS_PDMA->CHANNEL[channel_id].CRTL |= CLEAR_PORT_A_DONE_MASK;
		g_pdma_started_a[channel_id] = CHANNEL_STOPPED;
	}

	if (MSS_PDMA->CHANNEL[channel_id].STATUS & PORT_B_COMPLETE_MASK){
		MSS_PDMA->CHANNEL[channel_id].CRTL |= CLEAR_PORT_B_DONE_MASK;
		g_pdma_started_b[channel_id] = CHANNEL_STOPPED;
	}

	/* Load source, destination and transfer count. */
	if (MSS_PDMA->CHANNEL[channel_id].STATUS & BUFFER_B_SELECT_MASK){
		g_pdma_next_channel[channel_id] = NEXT_CHANNEL_A;
		g_pdma_started_b[channel_id] = CHANNEL_STARTED;

		MSS_PDMA->CHANNEL[channel_id].BUFFER_B_SRC_ADDR = src_addr;
		MSS_PDMA->CHANNEL[channel_id].BUFFER_B_DEST_ADDR = dest_addr;
		MSS_PDMA->CHANNEL[channel_id].BUFFER_B_TRANSFER_COUNT = transfer_count;
	}
	else{
		g_pdma_next_channel[channel_id] = NEXT_CHANNEL_B;
		g_pdma_started_a[channel_id] = CHANNEL_STARTED;

		MSS_PDMA->CHANNEL[channel_id].BUFFER_A_SRC_ADDR = src_addr;
		MSS_PDMA->CHANNEL[channel_id].BUFFER_A_DEST_ADDR = dest_addr;
		MSS_PDMA->CHANNEL[channel_id].BUFFER_A_TRANSFER_COUNT = transfer_count;
	}

	/* Start transfer */
	MSS_PDMA->CHANNEL[channel_id].CRTL &= ~PAUSE_MASK;
}

void PDMA_load_next_buffer(PDMA_CHANNEL_ID_Type channel_id, unsigned int src_addr, unsigned int dest_addr, unsigned short transfer_count){
	if (NEXT_CHANNEL_A == g_pdma_next_channel[channel_id]){
		/* Wait for channel A current transfer completion. */
		if (CHANNEL_STARTED == g_pdma_started_a[channel_id]){
			unsigned int completed;
			unsigned int channel_mask;
			channel_mask = (unsigned int)1 << ((unsigned int)channel_id * 2U);
			do {
				completed = MSS_PDMA->BUFFER_STATUS & channel_mask;
			}
			while( !completed );
			MSS_PDMA->CHANNEL[channel_id].CRTL |= CLEAR_PORT_A_DONE_MASK;
		}

		/* Load source, destination and transfer count. */
		MSS_PDMA->CHANNEL[channel_id].BUFFER_A_SRC_ADDR = src_addr;
		MSS_PDMA->CHANNEL[channel_id].BUFFER_A_DEST_ADDR = dest_addr;
		MSS_PDMA->CHANNEL[channel_id].BUFFER_A_TRANSFER_COUNT = transfer_count;

		/* Update channel state information. */
		g_pdma_next_channel[channel_id] = NEXT_CHANNEL_B;
		g_pdma_started_a[channel_id] = CHANNEL_STARTED;
	}
	else{
		/* Wait for channel B current transfer completion. */
		if ( CHANNEL_STARTED == g_pdma_started_b[channel_id]){

			unsigned int completed;
			unsigned int channel_mask;
			channel_mask = (unsigned int)1 << (((unsigned int)channel_id * 2U) + 1U);
			do {
				completed = MSS_PDMA->BUFFER_STATUS & channel_mask;
			}
			while( !completed );
			MSS_PDMA->CHANNEL[channel_id].CRTL |= CLEAR_PORT_B_DONE_MASK;
		} 

		/* Load source, destination and transfer count. */
		MSS_PDMA->CHANNEL[channel_id].BUFFER_B_SRC_ADDR = src_addr;
		MSS_PDMA->CHANNEL[channel_id].BUFFER_B_DEST_ADDR = dest_addr;
		MSS_PDMA->CHANNEL[channel_id].BUFFER_B_TRANSFER_COUNT = transfer_count;

		/* Update channel state information. */
		g_pdma_next_channel[channel_id] = NEXT_CHANNEL_A;
		g_pdma_started_b[channel_id] = CHANNEL_STARTED;
	}
} 

unsigned int PDMA_status(PDMA_CHANNEL_ID_Type  channel_id){
	unsigned int status;
	status = MSS_PDMA->CHANNEL[channel_id].STATUS & (PORT_A_COMPLETE_MASK | PORT_B_COMPLETE_MASK);
	return status;
}

inline void PDMA_pause( PDMA_CHANNEL_ID_Type channel_id ){
    MSS_PDMA->CHANNEL[channel_id].CRTL |= PDMA_PAUSE_MASK;
}

inline void PDMA_resume( PDMA_CHANNEL_ID_Type channel_id ){
    MSS_PDMA->CHANNEL[channel_id].CRTL &= ~PDMA_PAUSE_MASK;
}

#endif
