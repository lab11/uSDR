
#include "mss_pdma.h"

static uint8_t g_pdma_next_channel[NB_OF_PDMA_CHANNELS];
static uint8_t g_pdma_started_a[NB_OF_PDMA_CHANNELS];
static uint8_t g_pdma_started_b[NB_OF_PDMA_CHANNELS];


void PDMA_init(void){
	// Previous initialization goes to Kernel space
    /* Initialize channels state information. */
	int i;
    for(i = 0; i < NB_OF_PDMA_CHANNELS; ++i)
    {
        g_pdma_next_channel[i] = NEXT_CHANNEL_A;
        g_pdma_started_a[i] = CHANNEL_STOPPED;
        g_pdma_started_b[i] = CHANNEL_STOPPED;
    }
}


void PDMA_configure
(
    pdma_channel_id_t channel_id,
    pdma_src_dest_t src_dest,
    uint32_t channel_cfg,
    uint8_t write_adjust
)
{
    const uint32_t src_dest_to_ctrl_reg_lut[] =
    {
        CHANNEL_N_CTRL_PDMA_MASK,                                                                               /* PDMA_FROM_UART_0 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)1 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_UART_0 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)2 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_UART_1 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)3 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_UART_1 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)4 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_SPI_0 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)5 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_SPI_0 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)6 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_SPI_1 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)7 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_SPI_1 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)8 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_FIC_0_DMAREADY_1 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)8 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_FIC_0_DMAREADY_1 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)9 << CHANNEL_N_PERIPH_SELECT_SHIFT),                             /* PDMA_FROM_FIC_0_DMAREADY_0 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)9 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK,  /* PDMA_TO_FIC_0_DMAREADY_0 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)10 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK, /* PDMA_TO_CAN */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)11 << CHANNEL_N_PERIPH_SELECT_SHIFT),                            /* PDMA_FROM_CAN */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)12 << CHANNEL_N_PERIPH_SELECT_SHIFT),                            /* PDMA_FROM_FIC_1_DMAREADY_1 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)12 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK, /* PDMA_TO_FIC_1_DMAREADY_1 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)13 << CHANNEL_N_PERIPH_SELECT_SHIFT),                            /* PDMA_FROM_FIC_1_DMAREADY_0 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)13 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK, /* PDMA_TO_FIC_1_DMAREADY_0 */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)14 << CHANNEL_N_PERIPH_SELECT_SHIFT),                            /* PDMA_FROM_COMBLK */
        CHANNEL_N_CTRL_PDMA_MASK | ( (uint32_t)15 << CHANNEL_N_PERIPH_SELECT_SHIFT) | CHANNEL_N_DIRECTION_MASK  /* PDMA_TO_COMBLK */
    };
    
    /* Reset the channel. */
    PDMA->CHANNEL[channel_id].CTRL |= CHANNEL_RESET_MASK;
    PDMA->CHANNEL[channel_id].CTRL &= ~CHANNEL_RESET_MASK;

    /* Configure PDMA channel's data source and destination. */
    if(src_dest != PDMA_MEM_TO_MEM)
    {
        PDMA->CHANNEL[channel_id].CTRL |= src_dest_to_ctrl_reg_lut[src_dest];
    }
    
    /* Configure PDMA channel trnasfer size, priority, source and destination address increment. */
    PDMA->CHANNEL[channel_id].CTRL |= channel_cfg;

    /* Posted write adjust. */
    PDMA->CHANNEL[channel_id].CTRL |= ((uint32_t)write_adjust << CHAN_POSTED_WRITE_ADJUST_SHIFT);
}

#define PAUSE_MASK                  0x00000010u

#define BUFFER_B_SELECT_MASK        0x00000004u

#define PORT_A_COMPLETE_MASK        0x00000001u
#define PORT_B_COMPLETE_MASK        0x00000002u

void PDMA_start
(
    pdma_channel_id_t channel_id,
    uint32_t src_addr,
    uint32_t dest_addr,
    uint16_t transfer_count
)
{
    /* Pause transfer. */
    PDMA->CHANNEL[channel_id].CTRL |= PAUSE_MASK;
    
    /* Clear complete transfers. */
    if(PDMA->CHANNEL[channel_id].STATUS & PORT_A_COMPLETE_MASK)
    {
        PDMA->CHANNEL[channel_id].CTRL |= CLEAR_PORT_A_DONE_MASK;
        g_pdma_started_a[channel_id] = CHANNEL_STOPPED;
    }
    if(PDMA->CHANNEL[channel_id].STATUS & PORT_B_COMPLETE_MASK)
    {
        PDMA->CHANNEL[channel_id].CTRL |= CLEAR_PORT_B_DONE_MASK;
        g_pdma_started_b[channel_id] = CHANNEL_STOPPED;
    }
    
    /* Load source, destination and transfer count. */
    if(PDMA->CHANNEL[channel_id].STATUS & BUFFER_B_SELECT_MASK)
    {
        g_pdma_next_channel[channel_id] = NEXT_CHANNEL_A;
        g_pdma_started_b[channel_id] = CHANNEL_STARTED;
        
        PDMA->CHANNEL[channel_id].BUFFER_B_SRC_ADDR = src_addr;
        PDMA->CHANNEL[channel_id].BUFFER_B_DEST_ADDR = dest_addr;
        PDMA->CHANNEL[channel_id].BUFFER_B_TRANSFER_COUNT = transfer_count;
    }
    else
    {
        g_pdma_next_channel[channel_id] = NEXT_CHANNEL_B;
        g_pdma_started_a[channel_id] = CHANNEL_STARTED;
        
        PDMA->CHANNEL[channel_id].BUFFER_A_SRC_ADDR = src_addr;
        PDMA->CHANNEL[channel_id].BUFFER_A_DEST_ADDR = dest_addr;
        PDMA->CHANNEL[channel_id].BUFFER_A_TRANSFER_COUNT = transfer_count;
    }
    
    /* Start transfer */
    PDMA->CHANNEL[channel_id].CTRL &= ~PAUSE_MASK;
}

uint32_t PDMA_status(pdma_channel_id_t channel_id){
	uint32_t status;
	status = PDMA->CHANNEL[channel_id].STATUS & (PORT_A_COMPLETE_MASK | PORT_B_COMPLETE_MASK);
	return status;
		
}
