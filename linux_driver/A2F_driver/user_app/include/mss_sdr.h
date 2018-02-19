
#ifndef _MSS_SDR_H_
#define _MSS_SDR_H_

#include "a2f/a2fm3.h"
#include "sdr_def.h"

#define GPIO_DRIVER_EN
#define TIMER2_DRIVER_EN
#define PDMA_DRIVER_EN

#ifdef GPIO_DRIVER_EN

#define MSS_GPIO_INPUT_MODE			0x0000000002UL
#define MSS_GPIO_OUTPUT_MODE		0x0000000005UL
#define MSS_GPIO_INOUT_MODE			0x0000000003UL

#define MSS_GPIO_IRQ_LEVEL_HIGH		0x0000000000UL
#define MSS_GPIO_IRQ_LEVEL_LOW		0x0000000020UL
#define MSS_GPIO_IRQ_EDGE_POSITIVE	0x0000000040UL
#define MSS_GPIO_IRQ_EDGE_NEGATIVE	0x0000000060UL
#define MSS_GPIO_IRQ_EDGE_BOTH		0x0000000080UL

void MSS_GPIO_config(MSS_GPIO_Type port_id, unsigned int config);

void MSS_GPIO_set_output(MSS_GPIO_Type port_id, unsigned int value);

inline void MSS_GPIO_set_outputs(unsigned int value);

inline unsigned int MSS_GPIO_get_input(MSS_GPIO_Type port_id);

inline unsigned int MSS_GPIO_get_inputs(void);

inline unsigned int MSS_GPIO_get_outputs(void);
#endif

#ifdef TIMER2_DRIVER_EN

inline void MSS_TIMER2_start(void);

inline void MSS_TIMER2_stop(void);

inline unsigned int MSS_TIMER2_get_current_value(void);

inline void MSS_TIMER2_load_immediate(unsigned int value);

inline void MSS_TIMER2_load_background(unsigned int value);
#endif

#ifdef PDMA_DRIVER_EN

typedef enum pdma_src_dest{
	PDMA_FROM_UART_0 	= 0,
	PDMA_TO_UART_0		= 1,
	PDMA_FROM_UART_1	= 2,
	PDMA_TO_UART_1		= 3,
	PDMA_FROM_SPI_0		= 4,
	PDMA_TO_SPI_0		= 5,
	PDMA_FROM_SPI_1		= 6,
	PDMA_TO_SPI_1		= 7,
	PDMA_FROM_FPGA_1	= 8,
	PDMA_TO_FPGA_1		= 9,
	PDMA_FROM_FPGA_0	= 10,
	PDMA_TO_FPGA_0		= 11,
	PDMA_TO_ACE			= 12,
	PDMA_FROM_ACE		= 13,
	PDMA_MEM_TO_MEM		= 14
} PDMA_SRC_DEST_Type;

typedef enum pdma_priority_ratio{
	PDMA_ROUND_ROBIN = 0,
	PDMA_RATIO_HIGH_LOW_1_TO_1 = 1,
	PDMA_RATIO_HIGH_LOW_3_TO_1 = 3,
	PDMA_RATIO_HIGH_LOW_7_TO_1 = 7,
	PDMA_RATIO_HIGH_LOW_15_TO_1 = 15,
	PDMA_RATIO_HIGH_LOW_31_TO_1 = 31,
	PDMA_RATIO_HIGH_LOW_63_TO_1 = 63,
	PDMA_RATIO_HIGH_LOW_127_TO_1 = 127,
	PDMA_RATIO_HIGH_LOW_255_TO_1 = 255
} PDMA_PRIORITY_RATIO_Type;

/***************************************************************************
 *	The pdma_channel_isr_t type is a pointer to a PDMA channel interrupt handler
 *	function. It specifies the function prototype of functions that can be
 *	registered as PDMA channel interrupt handlers. It is used as parameter to
 *	function PDMA_set_irq_handler().
 */
//typedef void (*pdma_channel_isr_t)( void );

#define PDMA_LOW_PRIORITY    		0x0000
#define PDMA_HIGH_PRIORITY   		0x0200

#define PDMA_BYTE_TRANSFER       	0x0000     /* Byte transfers (8 bits) */
#define PDMA_HALFWORD_TRANSFER   	0x0004     /* Half-word transfers (16 bits) */
#define PDMA_WORD_TRANSFER       	0x0008     /* Word transfers (32 bits) */

/***************************************************************************
 * 	These constants are used to build the channel_cfg parameter of the
 * 	PDMA_configure() function. They specify the PDMA channel’s source and
 *	destination address increment.
 */
#define PDMA_NO_INC  				0
#define PDMA_INC_SRC_ONE_BYTE    	0x0400
#define PDMA_INC_SRC_TWO_BYTES   	0x0800
#define PDMA_INC_SRC_FOUR_BYTES  	0x0C00
#define PDMA_INC_DEST_ONE_BYTE   	0x1000
#define PDMA_INC_DEST_TWO_BYTES  	0x2000
#define PDMA_INC_DEST_FOUR_BYTES 	0x3000

/***************************************************************************
 * Mask for various control register bits.
 */
#define PDMA_PAUSE_MASK         	0x00000010

/***************************************************************************
 *	These constants are used to specify the src_addr parameter to the PDMA_start()
 *	and PDMA_load_next_buffer() functions. They specify the receive register
 *	address of peripherals that can be the source of a DMA transfer. 
 *	When a PDMA channel is configured for DMA transfers from a peripheral to memory,
 *	the constant specifying that peripheral’s receive register address must be used
 *	as the src_addr parameter.
 */
#define PDMA_SPI0_RX_REGISTER       0x40001010uL
#define PDMA_SPI1_RX_REGISTER       0x40011010uL
#define PDMA_UART0_RX_REGISTER      0x40000000uL
#define PDMA_UART1_RX_REGISTER      0x40010000uL
#define PDMA_ACE_PPE_DATAOUT        0x40021308uL

/***************************************************************************
 *	These constants are used to specify the dest_addr parameter to the PDMA_start()
 *	and PDMA_load_next_buffer() functions. They specify the transmit register
 *	address of peripherals that can be the destination of a DMA transfer. 
 *	When a PDMA channel is configured for DMA transfers from memory to a peripheral,
 *	the constant specifying that peripheral’s transmit register address must be used
 *	as the dest_addr parameter.
 */
#define PDMA_SPI0_TX_REGISTER       0x40001014uL
#define PDMA_SPI1_TX_REGISTER       0x40011014uL
#define PDMA_UART0_TX_REGISTER      0x40000000uL
#define PDMA_UART1_TX_REGISTER      0x40010000uL
#define PDMA_ACE_SSE_DATAIN         0x40020700uL

/***************************************************************************
 *	The PDMA_DEFAULT_WRITE_ADJ constant provides a suitable default value for the
 *	PDMA_configure() function write_adjust parameter.
 */
#define PDMA_DEFAULT_WRITE_ADJ      10u

#define CHANNEL_RESET_MASK  		0x00000020

/***************************************************************************
 *	Look-up table use to derice a channel's control register value from the
 * 	requested source/destination. This table is incexed on the pdma_src_dest_t
 * 	enumeration.
 */

/***************************************************************************
 *	Offset of the posted writes WRITE_ADJ bits in a PDMA channel's configuration
 *	register.
 */
#define CHANNEL_N_POSTED_WRITE_ADJUST_SHIFT	14
#define CHANNEL_N_CTRL_PDMA_MASK        	0x00000001
#define CHANNEL_N_PERIPH_SELECT_SHIFT   	23
#define CHANNEL_N_DIRECTION_MASK			0x00000002

void PDMA_configure(PDMA_CHANNEL_ID_Type channel_id, PDMA_SRC_DEST_Type src_dest, unsigned int channel_cfg, unsigned char write_adjust);

/***************************************************************************//**
 *	See mss_pdma.h for description of this function.
 */
#define PAUSE_MASK  			0x00000010

#define BUFFER_B_SELECT_MASK	0x00000004


#define PORT_A_COMPLETE_MASK	0x00000001
#define PORT_B_COMPLETE_MASK	0x00000002

#define CHANNEL_STOPPED     	0U
#define CHANNEL_STARTED     	1U
#define NEXT_CHANNEL_A      	0U
#define NEXT_CHANNEL_B      	1U

void PDMA_init(void);

void PDMA_start(PDMA_CHANNEL_ID_Type channel_id, unsigned int src_addr, unsigned int dest_addr, unsigned short transfer_count);

void PDMA_load_next_buffer(PDMA_CHANNEL_ID_Type channel_id, unsigned int src_addr, unsigned int dest_addr, unsigned short transfer_count);

unsigned int PDMA_status(PDMA_CHANNEL_ID_Type  channel_id);

inline void PDMA_pause( PDMA_CHANNEL_ID_Type channel_id );

inline void PDMA_resume( PDMA_CHANNEL_ID_Type channel_id );


#endif


#endif

