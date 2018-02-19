
#ifndef _MSS_PDMA_H
#define _MSS_PDMA_H

#include <stdint.h>
#include "../include/sdr_def.h"


typedef enum{
	PDMA_FROM_UART_0 = 0,
	PDMA_TO_UART_0,
	PDMA_FROM_UART_1,
	PDMA_TO_UART_1,
	PDMA_FROM_SPI_0,
	PDMA_TO_SPI_0,
	PDMA_FROM_SPI_1,
	PDMA_TO_SPI_1,
	PDMA_FROM_FIC_0_DMAREADY_1,
	PDMA_TO_FIC_0_DMAREADY_1,
	PDMA_FROM_FIC_0_DMAREADY_0,
	PDMA_TO_FIC_0_DMAREADY_0,
	PDMA_TO_CAN,
	PDMA_FROM_CAN,
	PDMA_FROM_FIC_1_DMAREADY_1,
	PDMA_TO_FIC_1_DMAREADY_1,
	PDMA_FROM_FIC_1_DMAREADY_0,
	PDMA_TO_FIC_1_DMAREADY_0,
	PDMA_FROM_COMBLK,
	PDMA_TO_COMBLK,
	PDMA_MEM_TO_MEM
} pdma_src_dest_t;

typedef enum{
	PDMA_ROUND_ROBIN = 0,
	PDMA_RATIO_HIGH_LOW_1_TO_1 = 1,
	PDMA_RATIO_HIGH_LOW_3_TO_1 = 3,
	PDMA_RATIO_HIGH_LOW_7_TO_1 = 7,
	PDMA_RATIO_HIGH_LOW_15_TO_1 = 15,
	PDMA_RATIO_HIGH_LOW_31_TO_1 = 31,
	PDMA_RATIO_HIGH_LOW_63_TO_1 = 63,
	PDMA_RATIO_HIGH_LOW_127_TO_1 = 127,
	PDMA_RATIO_HIGH_LOW_255_TO_1 = 255
} pdma_priority_ratio_t;

#define PDMA_LOW_PRIORITY    0x0000
#define PDMA_HIGH_PRIORITY   0x0200

#define PDMA_BYTE_TRANSFER       0x0000     /* Byte transfers (8 bits) */
#define PDMA_HALFWORD_TRANSFER   0x0004     /* Half-word transfers (16 bits) */
#define PDMA_WORD_TRANSFER       0x0008     /* Word transfers (32 bits) */

#define PDMA_NO_INC  0
#define PDMA_INC_SRC_ONE_BYTE    0x0400
#define PDMA_INC_SRC_TWO_BYTES   0x0800
#define PDMA_INC_SRC_FOUR_BYTES  0x0C00
#define PDMA_INC_DEST_ONE_BYTE   0x1000
#define PDMA_INC_DEST_TWO_BYTES  0x2000
#define PDMA_INC_DEST_FOUR_BYTES 0x3000

#define PDMA_IRQ_ENABLE_MASK    (uint32_t)0x00000040
#define PDMA_PAUSE_MASK         (uint32_t)0x00000010

/***************************************************************************//**
  These constants are used to specify the src_addr parameter to the PDMA_start()
  and PDMA_load_next_buffer() functions. They specify the receive register
  address of peripherals that can be the source of a DMA transfer. 
  When a PDMA channel is configured for DMA transfers from a peripheral to memory,
  the constant specifying that peripherals receive register address must be used
  as the src_addr parameter.
 */
#define PDMA_SPI0_RX_REGISTER       0x40001010uL
#define PDMA_SPI1_RX_REGISTER       0x40011010uL

#define PDMA_UART0_RX_REGISTER      0x40000000uL
#define PDMA_UART1_RX_REGISTER      0x40010000uL

#define PDMA_CAN_RX_MSG_0_BUFFER    0x40015220uL
#define PDMA_CAN_RX_MSG_1_BUFFER    0x40015240uL
#define PDMA_CAN_RX_MSG_2_BUFFER    0x40015260uL
#define PDMA_CAN_RX_MSG_3_BUFFER    0x40015280uL
#define PDMA_CAN_RX_MSG_4_BUFFER    0x400152A0uL
#define PDMA_CAN_RX_MSG_5_BUFFER    0x400152C0uL
#define PDMA_CAN_RX_MSG_6_BUFFER    0x400152E0uL
#define PDMA_CAN_RX_MSG_7_BUFFER    0x40015300uL
#define PDMA_CAN_RX_MSG_8_BUFFER    0x40015320uL
#define PDMA_CAN_RX_MSG_9_BUFFER    0x40015340uL
#define PDMA_CAN_RX_MSG_10_BUFFER   0x40015360uL
#define PDMA_CAN_RX_MSG_11_BUFFER   0x40015380uL
#define PDMA_CAN_RX_MSG_12_BUFFER   0x400153A0uL
#define PDMA_CAN_RX_MSG_13_BUFFER   0x400153C0uL
#define PDMA_CAN_RX_MSG_14_BUFFER   0x400153E0uL
#define PDMA_CAN_RX_MSG_15_BUFFER   0x40015400uL
#define PDMA_CAN_RX_MSG_16_BUFFER   0x40015420uL
#define PDMA_CAN_RX_MSG_17_BUFFER   0x40015440uL
#define PDMA_CAN_RX_MSG_18_BUFFER   0x40015460uL
#define PDMA_CAN_RX_MSG_19_BUFFER   0x40015480uL
#define PDMA_CAN_RX_MSG_20_BUFFER   0x400154A0uL
#define PDMA_CAN_RX_MSG_21_BUFFER   0x400154C0uL
#define PDMA_CAN_RX_MSG_22_BUFFER   0x400154E0uL
#define PDMA_CAN_RX_MSG_23_BUFFER   0x40015500uL
#define PDMA_CAN_RX_MSG_24_BUFFER   0x40015520uL
#define PDMA_CAN_RX_MSG_25_BUFFER   0x40015540uL
#define PDMA_CAN_RX_MSG_26_BUFFER   0x40015560uL
#define PDMA_CAN_RX_MSG_27_BUFFER   0x40015580uL
#define PDMA_CAN_RX_MSG_28_BUFFER   0x400155A0uL
#define PDMA_CAN_RX_MSG_29_BUFFER   0x400155C0uL
#define PDMA_CAN_RX_MSG_30_BUFFER   0x400155E0uL
#define PDMA_CAN_RX_MSG_31_BUFFER   0x40015600uL

/***************************************************************************//**
  These constants are used to specify the dest_addr parameter to the PDMA_start()
  and PDMA_load_next_buffer() functions. They specify the transmit register
  address of peripherals that can be the destination of a DMA transfer. 
  When a PDMA channel is configured for DMA transfers from memory to a peripheral,
  the constant specifying that peripherals transmit register address must be used
  as the dest_addr parameter.
 */
#define PDMA_SPI0_TX_REGISTER       0x40001014uL
#define PDMA_SPI1_TX_REGISTER       0x40011014uL

#define PDMA_UART0_TX_REGISTER      0x40000000uL
#define PDMA_UART1_TX_REGISTER      0x40010000uL

#define PDMA_CAN_TX_MSG_0_BUFFER    0x40015020uL
#define PDMA_CAN_TX_MSG_1_BUFFER    0x40015030uL
#define PDMA_CAN_TX_MSG_2_BUFFER    0x40015040uL
#define PDMA_CAN_TX_MSG_3_BUFFER    0x40015050uL
#define PDMA_CAN_TX_MSG_4_BUFFER    0x40015060uL
#define PDMA_CAN_TX_MSG_5_BUFFER    0x40015070uL
#define PDMA_CAN_TX_MSG_6_BUFFER    0x40015080uL
#define PDMA_CAN_TX_MSG_7_BUFFER    0x40015090uL
#define PDMA_CAN_TX_MSG_8_BUFFER    0x400150A0uL
#define PDMA_CAN_TX_MSG_9_BUFFER    0x400150B0uL
#define PDMA_CAN_TX_MSG_10_BUFFER   0x400150C0uL
#define PDMA_CAN_TX_MSG_11_BUFFER   0x400150D0uL
#define PDMA_CAN_TX_MSG_12_BUFFER   0x400150E0uL
#define PDMA_CAN_TX_MSG_13_BUFFER   0x400150F0uL
#define PDMA_CAN_TX_MSG_14_BUFFER   0x40015100uL
#define PDMA_CAN_TX_MSG_15_BUFFER   0x40015110uL
#define PDMA_CAN_TX_MSG_16_BUFFER   0x40015120uL
#define PDMA_CAN_TX_MSG_17_BUFFER   0x40015130uL
#define PDMA_CAN_TX_MSG_18_BUFFER   0x40015140uL
#define PDMA_CAN_TX_MSG_19_BUFFER   0x40015150uL
#define PDMA_CAN_TX_MSG_20_BUFFER   0x40015160uL
#define PDMA_CAN_TX_MSG_21_BUFFER   0x40015170uL
#define PDMA_CAN_TX_MSG_22_BUFFER   0x40015180uL
#define PDMA_CAN_TX_MSG_23_BUFFER   0x40015190uL
#define PDMA_CAN_TX_MSG_24_BUFFER   0x400151A0uL
#define PDMA_CAN_TX_MSG_25_BUFFER   0x400151B0uL
#define PDMA_CAN_TX_MSG_26_BUFFER   0x400151C0uL
#define PDMA_CAN_TX_MSG_27_BUFFER   0x400151D0uL
#define PDMA_CAN_TX_MSG_28_BUFFER   0x400151E0uL
#define PDMA_CAN_TX_MSG_29_BUFFER   0x400151F0uL
#define PDMA_CAN_TX_MSG_30_BUFFER   0x40015200uL
#define PDMA_CAN_TX_MSG_31_BUFFER   0x40015210uL

/***************************************************************************//**
  These constants are used to specify the src_addr or dest_addr parameter to the
  PDMA_start() and PDMA_load_next_buffer() functions. They specify the register
  address of peripherals that can be both the source and the destination of a
  DMA transfer. 
 */
#define PDMA_COMBLK_DATA8_REGISTER  0x40016010uL
#define PDMA_COMBLK_DATA32_REGISTER 0x40016014uL
 
/***************************************************************************//**
  The PDMA_DEFAULT_WRITE_ADJ constant provides a suitable default value for the
  PDMA_configure() function write_adjust parameter.
 */
#define PDMA_DEFAULT_WRITE_ADJ      10u

#define CHANNEL_RESET_MASK			0x00000020u
void PDMA_configure
(
    pdma_channel_id_t channel_id,
    pdma_src_dest_t src_dest,
    uint32_t channel_cfg,
    uint8_t write_adjust
);

void PDMA_start
(
    pdma_channel_id_t channel_id,
    uint32_t src_addr,
    uint32_t dest_addr,
    uint16_t transfer_count
);

uint32_t PDMA_status(pdma_channel_id_t channel_id);


void PDMA_init(void);

/***************************************************************************//**
  Offset of the posted writes WRITE_ADJ bits in a PDMA channel's configuration
  register.
 */
#define CHAN_POSTED_WRITE_ADJUST_SHIFT   14

/*-------------------------------------------------------------------------*//**
 * Look-up table use to derice a channel's control register value from the
 * requested source/destination. This table is incexed on the pdma_src_dest_t
 * enumeration.
 */
#define CHANNEL_N_CTRL_PDMA_MASK        0x00000001u
#define CHANNEL_N_PERIPH_SELECT_SHIFT   23u
#define CHANNEL_N_DIRECTION_MASK        0x00000002u

#define NEXT_CHANNEL_A      0U
#define NEXT_CHANNEL_B      1U

#define CHANNEL_STOPPED     0U
#define CHANNEL_STARTED     1U

#endif
