
#ifndef _SDR_PDMA_H_
#define _SDR_PDMA_H_

#define PDMA_SOFT_RESET     0x20u
#define PDMA_IRQ_ENABLE_MASK 0x40u

void M2S_PDMA_init(void);

inline void M2S_PDMA_enable_irq(pdma_channel_id_t);

inline void M2S_PDMA_disable_irq(pdma_channel_id_t);

inline void M2S_PDMA_clear_irq(pdma_channel_id_t);

pdma_channel_id_t get_channel_id_from_status(unsigned short);

#endif
