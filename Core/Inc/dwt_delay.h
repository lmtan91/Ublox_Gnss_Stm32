#include <stdint.h>

#ifndef INC_DWT_DELAY_H_
#define INC_DWT_DELAY_H_

#define DWT_DELAY_NEWBIE 0

uint32_t DWT_Init(void);
/**
 * @brief This function provides a delay (in microseconds)
 * @param microseconds: delay in microseconds
 */
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
 uint32_t clk_cycle_start = DWT->CYCCNT;

/* Go to number of cycles for system */
 microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

/* Delay till end */
 while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void DWT_Delay(uint32_t us);
void delayMicroseconds(uint32_t us);

#endif /* INC_DWT_DELAY_DWT_DELAY_H_ */
