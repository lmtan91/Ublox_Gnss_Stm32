#ifndef TIMER_H_
#define TIMER_H_

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "stm32f1xx_hal.h"
/**********************************************************************************************************************
 *  GLOBAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/
/**
 * @brief Get the system tick, unit is millisecond
 * @return Current system tick in ms
 */
uint32_t timer_getCurrentTimeStampMs(void);

/**
 * @brief Get the time elapsed from a specific timestamp, unit is millisecond
 * @param TimeStamp A reference to a timestamp
 * @return Elapsed time in ms
 */
uint32_t timer_getTimeElapsed(uint32_t TimeStamp);

#endif /* TIMER_H_ */
