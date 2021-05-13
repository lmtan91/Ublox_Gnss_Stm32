/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "timer.h"
/**********************************************************************************************************************
 *  Internal macro definition
 *********************************************************************************************************************/
/* Macro to indicate the maximum time stamp then timestamp will be roll to 0 */
#define TIMER_MAXIMUM_TIME_STAMP 0xFFFFFFFFu

/**********************************************************************************************************************
 *  Global function definition
 *********************************************************************************************************************/
uint32_t timer_getCurrentTimeStampMs(void)
{
    return (HAL_GetTick());
}

uint32_t timer_getTimeElapsed(uint32_t TimeStamp)
{
    uint32_t time_tmp;
    time_tmp = HAL_GetTick();

    if (time_tmp >= TimeStamp)
    {
        return (time_tmp - TimeStamp);
    }
    else
    {
        return (TIMER_MAXIMUM_TIME_STAMP - TimeStamp + time_tmp);
    }
}
