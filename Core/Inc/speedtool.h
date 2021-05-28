

#ifndef INC_SPEEDTOOL_H_
#define INC_SPEEDTOOL_H_

#include <stdint.h>

typedef struct
{
	uint8_t 	hour_u8;
	uint8_t 	min_u8;
	uint8_t 	sec_u8;
	uint32_t 	milli_u32;
} Spd_Time_tst;

uint32_t SpdTool_DiffInSeconds(Spd_Time_tst time1_ptr, Spd_Time_tst time2_ptr);

#endif /* INC_SPEEDTOOL_H_ */
