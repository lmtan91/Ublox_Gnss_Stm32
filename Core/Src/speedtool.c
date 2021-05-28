
#include "speedtool.h"

uint32_t SpdTool_DiffInSeconds(Spd_Time_tst time1_ptr, Spd_Time_tst time2_ptr)
{
	uint32_t seconds;

	seconds = (time2_ptr.hour_u8 * 3600 + time2_ptr.min_u8 * 60 + time2_ptr.sec_u8 + time2_ptr.milli_u32/1000)
			- (time1_ptr.hour_u8 * 3600 + time1_ptr.min_u8 * 60 + time1_ptr.sec_u8 + time1_ptr.milli_u32/1000);
	return seconds;
}
