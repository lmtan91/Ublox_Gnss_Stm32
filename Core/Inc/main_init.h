#ifndef INC_MAIN_INIT_H_
#define INC_MAIN_INIT_H_
#include <ublox_gnss.h>

#ifdef __cplusplus
extern "C" {
#endif

extern Ublox_Gnss_tst ubx_st;

void Main_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAIN_INIT_H_ */
