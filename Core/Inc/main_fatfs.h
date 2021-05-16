
#ifndef INC_MAIN_FATFS_H_
#define INC_MAIN_FATFS_H_

#include <stdarg.h>

extern FIL g_fil_st;
extern FRESULT g_fres_st;

void Fatfs_Init(void);
void Fatfs_Open(void);
void Fatfs_Close(void);
void Fatfs_Printf(const char *fmt, ...);
void vprint1(const char *fmt, va_list argp);

#endif /* INC_MAIN_FATFS_H_ */
