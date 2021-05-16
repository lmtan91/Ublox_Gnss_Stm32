#include "fatfs.h"
#include "main_fatfs.h"

#include <stdbool.h>

FIL g_fil_st;
FRESULT g_fres_st;
FATFS FatFs;

bool g_isOpen_b = false;
void Fatfs_Init(void)
{
    g_fres_st = f_mount(&FatFs, "", 1); //1=mount now
    if (g_fres_st != FR_OK)
    {
        my_printf("f_mount error (%i)\r\n", g_fres_st);
        while(1);
    }
    //Let's get some statistics from the SD card
    DWORD free_clusters, free_sectors, total_sectors;
    FATFS* getFreeFs;

    g_fres_st = f_getfree("", &free_clusters, &getFreeFs);
    if (g_fres_st != FR_OK)
    {
        my_printf("f_getfree error (%i)\r\n", g_fres_st);
        while(1);
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    my_printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    Fatfs_Open();
}

void Fatfs_Open(void)
{
    g_fres_st = f_open(&g_fil_st, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
    if(g_fres_st == FR_OK)
    {
        g_isOpen_b = true;
        my_printf("I was able to open 'write.txt' for writing\r\n");
    }
    else
    {
      my_printf("f_open error (%i)\r\n", g_fres_st);
    }
}

void Fatfs_Close(void)
{
    if(g_isOpen_b)
    {
        f_close(&g_fil_st);
    }
}

void vprint1(const char *fmt, va_list argp)
{
    char string[200];
    UINT bytesWrote;
    UINT len2Write;

    string[0] = '\r';
    if(0 < vsprintf(string+1,fmt,argp)) // build string
    {
        len2Write = strlen(string);
        string[len2Write+1] = '\n';
        g_fres_st = f_write(&g_fil_st, string, strlen(string), &bytesWrote);
        if(g_fres_st == FR_OK)
        {
            my_printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
        }
        else
        {
            my_printf("f_write error (%i)\r\n");
        }
    }
}

void Fatfs_Printf(const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vprint1(fmt, argp);
    va_end(argp);
}
