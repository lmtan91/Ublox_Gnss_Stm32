#include "fatfs.h"
#include "main_fatfs.h"

FIL g_fil_st;
FRESULT g_fres_st;

void Main_Fatfs_Init(void)
{

    FATFS FatFs;  //Fatfs handle


    //Open the g_fil_ste system
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




}
