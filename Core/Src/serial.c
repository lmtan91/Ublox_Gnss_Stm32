/* *****************************************************************************
 *
 *
 *******************************************************************************

/*******************************************************************************
 * Files inclusion
 ******************************************************************************/
#include <serial.h>
#include <stdarg.h>
#include "fatfs.h"
#include "main_fatfs.h"
/*******************************************************************************
 * Internal Macros
 ******************************************************************************/


/*******************************************************************************
 * Static function declaration
 ******************************************************************************/


/* *****************************************************************************
 * Variables
 ******************************************************************************/
Serial_tst serial_st;

FIL g_fil_st;      //File handle
FRESULT g_fres_st;
/* *****************************************************************************
 * Static function definition
 ******************************************************************************/



/* *****************************************************************************
 * Public function definition
 ******************************************************************************/
void Serial_Write(Serial_tst *serial_pst, uint8_t *buff_pu8, uint16_t len_u16)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    ret = HAL_UART_Transmit(serial_pst->uartHdl_pst, buff_pu8, len_u16, 0xFFFF);
    if(ret == HAL_OK)
    {
        if(HAL_UART_Receive_IT(serial_pst->uartHdl_pst, (uint8_t *)&g_incomming, 1)==HAL_OK)
        {

        }
    }
}

void Serial_Init(struct Serial *serial_pst)
{
    rb_init(&serial_pst->rbRx_st, USART_RX_BUF_SIZE, serial_pst->rx_buf);
}

uint16_t Serial_Available(struct Serial *serial_pst)
{
    uint16_t ret = rb_full_count(&serial_pst->rbRx_st);
    if(ret > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

void Serial_Read(struct Serial *serial_pst, uint8_t* incomming_u8)
{
    uint8_t ret = rb_remove(&serial_pst->rbRx_st);
    *incomming_u8 = ret;

    return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static int i = 0;
    if(huart->Instance == UART4)
    {
        rb_push_insert(&serial_st.rbRx_st, g_incomming);
        if(HAL_UART_Receive_IT(serial_st.uartHdl_pst, (uint8_t *)&g_incomming, 1)==HAL_OK)
        {

        }
    }
}

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
                printf("\r%s\n", string);
    }
}

void vprint1(const char *fmt, va_list argp)
{
    char string[200];
    BYTE readBuf[30];

    strncpy((char*)readBuf, "a new file is made!", 19);

    g_fres_st = f_open(&g_fil_st, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
    if(g_fres_st == FR_OK)
    {
          my_printf("I was able to open 'write.txt' for writing\r\n");
    }
        else
        {
          my_printf("f_open error (%i)\r\n", g_fres_st);
        }
//    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        UINT bytesWrote;
        g_fres_st = f_write(&g_fil_st, readBuf, 19, &bytesWrote);
        if(g_fres_st == FR_OK)
        {
            my_printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
        }
        else
        {
            my_printf("f_write error (%i)\r\n");
        }

            //Be a tidy kiwi - don't forget to close your g_fil_ste!
        f_close(&g_fil_st);
//        printf("\r%s\n", string);
    }
}

void my_printf(const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

void fx_printf(const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vprint1(fmt, argp);
    va_end(argp);
}


