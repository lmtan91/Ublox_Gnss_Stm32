#ifndef EXAMPLE_USER_INC_SERIAL_H_
#define EXAMPLE_USER_INC_SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "Ublox_Gnss.h"
#include "ring_buffer.h"

extern bool dataComing_b;


#ifndef USART_RX_BUF_SIZE
#define USART_RX_BUF_SIZE               256
#endif

typedef struct Serial
{
    UART_HandleTypeDef      *uartHdl_pst;
    ring_buffer             rbRx_st;
    uint8_t rx_buf[USART_RX_BUF_SIZE];
    void (*init)(struct Serial *serial_pst);
    void (*write)(struct Serial *serial_pst, uint8_t *buff_pu8, uint16_t len_u16);
    uint16_t (*available)(struct Serial *serial_pst);
    void (*read)(struct Serial *serial_pst, uint8_t* incomming_pu8);
}Serial_tst;

void Serial_Write(Serial_tst *serial_pst, uint8_t *buff_pu8, uint16_t len_u16);
uint16_t Serial_Available(struct Serial *serial_pst);
void Serial_Read(struct Serial *serial_pst, uint8_t* incomming_pu8);
void Serial_Init(struct Serial *serial_pst);

void my_printf(const char *fmt, ...);

extern Serial_tst serial_st;

#ifdef __cplusplus
}
#endif

#endif /* EXAMPLE_USER_INC_SERIAL_H_ */
