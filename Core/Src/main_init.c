#include "main_init.h"

#include "dwt_delay.h"
#include "main.h"

UART_HandleTypeDef huart4;
Serial_tst serial_st;
Ublox_Gnss_tst ubx_st;

static void MX_UART4_Init(uint32_t BaudRate_u32);
static void MX_UART4_Init(uint32_t BaudRate_u32)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = BaudRate_u32;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}


void printPVTdata(UBX_NAV_PVT_data_t ubxDataStruct)
{

    uint8_t hms = ubxDataStruct.hour; // Print the hours
    uint8_t min = ubxDataStruct.min; // Print the minutes
    uint8_t sec = ubxDataStruct.sec; // Print the seconds

    unsigned long millisecs = ubxDataStruct.iTOW % 1000; // Print the milliseconds


    long latitude = ubxDataStruct.lat; // Print the latitude
    long longitude = ubxDataStruct.lon; // Print the longitude

    long altitude = ubxDataStruct.hMSL; // Print the height above mean sea level

    my_printf("Time: %d:%d:%d.%lu Lat: %ld Long: %ld (degrees * 10^-7) Height above MSL: %ld (mm)",
            hms,min,sec, millisecs, latitude, longitude, altitude);
}

void Main_Init(void)
{
    HAL_GPIO_WritePin(GPIOE, EXT_ANT_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    /* Init us timer */
    DWT_Init();
    MX_UART4_Init(9600);

    serial_st.uartHdl_pst = &huart4;
    Serial_Init(&serial_st);
    if(Gnss_Init(&ubx_st, &serial_st) == true)
    {
        if(Gnss_DisableNMEAMessage(&ubx_st, UBX_NMEA_GLL, COM_PORT_UART1, 1100) == true)
        {

        }
        Gnss_SetSerialRate(&ubx_st, 115200, COM_PORT_UART1, 1100);
        HAL_Delay(100);
        HAL_UART_DeInit(&huart4);
        HAL_Delay(100);
        MX_UART4_Init(115200);
        Serial_Init(&serial_st);
        if(Gnss_Init(&ubx_st, &serial_st) == true)
        {
            my_printf("init2 ok");
            if(Gnss_GetPortSettings(&ubx_st, COM_PORT_UART1, 1100) ==  true)
            {
                my_printf("get ok");
            }
        }
    }
    else
    {
        HAL_Delay(100);
        HAL_UART_DeInit(&huart4);
        HAL_Delay(100);
        MX_UART4_Init(115200);
        Serial_Init(&serial_st);
        if(Gnss_Init(&ubx_st, &serial_st) == true)
        {
            my_printf("init3 ok");
            if(Gnss_SetUART1Output(&ubx_st, COM_TYPE_UBX, 1100) == true)
            {
                my_printf("enable UBX");
            }
            if(Gnss_SetNavigationFrequency(&ubx_st, 2, 1100) == true)
            {
                my_printf("setNavigationFrequency ok");
            }
            my_printf("sw version %02x.%02x",
                    Gnss_GetProtocolVersionHigh(&ubx_st, 1000),
                    Gnss_GetProtocolVersionLow(&ubx_st, 1000));

            if(Gnss_GetEsfInfo(&ubx_st, 1000) == false)
            {
                my_printf("cannot get ESF");
            }
            else
            {
                my_printf("Fusion Mode: %d", ubx_st.packetUBXESFSTATUS_pst->data.fusionMode);
            }

            Gnss_SetAutoPVTcallback(&ubx_st, &printPVTdata, 1000);
        }
    }
}