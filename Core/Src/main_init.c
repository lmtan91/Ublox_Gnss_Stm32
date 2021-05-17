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

void printODOdata(UBX_NAV_ODO_data_t ubxDataStruct)
{
    unsigned long iTOW = ubxDataStruct.iTOW; // iTOW is in milliseconds
    unsigned long distance = ubxDataStruct.distance; // Print the distance
    unsigned long totalDistance = ubxDataStruct.totalDistance; // Print the total distance

    my_printf("TOW: %lu (ms) Distance: %lu (m) Total Distance: %lu (m)",
    		iTOW, distance, totalDistance);
    Fatfs_Printf("TOW: %lu (ms) Distance: %lu (m) Total Distance: %lu (m)",
        		iTOW, distance, totalDistance);
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
    uint8_t numSv_u8 = ubxDataStruct.numSV;

    uint8_t fixOk_u8 = ubxDataStruct.flags.bits.gnssFixOK;
    uint8_t fixType_u8 = ubxDataStruct.fixType;

    long height = ubxDataStruct.height;

    my_printf("Time: %d:%d:%d.%lu Lat: %ld Long: %ld (degrees * 10^-7) hMSL: %ld (mm) NumSV: %d Fix: %d FixType: %d",
            hms,min,sec, millisecs, latitude, longitude, altitude, numSv_u8, fixOk_u8, fixType_u8);
    my_printf("velN: %ld velE: %ld velD: %ld gSpeed: %ld (mm/s)",
        		ubxDataStruct.velN,ubxDataStruct.velE,ubxDataStruct.velD,
    			ubxDataStruct.gSpeed);
    Fatfs_Printf("Time: %d:%d:%d.%lu Lat: %ld Long: %ld Height: %ld hMSL: %ld (mm) NumSV: %d Fix: %d FixType: %d",
                hms,min,sec, millisecs, latitude, longitude, height, altitude, numSv_u8, fixOk_u8, fixType_u8);
    Fatfs_Printf("velN: %ld velE: %ld velD: %ld gSpeed: %ld (mm/s)",
    		ubxDataStruct.velN,ubxDataStruct.velE,ubxDataStruct.velD,
			ubxDataStruct.gSpeed);
}

void printESFALGdata(UBX_ESF_ALG_data_t ubxDataStruct)
{

  unsigned long iTOW = ubxDataStruct.iTOW; // iTOW is in milliseconds

  my_printf("ESFALG TOW: %lu (ms) Roll: %.2f Pitch: %.2f Yaw: %.2f (Degrees)", iTOW,
		  (float)ubxDataStruct.roll / 100.0, (float)ubxDataStruct.pitch / 100.0,(float)ubxDataStruct.yaw / 100.0);
  Fatfs_Printf("ESFALG TOW: %lu (ms) Roll: %.2f Pitch: %.2f Yaw: %.2f (Degrees)", iTOW,
  		  (float)ubxDataStruct.roll / 100.0, (float)ubxDataStruct.pitch / 100.0,(float)ubxDataStruct.yaw / 100.0);
}

void printESFSTATUSdata(UBX_ESF_STATUS_data_t ubxDataStruct)
{
  my_printf("ESFSTATUS fusionMode: %d numSens: %d", ubxDataStruct.fusionMode,ubxDataStruct.numSens);
  Fatfs_Printf("ESFSTATUS fusionMode: %d numSens: %d", ubxDataStruct.fusionMode,ubxDataStruct.numSens);
  for (uint8_t num = 0; num < ubxDataStruct.numSens; num++) // For each sensor
  {
    UBX_ESF_STATUS_sensorStatus_t sensorStatus;
//    myGNSS.getSensorFusionStatus(&sensorStatus, ubxDataStruct, num); // Extract the data for one sensor
//
//    Serial.print(F(": Type: "));
//    Serial.print(sensorStatus.sensStatus1.bits.type);
//    Serial.print(F(" Used: "));
//    Serial.print(sensorStatus.sensStatus1.bits.used);
//    Serial.print(F(" Ready: "));
//    Serial.print(sensorStatus.sensStatus1.bits.ready);
//    Serial.print(F(" Calib Status: "));
//    Serial.print(sensorStatus.sensStatus2.bits.calibStatus);
//    Serial.print(F(" Noisy: "));
//    Serial.println(sensorStatus.faults.bits.noisyMeas);

  }
}

void Main_Init(void)
{
    Fatfs_Init();

    HAL_GPIO_WritePin(GPIOE, EXT_ANT_Pin, GPIO_PIN_SET);
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
            Fatfs_Printf("init3 ok");
            if(Gnss_SetUART1Output(&ubx_st, COM_TYPE_UBX, 1100) == true)
            {
                my_printf("enable UBX");
                Fatfs_Printf("enable UBX");
            }
            if(Gnss_SetNavigationFrequency(&ubx_st, 2, 1100) == true)
            {
                my_printf("setNavigationFrequency ok");
                Fatfs_Printf("setNavigationFrequency ok");
            }

//            if(Gnss_SetHNRNavigationRate(&ubx_st, 50, 1000) == true)
//            {
//            	my_printf("Gnss_SetHNRNavigationRate ok");
//				Fatfs_Printf("Gnss_SetHNRNavigationRate ok");
//            }
//            else
//            {
//            	my_printf("Gnss_SetHNRNavigationRate failed");
//				Fatfs_Printf("Gnss_SetHNRNavigationRate failed");
//            }

            my_printf("sw version %02x.%02x",
                    Gnss_GetProtocolVersionHigh(&ubx_st, 1000),
                    Gnss_GetProtocolVersionLow(&ubx_st, 1000));
            Fatfs_Printf("sw version %02x.%02x",
                                Gnss_GetProtocolVersionHigh(&ubx_st, 1000),
                                Gnss_GetProtocolVersionLow(&ubx_st, 1000));

            Gnss_SetAutoPVTcallback(&ubx_st, &printPVTdata, 1000);
            Gnss_SetAutoNAVODOcallback(&ubx_st, &printODOdata, 1000);

//            Gnss_SetAutoESFALGcallback(&ubx_st, &printESFALGdata, 1000);
        }
    }
}
