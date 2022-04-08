/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "sample_pnp_device_component.h"
#include "main.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_tsensor.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Azure Provisioning/IoT Hub library includes */
#include "azure_iot_hub_client.h"
#include "azure_iot_hub_client_properties.h"
#include "azure_iot_provisioning_client.h"


/* Azure JSON includes */
#include "azure_iot_json_reader.h"
#include "azure_iot_json_writer.h"

#include "azure_iot_hub_client_properties.h"

#include <math.h>

#define DOUBLE_DECIMAL_PLACE_DIGITS   (2)
#define SAMPLE_COMMAND_SUCCESS_STATUS (200)
#define SAMPLE_COMMAND_ERROR_STATUS   (500)

#define DS18B20_1_PORT GPIOA
#define DS18B20_1_PIN  GPIO_PIN_15
#define DS18B20_2_PORT GPIOB
#define DS18B20_2_PIN  GPIO_PIN_2

#define US_100_PORT GPIOB
#define US_100_PIN  GPIO_PIN_2

#define UART_BUFFER_LENGTH 100

/* Telemetry key */
static const CHAR telemetry_name_soilMoistureExternal1Raw[] = "soilMoistureExternal1";
static const CHAR telemetry_name_soilMoistureExternal2Raw[] = "soilMoistureExternal2";
static const CHAR telemetry_name_pmsExternal1Raw[]  = "PMSExternal1";
static const CHAR telemetry_name_temperatureExternal2Raw[]  = "temperatureExternal2";
static const CHAR telemetry_name_sensorTemperature[]        = "temperature";
static const CHAR telemetry_name_sensorPressure[]           = "pressure";
static const CHAR telemetry_name_sensorHumidity[]           = "humidityPercentage";
static const CHAR telemetry_name_sensorAcceleration[]       = "acceleration";
static const CHAR telemetry_name_sensorMagnetic[]           = "magnetic";

/* Pnp command supported */
static const CHAR set_led_state[] = "setLedState";

/* Names of properties for desired/reporting */
static const CHAR reported_led_state[] = "ledState";

static UCHAR scratch_buffer[512];
uint8_t UART4_rxBuffer[UART_BUFFER_LENGTH];
static uint8_t ucPropertyPayloadBuffer[256];

static void set_led_state_action(bool level)
{
    if (level)
    {
        printf("LED is turned ON\r\n");
        BSP_LED_On(LED_GREEN);
    }
    else
    {
        printf("LED is turned OFF\r\n");
        BSP_LED_Off(LED_GREEN);
    }
}

UINT adc_read(ADC_HandleTypeDef* ADC_Controller, UINT ADC_Channel)
{
    UINT value                     = 0;
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = ADC_Channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;

    HAL_ADC_ConfigChannel(ADC_Controller, &sConfig);

    HAL_ADC_Start(ADC_Controller);
    if (HAL_ADC_PollForConversion(ADC_Controller, 10) == HAL_OK)
    {
        value = HAL_ADC_GetValue(ADC_Controller);
    }
    HAL_ADC_Stop(ADC_Controller);
    HAL_Delay(200);

    return value;
}
// VT_VOID co2_start_measurement(){

//     uint8_t UART4_txBuffer[]={0x61,0x06,0x00,0x36,0x00,0x00,0x60,0x64};
//             for (int i =0;i <UART_BUFFER_LENGTH;i++){
//         UART4_rxBuffer[i]=0;
//     }
   
//     HAL_UART_Transmit(&UartHandle4, (uint8_t*)UART4_txBuffer, sizeof(UART4_txBuffer), 1000);
//     HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 3000);



//     printf("sent start\n");
//             for (int j=0;j<UART_BUFFER_LENGTH;j++){

//             printf("%x-", UART4_rxBuffer[j]);
//         }
//     #if VT_LOG_LEVEL > 2
//     VTLogDebugNoTag("sent start\n");
//     VTLogDebugNoTag("received packet:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             VTLogDebugNoTag("%x-", UART4_rxBuffer[j]);
//         }
//        VTLogDebugNoTag("\n");
//        #endif

// }

// VT_VOID co2_stop_measurement(){

//     uint8_t UART4_txBuffer[]={0x61,0x06,0x00,0x37,0x00,0x01,0xF0,0x64};

//             for (int i =0;i <UART_BUFFER_LENGTH;i++){
//         UART4_rxBuffer[i]=0;
//     }
//     HAL_UART_Transmit(&UartHandle4, UART4_txBuffer, sizeof(UART4_txBuffer), 1000);
//         HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 3000);
       
//            printf("sent stop\n");
//             for (int j=0;j<UART_BUFFER_LENGTH;j++){

//             printf("%x-", UART4_rxBuffer[j]);
//         }

//     #if VT_LOG_LEVEL > 2
//     VTLogDebugNoTag("sent stop\n");
//     VTLogDebugNoTag("received packet:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             VTLogDebugNoTag("%x-", UART4_rxBuffer[j]);
//         }
//        VTLogDebugNoTag("\n");
//        #endif


// }

// VT_VOID co2_read_measurement(){

//     uint8_t UART4_txBuffer[]={0x61,0x03,0x00,0x27,0x00,0x01,0x3d,0xa1};

//             for (int i =0;i <UART_BUFFER_LENGTH;i++){
//         UART4_rxBuffer[i]=0;
//     }
//     HAL_UART_Transmit(&UartHandle4, UART4_txBuffer, sizeof(UART4_txBuffer), 1000);
//         HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 3000);
       
//            printf("sent read ready\n");
//             for (int j=0;j<UART_BUFFER_LENGTH;j++){

//             printf("%x-", UART4_rxBuffer[j]);
//         }

// uint8_t UART4_txBuffer2[]={0x61,0x03,0x00,0x28,0x00,0x06,0x4c,0x60};

//             for (int i =0;i <UART_BUFFER_LENGTH;i++){
//         UART4_rxBuffer[i]=0;
//     }
//     HAL_UART_Transmit(&UartHandle4, UART4_txBuffer2, sizeof(UART4_txBuffer2), 1000);
//         HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 4000);
       
//            printf("sent read\n");
//             for (int j=0;j<UART_BUFFER_LENGTH;j++){

//             printf("%x-", UART4_rxBuffer[j]);
//         }

//     #if VT_LOG_LEVEL > 2
//     VTLogDebugNoTag("sent stop\n");
//     VTLogDebugNoTag("received packet:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             VTLogDebugNoTag("%x-", UART4_rxBuffer[j]);
//         }
//        VTLogDebugNoTag("\n");
//        #endif


// }

// VT_VOID hpma_start_measurement(){

//     uint8_t UART4_txBuffer[]={0x68,0x01,0x01,0x96};
//             for (int i =0;i <UART_BUFFER_LENGTH;i++){
//         UART4_rxBuffer[i]=0;
//     }
   
//     HAL_UART_Transmit(&UartHandle4, (uint8_t*)UART4_txBuffer, sizeof(UART4_txBuffer), 1000);
//     HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 3000);



//     printf("sent start\n");
//             for (int j=0;j<UART_BUFFER_LENGTH;j++){

//             printf("%x-", UART4_rxBuffer[j]);
//         }
//     #if VT_LOG_LEVEL > 2
//     VTLogDebugNoTag("sent start\n");
//     VTLogDebugNoTag("received packet:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             VTLogDebugNoTag("%x-", UART4_rxBuffer[j]);
//         }
//        VTLogDebugNoTag("\n");
//        #endif

// }

// VT_VOID hpma_stop_measurement(){

//     uint8_t UART4_txBuffer[]={0x68,0x01,0x02,0x95};

//             for (int i =0;i <UART_BUFFER_LENGTH;i++){
//         UART4_rxBuffer[i]=0;
//     }
//     HAL_UART_Transmit(&UartHandle4, UART4_txBuffer, sizeof(UART4_txBuffer), 1000);
//         HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 3000);
       
//            printf("sent stop\n");
//             for (int j=0;j<UART_BUFFER_LENGTH;j++){

//             printf("%x-", UART4_rxBuffer[j]);
//         }

//     #if VT_LOG_LEVEL > 2
//     VTLogDebugNoTag("sent stop\n");
//     VTLogDebugNoTag("received packet:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             VTLogDebugNoTag("%x-", UART4_rxBuffer[j]);
//         }
//        VTLogDebugNoTag("\n");
//        #endif


// }


// VT_VOID sps30_start_measurement(){

//     uint8_t UART4_txBuffer[]={0x7E,0x00,0x00,0x02,0x01,0x03,0xF9,0x7E};
   
//     HAL_UART_Transmit(&UartHandle4, (uint8_t*)UART4_txBuffer, sizeof(UART4_txBuffer), 1000);
//     HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 3000);
//         // printf("sent start:\n");
    
//         // for (int j=0;j<UART_BUFFER_LENGTH;j++){
//         //     printf("%x-", UART4_rxBuffer[j]);
//         // }

//     #if VT_LOG_LEVEL > 2
//     VTLogDebugNoTag("sent start\n");
//     VTLogDebugNoTag("received packet:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             VTLogDebugNoTag("%x-", UART4_rxBuffer[j]);
//         }
//        VTLogDebugNoTag("\n");
//        #endif

// }

// VT_VOID sps30_stop_measurement(){

//     uint8_t UART4_txBuffer[]={0x7E,0x00,0x01,0x00,0xFE,0x7E};

    
//     HAL_UART_Transmit(&UartHandle4, UART4_txBuffer, sizeof(UART4_txBuffer), 1000);
//         HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 3000);
//         //       printf("sent sotp:\n");
    
//         // for (int j=0;j<UART_BUFFER_LENGTH;j++){
//         //     printf("%x-", UART4_rxBuffer[j]);
//         // }
//     #if VT_LOG_LEVEL > 2
//     VTLogDebugNoTag("sent stop\n");
//     VTLogDebugNoTag("received packet:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             VTLogDebugNoTag("%x-", UART4_rxBuffer[j]);
//         }
//        VTLogDebugNoTag("\n");
//        #endif


// }

// VT_VOID sps30_read_measurement(){
//         union {
//         char c[4];
//         float f;
//     } u;

//     uint8_t UART4_txBuffer[]={0x7E,0x00,0x03,0x00,0xFC,0x7E};
//         VT_INT decimal;
//     VT_FLOAT frac_float;
//     VT_INT frac;

    
//     HAL_UART_Transmit(&UartHandle4, UART4_txBuffer, sizeof(UART4_txBuffer), 1000);
//         HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 3000);
//               printf("sent read:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             printf("%x-", UART4_rxBuffer[j]);
//         }

//     for (VT_UINT iter=0;iter<UART_BUFFER_LENGTH-31;iter++){
//             if (UART4_rxBuffer[iter]==0x7e){
//                 if (UART4_rxBuffer[iter+1]==0x00){
//                         u.c[3] = UART4_rxBuffer[iter+9];
//                         u.c[2] = UART4_rxBuffer[iter+10];
//                         u.c[1] = UART4_rxBuffer[iter+11];
//                         u.c[0] = UART4_rxBuffer[iter+12];
//                         break;
//                 }}}

//             decimal    = u.f;
//         frac_float = u.f - (VT_FLOAT)decimal;
//         frac       = fabsf(frac_float) * 10000;
//        printf("\npm2.5 val %d.%04d : \n", decimal, frac);

    
//     #if VT_LOG_LEVEL > 2
//     VTLogDebugNoTag("sent stop\n");
//     VTLogDebugNoTag("received packet:\n");
    
//         for (int j=0;j<UART_BUFFER_LENGTH;j++){
//             VTLogDebugNoTag("%x-", UART4_rxBuffer[j]);
//         }
//        VTLogDebugNoTag("\n");
//        #endif


// }



VT_UINT getpmdata()
{
    uint16_t _checksum;
    uint16_t _calculatedChecksum;
    int flag=0;
    VT_UINT val1=0;

    for (int i =0;i <UART_BUFFER_LENGTH;i++){
        UART4_rxBuffer[i]=0;
    }

   // HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, 1, 5000);
        //printf(" UART4_rxBuffer: %x\n", *UART4_rxBuffer);
    //if((*UART4_rxBuffer == 0x42) || (*UART4_rxBuffer == 0x52))
    //{
        HAL_UART4_Receive (&UartHandle4, UART4_rxBuffer, UART_BUFFER_LENGTH, 2000);
        //HAL_UART_Transmit(&UartHandle4, UART4_rxBuffer, sizeof(UART4_rxBuffer), 1000);
        for (int j=0;j<UART_BUFFER_LENGTH;j++){
            printf("%x-", UART4_rxBuffer[j]);
        }
       printf("\n");
        for (VT_UINT iter=0;iter<UART_BUFFER_LENGTH-31;iter++){
            if (UART4_rxBuffer[iter]==0x42){
                if (UART4_rxBuffer[iter+1]==0x4d){
                    for (VT_UINT iter2=0;iter2<30;iter2++){
                        
                        _calculatedChecksum += UART4_rxBuffer[iter+iter2];

                    }
                    _checksum = UART4_rxBuffer[iter+30] << 8;
                    _checksum |= UART4_rxBuffer[iter+31];

                    #if VT_LOG_LEVEL > 2
                    VTLogDebug("_checksum: %x\n", _checksum);
                    VTLogDebug("_calculatedChecksum: %x\n", _calculatedChecksum);
                    #endif

                    if (_checksum==_calculatedChecksum){
                         val1=  ((UART4_rxBuffer[iter+12]) << 8 | (UART4_rxBuffer[iter+13]));
                        printf("\nPMS Sensor Val: %d\n", val1);
                        flag=1;
                        break;
                    }
                    else{
                        _calculatedChecksum=0;
                    }

                }

            }



        }
    if (flag==0){printf("Error in getting PM2.5 value");}

    return val1;
}

//temp sensor requiremnts

// static void set_pin_output(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
// {
//     GPIO_InitTypeDef GPIO_InitStruct = {0};

//     GPIO_InitStruct.Pin   = GPIO_Pin;
//     GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull  = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
// }

// static void set_pin_input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
// {
//     GPIO_InitTypeDef GPIO_InitStruct = {0};

//     GPIO_InitStruct.Pin   = GPIO_Pin;
//     GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull  = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
// }

// static void delay_usec(uint32_t delay_usec)
// {
//     TIM_HandleTypeDef delay_usec_timer;
//     delay_usec_timer.Instance               = TIM7;
//     delay_usec_timer.Init.Prescaler         = (uint32_t)((SystemCoreClock / 1000000) - 1);
//     delay_usec_timer.Init.CounterMode       = TIM_COUNTERMODE_UP;
//     delay_usec_timer.Init.Period            = 65535;
//     delay_usec_timer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//     delay_usec_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//     if (HAL_TIM_Base_Init(&delay_usec_timer) != HAL_OK)
//     {
//         // add error handling
//     }
//     HAL_TIM_Base_Start(&delay_usec_timer);
//     while ((__HAL_TIM_GET_COUNTER(&delay_usec_timer)) < delay_usec)
//         ;
// }

// static uint8_t ds18b20_start(GPIO_TypeDef* ds18b20_port, uint16_t ds18b20_pin)
// {
//     uint8_t response = 0;
//     set_pin_output(ds18b20_port, ds18b20_pin);       // set the pin as output
//     HAL_GPIO_WritePin(ds18b20_port, ds18b20_pin, 0); // pull the pin low
//     delay_usec(480);                                 // delay according to datasheet

//     set_pin_input(ds18b20_port, ds18b20_pin); // set the pin as input
//     delay_usec(80);                           // delay according to datasheet

//     if (!(HAL_GPIO_ReadPin(ds18b20_port, ds18b20_pin)))
//     {
//         response = 1; // if the pin is low i.e the presence pulse is detected
//     }
//     else
//     {
//         response = 0;
//     }

//     delay_usec(400); // 480 us delay totally.
//     return response;
// }

// static void ds18b20_write(GPIO_TypeDef* ds18b20_port, uint16_t ds18b20_pin, uint8_t data)
// {
//     set_pin_output(ds18b20_port, ds18b20_pin); // set as output

//     for (int i = 0; i < 8; i++)
//     {
//         if ((data & (1 << i)) != 0) // if the bit is high
//         {
//             // write 1
//             set_pin_output(ds18b20_port, ds18b20_pin);       // set as output
//             HAL_GPIO_WritePin(ds18b20_port, ds18b20_pin, 0); // pull the pin LOW
//             delay_usec(1);                                   // wait for 1 us

//             set_pin_input(ds18b20_port, ds18b20_pin); // set as input
//             delay_usec(50);                           // wait for 60 us
//         }

//         else // if the bit is low
//         {
//             // write 0
//             set_pin_output(ds18b20_port, ds18b20_pin);
//             HAL_GPIO_WritePin(ds18b20_port, ds18b20_pin, 0); // pull the pin LOW
//             delay_usec(50);                                  // wait for 60 us

//             set_pin_input(ds18b20_port, ds18b20_pin);
//         }
//     }
// }

// static uint8_t ds18b20_read(GPIO_TypeDef* ds18b20_port, uint16_t ds18b20_pin)
// {
//     uint8_t value = 0;
//     set_pin_input(ds18b20_port, ds18b20_pin);

//     for (int i = 0; i < 8; i++)
//     {
//         set_pin_output(ds18b20_port, ds18b20_pin); // set as output

//         HAL_GPIO_WritePin(ds18b20_port, ds18b20_pin, 0); // pull the data pin LOW
//         delay_usec(2);                                   // wait for 2 us

//         set_pin_input(ds18b20_port, ds18b20_pin);        // set as input
//         if (HAL_GPIO_ReadPin(ds18b20_port, ds18b20_pin)) // if the pin is HIGH
//         {
//             value |= 1 << i; // read = 1
//         }
//         delay_usec(60); // wait for 60 us
//     }
//     return value;
// }

// static float ds18b20_temperature_read(GPIO_TypeDef* ds18b20_port, uint16_t ds18b20_pin)
// {
//     uint8_t byte1 = 0;
//     uint8_t byte2 = 0;
//     float integer;
//     float decimal;
//     if (ds18b20_start(ds18b20_port, ds18b20_pin))
//     {
//         HAL_Delay(1);
//         ds18b20_write(ds18b20_port, ds18b20_pin, 0xCC); // skip ROM
//         ds18b20_write(ds18b20_port, ds18b20_pin, 0x44); // convert t
//         HAL_Delay(800);

//         if (ds18b20_start(ds18b20_port, ds18b20_pin))
//         {
//             HAL_Delay(1);
//             ds18b20_write(ds18b20_port, ds18b20_pin, 0xCC); // skip ROM
//             ds18b20_write(ds18b20_port, ds18b20_pin, 0xBE); // Read Scratch-pad

//             byte1 = ds18b20_read(ds18b20_port, ds18b20_pin);
//             byte2 = ds18b20_read(ds18b20_port, ds18b20_pin);

//             integer = (int8_t)((byte1 >> 4) | (byte2 << 4));
//             decimal = (float)(byte1 & 0x0F) * 0.0625f;

//             return (integer + decimal);
//         }
//     }
//     return (0);
// }

/* Implementation of Set LED state command of device component  */
static UINT sample_pnp_device_set_led_state_command(
    SAMPLE_PNP_DEVICE_COMPONENT* handle, AzureIoTJSONReader_t* xReader, AzureIoTJSONWriter_t* xWriter)
{
    bool state;
    AzureIoTResult_t xResult;

    xResult = AzureIoTJSONReader_NextToken(xReader);
    configASSERT(xResult == eAzureIoTSuccess);
    xResult = AzureIoTJSONReader_GetTokenBool(xReader, &state);
    configASSERT(xResult == eAzureIoTSuccess);

    set_led_state_action((bool)state);
    handle->sensorLEDState = state;
    return (eAzureIoTSuccess);
}

UINT sample_pnp_device_init(SAMPLE_PNP_DEVICE_COMPONENT* handle,
    UCHAR* component_name_ptr,
    UINT component_name_length,
    double default_sensor_reading,
    FreeRTOS_VERIFIED_TELEMETRY_DB* verified_telemetry_DB)
{
    if (handle == NULL)
    {
        return (eAzureIoTErrorFailed);
    }

    handle->component_name_ptr       = component_name_ptr;
    handle->component_name_length    = component_name_length;
    handle->soilMoistureExternal1Raw = default_sensor_reading;
    handle->soilMoistureExternal2Raw = default_sensor_reading;
    handle->pmsExternal1Raw  = default_sensor_reading;
    handle->temperatureExternal2Raw  = default_sensor_reading;
    handle->sensorTemperature        = default_sensor_reading;
    handle->sensorPressure           = default_sensor_reading;
    handle->sensorHumidity           = default_sensor_reading;
    handle->sensorAcceleration       = default_sensor_reading;
    handle->sensorMagnetic           = default_sensor_reading;
    handle->sensorLEDState           = false;
    handle->verified_telemetry_DB    = verified_telemetry_DB;

    return (eAzureIoTSuccess);
}

UINT get_sensor_data(SAMPLE_PNP_DEVICE_COMPONENT* handle)
{
    if (handle == NULL)
    {
        return (eAzureIoTErrorFailed);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(10);

    UINT soilMoisture1ADCData = adc_read(&hadc1, ADC_CHANNEL_1);
    UINT soilMoisture2ADCData = adc_read(&hadc1, ADC_CHANNEL_2);

        printf("******* CS PART *******\n");

    //for (int i=0;i<5;i++){

    //sps30_start_measurement();
    //hpma_start_measurement();
    //co2_start_measurement();
    
    FreeRTOS_vt_signature_read(handle->verified_telemetry_DB,
        (UCHAR*)telemetry_name_pmsExternal1Raw,
        sizeof(telemetry_name_pmsExternal1Raw) - 1,1);
    

    // int i=0;
    // while(i<100000){
    //     i++;
    // }
    //co2_read_measurement();

    //sps30_read_measurement();

    //float pmsexternal1 = ds18b20_temperature_read(DS18B20_1_PORT, DS18B20_1_PIN);
    float pms_extrernal1= (float) getpmdata();
    //printf("%.2f",pms_extrernal1);

    //float temperatureExternal1 = ds18b20_temperature_read(DS18B20_1_PORT, DS18B20_1_PIN);



    FreeRTOS_vt_signature_process(handle->verified_telemetry_DB,
        (UCHAR*)telemetry_name_pmsExternal1Raw,
        sizeof(telemetry_name_pmsExternal1Raw) - 1);
      //  }
      //sps30_stop_measurement();

      //hpma_stop_measurement();
      //co2_stop_measurement();

    printf("\n******* CS PART END*******\n");

    float temperature = BSP_TSENSOR_ReadTemp();
    float humidity    = BSP_HSENSOR_ReadHumidity();
    float pressure    = BSP_PSENSOR_ReadPressure();
    int16_t magnetoXYZ[3];
    BSP_MAGNETO_GetXYZ(magnetoXYZ);
    int16_t accXYZ[3];
    BSP_ACCELERO_AccGetXYZ(accXYZ);

    handle->soilMoistureExternal1Raw = soilMoisture1ADCData;
    handle->soilMoistureExternal2Raw = soilMoisture2ADCData;
    handle->pmsExternal1Raw  = pms_extrernal1;//temperatureExternal1;
    handle->temperatureExternal2Raw  = 30;// temperatureExternal2;

    handle->sensorTemperature  = temperature;
    handle->sensorPressure     = pressure;
    handle->sensorHumidity     = humidity;
    handle->sensorAcceleration = accXYZ[0];
    handle->sensorMagnetic     = magnetoXYZ[0];

    return (eAzureIoTSuccess);
}

UINT sample_pnp_device_process_command(SAMPLE_PNP_DEVICE_COMPONENT* handle,
    UCHAR* component_name_ptr,
    UINT component_name_length,
    UCHAR* pnp_command_name_ptr,
    UINT pnp_command_name_length,
    AzureIoTJSONReader_t* json_reader_ptr,
    AzureIoTJSONWriter_t* json_response_ptr,
    UINT* status_code)
{
    UINT dm_status;

    if (handle == NULL)
    {
        return (eAzureIoTErrorFailed);
    }

    if (handle->component_name_length != component_name_length ||
        strncmp((CHAR*)handle->component_name_ptr, (CHAR*)component_name_ptr, component_name_length) != 0)
    {
        return (eAzureIoTErrorFailed);
    }

    if (pnp_command_name_length != (sizeof(set_led_state) - 1) ||
        strncmp((CHAR*)pnp_command_name_ptr, (CHAR*)set_led_state, pnp_command_name_length) != 0)
    {
        printf(
            "PnP command=%.*s is not supported on device component\r\n", pnp_command_name_length, pnp_command_name_ptr);
        dm_status = 404;
    }
    else
    {
        dm_status =
            (sample_pnp_device_set_led_state_command(handle, json_reader_ptr, json_response_ptr) != eAzureIoTSuccess)
                ? SAMPLE_COMMAND_ERROR_STATUS
                : SAMPLE_COMMAND_SUCCESS_STATUS;
    }

    *status_code = dm_status;

    return (eAzureIoTSuccess);
}

AzureIoTResult_t sample_pnp_device_telemetry_send(
    SAMPLE_PNP_DEVICE_COMPONENT* handle, AzureIoTHubClient_t* xAzureIoTHubClient)
{
    AzureIoTJSONWriter_t json_writer;
    UINT lBytesWritten;
    AzureIoTResult_t xResult;

    if (handle == NULL)
    {
        return (eAzureIoTErrorFailed);
    }

    /* Get sensor data. */
    if ((xResult = get_sensor_data(handle)))
    {
        printf("Fetching Sensor data failed!: error code = 0x%08x\r\n", xResult);
        return (xResult);
    }
    memset(scratch_buffer, 0, sizeof(scratch_buffer));

    xResult = AzureIoTJSONWriter_Init(&json_writer, scratch_buffer, sizeof(scratch_buffer));
    configASSERT(xResult == eAzureIoTSuccess);
    // printf("init");

    xResult = AzureIoTJSONWriter_AppendBeginObject(&json_writer);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_soilMoistureExternal1Raw,
        strlen(telemetry_name_soilMoistureExternal1Raw),
        handle->soilMoistureExternal1Raw,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_soilMoistureExternal2Raw,
        strlen(telemetry_name_soilMoistureExternal2Raw),
        handle->soilMoistureExternal2Raw,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_pmsExternal1Raw,
        strlen(telemetry_name_pmsExternal1Raw),
        handle->pmsExternal1Raw,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_temperatureExternal2Raw,
        strlen(telemetry_name_temperatureExternal2Raw),
        handle->temperatureExternal2Raw,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);


    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_sensorTemperature,
        strlen(telemetry_name_sensorTemperature),
        handle->sensorTemperature,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_sensorPressure,
        strlen(telemetry_name_sensorPressure),
        handle->sensorPressure,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_sensorHumidity,
        strlen(telemetry_name_sensorHumidity),
        handle->sensorHumidity,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_sensorAcceleration,
        strlen(telemetry_name_sensorAcceleration),
        handle->sensorAcceleration,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&json_writer,
        (const uint8_t*)telemetry_name_sensorMagnetic,
        strlen(telemetry_name_sensorMagnetic),
        handle->sensorMagnetic,
        DOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendEndObject(&json_writer);
    configASSERT(xResult == eAzureIoTSuccess);

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed(&json_writer);

    /* Create and send the telemetry message packet. */
    if ((xResult = FreeRTOS_vt_verified_telemetry_message_create_send(handle->verified_telemetry_DB,
             xAzureIoTHubClient,
             handle->component_name_ptr,
             handle->component_name_length,
             (UCHAR*)scratch_buffer,
             lBytesWritten)))
    {
        printf("Verified Telemetry message create and send failed!: error code = 0x%08x\r\n", xResult);
        return (eAzureIoTErrorFailed);
    }

    printf("Component %.*s Telemetry message send: %.*s.\r\n\n",
        handle->component_name_length,
        handle->component_name_ptr,
        lBytesWritten,
        scratch_buffer);

    return (eAzureIoTSuccess);
}

AzureIoTResult_t sample_pnp_device_led_state_property(
    SAMPLE_PNP_DEVICE_COMPONENT* handle, AzureIoTHubClient_t* xAzureIoTHubClient)
{
    AzureIoTResult_t xResult;

    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    memset(ucPropertyPayloadBuffer, 0, sizeof(ucPropertyPayloadBuffer));

    xResult = AzureIoTJSONWriter_Init(&xWriter, ucPropertyPayloadBuffer, sizeof(ucPropertyPayloadBuffer));
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendBeginObject(&xWriter);
    configASSERT(xResult == eAzureIoTSuccess);

    AzureIoTHubClientProperties_BuilderBeginComponent(
        xAzureIoTHubClient, &xWriter, (const uint8_t*)handle->component_name_ptr, handle->component_name_length);

    xResult =
        AzureIoTJSONWriter_AppendPropertyName(&xWriter, (const uint8_t*)reported_led_state, strlen(reported_led_state));
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendBool(&xWriter, handle->sensorLEDState);
    configASSERT(xResult == eAzureIoTSuccess);

    AzureIoTHubClientProperties_BuilderEndComponent(xAzureIoTHubClient, &xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject(&xWriter);
    configASSERT(xResult == eAzureIoTSuccess);

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed(&xWriter);

    if (lBytesWritten < 0)
    {
        LogError(("Error getting the bytes written for the properties confirmation JSON"));
    }
    else
    {
        xResult =
            AzureIoTHubClient_SendPropertiesReported(xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL);

        if (xResult != eAzureIoTSuccess)
        {
            LogError(("There was an error sending the reported properties: 0x%08x", xResult));
        }
    }
    printf(" next2 ");

    return xResult;
}
