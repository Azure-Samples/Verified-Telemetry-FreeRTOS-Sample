/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "sample_vt_device_driver.h"
#include <stdio.h>


#define SAMPLE_INTERNAL_ADC_TYPE_ID  0x01
#define SAMPLE_INTERNAL_GPIO_TYPE_ID 0x01
#define SAMPLE_EXTERNAL_ADC_TYPE_ID  0x02

#define SAMPLE_MCP3204_SLAVE_SELECT_GPIO_PORT          GPIOA
#define SAMPLE_MCP3204_SLAVE_SELECT_GPIO_PIN           GPIO_PIN_2
#define SAMPLE_MCP3204_SPI_CLOCK_CYCLES_FOR_CONVERSION 24

TIM_HandleTypeDef vt_tick_timer;

/* ADC Definitions */
uint16_t vt_adc_id_sensor_1 = SAMPLE_INTERNAL_ADC_TYPE_ID;
uint16_t vt_adc_id_sensor_2 = SAMPLE_INTERNAL_ADC_TYPE_ID;
uint16_t vt_adc_id_sensor_3 = SAMPLE_EXTERNAL_ADC_TYPE_ID;
uint16_t vt_adc_id_sensor_4 = SAMPLE_EXTERNAL_ADC_TYPE_ID;

ADC_HandleTypeDef vt_adc_controller_sensor_1 = {.Instance = ADC1};
ADC_HandleTypeDef vt_adc_controller_sensor_2 = {.Instance = ADC1};
SPI_HandleTypeDef vt_adc_controller_sensor_3 = {.Instance = SPI1};
SPI_HandleTypeDef vt_adc_controller_sensor_4 = {.Instance = SPI1};

uint32_t vt_adc_channel_sensor_1 = ADC_CHANNEL_1;
uint32_t vt_adc_channel_sensor_2 = ADC_CHANNEL_2;
uint32_t vt_adc_channel_sensor_3 = 0;
uint32_t vt_adc_channel_sensor_4 = 1;

/* GPIO Definitions */
uint16_t vt_gpio_id_sensor_1 = SAMPLE_INTERNAL_GPIO_TYPE_ID;
uint16_t vt_gpio_id_sensor_2 = SAMPLE_INTERNAL_GPIO_TYPE_ID;

GPIO_TypeDef* vt_gpio_port_sensor_1 = GPIOB;
GPIO_TypeDef* vt_gpio_port_sensor_2 = GPIOB;

uint16_t vt_gpio_pin_sensor_1 = GPIO_PIN_9;
uint16_t vt_gpio_pin_sensor_2 = GPIO_PIN_8;

/* Variables needed for External ADC Buffer Read */
uint8_t adc_mcp3204_TxData[3];
uint8_t adc_mcp3204_RxData[3];
void* adc_mcp3204_spi_handle;
float* adc_mcp3204_read_buffer_local;
uint16_t adc_mcp3204_read_buffer_length_local;
uint16_t adc_mcp3204_read_buffer_datapoints_stored = 0;
typedef void (*VT_ADC_BUFFER_READ_CALLBACK_FUNC)(void);
VT_ADC_BUFFER_READ_CALLBACK_FUNC adc_mcp3204_read_buffer_half_complete_callback;
VT_ADC_BUFFER_READ_CALLBACK_FUNC adc_mcp3204_read_buffer_full_complete_callback;

uint16_t vt_adc_single_read_init(
    uint16_t adc_id, void* adc_controller, void* adc_channel, uint16_t* adc_resolution, float* adc_ref_volt)
{
    if (adc_id == SAMPLE_INTERNAL_ADC_TYPE_ID)
    {
        ADC_MultiModeTypeDef multimode       = {0};
        ADC_ChannelConfTypeDef sConfig       = {0};
        ADC_HandleTypeDef* vt_adc_controller = (ADC_HandleTypeDef*)adc_controller;

        vt_adc_controller->Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
        vt_adc_controller->Init.Resolution            = ADC_RESOLUTION_12B;
        vt_adc_controller->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        vt_adc_controller->Init.ScanConvMode          = ADC_SCAN_DISABLE;
        vt_adc_controller->Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
        vt_adc_controller->Init.LowPowerAutoWait      = DISABLE;
        vt_adc_controller->Init.ContinuousConvMode    = DISABLE;
        vt_adc_controller->Init.NbrOfConversion       = 1;
        vt_adc_controller->Init.DiscontinuousConvMode = DISABLE;
        vt_adc_controller->Init.NbrOfDiscConversion   = 1;
        vt_adc_controller->Init.ExternalTrigConv      = ADC_SOFTWARE_START;
        vt_adc_controller->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
        vt_adc_controller->Init.DMAContinuousRequests = DISABLE;
        vt_adc_controller->Init.Overrun               = ADC_OVR_DATA_PRESERVED;
        vt_adc_controller->Init.OversamplingMode      = DISABLE;
        if (HAL_ADC_Init(vt_adc_controller) != HAL_OK)
        {
            // add error handling
        }
        /** Configure the ADC multi-mode
         */
        multimode.Mode = ADC_MODE_INDEPENDENT;
        if (HAL_ADCEx_MultiModeConfigChannel(vt_adc_controller, &multimode) != HAL_OK)
        {
            // add error handling
        }

        sConfig.Channel      = ADC_CHANNEL_1;
        sConfig.Rank         = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
        sConfig.SingleDiff   = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset       = 0;
        if (HAL_ADC_ConfigChannel(vt_adc_controller, &sConfig) != HAL_OK)
        {
            // add error handling
        }
        *adc_resolution = 12;
        *adc_ref_volt   = 3.3f;
    }
    return 0;
}

uint16_t vt_adc_single_read(uint16_t adc_id, void* adc_controller, void* adc_channel)
{
    uint16_t value = 0;

    if (adc_id == SAMPLE_INTERNAL_ADC_TYPE_ID)
    {
        ADC_HandleTypeDef* vt_adc_controller = (ADC_HandleTypeDef*)adc_controller;
        ADC_ChannelConfTypeDef sConfig       = {0};
        sConfig.Channel                      = *((uint32_t*)adc_channel);
        sConfig.Rank                         = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime                 = ADC_SAMPLETIME_2CYCLES_5;
        sConfig.SingleDiff                   = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber                 = ADC_OFFSET_NONE;
        sConfig.Offset                       = 0;
        HAL_ADC_ConfigChannel(vt_adc_controller, &sConfig);
        HAL_ADC_Start(vt_adc_controller);
        if (HAL_ADC_PollForConversion(vt_adc_controller, 10) == HAL_OK)
        {
            value = HAL_ADC_GetValue(vt_adc_controller);
        }
        HAL_ADC_Stop(vt_adc_controller);
    }

    return value;
}

void sample_mcp3208_read_start()
{
    
    HAL_GPIO_WritePin(SAMPLE_MCP3204_SLAVE_SELECT_GPIO_PORT, SAMPLE_MCP3204_SLAVE_SELECT_GPIO_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Init((SPI_HandleTypeDef*)adc_mcp3204_spi_handle) != HAL_OK)
    {
        // Handle error
    }
    HAL_SPI_TransmitReceive_IT((SPI_HandleTypeDef*)adc_mcp3204_spi_handle, adc_mcp3204_TxData, adc_mcp3204_RxData, 3);
}

void sample_mcp3208_read_stop()
{
    
    HAL_GPIO_WritePin(SAMPLE_MCP3204_SLAVE_SELECT_GPIO_PORT, SAMPLE_MCP3204_SLAVE_SELECT_GPIO_PIN, GPIO_PIN_SET);
    if (HAL_SPI_DeInit((SPI_HandleTypeDef*)adc_mcp3204_spi_handle) != HAL_OK)
    {
        // Handle error
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{   
    sample_mcp3208_read_stop();

    adc_mcp3204_read_buffer_local[adc_mcp3204_read_buffer_datapoints_stored] =
        (uint16_t)(adc_mcp3204_RxData[1] & 0x0F) << 8 | (uint16_t)adc_mcp3204_RxData[2];
    adc_mcp3204_read_buffer_datapoints_stored++;

    if (adc_mcp3204_read_buffer_datapoints_stored == adc_mcp3204_read_buffer_length_local / 2)
    {
        adc_mcp3204_read_buffer_half_complete_callback();
    }
    else if (adc_mcp3204_read_buffer_datapoints_stored == adc_mcp3204_read_buffer_length_local)
    {
        adc_mcp3204_read_buffer_full_complete_callback();
    }
    if (adc_mcp3204_read_buffer_datapoints_stored < adc_mcp3204_read_buffer_length_local)
    {
        sample_mcp3208_read_start();
    }
}

void SPI1_IRQHandler(void)
{
    /* USER CODE BEGIN SPI1_IRQn 0 */

    /* USER CODE END SPI1_IRQn 0 */
    HAL_SPI_IRQHandler((SPI_HandleTypeDef*)adc_mcp3204_spi_handle);
    /* USER CODE BEGIN SPI1_IRQn 1 */

    /* USER CODE END SPI1_IRQn 1 */
}

void vt_adc_buffer_read(uint16_t adc_id,
    void* adc_controller,
    void* adc_channel,
    float* adc_read_buffer,
    uint16_t buffer_length,
    float desired_sampling_frequency,
    float* set_sampling_frequency,
    void (*vt_adc_buffer_read_conv_half_cplt_callback)(),
    void (*vt_adc_buffer_read_conv_cplt_callback)())
{
    
    if (adc_id == SAMPLE_EXTERNAL_ADC_TYPE_ID)
    {
        adc_mcp3204_spi_handle                         = adc_controller;
        adc_mcp3204_read_buffer_half_complete_callback = vt_adc_buffer_read_conv_half_cplt_callback;
        adc_mcp3204_read_buffer_full_complete_callback = vt_adc_buffer_read_conv_cplt_callback;
        adc_mcp3204_read_buffer_local                  = adc_read_buffer;
        adc_mcp3204_read_buffer_length_local           = buffer_length;
        adc_mcp3204_read_buffer_datapoints_stored      = 0;

        /* SPI1 parameter configuration*/
        SPI_HandleTypeDef* vt_adc_controller   = (SPI_HandleTypeDef*)adc_controller;
        vt_adc_controller->Init.Mode           = SPI_MODE_MASTER;
        vt_adc_controller->Init.Direction      = SPI_DIRECTION_2LINES;
        vt_adc_controller->Init.DataSize       = SPI_DATASIZE_8BIT;
        vt_adc_controller->Init.CLKPolarity    = SPI_POLARITY_LOW;
        vt_adc_controller->Init.CLKPhase       = SPI_PHASE_1EDGE;
        vt_adc_controller->Init.NSS            = SPI_NSS_SOFT;
        vt_adc_controller->Init.FirstBit       = SPI_FIRSTBIT_MSB;
        vt_adc_controller->Init.TIMode         = SPI_TIMODE_DISABLE;
        vt_adc_controller->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        vt_adc_controller->Init.CRCPolynomial  = 7;
        vt_adc_controller->Init.CRCLength      = SPI_CRC_LENGTH_DATASIZE;
        vt_adc_controller->Init.NSSPMode       = SPI_NSS_PULSE_DISABLE;

        uint32_t baudrate_prescaler = 256;
        if (desired_sampling_frequency > 1)
        {
            baudrate_prescaler = (uint32_t)(
                (SystemCoreClock / (desired_sampling_frequency * SAMPLE_MCP3204_SPI_CLOCK_CYCLES_FOR_CONVERSION)) - 1);
        } 
        if (baudrate_prescaler < 2)
        {
            vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
            baudrate_prescaler                        = 2;
        }
        else if (baudrate_prescaler < 4)
        {
            vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
            baudrate_prescaler                        = 4;
        }
        else if (baudrate_prescaler < 8)
        {
            vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
            baudrate_prescaler                        = 8;
        }
        else if (baudrate_prescaler < 16)
        {
            vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
            baudrate_prescaler                        = 16;
        }
        else if (baudrate_prescaler < 21)
        {
            vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
            baudrate_prescaler                        = 32;
        }
        else if (baudrate_prescaler < 64)
        {
            vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
            baudrate_prescaler                        = 64;
        }
        else if (baudrate_prescaler < 128)
        {
            vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
            baudrate_prescaler                        = 128;
        }
        else
        {
            vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
            baudrate_prescaler                        = 256;
        }

        *set_sampling_frequency = ((float)SystemCoreClock / ((float)(baudrate_prescaler + 1) *
                                                                (float)SAMPLE_MCP3204_SPI_CLOCK_CYCLES_FOR_CONVERSION));

        adc_mcp3204_TxData[0] = 0b110;
        adc_mcp3204_TxData[1] = (*(uint8_t*)adc_channel) << 6;
        adc_mcp3204_TxData[2] = 0b0;
        
        sample_mcp3208_read_stop();
        sample_mcp3208_read_start();

        //printf("MCP3204 read start \r\n");
    }
}

uint16_t vt_gpio_on(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{
    if (gpio_id == SAMPLE_INTERNAL_GPIO_TYPE_ID)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef*)gpio_port, *((uint16_t*)gpio_pin), GPIO_PIN_SET);
    }
    return 0;
}

uint16_t vt_gpio_off(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{
    if (gpio_id == SAMPLE_INTERNAL_GPIO_TYPE_ID)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef*)gpio_port, *((uint16_t*)gpio_pin), GPIO_PIN_RESET);
    }
    return 0;
}

uint16_t vt_tick_init(uint16_t* max_value, uint16_t* resolution_usec)
{
    uint16_t default_max_tick        = 65535;
    uint16_t default_tick_resolution = 1;
    if (*max_value)
    {
        default_max_tick = *max_value;
    }
    else
    {
        *max_value = default_max_tick;
    }
    if (*resolution_usec)
    {
        default_tick_resolution = *resolution_usec;
    }
    else
    {
        *resolution_usec = default_tick_resolution;
    }

    vt_tick_timer.Instance               = TIM7;
    vt_tick_timer.Init.Prescaler         = (80 * default_tick_resolution) - 1;
    vt_tick_timer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    vt_tick_timer.Init.Period            = 65535;
    vt_tick_timer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    vt_tick_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&vt_tick_timer) != HAL_OK)
    {
        // add error handling
    }
    HAL_TIM_Base_Start(&vt_tick_timer);
    return 0;
}

unsigned long vt_tick_deinit()
{
    HAL_TIM_Base_Stop(&vt_tick_timer);
    HAL_TIM_Base_DeInit(&vt_tick_timer);
    return 0;
}

unsigned long vt_tick()
{
    return __HAL_TIM_GET_COUNTER(&vt_tick_timer);
}

void vt_interrupt_enable()
{
    __enable_irq();
}

void vt_interrupt_disable()
{
    __disable_irq();
}
