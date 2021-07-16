/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include <stdio.h>
#include "sample_vt_device_driver.h"

#define SAMPLE_INTERNAL_ADC_TYPE_ID 0x01
#define SAMPLE_INTERNAL_GPIO_TYPE_ID 0x01

TIM_HandleTypeDef vt_tick_timer;

/* ADC Definitions */
uint16_t vt_adc_id_sensor_1 = SAMPLE_INTERNAL_ADC_TYPE_ID;
uint16_t vt_adc_id_sensor_2 = SAMPLE_INTERNAL_ADC_TYPE_ID;

ADC_HandleTypeDef vt_adc_controller_sensor_1 = { .Instance = ADC1};
ADC_HandleTypeDef vt_adc_controller_sensor_2 = { .Instance = ADC1};

uint32_t vt_adc_channel_sensor_1 = ADC_CHANNEL_1;
uint32_t vt_adc_channel_sensor_2 = ADC_CHANNEL_2;

/* GPIO Definitions */
uint16_t vt_gpio_id_sensor_1 = SAMPLE_INTERNAL_GPIO_TYPE_ID;
uint16_t vt_gpio_id_sensor_2 = SAMPLE_INTERNAL_GPIO_TYPE_ID;

GPIO_TypeDef* vt_gpio_port_sensor_1 = GPIOB;
GPIO_TypeDef* vt_gpio_port_sensor_2 = GPIOB;

uint16_t vt_gpio_pin_sensor_1 = GPIO_PIN_9;
uint16_t vt_gpio_pin_sensor_2 = GPIO_PIN_8;


uint16_t vt_adc_init(uint16_t adc_id, void* adc_controller, void* adc_channel, uint16_t* adc_resolution, float* adc_ref_volt)
{
  if(adc_id == SAMPLE_INTERNAL_ADC_TYPE_ID)
  {
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
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
    *adc_ref_volt = 3.3f;
  }
  return 0;
}

uint16_t vt_adc_read(uint16_t adc_id, void* adc_controller, void* adc_channel)
{
  uint16_t value = 0;

  if(adc_id == SAMPLE_INTERNAL_ADC_TYPE_ID)
  {
    ADC_HandleTypeDef* vt_adc_controller = (ADC_HandleTypeDef*)adc_controller;
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = *((uint32_t*)adc_channel);
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    HAL_ADC_ConfigChannel(vt_adc_controller, &sConfig);
    HAL_ADC_Start(vt_adc_controller);
    if (HAL_ADC_PollForConversion(vt_adc_controller, 10) == HAL_OK)
    {
      value =  HAL_ADC_GetValue(vt_adc_controller);
    }
    HAL_ADC_Stop(vt_adc_controller);
  }

  return value;
}

uint16_t vt_gpio_on(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{
  if(gpio_id == SAMPLE_INTERNAL_GPIO_TYPE_ID)
  {
    HAL_GPIO_WritePin((GPIO_TypeDef*)gpio_port, *((uint16_t*)gpio_pin), GPIO_PIN_SET);
  }
  return 0;
}

uint16_t vt_gpio_off(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{
  if(gpio_id == SAMPLE_INTERNAL_GPIO_TYPE_ID)
  {
    HAL_GPIO_WritePin((GPIO_TypeDef*)gpio_port, *((uint16_t*)gpio_pin), GPIO_PIN_RESET);
  }
  return 0;
}

uint16_t vt_tick_init(uint16_t* max_value, uint16_t* resolution_usec)
{
  uint16_t default_max_tick = 65535;
  uint16_t default_tick_resolution = 1;
  if(*max_value)
  {
    default_max_tick = *max_value;
  }
  else
  {
    *max_value = default_max_tick;
  }
  if(*resolution_usec)
  {
    default_tick_resolution = *resolution_usec;
  }
  else
  {
    *resolution_usec = default_tick_resolution;
  }
  
  vt_tick_timer.Instance               = TIM7;
  vt_tick_timer.Init.Prescaler         = (80*default_tick_resolution) - 1;
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

