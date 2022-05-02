/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#ifndef _SAMPLE_VT_DEVICE_DRIVER_H
#define _SAMPLE_VT_DEVICE_DRIVER_H

#include "stm32l4xx_hal.h"
#include "vt_device_driver.h"
#include <stdint.h>

#define CURRENTSENSE_EXTERNAL_ADC_REF_VOLT   5.0f
#define CURRENTSENSE_EXTERNAL_ADC_RESOLUTION 12.0f
#define CURRENTSENSE_SHUNT_RESISTOR          1.0f
#define CURRENTSENSE_OPAMP_GAIN              50.0f


/* Sensor Hardware Declaration */

//sensor_1
extern uint16_t vt_adc_id_sensor_1;
extern ADC_HandleTypeDef vt_adc_controller_sensor_1;
extern uint32_t vt_adc_channel_sensor_1;
extern uint16_t vt_gpio_id_sensor_1;
extern GPIO_TypeDef* vt_gpio_port_sensor_1;
extern uint16_t vt_gpio_pin_sensor_1;

//sensor_2
extern uint16_t vt_adc_id_sensor_2;
extern ADC_HandleTypeDef vt_adc_controller_sensor_2;
extern uint32_t vt_adc_channel_sensor_2;
extern uint16_t vt_gpio_id_sensor_2;
extern GPIO_TypeDef* vt_gpio_port_sensor_2;
extern uint16_t vt_gpio_pin_sensor_2;

//sensor_3
extern uint16_t vt_adc_id_sensor_3;
extern SPI_HandleTypeDef vt_adc_controller_sensor_3;
extern uint32_t vt_adc_channel_sensor_3;

//sensor_4
extern uint16_t vt_adc_id_sensor_4;
extern SPI_HandleTypeDef vt_adc_controller_sensor_4;
extern uint32_t vt_adc_channel_sensor_4;


#endif // _SAMPLE_VT_DEVICE_DRIVER_H
