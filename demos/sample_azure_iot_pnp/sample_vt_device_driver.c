/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include <stdio.h>
#include "sample_vt_device_driver.h"
#include <stdint.h>


uint16_t vt_tick_timer;

/* ADC Definitions */
uint16_t vt_adc_id_sensor_1 ;
uint16_t vt_adc_id_sensor_2 ;

uint16_t vt_adc_controller_sensor_1 ;
uint16_t vt_adc_controller_sensor_2 ;

uint32_t vt_adc_channel_sensor_1 ;
uint32_t vt_adc_channel_sensor_2 ;

/* GPIO Definitions */
uint16_t vt_gpio_id_sensor_1 ;
uint16_t vt_gpio_id_sensor_2 ;

uint16_t* vt_gpio_port_sensor_1 ;
uint16_t* vt_gpio_port_sensor_2 ;

uint16_t vt_gpio_pin_sensor_1 ;
uint16_t vt_gpio_pin_sensor_2 ;


uint16_t vt_adc_init(uint16_t adc_id, void* adc_controller, void* adc_channel, uint16_t* adc_resolution, float* adc_ref_volt)
{
  return 0;

}

uint16_t vt_adc_read(uint16_t adc_id, void* adc_controller, void* adc_channel)
{
  return 0;

}

uint16_t vt_gpio_on(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{
  return 0;

}

uint16_t vt_gpio_off(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{
  return 0;

}

uint16_t vt_tick_init(uint16_t* max_value, uint16_t* resolution_usec)
{
    return 0;

}

unsigned long vt_tick_deinit()
{
  return 0;

}

unsigned long vt_tick()
{
  return 0;

}

void vt_interrupt_enable()
{

}

void vt_interrupt_disable()
{

}

