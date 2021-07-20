/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "FreeRTOS_verified_telemetry.h"
#include "FreeRTOS_vt_fallcurve_component.h"
#include "sample_vt_device_driver.h"
#include <stdint.h>


static FreeRTOS_VERIFIED_TELEMETRY_DB verified_telemetry_DB;

static FreeRTOS_VT_OBJECT sample_signature_sensor_1;

static FreeRTOS_VT_OBJECT sample_signature_sensor_2;

static VT_DEVICE_DRIVER sample_device_driver;

static VT_SENSOR_HANDLE sample_handle_sensor_1;
static VT_SENSOR_HANDLE sample_handle_sensor_2;

FreeRTOS_VERIFIED_TELEMETRY_DB* sample_nx_verified_telemetry_user_init()
{
    UINT status;

    sample_device_driver.adc_single_read_init = &vt_adc_single_read_init;
    sample_device_driver.adc_single_read      = &vt_adc_single_read;
    sample_device_driver.gpio_on              = &vt_gpio_on;
    sample_device_driver.gpio_off             = &vt_gpio_off;
    sample_device_driver.tick_init            = &vt_tick_init;
    sample_device_driver.tick_deinit          = &vt_tick_deinit;
    sample_device_driver.tick                 = &vt_tick;
    sample_device_driver.interrupt_enable     = &vt_interrupt_enable;
    sample_device_driver.interrupt_disable    = &vt_interrupt_disable;

    if ((status = FreeRTOS_vt_init(&verified_telemetry_DB, (UCHAR*)"vTDevice", true, &sample_device_driver)))
    {
        printf("Failed to configure Verified Telemetry settings: error code = 0x%08x\r\n", status);
    }

    sample_handle_sensor_1.adc_id         = vt_adc_id_sensor_1;
    sample_handle_sensor_1.adc_controller = (void*)&vt_adc_controller_sensor_1;
    sample_handle_sensor_1.adc_channel    = (void*)&vt_adc_channel_sensor_1;
    sample_handle_sensor_1.gpio_id        = vt_gpio_id_sensor_1;
    sample_handle_sensor_1.gpio_port      = (void*)vt_gpio_port_sensor_1;
    sample_handle_sensor_1.gpio_pin       = (void*)&vt_gpio_pin_sensor_1;

    if ((status = FreeRTOS_vt_signature_init(&verified_telemetry_DB,
             &sample_signature_sensor_1,
             (UCHAR*)"soilMoistureExternal1",
             VT_SIGNATURE_TYPE_FALLCURVE,
             (UCHAR*)"soilMoistureExternal1",
             true,
             &sample_handle_sensor_1)))
    {
        printf("Failed to initialize VT for soilMoistureExternal1 telemetry: error code = 0x%08x\r\n", status);
    }

    sample_handle_sensor_2.adc_id         = vt_adc_id_sensor_2;
    sample_handle_sensor_2.adc_controller = (void*)&vt_adc_controller_sensor_2;
    sample_handle_sensor_2.adc_channel    = (void*)&vt_adc_channel_sensor_2;
    sample_handle_sensor_2.gpio_id        = vt_gpio_id_sensor_2;
    sample_handle_sensor_2.gpio_port      = (void*)vt_gpio_port_sensor_2;
    sample_handle_sensor_2.gpio_pin       = (void*)&vt_gpio_pin_sensor_2;

    if ((status = FreeRTOS_vt_signature_init(&verified_telemetry_DB,
             &sample_signature_sensor_2,
             (UCHAR*)"soilMoistureExternal2",
             VT_SIGNATURE_TYPE_FALLCURVE,
             (UCHAR*)"soilMoistureExternal2",
             true,
             &sample_handle_sensor_2)))
    {
        printf("Failed to initialize VT for soilMoistureExternal2 telemetry: error code = 0x%08x\r\n", status);
    }

    return (&verified_telemetry_DB);
}