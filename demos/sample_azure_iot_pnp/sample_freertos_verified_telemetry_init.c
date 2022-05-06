/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "FreeRTOS_verified_telemetry.h"
#include "FreeRTOS_vt_fallcurve_component.h"
#include "FreeRTOS_vt_currentsense_component.h"
#include "sample_vt_device_driver.h"
#include <stdint.h>


//choose one of the below 3 configurations according to the set of sensors used 

#define BOTH_ANALOG_AND_DIGITAL_SENSORS
//#define ONLY_ANALOG_SENSORS
//#define ONLY_DIGITAL_SENSORS

static char scratch_buffer[VT_MINIMUM_BUFFER_SIZE_BYTES];

static FreeRTOS_VERIFIED_TELEMETRY_DB verified_telemetry_DB;

#if defined(BOTH_ANALOG_AND_DIGITAL_SENSORS) || defined(ONLY_ANALOG_SENSORS)
    static FreeRTOS_VT_OBJECT sample_signature_sensor_1;
    static FreeRTOS_VT_OBJECT sample_signature_sensor_2;
    static VT_SENSOR_HANDLE sample_handle_sensor_1;
    static VT_SENSOR_HANDLE sample_handle_sensor_2;
#endif

static VT_DEVICE_DRIVER sample_device_driver;

#if defined(BOTH_ANALOG_AND_DIGITAL_SENSORS) || defined(ONLY_DIGITAL_SENSORS)
    static FreeRTOS_VT_OBJECT sample_signature_sensor_3;
    static FreeRTOS_VT_OBJECT sample_signature_sensor_4;
    static VT_SENSOR_HANDLE sample_handle_sensor_3;
    static VT_SENSOR_HANDLE sample_handle_sensor_4;
#endif

AzureIoTResult_t vt_analog_sensor_signature_init(FreeRTOS_VERIFIED_TELEMETRY_DB* verified_telemetry_DB,
                                            FreeRTOS_VT_OBJECT* handle,
                                            UCHAR* component_name_ptr,
                                            bool telemetry_status_auto_update,
                                            uint16_t adc_id, 
                                            void * adc_controller, 
                                            void * adc_channel,
                                            uint16_t gpio_id,
                                            void * gpio_port,
                                            void * gpio_pin,
                                            VT_SENSOR_HANDLE* sensor_handle)
{
    UINT status;

    sensor_handle->adc_id         = adc_id;
    sensor_handle->adc_controller = adc_controller;
    sensor_handle->adc_channel    = adc_channel;
    sensor_handle->gpio_id        = gpio_id;
    sensor_handle->gpio_port      = gpio_port;
    sensor_handle->gpio_pin       = gpio_pin;

    if ((status = FreeRTOS_vt_signature_init(verified_telemetry_DB,
             handle,
             component_name_ptr,
             VT_SIGNATURE_TYPE_FALLCURVE,
             component_name_ptr,
             telemetry_status_auto_update,
             sensor_handle)))
    {   
        printf("Failed to initialize VT for soilMoistureExternal1 telemetry: error code = 0x%08x\r\n", status);
        return eAzureIoTErrorFailed;
    }

    return eAzureIoTSuccess;
}

AzureIoTResult_t vt_digital_sensor_signature_init(FreeRTOS_VERIFIED_TELEMETRY_DB* verified_telemetry_DB,
                                            FreeRTOS_VT_OBJECT* handle,
                                            UCHAR* component_name_ptr,
                                            bool telemetry_status_auto_update,
                                            uint16_t adc_id, 
                                            void * adc_controller, 
                                            void * adc_channel,
                                            VT_FLOAT currentsense_adc_ref_volt,
                                            VT_UINT currentsense_adc_resolution,
                                            VT_FLOAT currentsense_mV_to_mA,
                                            VT_SENSOR_HANDLE* sensor_handle)
{
    UINT status;

    sensor_handle->adc_id         = adc_id;
    sensor_handle->adc_controller = adc_controller;
    sensor_handle->adc_channel    = adc_channel;
    sensor_handle->currentsense_adc_ref_volt   = currentsense_adc_ref_volt;
    sensor_handle->currentsense_adc_resolution = currentsense_adc_resolution;
    sensor_handle->currentsense_mV_to_mA       = currentsense_mV_to_mA;

    if ((status = FreeRTOS_vt_signature_init(verified_telemetry_DB,
             handle,
             component_name_ptr,
             VT_SIGNATURE_TYPE_CURRENTSENSE,
             component_name_ptr,
             telemetry_status_auto_update,
             sensor_handle)))
    {   
        printf("Failed to initialize VT for soilMoistureExternal1 telemetry: error code = 0x%08x\r\n", status);
        return eAzureIoTErrorFailed;
    }

    return eAzureIoTSuccess;
}

FreeRTOS_VERIFIED_TELEMETRY_DB* sample_nx_verified_telemetry_user_init()
{
    UINT status;

    sample_device_driver.adc_single_read_init = &vt_adc_single_read_init;
    sample_device_driver.adc_single_read      = &vt_adc_single_read;
    sample_device_driver.adc_buffer_read      = &vt_adc_buffer_read;
    sample_device_driver.gpio_on              = &vt_gpio_on;
    sample_device_driver.gpio_off             = &vt_gpio_off;
    sample_device_driver.tick_init            = &vt_tick_init;
    sample_device_driver.tick_deinit          = &vt_tick_deinit;
    sample_device_driver.tick                 = &vt_tick;
    sample_device_driver.interrupt_enable     = &vt_interrupt_enable;
    sample_device_driver.interrupt_disable    = &vt_interrupt_disable;

    if ((status = FreeRTOS_vt_init(&verified_telemetry_DB, (UCHAR*)"vTDevice", true, &sample_device_driver,scratch_buffer,
             sizeof(scratch_buffer))))
    {
        printf("Failed to configure Verified Telemetry settings: error code = 0x%08x\r\n", status);
    }

    #if defined(BOTH_ANALOG_AND_DIGITAL_SENSORS) || defined(ONLY_ANALOG_SENSORS)

        vt_analog_sensor_signature_init(&verified_telemetry_DB,
                                        &sample_signature_sensor_1,
                                        (UCHAR*)"soilMoistureExternal1",
                                        true,
                                        vt_adc_id_sensor_1,
                                        (void*)&vt_adc_controller_sensor_1,
                                        (void*)&vt_adc_channel_sensor_1,
                                        vt_gpio_id_sensor_1,
                                        (void*)vt_gpio_port_sensor_1,
                                        (void*)&vt_gpio_pin_sensor_1,
                                        &sample_handle_sensor_1);

        vt_analog_sensor_signature_init(&verified_telemetry_DB,
                                        &sample_signature_sensor_2,
                                        (UCHAR*)"soilMoistureExternal2",
                                        true,
                                        vt_adc_id_sensor_2,
                                        (void*)&vt_adc_controller_sensor_2,
                                        (void*)&vt_adc_channel_sensor_2,
                                        vt_gpio_id_sensor_2,
                                        (void*)vt_gpio_port_sensor_2,
                                        (void*)&vt_gpio_pin_sensor_2,
                                        &sample_handle_sensor_2);

    #endif

    #if defined(BOTH_ANALOG_AND_DIGITAL_SENSORS) || defined(ONLY_DIGITAL_SENSORS)

        vt_digital_sensor_signature_init(&verified_telemetry_DB,
                                        &sample_signature_sensor_3,
                                        (UCHAR*)"PMSExternal1",
                                        true,
                                        vt_adc_id_sensor_3,
                                        (void*)&vt_adc_controller_sensor_3,
                                        (void*)&vt_adc_channel_sensor_3,
                                        CURRENTSENSE_EXTERNAL_ADC_REF_VOLT,
                                        CURRENTSENSE_EXTERNAL_ADC_RESOLUTION,
                                        (1.0f / (CURRENTSENSE_SHUNT_RESISTOR * CURRENTSENSE_OPAMP_GAIN)),
                                        &sample_handle_sensor_3);

        vt_digital_sensor_signature_init(&verified_telemetry_DB,
                                        &sample_signature_sensor_4,
                                        (UCHAR*)"temperatureExternal2",
                                        true,
                                        vt_adc_id_sensor_4,
                                        (void*)&vt_adc_controller_sensor_4,
                                        (void*)&vt_adc_channel_sensor_4,
                                        CURRENTSENSE_EXTERNAL_ADC_REF_VOLT,
                                        CURRENTSENSE_EXTERNAL_ADC_RESOLUTION,
                                        (1.0f / (CURRENTSENSE_SHUNT_RESISTOR * CURRENTSENSE_OPAMP_GAIN)),
                                        &sample_handle_sensor_4);

    #endif

    return (&verified_telemetry_DB);
}