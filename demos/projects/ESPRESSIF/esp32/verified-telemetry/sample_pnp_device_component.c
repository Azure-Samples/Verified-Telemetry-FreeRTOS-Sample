/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "sample_pnp_device_component.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/adc.h"
//#include "esp_adc_cal.h"
/* Azure Provisioning/IoT Hub library includes */
#include "azure_iot_hub_client.h"
#include "azure_iot_hub_client_properties.h"
#include "azure_iot_provisioning_client.h"


/* Azure JSON includes */
#include "azure_iot_json_reader.h"
#include "azure_iot_json_writer.h"

#include "azure_iot_hub_client_properties.h"

#define DOUBLE_DECIMAL_PLACE_DIGITS   (2)
#define SAMPLE_COMMAND_SUCCESS_STATUS (200)
#define SAMPLE_COMMAND_ERROR_STATUS   (500)

/* Telemetry key */
static const CHAR telemetry_name_soilMoistureExternal1Raw[] = "soilMoistureExternal1";
static const CHAR telemetry_name_soilMoistureExternal2Raw[] = "soilMoistureExternal2";
static const CHAR telemetry_name_sensorTemperature[]        = "temperature";
static const CHAR telemetry_name_sensorPressure[]           = "pressure";
static const CHAR telemetry_name_sensorHumidity[]           = "humidityPercentage";
static const CHAR telemetry_name_sensorAcceleration[]       = "acceleration";
static const CHAR telemetry_name_sensorMagnetic[]           = "magnetic";

/* Pnp command supported */
static const CHAR set_led_state[] = "setLedState";

/* Names of properties for desired/reporting */
static const CHAR reported_led_state[] = "ledState";
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static UCHAR scratch_buffer[512];
static uint8_t ucPropertyPayloadBuffer[256];

adc_unit_t vt_adc_controller = ADC_UNIT_1;
extern uint32_t vt_adc_channel_sensor_1 ;
extern uint32_t vt_adc_channel_sensor_2 ;

#define GPIO_LED_PIN    2
#define GPIO_LED_PIN_SEL  (1ULL<<GPIO_LED_PIN)

#define GPIO_LED_PIN    2
#define GPIO_LED_PIN_SEL  (1ULL<<GPIO_LED_PIN)

static void set_led_state_action(bool level)
{
    if (level)
    {
        printf("LED is turned OFF\r\n");

        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        gpio_set_level(GPIO_LED_PIN, 1);
    }
    else
    {
        printf("LED is turned OFF\r\n");

        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        gpio_set_level(GPIO_LED_PIN, 0);
    }
}

UINT adc_read(adc_unit_t* ADC_Controller, UINT ADC_Channel)
{
    int adc_raw = 0;
    int adc_raw_abs = 0;

    adc_unit_t unit = *ADC_Controller;
    adc_channel_t channel = (adc_channel_t)ADC_Channel;

    if (unit == ADC_UNIT_1)
    {
        adc_raw=adc1_get_raw(channel);
    }
    else
    {
        adc2_get_raw(channel, width, &adc_raw);
    }
    printf("adc_raw - %d \n",adc_raw);
    adc_raw_abs=abs(adc_raw-4095);
    return (UINT)adc_raw_abs;
}

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

        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = ((1ULL<<GPIO_NUM_18) | (1ULL<<GPIO_NUM_19));
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        gpio_set_level(GPIO_NUM_18, 1);
        gpio_set_level(GPIO_NUM_19, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);

    UINT soilMoisture1ADCData = adc_read(&vt_adc_controller, vt_adc_channel_sensor_1);
    UINT soilMoisture2ADCData = adc_read(&vt_adc_controller, vt_adc_channel_sensor_2);
    /*
    float temperature = BSP_TSENSOR_ReadTemp();
    float humidity    = BSP_HSENSOR_ReadHumidity();
    float pressure    = BSP_PSENSOR_ReadPressure();
    int16_t magnetoXYZ[3];
    BSP_MAGNETO_GetXYZ(magnetoXYZ);
    int16_t accXYZ[3];
    BSP_ACCELERO_AccGetXYZ(accXYZ);
    */
    handle->soilMoistureExternal1Raw = soilMoisture1ADCData;
    handle->soilMoistureExternal2Raw = soilMoisture2ADCData;

    handle->sensorTemperature  = 0;
    handle->sensorPressure     = 0;
    handle->sensorHumidity     = 0;
    handle->sensorAcceleration = 0;
    handle->sensorMagnetic     = 0;

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
