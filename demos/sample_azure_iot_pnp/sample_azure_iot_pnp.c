/* Copyright (c) Microsoft Corporation. All rights reserved. */
/* SPDX-License-Identifier: MIT */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <time.h>


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

/* Exponential backoff retry include. */
#include "backoff_algorithm.h"

/* Transport interface implementation include header for TLS. */
#include "sample_pnp_device_component.h"
#include "transport_tls_socket.h"


/* Crypto helper header. */
#include "crypto.h"

/* Demo Specific configs. */
#include "demo_config.h"

#include "FreeRTOS_verified_telemetry.h"
#include "FreeRTOS_vt_fallcurve_component.h"
#include "FreeRTOS_vt_currentsense_component.h"
#include "sample_freertos_verified_telemetry_init.h"
#include "sample_vt_device_driver.h"      


#ifdef USINGESP
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = SPICOMMON_BUSFLAG_MASTER, 
        .intr_flags= ESP_INTR_FLAG_IRAM
        };
#endif

/*-----------------------------------------------------------*/

/* Compile time error for undefined configs. */
#if !defined(democonfigHOSTNAME) && !defined(democonfigENABLE_DPS_SAMPLE)
#error "Define the config democonfigHOSTNAME by following the instructions in file demo_config.h."
#endif

#if !defined(democonfigENDPOINT) && defined(democonfigENABLE_DPS_SAMPLE)
#error "Define the config dps endpoint by following the instructions in file demo_config.h."
#endif

#ifndef democonfigROOT_CA_PEM
#error "Please define Root CA certificate of the IoT Hub(democonfigROOT_CA_PEM) in demo_config.h."
#endif

#if defined(democonfigDEVICE_SYMMETRIC_KEY) && defined(democonfigCLIENT_CERTIFICATE_PEM)
#error                                                                                                                 \
    "Please define only one auth democonfigDEVICE_SYMMETRIC_KEY or democonfigCLIENT_CERTIFICATE_PEM in demo_config.h."
#endif

#if !defined(democonfigDEVICE_SYMMETRIC_KEY) && !defined(democonfigCLIENT_CERTIFICATE_PEM)
#error "Please define one auth democonfigDEVICE_SYMMETRIC_KEY or democonfigCLIENT_CERTIFICATE_PEM in demo_config.h."
#endif

/*-----------------------------------------------------------*/

/**
 * @brief The maximum number of retries for network operation with server.
 */
#define sampleazureiotRETRY_MAX_ATTEMPTS (5U)

/**
 * @brief The maximum back-off delay (in milliseconds) for retrying failed operation
 *  with server.
 */
#define sampleazureiotRETRY_MAX_BACKOFF_DELAY_MS (5000U)

/**
 * @brief The base back-off delay (in milliseconds) to use for network operation retry
 * attempts.
 */
#define sampleazureiotRETRY_BACKOFF_BASE_MS (500U)

/**
 * @brief Timeout for receiving CONNACK packet in milliseconds.
 */
#define sampleazureiotCONNACK_RECV_TIMEOUT_MS (10 * 1000U)


#define sampleazureiotMODEL_ID "dtmi:azure:verifiedtelemetry:sample:GSG;1"

/**
 * @brief Property Values
 */
#define sampleazureiotCOMPONENT_NAME                        "sampleComponent"
#define sampleazureiotPROPERTY_REPORTED_STATUS              "Status"
#define sampleazureiotsampleDeviceCOMPONENT_NAME            "sampleDevice"
#define sampleazureiotvTDeviceCOMPONENT_NAME                "vTDevice"

/**
 * @brief Command values
 */

#define sampleazureiotCOMMAND_EMPTY_PAYLOAD            "{}"



// static int32_t ConfidenceMetriclocalone;
// static int32_t ConfidenceMetriclocaltwo;

// vt variables

#define NUM_COMPONENTS 6
static bool deviceStatusReportedProperty;
static bool enableVerifiedTelemetryWritableProperty;
//static bool setLedState;
// static bool telemetryStatusSoilMoisture1;
// static bool telemetryStatusSoilMoisture2;
#define sampleazureiotPROPERTY_ENABLE_VERIFIED_TELEMETRY "enableVerifiedTelemetry"
#define sampleazureiotSAMPLE_DEVICE_COMPONENT_NAME       "sampleDevice"
#define sampleazureiotSOIL_MOISTURE_ONE_COMPONENT_NAME   "vTsoilMoistureExternal1"
#define sampleazureiotSOIL_MOISTURE_TWO_COMPONENT_NAME   "vTsoilMoistureExternal2"
#define sampleazureiotPMS_ONE_COMPONENT_NAME   "vTPMSExternal1"
#define sampleazureiotTEMPERATURE_TWO_COMPONENT_NAME   "vTtemperatureExternal2"
#define sampleazureiotSET_LED_STATUS_COMMAND             "setLedState"


FreeRTOS_VERIFIED_TELEMETRY_DB* verified_telemetry_DB;
#define SAMPLE_DEFAULT_DEVICE_SENSOR_READING (22)

SAMPLE_PNP_DEVICE_COMPONENT sample_device;
static const CHAR sample_device_component[] = "sampleDevice";

// vt variables end
/**
 * @brief The payload to send to the Device Provisioning Service
 */
#define sampleazureiotPROVISIONING_PAYLOAD "{\"modelId\":\"" sampleazureiotMODEL_ID "\"}"

/**
 * @brief Time in ticks to wait between each cycle of the demo implemented
 * by prvMQTTDemoTask().
 */
#define sampleazureiotDELAY_BETWEEN_DEMO_ITERATIONS_TICKS (pdMS_TO_TICKS(5000U))

/**
 * @brief Timeout for MQTT_ProcessLoop in milliseconds.
 */
#define sampleazureiotPROCESS_LOOP_TIMEOUT_MS (500U)

/**
 * @brief Delay (in ticks) between consecutive cycles of MQTT publish operations in a
 * demo iteration.
 *
 * Note that the process loop also has a timeout, so the total time between
 * publishes is the sum of the two delays.
 */
#define sampleazureiotDELAY_BETWEEN_PUBLISHES_TICKS (pdMS_TO_TICKS(5000U))

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS (2000U)

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define sampleazureiotProvisioning_Registration_TIMEOUT_MS (20U)

/**
 * @brief Wait timeout for subscribe to finish.
 */
#define sampleazureiotSUBSCRIBE_TIMEOUT (10 * 1000U)
/*-----------------------------------------------------------*/

/**
 * @brief Unix time.
 *
 * @return Time in milliseconds.
 */
uint64_t ullGetUnixTime(void);
/*-----------------------------------------------------------*/

/* Define buffer for IoT Hub info.  */
#ifdef democonfigENABLE_DPS_SAMPLE
static uint8_t ucSampleIotHubHostname[128];
static uint8_t ucSampleIotHubDeviceId[128];
static AzureIoTProvisioningClient_t xAzureIoTProvisioningClient;
#endif /* democonfigENABLE_DPS_SAMPLE */

// static uint8_t ucScratchBuffer[ 256 ];

/* Each compilation unit must define the NetworkContext struct. */
struct NetworkContext
{
    TlsTransportParams_t* pParams;
};

static AzureIoTHubClient_t xAzureIoTHubClient;
/*-----------------------------------------------------------*/




//     mcp320x_config_t mcp320x_cfg = {
//         .host = SPI2_HOST,
//         .device_model = MCP3204_MODEL,
//         .clock_speed_hz = 1 * 1000 * 1000, // 1 Mhz.
//         .reference_voltage = 5000,         // 5V
//         .cs_io_num = GPIO_CS};

//     mcp320x_handle_t mcp320x_handle;

#ifdef democonfigENABLE_DPS_SAMPLE

/**
 * @brief Gets the IoT Hub endpoint and deviceId from Provisioning service.
 *   This function will block for Provisioning service for result or return failure.
 *
 * @param[in] pXNetworkCredentials  Network credential used to connect to Provisioning service
 * @param[out] ppucIothubHostname  Pointer to uint8_t* IoT Hub hostname return from Provisioning Service
 * @param[in,out] pulIothubHostnameLength  Length of hostname
 * @param[out] ppucIothubDeviceId  Pointer to uint8_t* deviceId return from Provisioning Service
 * @param[in,out] pulIothubDeviceIdLength  Length of deviceId
 */
static uint32_t prvIoTHubInfoGet(NetworkCredentials_t* pXNetworkCredentials,
    uint8_t** ppucIothubHostname,
    uint32_t* pulIothubHostnameLength,
    uint8_t** ppucIothubDeviceId,
    uint32_t* pulIothubDeviceIdLength);

#endif /* democonfigENABLE_DPS_SAMPLE */

/**
 * @brief The task used to demonstrate the MQTT API.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation. Not
 * used in this example.
 */
static void prvAzureDemoTask(void* pvParameters);

/**
 * @brief Connect to IoT Hub with reconnection retries.
 *
 * If connection fails, retry is attempted after a timeout.
 * Timeout value will exponentially increase until maximum
 * timeout value is reached or the number of attempts are exhausted.
 *
 * @param pcHostName Hostname of the endpoint to connect to.
 * @param ulPort Endpoint port.
 * @param pxNetworkCredentials Pointer to Network credentials.
 * @param pxNetworkContext Point to Network context created.
 * @return uint32_t The status of the final connection attempt.
 */
static uint32_t prvConnectToServerWithBackoffRetries(const char* pcHostName,
    uint32_t ulPort,
    NetworkCredentials_t* pxNetworkCredentials,
    NetworkContext_t* pxNetworkContext);
/*-----------------------------------------------------------*/

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t ucMQTTMessageBuffer[democonfigNETWORK_BUFFER_SIZE];

/*-----------------------------------------------------------*/

static int32_t prvdesiredPropertiesSend_bootup()
{
    AzureIoTResult_t xResult;

    xResult = FreeRTOS_vt_send_desired_property_after_boot(
        verified_telemetry_DB, &xAzureIoTHubClient, FREERTOS_AZURE_IOT_PNP_PROPERTIES);

    return xResult;
}


/**
 * @brief Command message callback handler
 */
static void prvHandleCommand(AzureIoTHubClientCommandRequest_t* pxMessage, void* pvContext)
{
    int32_t setLedStateCommandNameLength;
    AzureIoTHubClient_t* xHandle = (AzureIoTHubClient_t*)pvContext;
    int32_t sampleDeviceComponontNameLength;
    int32_t soilMoistureOneComponontNameLength;
    int32_t soilMoistureTwoComponontNameLength;
    int32_t pmsoneComponontNameLength;
    int32_t temperatureTwoComponontNameLength;

    LogInfo( ( "Command payload : %.*s \r\n",
               pxMessage->ulPayloadLength,
               ( const char * ) pxMessage->pvMessagePayload ) );

    AzureIoTJSONReader_t xReader;
    AzureIoTJSONWriter_t xWriter;
    AzureIoTResult_t xResult;
    UINT status_code;
    xResult = AzureIoTJSONReader_Init(&xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength);
    configASSERT(xResult == eAzureIoTSuccess);

    sampleDeviceComponontNameLength    = strlen(sampleazureiotSAMPLE_DEVICE_COMPONENT_NAME);
    soilMoistureOneComponontNameLength = strlen(sampleazureiotSOIL_MOISTURE_ONE_COMPONENT_NAME);
    soilMoistureTwoComponontNameLength = strlen(sampleazureiotSOIL_MOISTURE_TWO_COMPONENT_NAME);
    pmsoneComponontNameLength          = strlen(sampleazureiotPMS_ONE_COMPONENT_NAME);
    temperatureTwoComponontNameLength  = strlen(sampleazureiotTEMPERATURE_TWO_COMPONENT_NAME);
    setLedStateCommandNameLength       = strlen(sampleazureiotSET_LED_STATUS_COMMAND);

    if ((sampleDeviceComponontNameLength == pxMessage->usComponentNameLength) &&
             (strncmp(sampleazureiotSAMPLE_DEVICE_COMPONENT_NAME,
                  (const char*)pxMessage->pucComponentName,
                  sampleDeviceComponontNameLength) == 0))
    {

        if ((setLedStateCommandNameLength == pxMessage->usCommandNameLength) &&
            (strncmp(sampleazureiotSET_LED_STATUS_COMMAND,
                 (const char*)pxMessage->pucCommandName,
                 setLedStateCommandNameLength) == 0))
        {

            sample_pnp_device_process_command(&sample_device,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                &xReader,
                &xWriter,
                &status_code);

            sample_pnp_device_led_state_property(&sample_device, &xAzureIoTHubClient);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, status_code, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else
        {
            LogInfo(("Received command is not for this device"));

            if (AzureIoTHubClient_SendCommandResponse(xHandle,
                    pxMessage,
                    404,
                    (const uint8_t*)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                    strlen(sampleazureiotCOMMAND_EMPTY_PAYLOAD)) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }
    }

    else if ((soilMoistureOneComponontNameLength == pxMessage->usComponentNameLength) &&
             (strncmp(sampleazureiotSOIL_MOISTURE_ONE_COMPONENT_NAME,
                  (const char*)pxMessage->pucComponentName,
                  soilMoistureOneComponontNameLength) == 0))
    {

        if ((strlen("setResetFingerprintTemplate") == pxMessage->usCommandNameLength) &&
            (strncmp("setResetFingerprintTemplate",
                 (const char*)pxMessage->pucCommandName,
                 strlen("setResetFingerprintTemplate")) == 0))
        {
            printf("vTsoilMoistureExternal1 command \n");

            FreeRTOS_vt_process_command(verified_telemetry_DB,
                xHandle,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                0);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, 200, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else if ((strlen("retrainFingerprintTemplate") == pxMessage->usCommandNameLength) &&
                 (strncmp("retrainFingerprintTemplate",
                      (const char*)pxMessage->pucCommandName,
                      strlen("retrainFingerprintTemplate")) == 0))
        {
            FreeRTOS_vt_process_command(verified_telemetry_DB,
                xHandle,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                0);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, 200, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else
        {
            /* Not for max min report (not for this device) */
            LogInfo(("Received command is not for this device"));

            if (AzureIoTHubClient_SendCommandResponse(xHandle,
                    pxMessage,
                    404,
                    (const uint8_t*)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                    strlen(sampleazureiotCOMMAND_EMPTY_PAYLOAD)) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }
    }

    else if ((soilMoistureTwoComponontNameLength == pxMessage->usComponentNameLength) &&
             (strncmp(sampleazureiotSOIL_MOISTURE_TWO_COMPONENT_NAME,
                  (const char*)pxMessage->pucComponentName,
                  soilMoistureTwoComponontNameLength) == 0))
    {

        if ((strlen("setResetFingerprintTemplate") == pxMessage->usCommandNameLength) &&
            (strncmp("setResetFingerprintTemplate",
                 (const char*)pxMessage->pucCommandName,
                 strlen("setResetFingerprintTemplate")) == 0))
        {
            FreeRTOS_vt_process_command(verified_telemetry_DB,
                xHandle,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                0);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, 200, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else if ((strlen("retrainFingerprintTemplate") == pxMessage->usCommandNameLength) &&
                 (strncmp("retrainFingerprintTemplate",
                      (const char*)pxMessage->pucCommandName,
                      strlen("retrainFingerprintTemplate")) == 0))
        {
            FreeRTOS_vt_process_command(verified_telemetry_DB,
                xHandle,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                0);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, 200, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else
        {
            /* Not for max min report (not for this device) */
            LogInfo(("Received command is not for this device"));

            if (AzureIoTHubClient_SendCommandResponse(xHandle,
                    pxMessage,
                    404,
                    (const uint8_t*)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                    strlen(sampleazureiotCOMMAND_EMPTY_PAYLOAD)) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }
    }

    else if ((pmsoneComponontNameLength == pxMessage->usComponentNameLength) &&
             (strncmp(sampleazureiotPMS_ONE_COMPONENT_NAME,
                  (const char*)pxMessage->pucComponentName,
                  pmsoneComponontNameLength) == 0))
    {

        if ((strlen("setResetFingerprintTemplate") == pxMessage->usCommandNameLength) &&
            (strncmp("setResetFingerprintTemplate",
                 (const char*)pxMessage->pucCommandName,
                 strlen("setResetFingerprintTemplate")) == 0))
        {
            FreeRTOS_vt_process_command(verified_telemetry_DB,
                xHandle,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                0);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, 200, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else if ((strlen("retrainFingerprintTemplate") == pxMessage->usCommandNameLength) &&
                 (strncmp("retrainFingerprintTemplate",
                      (const char*)pxMessage->pucCommandName,
                      strlen("retrainFingerprintTemplate")) == 0))
        {
            FreeRTOS_vt_process_command(verified_telemetry_DB,
                xHandle,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                0);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, 200, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else
        {
            /* Not for max min report (not for this device) */
            LogInfo(("Received command is not for this device"));

            if (AzureIoTHubClient_SendCommandResponse(xHandle,
                    pxMessage,
                    404,
                    (const uint8_t*)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                    strlen(sampleazureiotCOMMAND_EMPTY_PAYLOAD)) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }
    }

    else if ((temperatureTwoComponontNameLength == pxMessage->usComponentNameLength) &&
             (strncmp(sampleazureiotTEMPERATURE_TWO_COMPONENT_NAME,
                  (const char*)pxMessage->pucComponentName,
                  temperatureTwoComponontNameLength) == 0))
    {

        if ((strlen("setResetFingerprintTemplate") == pxMessage->usCommandNameLength) &&
            (strncmp("setResetFingerprintTemplate",
                 (const char*)pxMessage->pucCommandName,
                 strlen("setResetFingerprintTemplate")) == 0))
        {
            FreeRTOS_vt_process_command(verified_telemetry_DB,
                xHandle,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                0);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, 200, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else if ((strlen("retrainFingerprintTemplate") == pxMessage->usCommandNameLength) &&
                 (strncmp("retrainFingerprintTemplate",
                      (const char*)pxMessage->pucCommandName,
                      strlen("retrainFingerprintTemplate")) == 0))
        {
            FreeRTOS_vt_process_command(verified_telemetry_DB,
                xHandle,
                (UCHAR*)pxMessage->pucComponentName,
                pxMessage->usComponentNameLength,
                (UCHAR*)pxMessage->pucCommandName,
                pxMessage->usCommandNameLength,
                0);

            if (AzureIoTHubClient_SendCommandResponse(xHandle, pxMessage, 200, NULL, 0) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }

        else
        {
            /* Not for max min report (not for this device) */
            LogInfo(("Received command is not for this device"));

            if (AzureIoTHubClient_SendCommandResponse(xHandle,
                    pxMessage,
                    404,
                    (const uint8_t*)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                    strlen(sampleazureiotCOMMAND_EMPTY_PAYLOAD)) != eAzureIoTSuccess)
            {
                LogError(("Error sending command response"));
            }
        }
    }

    else
    {
        /* Not for max min report (not for this device) */
        LogInfo(("Received component is not for this device"));

        if (AzureIoTHubClient_SendCommandResponse(xHandle,
                pxMessage,
                404,
                (const uint8_t*)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                strlen(sampleazureiotCOMMAND_EMPTY_PAYLOAD)) != eAzureIoTSuccess)
        {
            LogError(("Error sending command response"));
        }
    }
}
/*-----------------------------------------------------------*/

static AzureIoTResult_t prvdeviceStatusReportedPropertyProcess(
    AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
{

    AzureIoTResult_t xResult;

    xResult = AzureIoTJSONReader_NextToken(xReader);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
    configASSERT(xResult == eAzureIoTSuccess);

    /* Get desired temperature */
    xResult = AzureIoTJSONReader_GetTokenBool(xReader, &deviceStatusReportedProperty);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONReader_NextToken(xReader);
    configASSERT(xResult == eAzureIoTSuccess);

    printf(" vtDevice deviceStatus %d \n", deviceStatusReportedProperty);

    return xResult;
}
/*
static AzureIoTResult_t prvledStateReportedPropertyProcess(
    AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
{

    AzureIoTResult_t xResult;

    xResult = AzureIoTJSONReader_NextToken(xReader);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONReader_GetTokenBool(xReader, &setLedState);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONReader_NextToken(xReader);
    configASSERT(xResult == eAzureIoTSuccess);

    printf(" sampleDevice ledState %d \n", setLedState);

    return xResult;
}
*/
// static AzureIoTResult_t prvSoilMoisture1TelemetryStatusPropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenBool(xReader, &telemetryStatusSoilMoisture1);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" telemetryStatus SoilMoisture1 %d \n", telemetryStatusSoilMoisture1);

//     return xResult;
// }

// static AzureIoTResult_t prvSoilMoisture1FingerprintTypePropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;
//     uint8_t pucBufferLocal[64];
//     uint32_t pusBytesCopied;
//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenString(xReader, pucBufferLocal, 64, &pusBytesCopied);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" FingerprintType SoilMoisture1 - %s \n", pucBufferLocal);

//     return xResult;
// }

// static AzureIoTResult_t prvSoilMoisture1ConfidenceMetricPropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenInt32(xReader, &ConfidenceMetriclocalone);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" ConfidenceMetriclocal SoilMoisture1 - %ld \n", (long int)ConfidenceMetriclocalone);

//     return xResult;
// }

// static AzureIoTResult_t prvSoilMoisture2ConfidenceMetricPropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;
//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenInt32(xReader, &ConfidenceMetriclocaltwo);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" ConfidenceMetriclocal SoilMoisture2 - %ld \n", (long int)ConfidenceMetriclocaltwo);

//     return xResult;
// }

// static AzureIoTResult_t prvSoilMoisture2FingerprintTypePropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;
//     uint8_t pucBufferLocal[64];
//     uint32_t pusBytesCopied;
//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenString(xReader, pucBufferLocal, 64, &pusBytesCopied);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" FingerprintType SoilMoisture2 - %s \n", pucBufferLocal);

//     return xResult;
// }

// static AzureIoTResult_t prvSoilMoisture2TelemetryStatusPropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenBool(xReader, &telemetryStatusSoilMoisture2);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" telemetryStatus SoilMoisture2 %d \n", telemetryStatusSoilMoisture2);

//     return xResult;
// }

// static AzureIoTResult_t prvPMS1ConfidenceMetricPropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;
//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenInt32(xReader, &ConfidenceMetriclocaltwo);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" ConfidenceMetriclocal SoilMoisture2 - %ld \n", (long int)ConfidenceMetriclocaltwo);

//     return xResult;
// }

// static AzureIoTResult_t prvPMS1FingerprintTypePropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;
//     uint8_t pucBufferLocal[64];
//     uint32_t pusBytesCopied;
//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenString(xReader, pucBufferLocal, 64, &pusBytesCopied);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" FingerprintType SoilMoisture2 - %s \n", pucBufferLocal);

//     return xResult;
// }

// static AzureIoTResult_t prvPMS1TelemetryStatusPropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenBool(xReader, &telemetryStatusSoilMoisture2);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" telemetryStatus SoilMoisture2 %d \n", telemetryStatusSoilMoisture2);

//     return xResult;
// }

// static AzureIoTResult_t prvTemperature2ConfidenceMetricPropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;
//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenInt32(xReader, &ConfidenceMetriclocaltwo);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" ConfidenceMetriclocal SoilMoisture2 - %ld \n", (long int)ConfidenceMetriclocaltwo);

//     return xResult;
// }

// static AzureIoTResult_t prvTemperature2FingerprintTypePropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;
//     uint8_t pucBufferLocal[64];
//     uint32_t pusBytesCopied;
//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenString(xReader, pucBufferLocal, 64, &pusBytesCopied);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" FingerprintType SoilMoisture2 - %s \n", pucBufferLocal);

//     return xResult;
// }

// static AzureIoTResult_t prvTemperature2TelemetryStatusPropertyProcess(
//     AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
// {

//     AzureIoTResult_t xResult;

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
//     configASSERT(xResult == eAzureIoTSuccess);

//     /* Get desired temperature */
//     xResult = AzureIoTJSONReader_GetTokenBool(xReader, &telemetryStatusSoilMoisture2);
//     configASSERT(xResult == eAzureIoTSuccess);

//     xResult = AzureIoTJSONReader_NextToken(xReader);
//     configASSERT(xResult == eAzureIoTSuccess);

//     printf(" telemetryStatus SoilMoisture2 %d \n", telemetryStatusSoilMoisture2);

//     return xResult;
// }

static AzureIoTResult_t prvenableVerifiedTelemetryReportedPropertyProcess(
    AzureIoTJSONReader_t* xReader, AzureIoTJSONTokenType_t* xTokenType)
{

    AzureIoTResult_t xResult;
    for (int iter = 0; iter < 9; iter++)
    {
        xResult = AzureIoTJSONReader_NextToken(xReader);
        configASSERT(xResult == eAzureIoTSuccess);
    }
    xResult = AzureIoTJSONReader_TokenType(xReader, xTokenType);
    configASSERT(xResult == eAzureIoTSuccess);

    /* Get desired temperature */
    xResult = AzureIoTJSONReader_GetTokenBool(xReader, &enableVerifiedTelemetryWritableProperty);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONReader_NextToken(xReader);
    configASSERT(xResult == eAzureIoTSuccess);

    verified_telemetry_DB->enable_verified_telemetry=enableVerifiedTelemetryWritableProperty;


    printf(" enableVerifiedTelemetry WritableProperty %d \n", enableVerifiedTelemetryWritableProperty);

    return xResult;
}

/**
 * @brief Properties callback handler
 */

static AzureIoTResult_t prvProcessProperties(
    AzureIoTHubClientPropertiesResponse_t* pxMessage, AzureIoTHubClientPropertyType_t xPropertyType)
{
    AzureIoTResult_t xResult;
    AzureIoTJSONReader_t xReader;
    AzureIoTJSONTokenType_t xTokenType;
    const uint8_t* pucComponentName;
    uint32_t ulComponentNameLength;
    uint32_t ulVersion;

    uint32_t sampleDevicecomponontNameLength = strlen(sampleazureiotvTDeviceCOMPONENT_NAME);

    xResult = AzureIoTJSONReader_Init(&xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTHubClientProperties_GetPropertiesVersion(
        &xAzureIoTHubClient, &xReader, pxMessage->xMessageType, &ulVersion);

    if (xResult != eAzureIoTSuccess)
    {
        LogError(("Error getting the property version"));
    }
    else
    { // printf(" iteration start \n");
        /* Reset JSON reader to the beginning */
        xResult = AzureIoTJSONReader_Init(&xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength);
        configASSERT(xResult == eAzureIoTSuccess);
        // here this fucntion would check for that articular commPonet in the Payload recieved. and whould then go on to
        // rocess
        while ((xResult = AzureIoTHubClientProperties_GetNextComponentProperty(&xAzureIoTHubClient,
                    &xReader,
                    pxMessage->xMessageType,
                    xPropertyType,
                    &pucComponentName,
                    &ulComponentNameLength)) == eAzureIoTSuccess)
        { // printf(" iteration \n");
            xResult = AzureIoTJSONReader_TokenType(&xReader, &xTokenType);
            configASSERT(xResult == eAzureIoTSuccess);
            /*
            if( ( strlen("componentSample") == ulComponentNameLength ) &&
                ( strncmp( "componentSample", (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {
                printf(" same com \n");
            }

            if( ( strlen("componentSampleuno") == ulComponentNameLength ) &&
                ( strncmp( "componentSampleuno", (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {
                printf(" componentSampleuno \n");
            }
            */

            if ((sampleDevicecomponontNameLength == ulComponentNameLength) &&
                (strncmp(sampleazureiotvTDeviceCOMPONENT_NAME, (const char*)pucComponentName, ulComponentNameLength) ==
                    0))
            {
                printf("vtdevice");
                FreeRTOS_vt_process_property_update(verified_telemetry_DB,
                    &xAzureIoTHubClient,
                    pucComponentName,
                    ulComponentNameLength,
                    &xReader,
                    ulVersion);

                /*
                if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)sampleazureiotPROPERTY_TARGET_TEMPERATURE_TEXT,
                                                        strlen( sampleazureiotPROPERTY_TARGET_TEMPERATURE_TEXT ) ) )
                {
                    //printf(" okay \n");
                    prvTargetTemperatureProcess(&xReader, &xTokenType,ulVersion);
                }


                if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t
                *)sampleazureiotPROPERTY_ENABLE_VERIFIED_TELEMETRY, strlen(
                sampleazureiotPROPERTY_ENABLE_VERIFIED_TELEMETRY ) ) )
                {
                    //printf(" okay \n");
                    prvEnableVerifiedTelemetryPropertyProcess(&xReader, &xTokenType,ulVersion);
                }
                else
                {
                    xResult = AzureIoTJSONReader_NextToken( &xReader );
                    configASSERT( xResult == eAzureIoTSuccess );

                    xResult = AzureIoTJSONReader_SkipChildren( &xReader );
                    configASSERT( xResult == eAzureIoTSuccess );

                    xResult = AzureIoTJSONReader_NextToken( &xReader );
                    configASSERT( xResult == eAzureIoTSuccess );
                }
                */
            }

            else
            {
                xResult = AzureIoTJSONReader_NextToken(&xReader);
                configASSERT(xResult == eAzureIoTSuccess);

                xResult = AzureIoTJSONReader_SkipChildren(&xReader);
                configASSERT(xResult == eAzureIoTSuccess);

                xResult = AzureIoTJSONReader_NextToken(&xReader);
                configASSERT(xResult == eAzureIoTSuccess);
            }
        }

        if (xResult != eAzureIoTErrorEndOfProperties)
        {
            LogError( ( "There was an error parsing the properties: result 0x%08x", xResult ) );
        }
        else
        {
            LogInfo(("Successfully parsed properties"));
            xResult = eAzureIoTSuccess;
        }
    }

    return xResult;
}
/*-----------------------------------------------------------*/

static AzureIoTResult_t prvProcessReportedProperties(
    AzureIoTHubClientPropertiesResponse_t* pxMessage, AzureIoTHubClientPropertyType_t xPropertyType)
{
    AzureIoTResult_t xResult;
    AzureIoTJSONReader_t xReader;
    AzureIoTJSONTokenType_t xTokenType;
    const uint8_t* pucComponentName;
    uint32_t ulComponentNameLength=0;
    uint32_t ulVersion;
   //uint8_t pucbuffer[50];
   // uint32_t bt;

    uint32_t vTDevicecomponontNameLength     = strlen(sampleazureiotvTDeviceCOMPONENT_NAME);
    //uint32_t sampleDevicecomponontNameLength = strlen(sampleazureiotsampleDeviceCOMPONENT_NAME);

    xResult = AzureIoTJSONReader_Init(&xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTHubClientProperties_GetPropertiesVersion(
        &xAzureIoTHubClient, &xReader, pxMessage->xMessageType, &ulVersion);

    if (xResult != eAzureIoTSuccess)
    {
        LogError(("Error getting the property version"));
    }
    else
    {
        /* Reset JSON reader to the beginning */
        xResult = AzureIoTJSONReader_Init(&xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength);
        configASSERT(xResult == eAzureIoTSuccess);

        // here this fucntion would check for that articular commPonet in the Payload recieved. and whould then go on to
        // rocess here it was weird accidentaly i store some value in the reorted roerty dock and it stayed true but even
        // with the comonentc name initalized it still icked u the Status reorted roerty where as it should only check
        // inside comonent
        while ((xResult = AzureIoTHubClientProperties_GetNextComponentProperty(&xAzureIoTHubClient,
                    &xReader,
                    pxMessage->xMessageType,
                    xPropertyType,
                    &pucComponentName,
                    &ulComponentNameLength)) == eAzureIoTSuccess)
        {
            
            xResult = AzureIoTJSONReader_TokenType(&xReader, &xTokenType);
            configASSERT(xResult == eAzureIoTSuccess);

            //LogInfo(("component name : %.*s \r\n", ulComponentNameLength, pucComponentName));

    LogInfo( ( "component name : %.*s \r\n",
               ulComponentNameLength,
               ( const char * ) pucComponentName ) );


                // xResult = AzureIoTJSONReader_NextToken( &xReader );
                // configASSERT( xResult == eAzureIoTSuccess );

                //                 AzureIoTJSONReader_GetTokenString( &xReader,pucbuffer,50,&bt);
                //  printf("\n0token: %s\n",pucbuffer);
            // printf(" %s \n",pucComponentName);

            /*
            if( ( strlen("componentSample") == ulComponentNameLength ) &&
                ( strncmp( "componentSample", (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {
                printf(" same com \n");
            }

            if( ( strlen("componentSampleuno") == ulComponentNameLength ) &&
                ( strncmp( "componentSampleuno", (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {
                printf(" componentSampleuno \n");
            }
         */

            if ((vTDevicecomponontNameLength == ulComponentNameLength) &&
                (strncmp(sampleazureiotvTDeviceCOMPONENT_NAME, (const char*)pucComponentName, ulComponentNameLength) ==
                    0))
            {


                //while()
                // printf(" found comp \n");
                if (AzureIoTJSONReader_TokenIsTextEqual(
                        &xReader, (const uint8_t*)"deviceStatus", strlen("deviceStatus")))
                {
                    // printf(" found \n");
                    prvdeviceStatusReportedPropertyProcess(&xReader, &xTokenType);

                // AzureIoTJSONReader_GetTokenString( &xReader,pucbuffer,50,&bt);
                //  printf("\naftertoken: %s\n",pucbuffer);
                }

                else if (AzureIoTJSONReader_TokenIsTextEqual(
                             &xReader, (const uint8_t*)"enableVerifiedTelemetry", strlen("enableVerifiedTelemetry")))
                {
                    // printf(" found \n");
                    prvenableVerifiedTelemetryReportedPropertyProcess(&xReader, &xTokenType);

                //                     AzureIoTJSONReader_GetTokenString( &xReader,pucbuffer,50,&bt);
                //  printf("\nafter1token: %s\n",pucbuffer);
                }
                else
                {


                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_SkipChildren(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);
                //                             AzureIoTJSONReader_GetTokenString( &xReader,pucbuffer,50,&bt);
                //  printf("\n2token: %s\n",pucbuffer);
                }
            }
            /*
            else if ((sampleDevicecomponontNameLength == ulComponentNameLength) &&
                     (strncmp(sampleazureiotsampleDeviceCOMPONENT_NAME,
                          (const char*)pucComponentName,
                          ulComponentNameLength) == 0))
            {

                if (AzureIoTJSONReader_TokenIsTextEqual(&xReader, (const uint8_t*)"ledState", strlen("ledState")))
                {
                    // printf(" found \n");
                    prvledStateReportedPropertyProcess(&xReader, &xTokenType);
                }
                else
                {

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_SkipChildren(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);
                }
            }
            */
            else if ((strlen("vTsoilMoistureExternal1") == ulComponentNameLength) &&
                     (strncmp("vTsoilMoistureExternal1", (const char*)pucComponentName, ulComponentNameLength) == 0))
            {
                 printf("in");
                // if (AzureIoTJSONReader_TokenIsTextEqual(
                //         &xReader, (const uint8_t*)"telemetryStatus", strlen("telemetryStatus")))
                // {
                //     // printf(" found in \n");
                //     prvSoilMoisture1TelemetryStatusPropertyProcess(&xReader, &xTokenType);
                // }
                // else if (AzureIoTJSONReader_TokenIsTextEqual(
                //              &xReader, (const uint8_t*)"fingerprintType", strlen("fingerprintType")))
                // {
                //     // printf(" found out \n");
                //     prvSoilMoisture1FingerprintTypePropertyProcess(&xReader, &xTokenType);
                // }
                // // using this else part causes problems and lags the code using configASSERT( xResult ==
                // // eAzureIoTSuccess );
                // else if (AzureIoTJSONReader_TokenIsTextEqual(&xReader,
                //              (const uint8_t*)"fingerprintTemplateConfidenceMetric",
                //              strlen("fingerprintTemplateConfidenceMetric")))
                // {
                //     // printf(" found out \n");
                //     prvSoilMoisture1ConfidenceMetricPropertyProcess(&xReader, &xTokenType);
                // }
                if (AzureIoTJSONReader_TokenIsTextEqual(
                             &xReader, (const uint8_t*)"fingerprintTemplate", strlen("fingerprintTemplate")))
                {
                    // printf(" found out \n");
                    // prvSoilMoisture1ConfidenceMetricMapPropertyProcess(&xReader, &xTokenType);

                    FreeRTOS_vt_process_reported_property_sync(verified_telemetry_DB,
                        pucComponentName,
                        ulComponentNameLength,
                        &xReader);
                }

                
                else
                {
                       // AzureIoTJSONReader_GetTokenString( &xReader,pucbuffer,50,&bt);
                //printf("token: %s",pucbuffer);
                    xResult = AzureIoTJSONReader_NextToken( &xReader );
                    configASSERT( xResult == eAzureIoTSuccess );

                    xResult = AzureIoTJSONReader_SkipChildren( &xReader );
                    configASSERT( xResult == eAzureIoTSuccess );

                    xResult = AzureIoTJSONReader_NextToken( &xReader );
                    configASSERT( xResult == eAzureIoTSuccess );
                }
                
            }
            else if ((strlen("vTsoilMoistureExternal2") == ulComponentNameLength) &&
                     (strncmp("vTsoilMoistureExternal2", (const char*)pucComponentName, ulComponentNameLength) == 0))
            {

                // if (AzureIoTJSONReader_TokenIsTextEqual(
                //         &xReader, (const uint8_t*)"telemetryStatus", strlen("telemetryStatus")))
                // {
                //     // printf(" found1 \n");
                //     prvSoilMoisture2TelemetryStatusPropertyProcess(&xReader, &xTokenType);
                // }
                // else if (AzureIoTJSONReader_TokenIsTextEqual(
                //              &xReader, (const uint8_t*)"fingerprintType", strlen("fingerprintType")))
                // {
                //     // printf(" found2 \n");
                //     prvSoilMoisture2FingerprintTypePropertyProcess(&xReader, &xTokenType);
                // }
                // else if (AzureIoTJSONReader_TokenIsTextEqual(&xReader,
                //              (const uint8_t*)"fingerprintTemplateConfidenceMetric",
                //              strlen("fingerprintTemplateConfidenceMetric")))
                // {
                //     // printf(" found out3 \n");
                //     prvSoilMoisture2ConfidenceMetricPropertyProcess(&xReader, &xTokenType);
                // }
                 if (AzureIoTJSONReader_TokenIsTextEqual(
                             &xReader, (const uint8_t*)"fingerprintTemplate", strlen("fingerprintTemplate")))
                {
                    // printf(" found out4 \n");
                    // prvSoilMoisture2ConfidenceMetricMapPropertyProcess(&xReader, &xTokenType);

                    FreeRTOS_vt_process_reported_property_sync(verified_telemetry_DB,
                        pucComponentName,
                        ulComponentNameLength,
                        &xReader);
                }
                else
                {

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_SkipChildren(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);
                }
            }

            else if ((strlen("vTPMSExternal1") == ulComponentNameLength) &&
                     (strncmp("vTPMSExternal1", (const char*)pucComponentName, ulComponentNameLength) == 0))
            {

                // if (AzureIoTJSONReader_TokenIsTextEqual(
                //         &xReader, (const uint8_t*)"telemetryStatus", strlen("telemetryStatus")))
                // {
                //     // printf(" found1 \n");
                //     prvPMS1TelemetryStatusPropertyProcess(&xReader, &xTokenType);
                // }
                // else if (AzureIoTJSONReader_TokenIsTextEqual(
                //              &xReader, (const uint8_t*)"fingerprintType", strlen("fingerprintType")))
                // {
                //     // printf(" found2 \n");
                //     prvPMS1FingerprintTypePropertyProcess(&xReader, &xTokenType);
                // }
                // else if (AzureIoTJSONReader_TokenIsTextEqual(&xReader,
                //              (const uint8_t*)"fingerprintTemplateConfidenceMetric",
                //              strlen("fingerprintTemplateConfidenceMetric")))
                // {
                //     // printf(" found out3 \n");
                //     prvPMS1ConfidenceMetricPropertyProcess(&xReader, &xTokenType);
                // }
                 if (AzureIoTJSONReader_TokenIsTextEqual(
                             &xReader, (const uint8_t*)"fingerprintTemplate", strlen("fingerprintTemplate")))
                {
                    // printf(" found out4 \n");
                    // prvSoilMoisture2ConfidenceMetricMapPropertyProcess(&xReader, &xTokenType);

                    FreeRTOS_vt_process_reported_property_sync(verified_telemetry_DB,
                        pucComponentName,
                        ulComponentNameLength,
                        &xReader);
                }
                else
                {

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_SkipChildren(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);
                }
            }

            else if ((strlen("vTtemperatureExternal2") == ulComponentNameLength) &&
                     (strncmp("vTtemperatureExternal2", (const char*)pucComponentName, ulComponentNameLength) == 0))
            {

                // if (AzureIoTJSONReader_TokenIsTextEqual(
                //         &xReader, (const uint8_t*)"telemetryStatus", strlen("telemetryStatus")))
                // {
                //     // printf(" found1 \n");
                //     prvTemperature2TelemetryStatusPropertyProcess(&xReader, &xTokenType);
                // }
                // else if (AzureIoTJSONReader_TokenIsTextEqual(
                //              &xReader, (const uint8_t*)"fingerprintType", strlen("fingerprintType")))
                // {
                //     // printf(" found2 \n");
                //     prvTemperature2FingerprintTypePropertyProcess(&xReader, &xTokenType);
                // }
                // else if (AzureIoTJSONReader_TokenIsTextEqual(&xReader,
                //              (const uint8_t*)"fingerprintTemplateConfidenceMetric",
                //              strlen("fingerprintTemplateConfidenceMetric")))
                // {
                //     // printf(" found out3 \n");
                //     prvTemperature2ConfidenceMetricPropertyProcess(&xReader, &xTokenType);
                // }
                 if (AzureIoTJSONReader_TokenIsTextEqual(
                             &xReader, (const uint8_t*)"fingerprintTemplate", strlen("fingerprintTemplate")))
                {
                    // printf(" found out4 \n");
                    // prvSoilMoisture2ConfidenceMetricMapPropertyProcess(&xReader, &xTokenType);

                    FreeRTOS_vt_process_reported_property_sync(verified_telemetry_DB,
                        pucComponentName,
                        ulComponentNameLength,
                        &xReader);
                }
                else
                {

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_SkipChildren(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);

                    xResult = AzureIoTJSONReader_NextToken(&xReader);
                    configASSERT(xResult == eAzureIoTSuccess);
                }
            }

            else
            {


                xResult = AzureIoTJSONReader_NextToken(&xReader);
                configASSERT(xResult == eAzureIoTSuccess);

                xResult = AzureIoTJSONReader_SkipChildren(&xReader);
                configASSERT(xResult == eAzureIoTSuccess);

                xResult = AzureIoTJSONReader_NextToken(&xReader);
                configASSERT(xResult == eAzureIoTSuccess);

            }
            
        }

        if (xResult != eAzureIoTErrorEndOfProperties)
        {
            LogError( ( "There was an error parsing the properties: result 0x%08x", xResult ) );
        }
        else
        {
            LogInfo(("Successfully parsed properties"));
            xResult = eAzureIoTSuccess;
        }
    }

    return xResult;
}

/**
 * @brief Property mesage callback handler
 *  * "reported": {
 *     "<pucComponentName>": {
 *         "__t": "c",
 *         "temperature": 23
 *     }
 * }
 * //here the pxMessage that we recive only has the ayload inside which there would be message like this
 * so need to rocess this in the resective prvProcessProperties fucntion
 */
// here the pxMessage that we recive only has the ayload inside which there would be message like this
static void prvHandleProperties(AzureIoTHubClientPropertiesResponse_t* pxMessage, void* pvContext)
{
    (void)pvContext;

    AzureIoTResult_t xResult;

    //LogInfo(("Property document payload : %.*s \r\n", pxMessage->ulPayloadLength, pxMessage->pvMessagePayload));

    LogInfo( ( "Property document payload : %.*s \r\n",
               pxMessage->ulPayloadLength,
               ( const char * ) pxMessage->pvMessagePayload ) );

    switch (pxMessage->xMessageType)
    {
        case eAzureIoTHubPropertiesRequestedMessage:
            LogInfo(("Device property document GET received"));
            // printf("first");
            xResult = prvProcessReportedProperties(pxMessage, eAzureIoTHubClientReportedFromDevice);
            if (xResult != eAzureIoTSuccess)
            {
                LogError(("There was an error processing incoming properties"));
            }

            break;

        case eAzureIoTHubPropertiesWritablePropertyMessage:
            LogInfo(("Device writeable property received"));
            xResult = prvProcessProperties(pxMessage, eAzureIoTHubClientPropertyWritable);

            if (xResult != eAzureIoTSuccess)
            {
                LogError(("There was an error processing incoming properties"));
            }

            break;

        case eAzureIoTHubPropertiesReportedResponseMessage:
            LogInfo(("Device reported property response received"));
            break;

        default:
            printf("default");
            LogError(("Unknown property message"));
    }
}
/*-----------------------------------------------------------*/
// void mcp320x_read()

// {
//         unsigned short raw;
//         unsigned short voltage;
//         ESP_ERROR_CHECK(mcp320x_read_raw(mcp320x_handle, MCP320X_CHANNEL_0, MCP320X_READ_MODE_SINGLE, &raw));
//         ESP_ERROR_CHECK(mcp320x_read_voltage(mcp320x_handle, MCP320X_CHANNEL_0, MCP320X_READ_MODE_SINGLE, &voltage));
//         //ESP_LOGI("mcp320x", "Raw: %d", raw);
//         //ESP_LOGI("mcp320x", "Voltage: %d mV", voltage);
//         printf("Raw: %d \n", raw);
//         printf("Voltage: %d mV \n", voltage);
// }
/**
 * @brief Setup transport credentials.
 */
static uint32_t prvSetupNetworkCredentials(NetworkCredentials_t* pxNetworkCredentials)
{
    pxNetworkCredentials->xDisableSni = pdFALSE;
    /* Set the credentials for establishing a TLS connection. */
    pxNetworkCredentials->pucRootCa   = (const unsigned char*)democonfigROOT_CA_PEM;
    pxNetworkCredentials->xRootCaSize = sizeof(democonfigROOT_CA_PEM);
#ifdef democonfigCLIENT_CERTIFICATE_PEM
    pxNetworkCredentials->pucClientCert   = (const unsigned char*)democonfigCLIENT_CERTIFICATE_PEM;
    pxNetworkCredentials->xClientCertSize = sizeof(democonfigCLIENT_CERTIFICATE_PEM);
    pxNetworkCredentials->pucPrivateKey   = (const unsigned char*)democonfigCLIENT_PRIVATE_KEY_PEM;
    pxNetworkCredentials->xPrivateKeySize = sizeof(democonfigCLIENT_PRIVATE_KEY_PEM);
#endif

    return 0;
}
/*-----------------------------------------------------------*/

/**
 * @brief Azure IoT demo task that gets started in the platform specific project.
 *  In this demo task, middleware API's are used to connect to Azure IoT Hub.
 */
static void prvAzureDemoTask(void* pvParameters)
{
    NetworkCredentials_t xNetworkCredentials = {0};
    AzureIoTTransportInterface_t xTransport;
    NetworkContext_t xNetworkContext         = {0};
    TlsTransportParams_t xTlsTransportParams = {0};
    AzureIoTResult_t xResult;
    uint32_t ulStatus;
    AzureIoTHubClientOptions_t xHubOptions = {0};
    bool xSessionPresent;

    verified_telemetry_DB = sample_nx_verified_telemetry_user_init();

    sample_pnp_device_init(&sample_device,
        (UCHAR*)sample_device_component,
        sizeof(sample_device_component) - 1,
        SAMPLE_DEFAULT_DEVICE_SENSOR_READING,
        verified_telemetry_DB);

    AzureIoTHubClientComponent_t component_list[NUM_COMPONENTS];
    component_list[0] = azureiothubCREATE_COMPONENT("sampleDevice");
    component_list[1] = azureiothubCREATE_COMPONENT("vTDevice");
    component_list[2] = azureiothubCREATE_COMPONENT("vTsoilMoistureExternal1");
    component_list[3] = azureiothubCREATE_COMPONENT("vTsoilMoistureExternal2");
    component_list[4] = azureiothubCREATE_COMPONENT("vTPMSExternal1");
    component_list[5] = azureiothubCREATE_COMPONENT("vTtemperatureExternal2");

#ifdef democonfigENABLE_DPS_SAMPLE
    uint8_t* pucIotHubHostname       = NULL;
    uint8_t* pucIotHubDeviceId       = NULL;
    uint32_t pulIothubHostnameLength = 0;
    uint32_t pulIothubDeviceIdLength = 0;
#else
    uint8_t* pucIotHubHostname       = (uint8_t*)democonfigHOSTNAME;
    uint8_t* pucIotHubDeviceId       = (uint8_t*)democonfigDEVICE_ID;
    uint32_t pulIothubHostnameLength = sizeof(democonfigHOSTNAME) - 1;
    uint32_t pulIothubDeviceIdLength = sizeof(democonfigDEVICE_ID) - 1;
#endif /* democonfigENABLE_DPS_SAMPLE */

    (void)pvParameters;

    /* Initialize Azure IoT Middleware.  */
    configASSERT(AzureIoT_Init() == eAzureIoTSuccess);

    ulStatus = prvSetupNetworkCredentials(&xNetworkCredentials);
    configASSERT(ulStatus == 0);

#ifdef democonfigENABLE_DPS_SAMPLE
    /* Run DPS.  */
    if ((ulStatus = prvIoTHubInfoGet(&xNetworkCredentials,
             &pucIotHubHostname,
             &pulIothubHostnameLength,
             &pucIotHubDeviceId,
             &pulIothubDeviceIdLength)) != 0)
    {
        //LogError(("Failed on sample_dps_entry!: error code = 0x%08x\r\n", ulStatus));
        return;
    }
#endif /* democonfigENABLE_DPS_SAMPLE */

    xNetworkContext.pParams = &xTlsTransportParams;

    xNetworkContext.pParams = &xTlsTransportParams;
    AzureIoTMessageProperties_t telemetrymessageProperties;
    uint8_t pucBuffer[128];

    AzureIoTMessage_PropertiesInit(&telemetrymessageProperties, pucBuffer, 0, 128);
    AzureIoTMessage_PropertiesAppend(&telemetrymessageProperties,
        (const uint8_t*)"$.sub",
        sizeof("$.sub") - 1,
        (const uint8_t*)"sampleDevice",
        sizeof("sampleDevice") - 1);
    AzureIoTMessage_PropertiesAppend(&telemetrymessageProperties,
        (const uint8_t*)"verifiedTelemetry",
        sizeof("verifiedTelemetry") - 1,
        (const uint8_t*)"Demo",
        sizeof("Demo") - 1);

    for (;;)
    {
        /* Attempt to establish TLS session with IoT Hub. If connection fails,
         * retry after a timeout. Timeout value will be exponentially increased
         * until  the maximum number of attempts are reached or the maximum timeout
         * value is reached. The function returns a failure status if the TCP
         * connection cannot be established to the IoT Hub after the configured
         * number of attempts. */
        ulStatus = prvConnectToServerWithBackoffRetries(
            (const char*)pucIotHubHostname, democonfigIOTHUB_PORT, &xNetworkCredentials, &xNetworkContext);
        configASSERT(ulStatus == 0);

        /* Fill in Transport Interface send and receive function pointers. */
        xTransport.pxNetworkContext = &xNetworkContext;
        xTransport.xSend            = TLS_Socket_Send;
        xTransport.xRecv            = TLS_Socket_Recv;

        /* Init IoT Hub option */
        xResult = AzureIoTHubClient_OptionsInit(&xHubOptions);
        configASSERT(xResult == eAzureIoTSuccess);

        xHubOptions.pucModuleID      = (const uint8_t*)democonfigMODULE_ID;
        xHubOptions.ulModuleIDLength = sizeof(democonfigMODULE_ID) - 1;
        xHubOptions.pucModelID       = (const uint8_t*)sampleazureiotMODEL_ID;
        xHubOptions.ulModelIDLength  = strlen(sampleazureiotMODEL_ID);

        ////xHubOptions.pxComponentList=&(azureiothubCREATE_COMPONENT("componentSample"));
        // xHubOptions.ulComponentListLength=1;
        // works on 1 and on strlen( "componentSample" ) as well
        xHubOptions.pxComponentList       = component_list;
        xHubOptions.ulComponentListLength = NUM_COMPONENTS;

        xResult = AzureIoTHubClient_Init(&xAzureIoTHubClient,
            pucIotHubHostname,
            pulIothubHostnameLength,
            pucIotHubDeviceId,
            pulIothubDeviceIdLength,
            &xHubOptions,
            ucMQTTMessageBuffer,
            sizeof(ucMQTTMessageBuffer),
            ullGetUnixTime,
            &xTransport);
        configASSERT(xResult == eAzureIoTSuccess);

#ifdef democonfigDEVICE_SYMMETRIC_KEY
        xResult = AzureIoTHubClient_SetSymmetricKey(&xAzureIoTHubClient,
            (const uint8_t*)democonfigDEVICE_SYMMETRIC_KEY,
            sizeof(democonfigDEVICE_SYMMETRIC_KEY) - 1,
            Crypto_HMAC);
        configASSERT(xResult == eAzureIoTSuccess);
#endif /* democonfigDEVICE_SYMMETRIC_KEY */

        /* Sends an MQTT Connect packet over the already established TLS connection,
         * and waits for connection acknowledgment (CONNACK) packet. */

        LogInfo( ( "Creating an MQTT connection to %s.\r\n", pucIotHubHostname ) );

        xResult = AzureIoTHubClient_Connect(
            &xAzureIoTHubClient, false, &xSessionPresent, sampleazureiotCONNACK_RECV_TIMEOUT_MS);
        configASSERT(xResult == eAzureIoTSuccess);

        xResult = AzureIoTHubClient_SubscribeCommand(
            &xAzureIoTHubClient, prvHandleCommand, &xAzureIoTHubClient, sampleazureiotSUBSCRIBE_TIMEOUT);
        configASSERT(xResult == eAzureIoTSuccess);
        
        xResult = AzureIoTHubClient_SubscribeProperties(
            &xAzureIoTHubClient, prvHandleProperties, &xAzureIoTHubClient, sampleazureiotSUBSCRIBE_TIMEOUT);
        configASSERT(xResult == eAzureIoTSuccess);
        
        /* Get property document after initial connection */
        xResult = AzureIoTHubClient_RequestPropertiesAsync(&xAzureIoTHubClient);
        configASSERT(xResult == eAzureIoTSuccess);
       
        xResult = AzureIoTHubClient_ProcessLoop(&xAzureIoTHubClient, sampleazureiotPROCESS_LOOP_TIMEOUT_MS);
        configASSERT(xResult == eAzureIoTSuccess);
   
        xResult = prvdesiredPropertiesSend_bootup();
        configASSERT(xResult == eAzureIoTSuccess);

        LogInfo( ( "Attempt to receive publish message from IoT Hub.\r\n" ) );

        xResult = AzureIoTHubClient_ProcessLoop(&xAzureIoTHubClient, sampleazureiotPROCESS_LOOP_TIMEOUT_MS);
        configASSERT(xResult == eAzureIoTSuccess);

        /* Publish messages with QoS1, send and process Keep alive messages. */
        for ( ; ; )
        {   
            //mcp320x_read();  

            xResult =FreeRTOS_vt_compute_evaluate_fingerprint_all_sensors(verified_telemetry_DB);
            //configASSERT(xResult == eAzureIoTSuccess);
            sample_pnp_device_telemetry_send(&sample_device, &xAzureIoTHubClient);
            //configASSERT(xResult == eAzureIoTSuccess);
            
            xResult =FreeRTOS_vt_properties(verified_telemetry_DB, &xAzureIoTHubClient);
            configASSERT(xResult == eAzureIoTSuccess);
            
            LogInfo(("Attempt to receive publish message from IoT Hub.\r\n"));
            xResult = AzureIoTHubClient_ProcessLoop(&xAzureIoTHubClient, sampleazureiotPROCESS_LOOP_TIMEOUT_MS);
            configASSERT(xResult == eAzureIoTSuccess);
            


            /* Leave Connection Idle for some time. */
            LogInfo(("Keeping Connection Idle...\r\n\r\n"));
            vTaskDelay(sampleazureiotDELAY_BETWEEN_PUBLISHES_TICKS);
        }

        xResult = AzureIoTHubClient_UnsubscribeProperties(&xAzureIoTHubClient);
        configASSERT(xResult == eAzureIoTSuccess);

        xResult = AzureIoTHubClient_UnsubscribeCommand(&xAzureIoTHubClient);
        configASSERT(xResult == eAzureIoTSuccess);

        /* Send an MQTT Disconnect packet over the already connected TLS over
         * TCP connection. There is no corresponding response for the disconnect
         * packet. After sending disconnect, client must close the network
         * connection. */
        xResult = AzureIoTHubClient_Disconnect(&xAzureIoTHubClient);
        configASSERT(xResult == eAzureIoTSuccess);

        /* Close the network connection.  */
        TLS_Socket_Disconnect(&xNetworkContext);

        /* Wait for some time between two iterations to ensure that we do not
         * bombard the IoT Hub. */
        LogInfo(("Demo completed successfully.\r\n"));
        LogInfo(("Short delay before starting the next iteration.... \r\n\r\n"));
        vTaskDelay(sampleazureiotDELAY_BETWEEN_DEMO_ITERATIONS_TICKS);
    }
}
/*-----------------------------------------------------------*/

#ifdef democonfigENABLE_DPS_SAMPLE

/**
 * @brief Get IoT Hub endpoint and device Id info, when Provisioning service is used.
 *   This function will block for Provisioning service for result or return failure.
 */
static uint32_t prvIoTHubInfoGet(NetworkCredentials_t* pXNetworkCredentials,
    uint8_t** ppucIothubHostname,
    uint32_t* pulIothubHostnameLength,
    uint8_t** ppucIothubDeviceId,
    uint32_t* pulIothubDeviceIdLength)
{
    NetworkContext_t xNetworkContext         = {0};
    TlsTransportParams_t xTlsTransportParams = {0};
    AzureIoTResult_t xResult;
    AzureIoTTransportInterface_t xTransport;
    uint32_t ucSamplepIothubHostnameLength = sizeof(ucSampleIotHubHostname);
    uint32_t ucSamplepIothubDeviceIdLength = sizeof(ucSampleIotHubDeviceId);
    uint32_t ulStatus;

    /* Set the pParams member of the network context with desired transport. */
    xNetworkContext.pParams = &xTlsTransportParams;

    ulStatus = prvConnectToServerWithBackoffRetries(
        democonfigENDPOINT, democonfigIOTHUB_PORT, pXNetworkCredentials, &xNetworkContext);
    configASSERT(ulStatus == 0);

    /* Fill in Transport Interface send and receive function pointers. */
    xTransport.pxNetworkContext = &xNetworkContext;
    xTransport.xSend            = TLS_Socket_Send;
    xTransport.xRecv            = TLS_Socket_Recv;

    xResult = AzureIoTProvisioningClient_Init(&xAzureIoTProvisioningClient,
        (const uint8_t*)democonfigENDPOINT,
        sizeof(democonfigENDPOINT) - 1,
        (const uint8_t*)democonfigID_SCOPE,
        sizeof(democonfigID_SCOPE) - 1,
        (const uint8_t*)democonfigREGISTRATION_ID,
        sizeof(democonfigREGISTRATION_ID) - 1,
        NULL,
        ucMQTTMessageBuffer,
        sizeof(ucMQTTMessageBuffer),
        ullGetUnixTime,
        &xTransport);
    configASSERT(xResult == eAzureIoTSuccess);

#ifdef democonfigDEVICE_SYMMETRIC_KEY
    xResult = AzureIoTProvisioningClient_SetSymmetricKey(&xAzureIoTProvisioningClient,
        (const uint8_t*)democonfigDEVICE_SYMMETRIC_KEY,
        sizeof(democonfigDEVICE_SYMMETRIC_KEY) - 1,
        Crypto_HMAC);
    configASSERT(xResult == eAzureIoTSuccess);
#endif /* democonfigDEVICE_SYMMETRIC_KEY */

    xResult = AzureIoTProvisioningClient_SetRegistrationPayload(
        &xAzureIoTProvisioningClient, sampleazureiotPROVISIONING_PAYLOAD, strlen(sampleazureiotPROVISIONING_PAYLOAD));
    configASSERT(xResult == eAzureIoTSuccess);

    do
    {
        xResult = AzureIoTProvisioningClient_Register(
            &xAzureIoTProvisioningClient, sampleazureiotProvisioning_Registration_TIMEOUT_MS);
    } while (xResult == eAzureIoTErrorPending);

    if (xResult == eAzureIoTSuccess)
    {
        //LogInfo(("Successfully acquired IoT Hub name and Device ID"));
    }
    else
    {
        //LogInfo(("Error geting IoT Hub name and Device ID: 0x%08", xResult));
    }

    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTProvisioningClient_GetDeviceAndHub(&xAzureIoTProvisioningClient,
        ucSampleIotHubHostname,
        &ucSamplepIothubHostnameLength,
        ucSampleIotHubDeviceId,
        &ucSamplepIothubDeviceIdLength);
    configASSERT(xResult == eAzureIoTSuccess);

    AzureIoTProvisioningClient_Deinit(&xAzureIoTProvisioningClient);

    /* Close the network connection.  */
    TLS_Socket_Disconnect(&xNetworkContext);

    *ppucIothubHostname      = ucSampleIotHubHostname;
    *pulIothubHostnameLength = ucSamplepIothubHostnameLength;
    *ppucIothubDeviceId      = ucSampleIotHubDeviceId;
    *pulIothubDeviceIdLength = ucSamplepIothubDeviceIdLength;

    return 0;
}

#endif /* democonfigENABLE_DPS_SAMPLE */
/*-----------------------------------------------------------*/

/**
 * @brief Connect to server with backoff retries.
 */
static uint32_t prvConnectToServerWithBackoffRetries(const char* pcHostName,
    uint32_t port,
    NetworkCredentials_t* pxNetworkCredentials,
    NetworkContext_t* pxNetworkContext)
{
    TlsTransportStatus_t xNetworkStatus;
    BackoffAlgorithmStatus_t xBackoffAlgStatus = BackoffAlgorithmSuccess;
    BackoffAlgorithmContext_t xReconnectParams;
    uint16_t usNextRetryBackOff = 0U;

    /* Initialize reconnect attempts and interval. */
    BackoffAlgorithm_InitializeParams(&xReconnectParams,
        sampleazureiotRETRY_BACKOFF_BASE_MS,
        sampleazureiotRETRY_MAX_BACKOFF_DELAY_MS,
        sampleazureiotRETRY_MAX_ATTEMPTS);

    /* Attempt to connect to IoT Hub. If connection fails, retry after
     * a timeout. Timeout value will exponentially increase till maximum
     * attempts are reached.
     */
    do
    {
        LogInfo( ( "Creating a TLS connection to %s:%u.\r\n", pcHostName, port ) );
        /* Attempt to create a mutually authenticated TLS connection. */
        xNetworkStatus = TLS_Socket_Connect(pxNetworkContext,
            pcHostName,
            port,
            pxNetworkCredentials,
            sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS,
            sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS);

        if (xNetworkStatus != eTLSTransportSuccess)
        {
            /* Generate a random number and calculate backoff value (in milliseconds) for
             * the next connection retry.
             * Note: It is recommended to seed the random number generator with a device-specific
             * entropy source so that possibility of multiple devices retrying failed network operations
             * at similar intervals can be avoided. */
            xBackoffAlgStatus = BackoffAlgorithm_GetNextBackoff(&xReconnectParams, configRAND32(), &usNextRetryBackOff);

            if (xBackoffAlgStatus == BackoffAlgorithmRetriesExhausted)
            {
                LogError(("Connection to the IoT Hub failed, all attempts exhausted."));
            }
            else if (xBackoffAlgStatus == BackoffAlgorithmSuccess)
            {
                LogWarn(("Connection to the IoT Hub failed [%d]. "
                         "Retrying connection with backoff and jitter [%d]ms.",
                    xNetworkStatus,
                    usNextRetryBackOff));
                vTaskDelay(pdMS_TO_TICKS(usNextRetryBackOff));
            }
        }
    } while ((xNetworkStatus != eTLSTransportSuccess) && (xBackoffAlgStatus == BackoffAlgorithmSuccess));

    return xNetworkStatus == eTLSTransportSuccess ? 0 : 1;
}


/*-----------------------------------------------------------*/

/*
 * @brief Create the task that demonstrates the AzureIoTHub demo
 */

#ifdef USINGESP
void uart_init(){

    const uart_port_t uart_num = UART_NUM_2;
uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE ,
    .source_clk = UART_SCLK_APB,
};
// Configure UART parameters
ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024 * 2, 0, 0, NULL, 0));
ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, -1, -1));

}
#endif



void vStartDemoTask(void)
{
        #ifdef USINGESP
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    
        uart_init();
    printf("\n spi_bus_initialize && mcp320x_initialize \n");
    #endif

// {
//     //extern spi_bus_config_t bus_cfg;
//         ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    
//         uart_init();
//     printf("\n spi_bus_initialize && mcp320x_initialize \n");
    //SemaphoreHandle_t  sema_vi=NULL; 
//sema_vi = xSemaphoreCreateBinary();
    /* This example uses a single application task, which in turn is used to
     * connect, subscribe, publish, unsubscribe and disconnect from the IoT Hub */
    xTaskCreate(prvAzureDemoTask, /* Function that implements the task. */
        "AzureDemoTask",          /* Text name for the task - only used for debugging. */
        democonfigDEMO_STACKSIZE, /* Size of stack (in words, not bytes) to allocate for the task. */
        NULL,                     /* Task parameter - not used in this case. */
        configMAX_PRIORITIES - 1,         /* Task priority, must be between 0 and configMAX_PRIORITIES - 1. */
        NULL);                    /* Used to pass out a handle to the created task - not used in this case. */
}
/*-----------------------------------------------------------*/
