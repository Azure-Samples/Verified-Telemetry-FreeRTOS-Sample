/* Copyright (c) Microsoft Corporation. All rights reserved. */
/* SPDX-License-Identifier: MIT */

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <time.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
 
/* Azure Provisioning/IoT Hub library includes */
#include "azure_iot_hub_client.h"
#include "azure_iot_provisioning_client.h"
#include "azure_iot_hub_client_properties.h"

/* Azure JSON includes */
#include "azure_iot_json_reader.h"
#include "azure_iot_json_writer.h"

/* Exponential backoff retry include. */
#include "backoff_algorithm.h"

/* Transport interface implementation include header for TLS. */
#include "transport_tls_socket.h"
#include "sample_pnp_device_component.h"

/* Crypto helper header. */
#include "crypto.h"

/* Demo Specific configs. */
#include "demo_config.h"

#include "FreeRTOS_verified_telemetry.h"
#include "FreeRTOS_vt_fallcurve_component.h"
#include "sample_freertos_verified_telemetry_init.h"
#include "sample_vt_device_driver.h"
/*-----------------------------------------------------------*/

/* Compile time error for undefined configs. */
#if !defined( democonfigHOSTNAME ) && !defined( democonfigENABLE_DPS_SAMPLE )
    #error "Define the config democonfigHOSTNAME by following the instructions in file demo_config.h."
#endif

#if !defined( democonfigENDPOINT ) && defined( democonfigENABLE_DPS_SAMPLE )
    #error "Define the config dps endpoint by following the instructions in file demo_config.h."
#endif

#ifndef democonfigROOT_CA_PEM
    #error "Please define Root CA certificate of the IoT Hub(democonfigROOT_CA_PEM) in demo_config.h."
#endif

#if defined( democonfigDEVICE_SYMMETRIC_KEY ) && defined( democonfigCLIENT_CERTIFICATE_PEM )
    #error "Please define only one auth democonfigDEVICE_SYMMETRIC_KEY or democonfigCLIENT_CERTIFICATE_PEM in demo_config.h."
#endif

#if !defined( democonfigDEVICE_SYMMETRIC_KEY ) && !defined( democonfigCLIENT_CERTIFICATE_PEM )
    #error "Please define one auth democonfigDEVICE_SYMMETRIC_KEY or democonfigCLIENT_CERTIFICATE_PEM in demo_config.h."
#endif

/*-----------------------------------------------------------*/

/**
 * @brief The maximum number of retries for network operation with server.
 */
#define sampleazureiotRETRY_MAX_ATTEMPTS                  ( 5U )

/**
 * @brief The maximum back-off delay (in milliseconds) for retrying failed operation
 *  with server.
 */
#define sampleazureiotRETRY_MAX_BACKOFF_DELAY_MS          ( 5000U )

/**
 * @brief The base back-off delay (in milliseconds) to use for network operation retry
 * attempts.
 */
#define sampleazureiotRETRY_BACKOFF_BASE_MS               ( 500U )

/**
 * @brief Timeout for receiving CONNACK packet in milliseconds.
 */
#define sampleazureiotCONNACK_RECV_TIMEOUT_MS             ( 10 * 1000U )

/**
 * @brief The model id for this device
 *
 * https://github.com/Azure/opendigitaltwins-dtdl/blob/master/DTDL/v2/samples/Thermostat.json
 *
 * The model id is the JSON document (also called the Digital Twins Model Identifier or DTMI) which
 * defines the capability of your device. The functionality of the device should match what is
 * described in the corresponding DTMI. Should you choose to program your own PnP capable device,
 * the functionality would need to match the DTMI and you would need to update the below 'model_id'.
 * Please see the sample README for more information on this DTMI.
 *
 */
#define sampleazureiotMODEL_ID                            "dtmi:azure:verifiedtelemetry:sample:GSG;1"

/**
 * @brief Date-time to use for the model id
 */
#define sampleazureiotDATE_TIME_FORMAT                    "%Y-%m-%dT%H:%M:%S.000Z"

/**
 * @brief Telemetry values
 */
#define sampleazureiotTELEMETRY_NAME                      "temperature"

/**
 * @brief Property Values
 */
#define sampleazureiotCOMPONENT_NAME                    "sampleComponent"
#define sampleazureiotPROPERTY_STATUS_SUCCESS             200
#define sampleazureiotPROPERTY_SUCCESS                    "success"
#define sampleazureiotPROPERTY_TARGET_TEMPERATURE_TEXT    "targetTemperature"
#define sampleazureiotPROPERTY_TEMPERATURE_THRESHOLD_TEXT "temperatureThreshold"
#define sampleazureiotPROPERTY_MAX_TEMPERATURE_TEXT       "maxTempSinceLastReboot"
#define sampleazureiotPROPERTY_REPORTED_STATUS       "Status"
#define sampleazureiotsampleDeviceCOMPONENT_NAME     "sampleDevice"
#define sampleazureiotvTDeviceCOMPONENT_NAME       "vTDevice"
#define sampleazureiotvTsoilMoistureExternal1COMPONENT_NAME       "vTsoilMoistureExternal1"
#define sampleazureiotvTsoilMoistureExternal2COMPONENT_NAME       "vTsoilMoistureExternal2"
#define sampleazureiotREPORTED     "\"reported\":"
static uint8_t ucPropertyPayloadBuffer[ 256 ];

/**
 * @brief Command values
 */
#define sampleazureiotCOMMAND_MAX_MIN_REPORTWITHCOM    "componentSample*getMaxMinReport"
#define sampleazureiotCOMMAND_MAX_MIN_REPORT    "getMaxMinReport"
#define sampleazureiotCOMMAND_STATUS_COMMAND    "statusCommand"
#define sampleazureiotCOMMAND_STATUS_COMMAND_REQUESTED    "getStatus"
#define sampleazureiotCOMMAND_SINCE             "since"
#define sampleazureiotCOMMAND_MAX_TEMP          "maxTemp"
#define sampleazureiotCOMMAND_MIN_TEMP          "minTemp"
#define sampleazureiotCOMMAND_AV_TEMP           "avgTemp"
#define sampleazureiotCOMMAND_START_TIME        "startTime"
#define sampleazureiotCOMMAND_END_TIME          "endTime"
#define sampleazureiotCOMMAND_EMPTY_PAYLOAD     "{}"
static uint8_t ucCommandPayloadBuffer[ 256 ];
static uint8_t ucCommandStartTimeValueBuffer[ 32 ];
static uint8_t ucCommandEndTimeValueBuffer[ 32 ];

/**
 *@brief The Telemetry message published in this example.
 */
#define sampleazureiotMESSAGE                        "{\"" sampleazureiotTELEMETRY_NAME "\":%0.2f}"

#define sampleazuretelemetryMESSAGE                        "{\"" sampleazureiotTELEMETRY_NAME "\": 5}"

/**
 * @brief Device values
 */
#define sampleazureiotDEFAULT_START_TEMP_COUNT       1
#define sampleazureiotDEFAULT_START_TEMP_CELSIUS     22.0
#define sampleazureiotDOUBLE_DECIMAL_PLACE_DIGITS    2
static double xDeviceMaximumTemperature = sampleazureiotDEFAULT_START_TEMP_CELSIUS;
static double xDeviceMinimumTemperature = sampleazureiotDEFAULT_START_TEMP_CELSIUS;
static double xDeviceAverageTemperature = sampleazureiotDEFAULT_START_TEMP_CELSIUS;
static bool statusBool=true;
int32_t ConfidenceMetriclocalone;
int32_t ConfidenceMetriclocaltwo;

 
//vt variables

#define NUM_COMPONENTS 4
static bool deviceStatusReportedProperty;
static bool enableVerifiedTelemetryWritableProperty;
static bool setLedState;
static bool telemetryStatusSoilMoisture1;
static bool telemetryStatusSoilMoisture2;
static bool vTDevicedeviceStatus;
#define sampleazureiotPROPERTY_ENABLE_VERIFIED_TELEMETRY "enableVerifiedTelemetry"
#define sampleazureiotSAMPLE_DEVICE_COMPONENT_NAME                    "sampleDevice"
#define sampleazureiotSOIL_MOISTURE_ONE_COMPONENT_NAME                    "vTsoilMoistureExternal1"
#define sampleazureiotSOIL_MOISTURE_TWO_COMPONENT_NAME                    "vTsoilMoistureExternal2"
#define sampleazureiotSET_LED_STATUS_COMMAND                    "setLedState"
#define sampleazureiotPROPERTY_REPORTED_LED_STATE       "ledState"

char * deviceTelemetryName[7]={   "soilMoistureExternal1",
                                    "soilMoistureExternal2",
                                    "temperature",
                                    "pressure",
                                    "humidityPercentage",
                                    "acceleration",
                                    "magnetic"};


    FreeRTOS_VERIFIED_TELEMETRY_DB * verified_telemetry_DB;
    #define SAMPLE_DEFAULT_DEVICE_SENSOR_READING (22)

    SAMPLE_PNP_DEVICE_COMPONENT sample_device;
    static const CHAR sample_device_component[] = "sampleDevice";

/*
static float deviceTelemetryValue[7];

static float soilMoistureExternal1=0;
static float soilMoistureExternal2=0;
static float temperature=0;
static float pressure=0;
static float humidityPercentage=0;
static float acceleration=0;
static float magnetic=0;
*/
/*
char * componentNames[NUM_COMPONENTS]={   "sampleDevice",
                                    "vTDevice",
                                    "vTsoilMoistureExternal1",
                                    "vTsoilMoistureExternal2"};

uint32_t numberOfReportedPropertiesInComponents[NUM_COMPONENTS]={1,1,2,2};

char * allReportedPropertyNames[6]={"ledState",
                                    "deviceStatus",
                                    "telemetryStatus",
                                    "fingerprintType",
                                    "telemetryStatus",
                                    "fingerprintType"};

char allReportedPropertyType[6]={'b',
                                    'b',
                                    'b',
                                    's',
                                    'b',
                                    's'};
*/
//vt variables end
/**
 * @brief The payload to send to the Device Provisioning Service
 */
#define sampleazureiotPROVISIONING_PAYLOAD                    "{\"modelId\":\"" sampleazureiotMODEL_ID "\"}"

/**
 * @brief Time in ticks to wait between each cycle of the demo implemented
 * by prvMQTTDemoTask().
 */
#define sampleazureiotDELAY_BETWEEN_DEMO_ITERATIONS_TICKS     ( pdMS_TO_TICKS( 5000U ) )

/**
 * @brief Timeout for MQTT_ProcessLoop in milliseconds.
 */
#define sampleazureiotPROCESS_LOOP_TIMEOUT_MS                 ( 500U )

/**
 * @brief Delay (in ticks) between consecutive cycles of MQTT publish operations in a
 * demo iteration.
 *
 * Note that the process loop also has a timeout, so the total time between
 * publishes is the sum of the two delays.
 */
#define sampleazureiotDELAY_BETWEEN_PUBLISHES_TICKS           ( pdMS_TO_TICKS( 3000U ) )

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS          ( 2000U )

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define sampleazureiotProvisioning_Registration_TIMEOUT_MS    ( 20U )

/**
 * @brief Wait timeout for subscribe to finish.
 */
#define sampleazureiotSUBSCRIBE_TIMEOUT                       ( 10 * 1000U )
/*-----------------------------------------------------------*/

/**
 * @brief Unix time.
 *
 * @return Time in milliseconds.
 */
uint64_t ullGetUnixTime( void );
/*-----------------------------------------------------------*/

/* Define buffer for IoT Hub info.  */
#ifdef democonfigENABLE_DPS_SAMPLE
    static uint8_t ucSampleIotHubHostname[ 128 ];
    static uint8_t ucSampleIotHubDeviceId[ 128 ];
    static AzureIoTProvisioningClient_t xAzureIoTProvisioningClient;
#endif /* democonfigENABLE_DPS_SAMPLE */

//static uint8_t ucScratchBuffer[ 256 ];

/* Each compilation unit must define the NetworkContext struct. */
struct NetworkContext
{
    TlsTransportParams_t * pParams;
};

static AzureIoTHubClient_t xAzureIoTHubClient;
/*-----------------------------------------------------------*/

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
    static uint32_t prvIoTHubInfoGet( NetworkCredentials_t * pXNetworkCredentials,
                                      uint8_t ** ppucIothubHostname,
                                      uint32_t * pulIothubHostnameLength,
                                      uint8_t ** ppucIothubDeviceId,
                                      uint32_t * pulIothubDeviceIdLength );

#endif /* democonfigENABLE_DPS_SAMPLE */

/**
 * @brief The task used to demonstrate the MQTT API.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation. Not
 * used in this example.
 */
static void prvAzureDemoTask( void * pvParameters );
 
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
static uint32_t prvConnectToServerWithBackoffRetries( const char * pcHostName,
                                                      uint32_t ulPort,
                                                      NetworkCredentials_t * pxNetworkCredentials,
                                                      NetworkContext_t * pxNetworkContext );
/*-----------------------------------------------------------*/

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t ucMQTTMessageBuffer[ democonfigNETWORK_BUFFER_SIZE ];
 
/*-----------------------------------------------------------*/
/*
static int32_t prvTelemetryPayloadCreate(char *ucScratchBuffer, 
                                        unsigned long ucScratchBufferSize,
                                        char *telemetryKey, double telemetryValue)
{
    AzureIoTJSONWriter_t xWriter;
    AzureIoTResult_t xResult;
    xResult = AzureIoTJSONWriter_Init(&xWriter, (uint8_t *)ucScratchBuffer, ucScratchBufferSize);
    configASSERT(xResult == eAzureIoTSuccess);

    //AzureIoTJSONWriter_AppendString(&xWriter,sampleazureiotREPORTED,strlen(sampleazureiotREPORTED));

    xResult = AzureIoTJSONWriter_AppendBeginObject(&xWriter);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue(&xWriter, (const uint8_t *)telemetryKey,
                                                               strlen(telemetryKey),
                                                               telemetryValue, sampleazureiotDOUBLE_DECIMAL_PLACE_DIGITS);
    configASSERT(xResult == eAzureIoTSuccess);

    xResult = AzureIoTJSONWriter_AppendEndObject(&xWriter);
    configASSERT(xResult == eAzureIoTSuccess);


    return AzureIoTJSONWriter_GetBytesUsed(&xWriter);
}
*/
 
static int32_t prvProcessSetLedStateCommand(const uint8_t * pucPayload,
                                        uint32_t ulPayloadLength )
{
      
    AzureIoTResult_t xResult;
    AzureIoTJSONReader_t xReader;

    xResult = AzureIoTJSONReader_Init( &xReader, pucPayload, ulPayloadLength );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONReader_NextToken( &xReader );
    configASSERT( xResult == eAzureIoTSuccess );
    xResult = AzureIoTJSONReader_GetTokenBool( &xReader,&setLedState);
    configASSERT( xResult == eAzureIoTSuccess );

    printf(" setLedState %d \n",setLedState);


//need to make this comonent reorted roerty send

    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;


    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)sampleazureiotSAMPLE_DEVICE_COMPONENT_NAME,
                                                    strlen(sampleazureiotSAMPLE_DEVICE_COMPONENT_NAME));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)sampleazureiotPROPERTY_REPORTED_LED_STATE,
                                                     strlen( sampleazureiotPROPERTY_REPORTED_LED_STATE ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBool( &xWriter, setLedState);
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }
    printf(" next2 ");

    return xResult;

}

static int32_t prvReportedPropertiesSendForSampleDevice()
{


    AzureIoTResult_t xResult;
    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)"sampleDevice",
                                                    strlen("sampleDevice"));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"ledState",
                                                     strlen( "ledState" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBool( &xWriter, setLedState);
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }

    return xResult;

}

static int32_t prvReportedPropertiesSendForvTDevice()
{
    AzureIoTResult_t xResult;
    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)"vTDevice",
                                                    strlen("vTDevice"));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"deviceStatus",
                                                     strlen( "deviceStatus" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBool( &xWriter, vTDevicedeviceStatus);
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }

    return xResult;

}

static int32_t prvReportedPropertiesSendForvTsoilMoistureExternal1()
{
    AzureIoTResult_t xResult;
    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)"vTsoilMoistureExternal1",
                                                    strlen("vTsoilMoistureExternal1"));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"telemetryStatus",
                                                     strlen( "telemetryStatus" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBool( &xWriter, true);
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"fingerprintType",
                                                     strlen( "fingerprintType" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendString( &xWriter, (const uint8_t *)"Fallcurve",strlen("Fallcurve"));
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }

    return xResult;

}


static int32_t prvReportedPropertiesSendForvTsoilMoistureExternal2()
{
    AzureIoTResult_t xResult;
    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)"vTsoilMoistureExternal2",
                                                    strlen("vTsoilMoistureExternal2"));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"telemetryStatus",
                                                     strlen( "telemetryStatus" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBool( &xWriter, true);
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"fingerprintType",
                                                     strlen( "fingerprintType" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendString( &xWriter, (const uint8_t *)"Fallcurve",strlen("Fallcurve"));
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }

    return xResult;

}


static int32_t prvReportedPropertiesSendForConfidenceMetricSoilMoisture1()
{
    AzureIoTResult_t xResult;
    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)"vTsoilMoistureExternal1",
                                                    strlen("vTsoilMoistureExternal1"));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"fingerprintTemplateConfidenceMetric",
                                                     strlen( "fingerprintTemplateConfidenceMetric" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendInt32( &xWriter, 50);
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }

    return xResult;

}


static int32_t prvReportedPropertiesSendfingerprintTemplateMapSoilMoisture1()
{
    AzureIoTResult_t xResult;
    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)"vTsoilMoistureExternal1",
                                                    strlen("vTsoilMoistureExternal1"));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"fingerprintTemplate",
                                                     strlen( "fingerprintTemplate" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"firstValue",
                                                     strlen( "firstValue" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendString( &xWriter,(const uint8_t *)"testStringone",strlen("testStringone"));
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"secondValue",
                                                     strlen( "secondValue" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendString( &xWriter,(const uint8_t *)"testString",strlen("testString"));
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }

    return xResult;

}


static int32_t prvReportedPropertiesSendfingerprintTemplateMapSoilMoisture2()
{
    AzureIoTResult_t xResult;
    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)"vTsoilMoistureExternal2",
                                                    strlen("vTsoilMoistureExternal2"));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"fingerprintTemplate",
                                                     strlen( "fingerprintTemplate" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"firstValueSensor2",
                                                     strlen( "firstValueSensor2" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendString( &xWriter,(const uint8_t *)"testStringSensor2",strlen("testStringSensor2"));
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"secondValueSensor2",
                                                     strlen( "secondValueSensor2" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendString( &xWriter,(const uint8_t *)"testStringSensor2",strlen("testStringSensor2"));
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }

    return xResult;

}

static int32_t prvReportedPropertiesSendForConfidenceMetricSoilMoisture2()
{
    AzureIoTResult_t xResult;
    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)"vTsoilMoistureExternal2",
                                                    strlen("vTsoilMoistureExternal2"));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)"fingerprintTemplateConfidenceMetric",
                                                     strlen( "fingerprintTemplateConfidenceMetric" ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendInt32( &xWriter, 0x64);
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }

    return xResult;

}

/*
char * componentNames[NUM_COMPONENTS]={   "sampleDevice",
                                    "vTDevice",
                                    "vTsoilMoistureExternal1",
                                    "vTsoilMoistureExternal2"};

uint32_t numberOfReportedPropertiesInComponents[NUM_COMPONENTS]={1,1,2,2};

char * allReportedPropertyNames[6]={"ledState",
                                    "deviceStatus",
                                    "telemetryStatus",
                                    "fingerprintType",
                                    "telemetryStatus",
                                    "fingerprintType"};

char allReportedPropertyType[6]={'b',
                                    'b',
                                    'b',
                                    's',
                                    'b',
                                    's'};
*/

static int32_t prvReportedPropertiesSend()
{
    AzureIoTResult_t xResult;

    //xResult = prvReportedPropertiesSendForSampleDevice();  
    //configASSERT( xResult == eAzureIoTSuccess );

    //xResult = prvReportedPropertiesSendForvTDevice();  
    //configASSERT( xResult == eAzureIoTSuccess );

    //xResult = prvReportedPropertiesSendForvTsoilMoistureExternal1();  
    //configASSERT( xResult == eAzureIoTSuccess );

    //xResult = prvReportedPropertiesSendForvTsoilMoistureExternal2();  
    //configASSERT( xResult == eAzureIoTSuccess );

    //xResult = prvReportedPropertiesSendForConfidenceMetricSoilMoisture1();  
    //configASSERT( xResult == eAzureIoTSuccess );

    //xResult = prvReportedPropertiesSendForConfidenceMetricSoilMoisture2();  
    //configASSERT( xResult == eAzureIoTSuccess );

    //xResult = prvReportedPropertiesSendfingerprintTemplateMapSoilMoisture1();  
    //configASSERT( xResult == eAzureIoTSuccess );

    //xResult = prvReportedPropertiesSendfingerprintTemplateMapSoilMoisture2();  
    //configASSERT( xResult == eAzureIoTSuccess );  

    xResult = FreeRTOS_vt_send_desired_property_after_boot(verified_telemetry_DB,&xAzureIoTHubClient,NX_AZURE_IOT_PNP_PROPERTIES);

    return xResult;
}

static void prvProcessSetResetFingerprintOneCommand()
{
    printf(" Fingerprint Reset one \n");
}


static void prvProcessRetrainFingerprintOneCommand()
{
    printf(" retrain Fingerprint one \n");
}

static void prvProcessSetResetFingerprintTwoCommand()
{
    printf(" Fingerprint Reset two \n");
}


static void prvProcessRetrainFingerprintTwoCommand()
{
    printf(" retrain Fingerprint two \n");
}

static int32_t prvProcessStatusCommand(const uint8_t * pucPayload,
                                        uint32_t ulPayloadLength )
{
     
    AzureIoTResult_t xResult;
    AzureIoTJSONReader_t xReader;

    xResult = AzureIoTJSONReader_Init( &xReader, pucPayload, ulPayloadLength );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONReader_NextToken( &xReader );
    configASSERT( xResult == eAzureIoTSuccess );
    xResult = AzureIoTJSONReader_GetTokenBool( &xReader,&statusBool);
    configASSERT( xResult == eAzureIoTSuccess );

    printf(" statusBool %d \n",statusBool);


//need to make this comonent reorted roerty send

    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;


    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)sampleazureiotCOMPONENT_NAME,
                                                    strlen(sampleazureiotCOMPONENT_NAME));

    xResult = AzureIoTJSONWriter_AppendPropertyName( &xWriter, (const uint8_t *)sampleazureiotPROPERTY_REPORTED_STATUS,
                                                     strlen( sampleazureiotPROPERTY_REPORTED_STATUS ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBool( &xWriter, statusBool);
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );


    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }
    printf(" next2 ");

    return xResult;

}

/**
 * @brief Generate max min payload.
 */
static int32_t prvInvokeMaxMinCommand( const uint8_t * pucPayload,
                                       uint32_t ulPayloadLength )
{
    AzureIoTResult_t xResult;
    AzureIoTJSONReader_t xReader;
    AzureIoTJSONWriter_t xWriter;
    uint32_t ulSinceTimeLength;
    time_t xRawTime;
    struct tm * pxTimeInfo;
    size_t xEndTimeLength;

    /* Get the start time */
    xResult = AzureIoTJSONReader_Init( &xReader, pucPayload, ulPayloadLength );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONReader_NextToken( &xReader );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONReader_GetTokenString( &xReader,
                                                 ucCommandStartTimeValueBuffer,
                                                 sizeof( ucCommandStartTimeValueBuffer ),
                                                 &ulSinceTimeLength );
    configASSERT( xResult == eAzureIoTSuccess );


    /* Get the current time as a string. */
    time( &xRawTime );
    pxTimeInfo = localtime( &xRawTime );
    xEndTimeLength = strftime(
        (char *)ucCommandEndTimeValueBuffer,
        sizeof( ucCommandEndTimeValueBuffer ),
        sampleazureiotDATE_TIME_FORMAT,
        pxTimeInfo );

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucCommandPayloadBuffer, sizeof( ucCommandPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue( &xWriter, (const uint8_t *)sampleazureiotCOMMAND_MAX_TEMP,
                                                                strlen( sampleazureiotCOMMAND_MAX_TEMP ),
                                                                xDeviceMaximumTemperature, sampleazureiotDOUBLE_DECIMAL_PLACE_DIGITS );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue( &xWriter, (const uint8_t *)sampleazureiotCOMMAND_MIN_TEMP,
                                                                strlen( sampleazureiotCOMMAND_MIN_TEMP ),
                                                                xDeviceMinimumTemperature, sampleazureiotDOUBLE_DECIMAL_PLACE_DIGITS );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyWithDoubleValue( &xWriter, (const uint8_t *)sampleazureiotCOMMAND_AV_TEMP,
                                                                strlen( sampleazureiotCOMMAND_AV_TEMP ),
                                                                xDeviceAverageTemperature, sampleazureiotDOUBLE_DECIMAL_PLACE_DIGITS );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyWithStringValue( &xWriter, (const uint8_t *)sampleazureiotCOMMAND_START_TIME,
                                                                strlen( sampleazureiotCOMMAND_START_TIME ),
                                                                ucCommandStartTimeValueBuffer, ulSinceTimeLength );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendPropertyWithStringValue( &xWriter, (const uint8_t *)sampleazureiotCOMMAND_END_TIME,
                                                                strlen( sampleazureiotCOMMAND_END_TIME ),
                                                                ucCommandEndTimeValueBuffer, xEndTimeLength );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    return AzureIoTJSONWriter_GetBytesUsed( &xWriter );
}
/*-----------------------------------------------------------*/

/**
 * @brief Command message callback handler
 */
//when ever we recive a command we recive a pxMessage struct, this would have the commonent name command name and command payload
//we need to check ommonent name command name and process command payload no need to ahve a secal fucntion for comonent command
static void prvHandleCommand( AzureIoTHubClientCommandRequest_t * pxMessage,
                              void * pvContext )
{
    int32_t lCommandNameLength;
    int32_t lCommandNameLengthStatus;
    int32_t setLedStateCommandNameLength;
    int32_t ulCommandPayloadLength;
    AzureIoTHubClient_t * xHandle = ( AzureIoTHubClient_t * ) pvContext;
    int32_t lcomponontNameLength;
    int32_t sampleDeviceComponontNameLength;
    int32_t soilMoistureOneComponontNameLength;
    int32_t soilMoistureTwoComponontNameLength;

    LogInfo( ( "Command payload : %.*s \r\n",
               pxMessage->ulPayloadLength,
               pxMessage->pvMessagePayload ) );


    AzureIoTJSONReader_t xReader;
    AzureIoTJSONWriter_t xWriter;
    AzureIoTResult_t xResult;
    UINT status_code;
    xResult = AzureIoTJSONReader_Init( &xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength );
    configASSERT( xResult == eAzureIoTSuccess );

    sampleDeviceComponontNameLength=strlen(sampleazureiotSAMPLE_DEVICE_COMPONENT_NAME);
    soilMoistureOneComponontNameLength=strlen(sampleazureiotSOIL_MOISTURE_ONE_COMPONENT_NAME);
    soilMoistureTwoComponontNameLength=strlen(sampleazureiotSOIL_MOISTURE_TWO_COMPONENT_NAME);
    lcomponontNameLength=strlen(sampleazureiotCOMPONENT_NAME);
    lCommandNameLength = strlen(sampleazureiotCOMMAND_MAX_MIN_REPORT);
    lCommandNameLengthStatus = strlen(sampleazureiotCOMMAND_STATUS_COMMAND);
    setLedStateCommandNameLength = strlen(sampleazureiotSET_LED_STATUS_COMMAND);
    
    if( ( lcomponontNameLength == pxMessage->usComponentNameLength ) &&
        ( strncmp( sampleazureiotCOMPONENT_NAME, (const char *)pxMessage->pucComponentName, lcomponontNameLength ) == 0 ) )
    {
        
        if( ( lCommandNameLength == pxMessage->usCommandNameLength ) &&
            ( strncmp( sampleazureiotCOMMAND_MAX_MIN_REPORT, (const char *)pxMessage->pucCommandName, lCommandNameLength ) == 0 ) )
        {
            /* Is for max min report */
            ulCommandPayloadLength = prvInvokeMaxMinCommand( pxMessage->pvMessagePayload, pxMessage->ulPayloadLength );

            if( ulCommandPayloadLength > 0 )
            {
                if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 200,
                                                        ucCommandPayloadBuffer,
                                                        ulCommandPayloadLength ) != eAzureIoTSuccess )
                {
                    LogError( ( "Error sending command response" ) );
                }
            }
            else
            {
                LogError( ( "Error generating command payload" ) );
            }
        }

        else if( ( lCommandNameLengthStatus == pxMessage->usCommandNameLength ) &&
            ( strncmp( sampleazureiotCOMMAND_STATUS_COMMAND, (const char *)pxMessage->pucCommandName, lCommandNameLengthStatus ) == 0 ) )
        {

                prvProcessStatusCommand(pxMessage->pvMessagePayload, pxMessage->ulPayloadLength );

                if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 200,
                                                        NULL,
                                                        0 ) != eAzureIoTSuccess )
                {
                    LogError( ( "Error sending command response" ) );
                }

                //prvStatusCommandPropertyUpdate();

        }

        else
        {
            /* Not for max min report (not for this device) */
            LogInfo( ( "Received command is not for this device" ) );

            if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 404,
                                                    (const uint8_t *)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                                                    strlen( sampleazureiotCOMMAND_EMPTY_PAYLOAD ) ) != eAzureIoTSuccess )
            {
                LogError( ( "Error sending command response" ) );
            }
        }
    }

    else if( ( sampleDeviceComponontNameLength == pxMessage->usComponentNameLength ) &&
        ( strncmp( sampleazureiotSAMPLE_DEVICE_COMPONENT_NAME, (const char *)pxMessage->pucComponentName, sampleDeviceComponontNameLength ) == 0 ) )
    {

        if( ( setLedStateCommandNameLength == pxMessage->usCommandNameLength ) &&
            ( strncmp( sampleazureiotSET_LED_STATUS_COMMAND, (const char *)pxMessage->pucCommandName, setLedStateCommandNameLength ) == 0 ) )
        {
                //prvProcessSetLedStateCommand(pxMessage->pvMessagePayload, pxMessage->ulPayloadLength );

                sample_pnp_device_process_command(&sample_device,
                        (UCHAR *)pxMessage->pucComponentName,
                        pxMessage->usComponentNameLength,
                        (UCHAR *)pxMessage->pucCommandName,
                        pxMessage->usCommandNameLength,
                        &xReader,
                        &xWriter,
                        &status_code);

                sample_pnp_device_led_state_property(&sample_device,&xAzureIoTHubClient);

                if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, status_code,
                                                        NULL,
                                                        0 ) != eAzureIoTSuccess )
                {
                    LogError( ( "Error sending command response" ) );
                }

                //prvStatusCommandPropertyUpdate();

        }

        else
        {
            /* Not for max min report (not for this device) */
            LogInfo( ( "Received command is not for this device" ) );

            if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 404,
                                                    (const uint8_t *)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                                                    strlen( sampleazureiotCOMMAND_EMPTY_PAYLOAD ) ) != eAzureIoTSuccess )
            {
                LogError( ( "Error sending command response" ) );
            }
        }


    }


    else if( ( soilMoistureOneComponontNameLength == pxMessage->usComponentNameLength ) &&
        ( strncmp( sampleazureiotSOIL_MOISTURE_ONE_COMPONENT_NAME, (const char *)pxMessage->pucComponentName, soilMoistureOneComponontNameLength ) == 0 ) )
    {

        
        if( ( strlen("setResetFingerprintTemplate") == pxMessage->usCommandNameLength ) &&
            ( strncmp( "setResetFingerprintTemplate", (const char *)pxMessage->pucCommandName, strlen("setResetFingerprintTemplate") ) == 0 ) )
        {       printf("vTsoilMoistureExternal1 command \n");

                FreeRTOS_vt_process_command(verified_telemetry_DB,
                                            xHandle,
                                            (UCHAR *)pxMessage->pucComponentName,
                                            pxMessage->usComponentNameLength,
                                            (UCHAR *)pxMessage->pucCommandName,
                                            pxMessage->usCommandNameLength,
                                            NULL,NULL,0);

                //prvProcessSetResetFingerprintOneCommand();

                if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 200,
                                                        NULL,
                                                        0 ) != eAzureIoTSuccess )
                {
                    LogError( ( "Error sending command response" ) );
                }

        }

        else if( ( strlen("retrainFingerprintTemplate") == pxMessage->usCommandNameLength ) &&
            ( strncmp( "retrainFingerprintTemplate", (const char *)pxMessage->pucCommandName, strlen("retrainFingerprintTemplate") ) == 0 ) )
        {
                FreeRTOS_vt_process_command(verified_telemetry_DB,
                                            xHandle,
                                            (UCHAR *)pxMessage->pucComponentName,
                                            pxMessage->usComponentNameLength,
                                            (UCHAR *)pxMessage->pucCommandName,
                                            pxMessage->usCommandNameLength,
                                            NULL,NULL,0);
                //prvProcessRetrainFingerprintOneCommand();

                if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 200,
                                                        NULL,
                                                        0 ) != eAzureIoTSuccess )
                {
                    LogError( ( "Error sending command response" ) );
                }

                //prvStatusCommandPropertyUpdate();

        }

        else
        {
            /* Not for max min report (not for this device) */
            LogInfo( ( "Received command is not for this device" ) );

            if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 404,
                                                    (const uint8_t *)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                                                    strlen( sampleazureiotCOMMAND_EMPTY_PAYLOAD ) ) != eAzureIoTSuccess )
            {
                LogError( ( "Error sending command response" ) );
            }
        }

    }
    
    else if( ( soilMoistureTwoComponontNameLength == pxMessage->usComponentNameLength ) &&
        ( strncmp( sampleazureiotSOIL_MOISTURE_TWO_COMPONENT_NAME, (const char *)pxMessage->pucComponentName, soilMoistureTwoComponontNameLength ) == 0 ) )
    {

        
        if( ( strlen("setResetFingerprintTemplate") == pxMessage->usCommandNameLength ) &&
            ( strncmp( "setResetFingerprintTemplate", (const char *)pxMessage->pucCommandName, strlen("setResetFingerprintTemplate") ) == 0 ) )
        {
                FreeRTOS_vt_process_command(verified_telemetry_DB,
                                            xHandle,
                                            (UCHAR *)pxMessage->pucComponentName,
                                            pxMessage->usComponentNameLength,
                                            (UCHAR *)pxMessage->pucCommandName,
                                            pxMessage->usCommandNameLength,
                                            NULL,NULL,0);
                //prvProcessSetResetFingerprintTwoCommand();

                if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 200,
                                                        NULL,
                                                        0 ) != eAzureIoTSuccess )
                {
                    LogError( ( "Error sending command response" ) );
                }

        }
 
        else if( ( strlen("retrainFingerprintTemplate") == pxMessage->usCommandNameLength ) &&
            ( strncmp( "retrainFingerprintTemplate", (const char *)pxMessage->pucCommandName, strlen("retrainFingerprintTemplate") ) == 0 ) )
        {
                FreeRTOS_vt_process_command(verified_telemetry_DB,
                                            xHandle,
                                            (UCHAR *)pxMessage->pucComponentName,
                                            pxMessage->usComponentNameLength,
                                            (UCHAR *)pxMessage->pucCommandName,
                                            pxMessage->usCommandNameLength,
                                            NULL,NULL,0);
                //prvProcessRetrainFingerprintTwoCommand();

                if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 200,
                                                        NULL,
                                                        0 ) != eAzureIoTSuccess )
                {
                    LogError( ( "Error sending command response" ) );
                }

                //prvStatusCommandPropertyUpdate();

        }

        else
        {
            /* Not for max min report (not for this device) */
            LogInfo( ( "Received command is not for this device" ) );

            if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 404,
                                                    (const uint8_t *)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                                                    strlen( sampleazureiotCOMMAND_EMPTY_PAYLOAD ) ) != eAzureIoTSuccess )
            {
                LogError( ( "Error sending command response" ) );
            }
        }

    }

    else
    {
        /* Not for max min report (not for this device) */
        LogInfo( ( "Received component is not for this device" ) );

        if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 404,
                                                (const uint8_t *)sampleazureiotCOMMAND_EMPTY_PAYLOAD,
                                                strlen( sampleazureiotCOMMAND_EMPTY_PAYLOAD ) ) != eAzureIoTSuccess )
        {
            LogError( ( "Error sending command response" ) );
        }
    }
}
/*-----------------------------------------------------------*/

static AzureIoTResult_t prvdeviceStatusReportedPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );


                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenBool( xReader, &deviceStatusReportedProperty );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" vtDevice deviceStatus %d \n",deviceStatusReportedProperty);

    return xResult;

}


static AzureIoTResult_t prvledStateReportedPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );


                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenBool( xReader, &setLedState );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" sampleDevice ledState %d \n",setLedState);

    return xResult;

}

static AzureIoTResult_t prvSoilMoisture1TelemetryStatusPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );


                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenBool( xReader, &telemetryStatusSoilMoisture1 );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" telemetryStatus SoilMoisture1 %d \n",telemetryStatusSoilMoisture1);

    return xResult;

}

static AzureIoTResult_t prvSoilMoisture1FingerprintTypePropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;
    uint8_t pucBufferLocal[64];
    uint32_t pusBytesCopied;
                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );


                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenString( xReader, pucBufferLocal, 64,&pusBytesCopied);
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" FingerprintType SoilMoisture1 - %s \n",pucBufferLocal);

    return xResult;

}

static AzureIoTResult_t prvSoilMoisture1ConfidenceMetricPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;
    //void* component_pointer = verified_telemetry_DB->first_component;
    //component_pointer = (((FreeRTOS_VT_OBJECT*)component_pointer)->next_component);

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );


                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenInt32( xReader,&ConfidenceMetriclocalone);
                configASSERT( xResult == eAzureIoTSuccess );

                //((FreeRTOS_VT_OBJECT*)component_pointer)->component.fc.template_confidence_metric=ConfidenceMetriclocal;

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" ConfidenceMetriclocal SoilMoisture1 - %ld \n",ConfidenceMetriclocalone);

    return xResult;

}

static AzureIoTResult_t prvSoilMoisture2ConfidenceMetricMapPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;
    uint8_t pucBufferLocal[64];
    uint32_t pusBytesCopied;
    uint8_t pucBufferLocalone[64];
    uint32_t pusBytesCopiedone;
                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenString( xReader, pucBufferLocal, 64,&pusBytesCopied);
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" FingerprintType SoilMoisture2 - %s \n",pucBufferLocal);

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenString( xReader, pucBufferLocalone, 64,&pusBytesCopiedone);
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" FingerprintType SoilMoisture2 - %s \n",pucBufferLocalone);

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

    return xResult;
}

static AzureIoTResult_t prvSoilMoisture1ConfidenceMetricMapPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;
    uint8_t pucBufferLocal[64];
    uint32_t pusBytesCopied;
    uint8_t pucBufferLocalone[64];
    uint32_t pusBytesCopiedone;
                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenString( xReader, pucBufferLocal, 64,&pusBytesCopied);
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" FingerprintType SoilMoisture1 - %s \n",pucBufferLocal);

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenString( xReader, pucBufferLocalone, 64,&pusBytesCopiedone);
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" FingerprintType SoilMoisture1 - %s \n",pucBufferLocalone);

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

    return xResult;

}

static AzureIoTResult_t prvSoilMoisture2ConfidenceMetricPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;
    int32_t ConfidenceMetriclocal;
                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );


                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenInt32( xReader,&ConfidenceMetriclocaltwo);
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" ConfidenceMetriclocal SoilMoisture2 - %ld \n",ConfidenceMetriclocaltwo);

    return xResult;

}

static AzureIoTResult_t prvSoilMoisture2FingerprintTypePropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;
    uint8_t pucBufferLocal[64];
    uint32_t pusBytesCopied;
                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );


                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenString( xReader, pucBufferLocal, 64,&pusBytesCopied);
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" FingerprintType SoilMoisture2 - %s \n",pucBufferLocal);

    return xResult;

}

static AzureIoTResult_t prvSoilMoisture2TelemetryStatusPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );


                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenBool( xReader, &telemetryStatusSoilMoisture2 );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" telemetryStatus SoilMoisture2 %d \n",telemetryStatusSoilMoisture2);

    return xResult;

}

static AzureIoTResult_t prvenableVerifiedTelemetryReportedPropertyProcess( AzureIoTJSONReader_t *xReader, 
                                                                AzureIoTJSONTokenType_t *xTokenType)
{

    AzureIoTResult_t xResult;
                for(int iter=0;iter<9;iter++)
                {
                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );
                }
                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );

                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenBool( xReader, &enableVerifiedTelemetryWritableProperty );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                printf(" enableVerifiedTelemetry WritableProperty %d \n",enableVerifiedTelemetryWritableProperty);

    return xResult;

}


static AzureIoTResult_t prvEnableVerifiedTelemetryPropertyProcess(AzureIoTJSONReader_t *xReader, AzureIoTJSONTokenType_t *xTokenType, uint32_t ulVersion)
{

        AzureIoTResult_t xResult;
        bool enableVerifiedTelemetryLocal;
                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_TokenType( xReader, xTokenType );
                configASSERT( xResult == eAzureIoTSuccess );

                /* Get desired temperature */
                xResult = AzureIoTJSONReader_GetTokenBool( xReader, &enableVerifiedTelemetryLocal );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( xReader );
                configASSERT( xResult == eAzureIoTSuccess );

    printf(" enableVerifiedTelemetryLocal %d \n",enableVerifiedTelemetryLocal);

    AzureIoTJSONWriter_t xWriter;
    int32_t lBytesWritten;

    xResult = AzureIoTJSONWriter_Init( &xWriter, ucPropertyPayloadBuffer, sizeof( ucPropertyPayloadBuffer ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBeginObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderBeginComponent(&xAzureIoTHubClient,&xWriter,
                                                    (const uint8_t *)sampleazureiotvTDeviceCOMPONENT_NAME,
                                                    strlen(sampleazureiotvTDeviceCOMPONENT_NAME));

    xResult = AzureIoTHubClientProperties_BuilderBeginResponseStatus( &xAzureIoTHubClient,
                                                                      &xWriter,
                                                                     (const uint8_t *) sampleazureiotPROPERTY_ENABLE_VERIFIED_TELEMETRY,
                                                                      strlen( sampleazureiotPROPERTY_ENABLE_VERIFIED_TELEMETRY ),
                                                                      sampleazureiotPROPERTY_STATUS_SUCCESS,
                                                                      ulVersion,
                                                                      (const uint8_t *)sampleazureiotPROPERTY_SUCCESS,
                                                                      strlen( sampleazureiotPROPERTY_SUCCESS ) );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTJSONWriter_AppendBool( &xWriter, enableVerifiedTelemetryLocal);
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTHubClientProperties_BuilderEndResponseStatus( &xAzureIoTHubClient,
                                                                    &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    AzureIoTHubClientProperties_BuilderEndComponent(&xAzureIoTHubClient,&xWriter);

    xResult = AzureIoTJSONWriter_AppendEndObject( &xWriter );
    configASSERT( xResult == eAzureIoTSuccess );

    lBytesWritten = AzureIoTJSONWriter_GetBytesUsed( &xWriter );

    if( lBytesWritten < 0 )
    {
        LogError( ( "Error getting the bytes written for the properties confirmation JSON" ) );
    }
    else
    {
        LogDebug( ( "Sending acknowledged writable property. Payload: %.*s", lBytesWritten, ucPropertyPayloadBuffer ) );
        xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient, ucPropertyPayloadBuffer, lBytesWritten, NULL );

        if( xResult != eAzureIoTSuccess )
        {
            LogError( ( "There was an error sending the reported properties: 0x%08x", xResult ) );
        }
    }
    
    return xResult;


}



/**
 * @brief Properties callback handler
 */

static AzureIoTResult_t prvProcessProperties( AzureIoTHubClientPropertiesResponse_t * pxMessage,
                                              AzureIoTHubClientPropertyType_t xPropertyType )
{
    AzureIoTResult_t xResult;
    AzureIoTJSONReader_t xReader;
    AzureIoTJSONTokenType_t xTokenType;
    const uint8_t * pucComponentName ;
    uint32_t ulComponentNameLength ;
    uint32_t ulVersion;

    uint32_t sampleDevicecomponontNameLength=strlen(sampleazureiotvTDeviceCOMPONENT_NAME);

    xResult = AzureIoTJSONReader_Init( &xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTHubClientProperties_GetPropertiesVersion( &xAzureIoTHubClient, &xReader, pxMessage->xMessageType, &ulVersion );

    if( xResult != eAzureIoTSuccess )
    {
        LogError( ( "Error getting the property version" ) );
    }
    else
    {   //printf(" iteration start \n");
        /* Reset JSON reader to the beginning */
        xResult = AzureIoTJSONReader_Init( &xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength );
        configASSERT( xResult == eAzureIoTSuccess );
        //here this fucntion would check for that articular commPonet in the Payload recieved. and whould then go on to rocess
        while( ( xResult = AzureIoTHubClientProperties_GetNextComponentProperty( &xAzureIoTHubClient, &xReader,
                                                                                 pxMessage->xMessageType, xPropertyType,
                                                                                 &pucComponentName, &ulComponentNameLength ) ) == eAzureIoTSuccess )
        {      //printf(" iteration \n");
            xResult = AzureIoTJSONReader_TokenType( &xReader, &xTokenType );
            configASSERT( xResult == eAzureIoTSuccess );
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

            if( ( sampleDevicecomponontNameLength == ulComponentNameLength ) &&
                ( strncmp( sampleazureiotvTDeviceCOMPONENT_NAME, (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {
                printf("vtdevice");
                FreeRTOS_vt_process_property_update(verified_telemetry_DB,&xAzureIoTHubClient,pucComponentName,ulComponentNameLength,&xReader,ulVersion);


                /*
                if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)sampleazureiotPROPERTY_TARGET_TEMPERATURE_TEXT,
                                                        strlen( sampleazureiotPROPERTY_TARGET_TEMPERATURE_TEXT ) ) )
                {
                    //printf(" okay \n");
                    prvTargetTemperatureProcess(&xReader, &xTokenType,ulVersion);
                }
            

                if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)sampleazureiotPROPERTY_ENABLE_VERIFIED_TELEMETRY,
                                                        strlen( sampleazureiotPROPERTY_ENABLE_VERIFIED_TELEMETRY ) ) )
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
                xResult = AzureIoTJSONReader_NextToken( &xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_SkipChildren( &xReader );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTJSONReader_NextToken( &xReader );
                configASSERT( xResult == eAzureIoTSuccess );
            }

        }

        if( xResult != eAzureIoTErrorEndOfProperties )
        {
            LogError( ( "There was an error parsing the properties: 0x%08x", xResult ) );
        }
        else
        {
            LogInfo( ( "Successfully parsed properties" ) );
            xResult = eAzureIoTSuccess;
        }
    }

    return xResult;
}
/*-----------------------------------------------------------*/

/*
AzureIoTResult_t sendTelemetrySampleDevice()
{
    AzureIoTResult_t xResult;
    uint32_t ulScratchBufferLength = 0U;
    AzureIoTMessageProperties_t telemetrymessageProperties;
    uint8_t pucBuffer[128];

    AzureIoT_MessagePropertiesInit(&telemetrymessageProperties, pucBuffer, 0, 128);
    AzureIoT_MessagePropertiesAppend(&telemetrymessageProperties, (const uint8_t *)"$.sub", 
                                    sizeof("$.sub") - 1, (const uint8_t *)"sampleDevice", 
                                    sizeof("sampleDevice") - 1);
    AzureIoT_MessagePropertiesAppend(
                                    &telemetrymessageProperties, 
                                    (const uint8_t *)"verifiedTelemetry", 
                                    sizeof("verifiedTelemetry") - 1, 
                                    (const uint8_t *)"freertosDemo", sizeof("freertosDemo") - 1);

    deviceTelemetryValue[0] = soilMoistureExternal1;
    deviceTelemetryValue[1] = soilMoistureExternal2;
    deviceTelemetryValue[2] = temperature;
    deviceTelemetryValue[3] = pressure;
    deviceTelemetryValue[4] = humidityPercentage;
    deviceTelemetryValue[5] = acceleration;
    deviceTelemetryValue[6] = magnetic;
 
    //just for simulation
soilMoistureExternal1++;soilMoistureExternal2++;temperature++;pressure++;humidityPercentage++;acceleration++;magnetic++;
    //just for simulation 
    for(uint32_t telemetryIter=0;telemetryIter<7;telemetryIter++)
    {
    memset((void *)ucScratchBuffer,0,sizeof(ucScratchBuffer));
    ulScratchBufferLength = prvTelemetryPayloadCreate((char *)ucScratchBuffer, sizeof(ucScratchBuffer),
                                                            deviceTelemetryName[telemetryIter], deviceTelemetryValue[telemetryIter]);
    //add a fucntion to toggle reported property deviceStatus
    printf("  %s \n",ucScratchBuffer);

    xResult = AzureIoTHubClient_SendTelemetry( &xAzureIoTHubClient,
                                               ucScratchBuffer, ulScratchBufferLength,
                                               &telemetrymessageProperties, eAzureIoTHubMessageQoS1, NULL );
    configASSERT( xResult == eAzureIoTSuccess );

    }

    return xResult;
    
}
*/



static AzureIoTResult_t prvProcessReportedProperties( AzureIoTHubClientPropertiesResponse_t * pxMessage,
                                            AzureIoTHubClientPropertyType_t xPropertyType )
{
    AzureIoTResult_t xResult;
    AzureIoTJSONReader_t xReader;
    AzureIoTJSONTokenType_t xTokenType;
    const uint8_t * pucComponentName;
    uint32_t ulComponentNameLength ;
    uint32_t ulVersion;

    uint32_t vTDevicecomponontNameLength=strlen(sampleazureiotvTDeviceCOMPONENT_NAME);
    uint32_t sampleDevicecomponontNameLength=strlen(sampleazureiotsampleDeviceCOMPONENT_NAME);


    xResult = AzureIoTJSONReader_Init( &xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength );
    configASSERT( xResult == eAzureIoTSuccess );

    xResult = AzureIoTHubClientProperties_GetPropertiesVersion( &xAzureIoTHubClient, &xReader, pxMessage->xMessageType, &ulVersion );

    if( xResult != eAzureIoTSuccess )
    {
        LogError( ( "Error getting the property version" ) );
    }
    else
    {
        /* Reset JSON reader to the beginning */
        xResult = AzureIoTJSONReader_Init( &xReader, pxMessage->pvMessagePayload, pxMessage->ulPayloadLength );
        configASSERT( xResult == eAzureIoTSuccess );


        //here this fucntion would check for that articular commPonet in the Payload recieved. and whould then go on to rocess
        //here it was weird accidentaly i store some value in the reorted roerty dock and it stayed true
        //but even with the comonentc name initalized it still icked u the Status reorted roerty where as it should only check inside comonent
        while( ( xResult = AzureIoTHubClientProperties_GetNextComponentProperty( &xAzureIoTHubClient, &xReader,
                                                                                 pxMessage->xMessageType, xPropertyType,
                                                                                 &pucComponentName, &ulComponentNameLength ) ) == eAzureIoTSuccess )
        {
            //printf(" in while \n");
            xResult = AzureIoTJSONReader_TokenType( &xReader, &xTokenType );
            configASSERT( xResult == eAzureIoTSuccess );

        LogInfo( ( "component name : %.*s \r\n",
               ulComponentNameLength,
               pucComponentName ) );

            //printf(" %s \n",pucComponentName);

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

            if( ( vTDevicecomponontNameLength == ulComponentNameLength ) &&
                ( strncmp( sampleazureiotvTDeviceCOMPONENT_NAME, (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {
                //printf(" found comp \n");
                if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"deviceStatus",
                                                        strlen( "deviceStatus" ) ) )
                {
                    //printf(" found \n");
                    prvdeviceStatusReportedPropertyProcess(&xReader, &xTokenType);
                }
                else if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"enableVerifiedTelemetry",
                                                        strlen( "enableVerifiedTelemetry" ) ) )
                {
                    //printf(" found \n");
                    prvenableVerifiedTelemetryReportedPropertyProcess(&xReader, &xTokenType);
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


            }
            else if( ( sampleDevicecomponontNameLength == ulComponentNameLength ) &&
                ( strncmp( sampleazureiotsampleDeviceCOMPONENT_NAME, (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {

                if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"ledState",
                                                        strlen( "ledState" ) ) )
                {
                    //printf(" found \n");
                    prvledStateReportedPropertyProcess(&xReader, &xTokenType);
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
            
            }
            else if( ( strlen("vTsoilMoistureExternal1") == ulComponentNameLength ) &&
                ( strncmp( "vTsoilMoistureExternal1", (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {
                                //printf("in");
                if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"telemetryStatus",
                                                        strlen("telemetryStatus") ) )
                {
                    //printf(" found in \n");
                    prvSoilMoisture1TelemetryStatusPropertyProcess(&xReader, &xTokenType);
                }
                else if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"fingerprintType",
                                                        strlen( "fingerprintType" ) ) )
                {
                    //printf(" found out \n");
                    prvSoilMoisture1FingerprintTypePropertyProcess(&xReader, &xTokenType);
                }
                //using this else part causes problems and lags the code using configASSERT( xResult == eAzureIoTSuccess );
                else if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"fingerprintTemplateConfidenceMetric",
                                                        strlen( "fingerprintTemplateConfidenceMetric" ) ) )
                {
                    //printf(" found out \n");
                    prvSoilMoisture1ConfidenceMetricPropertyProcess(&xReader, &xTokenType);
                } 
                else if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"fingerprintTemplate",
                                                        strlen( "fingerprintTemplate" ) ) )
                {
                    //printf(" found out \n");
                    //prvSoilMoisture1ConfidenceMetricMapPropertyProcess(&xReader, &xTokenType);
                    
                    FreeRTOS_vt_process_reported_property_sync(verified_telemetry_DB,
                                                                &xAzureIoTHubClient,
                                                                pucComponentName,
                                                                ulComponentNameLength,
                                                                &xReader,
                                                                ulVersion);
                                                                
                }   
                /*             
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
            else if( ( strlen("vTsoilMoistureExternal2") == ulComponentNameLength ) &&
                ( strncmp( "vTsoilMoistureExternal2", (const char *)pucComponentName, ulComponentNameLength ) == 0 ) )
            {

                if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"telemetryStatus",
                                                        strlen( "telemetryStatus" ) ) )
                {
                    //printf(" found1 \n");
                    prvSoilMoisture2TelemetryStatusPropertyProcess(&xReader, &xTokenType);
                }
                else if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"fingerprintType",
                                                        strlen( "fingerprintType" ) ) )
                {
                    //printf(" found2 \n");
                    prvSoilMoisture2FingerprintTypePropertyProcess(&xReader, &xTokenType);
                }
                else if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"fingerprintTemplateConfidenceMetric",
                                                        strlen( "fingerprintTemplateConfidenceMetric" ) ) )
                {
                    //printf(" found out3 \n");
                    prvSoilMoisture2ConfidenceMetricPropertyProcess(&xReader, &xTokenType);
                }
                else if( AzureIoTJSONReader_TokenIsTextEqual( &xReader,
                                                        (const uint8_t *)"fingerprintTemplate",
                                                        strlen( "fingerprintTemplate" ) ) )
                {
                    //printf(" found out4 \n");
                    //prvSoilMoisture2ConfidenceMetricMapPropertyProcess(&xReader, &xTokenType);

                    FreeRTOS_vt_process_reported_property_sync(verified_telemetry_DB,
                                                                &xAzureIoTHubClient,
                                                                pucComponentName,
                                                                ulComponentNameLength,
                                                                &xReader,
                                                                ulVersion);
                }  
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

        }



        if( xResult != eAzureIoTErrorEndOfProperties )
        {
            LogError( ( "There was an error parsing the properties: 0x%08x", xResult ) );
        }
        else
        {
            LogInfo( ( "Successfully parsed properties" ) );
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
//here the pxMessage that we recive only has the ayload inside which there would be message like this 
static void prvHandleProperties( AzureIoTHubClientPropertiesResponse_t * pxMessage,
                                 void * pvContext )
{
    ( void ) pvContext;

    AzureIoTResult_t xResult;


    LogInfo( ( "Property document payload : %.*s \r\n",
               pxMessage->ulPayloadLength,
               pxMessage->pvMessagePayload ) );

    switch( pxMessage->xMessageType )
    {
        case eAzureIoTHubPropertiesGetMessage:
            LogInfo( ( "Device property document GET received" ) );
            //printf("first");
            xResult = prvProcessReportedProperties( pxMessage, eAzureIoTHubClientReportedFromDevice );
            if( xResult != eAzureIoTSuccess )
                {LogError( ( "There was an error processing incoming properties" ) );}

            break;

        case eAzureIoTHubPropertiesWritablePropertyMessage:
            LogInfo( ( "Device writeable property received" ) );
            xResult = prvProcessProperties( pxMessage, eAzureIoTHubClientPropertyWritable);

            if( xResult != eAzureIoTSuccess )
                {LogError( ( "There was an error processing incoming properties" ) );}

            break;

        case eAzureIoTHubPropertiesReportedResponseMessage:
            LogInfo( ( "Device reported property response received" ) );
            break;

        default:
            LogError( ( "Unknown property message" ) );
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief Setup transport credentials.
 */
static uint32_t prvSetupNetworkCredentials( NetworkCredentials_t * pxNetworkCredentials )
{
    pxNetworkCredentials->xDisableSni = pdFALSE;
    /* Set the credentials for establishing a TLS connection. */
    pxNetworkCredentials->pucRootCa = ( const unsigned char * ) democonfigROOT_CA_PEM;
    pxNetworkCredentials->xRootCaSize = sizeof( democonfigROOT_CA_PEM );
    #ifdef democonfigCLIENT_CERTIFICATE_PEM
        pxNetworkCredentials->pucClientCert = ( const unsigned char * ) democonfigCLIENT_CERTIFICATE_PEM;
        pxNetworkCredentials->xClientCertSize = sizeof( democonfigCLIENT_CERTIFICATE_PEM );
        pxNetworkCredentials->pucPrivateKey = ( const unsigned char * ) democonfigCLIENT_PRIVATE_KEY_PEM;
        pxNetworkCredentials->xPrivateKeySize = sizeof( democonfigCLIENT_PRIVATE_KEY_PEM );
    #endif

    return 0;
}
/*-----------------------------------------------------------*/

/**
 * @brief Azure IoT demo task that gets started in the platform specific project.
 *  In this demo task, middleware API's are used to connect to Azure IoT Hub.
 */
static void prvAzureDemoTask( void * pvParameters )
{
    uint32_t ulPublishCount = 0U;
    const uint32_t ulMaxPublishCount = 200UL;
    NetworkCredentials_t xNetworkCredentials = { 0 };
    AzureIoTTransportInterface_t xTransport;
    NetworkContext_t xNetworkContext = { 0 };
    TlsTransportParams_t xTlsTransportParams = { 0 };
    AzureIoTResult_t xResult;
    uint32_t ulStatus;
    AzureIoTHubClientOptions_t xHubOptions = { 0 };
    bool xSessionPresent;
    /*
    FreeRTOS_VT_FALLCURVE_COMPONENT ComponentTag;
    FreeRTOS_VT_OBJECT FreeRTOS_Object;
    FreeRTOS_VERIFIED_TELEMETRY_DB verified_telemetry_DB;
    
    ComponentTag.associated_telemetry="soilMoistureExternal1";
    FreeRTOS_Object.component.fc.associated_telemetry="soilMoistureExternal1";

    FreeRTOS_Object.signature_type=VT_SIGNATURE_TYPE_FALLCURVE;

    verified_telemetry_DB.components_num=1;
    verified_telemetry_DB.first_component=((void *) &FreeRTOS_Object);
    */
 

    verified_telemetry_DB=sample_nx_verified_telemetry_user_init();

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

    #ifdef democonfigENABLE_DPS_SAMPLE
        uint8_t * pucIotHubHostname = NULL;
        uint8_t * pucIotHubDeviceId = NULL;
        uint32_t pulIothubHostnameLength = 0;
        uint32_t pulIothubDeviceIdLength = 0;
    #else
        uint8_t * pucIotHubHostname = ( uint8_t * ) democonfigHOSTNAME;
        uint8_t * pucIotHubDeviceId = ( uint8_t * ) democonfigDEVICE_ID;
        uint32_t pulIothubHostnameLength = sizeof( democonfigHOSTNAME ) - 1;
        uint32_t pulIothubDeviceIdLength = sizeof( democonfigDEVICE_ID ) - 1;
    #endif /* democonfigENABLE_DPS_SAMPLE */

    ( void ) pvParameters;

    /* Initialize Azure IoT Middleware.  */
    configASSERT( AzureIoT_Init() == eAzureIoTSuccess );

    ulStatus = prvSetupNetworkCredentials( &xNetworkCredentials );
    configASSERT( ulStatus == 0 );

    #ifdef democonfigENABLE_DPS_SAMPLE
        /* Run DPS.  */
        if( ( ulStatus = prvIoTHubInfoGet( &xNetworkCredentials, &pucIotHubHostname,
                                           &pulIothubHostnameLength, &pucIotHubDeviceId,
                                           &pulIothubDeviceIdLength ) ) != 0 )
        {
            LogError( ( "Failed on sample_dps_entry!: error code = 0x%08x\r\n", ulStatus ) );
            return;
        }
    #endif /* democonfigENABLE_DPS_SAMPLE */

    xNetworkContext.pParams = &xTlsTransportParams;

    xNetworkContext.pParams = &xTlsTransportParams;
    AzureIoTMessageProperties_t telemetrymessageProperties;
    uint8_t pucBuffer[128];

    AzureIoT_MessagePropertiesInit(&telemetrymessageProperties, pucBuffer, 0, 128);
    AzureIoT_MessagePropertiesAppend(&telemetrymessageProperties, (const uint8_t *)"$.sub", sizeof("$.sub") - 1, (const uint8_t *)"sampleDevice", sizeof("sampleDevice") - 1);
    AzureIoT_MessagePropertiesAppend(
        &telemetrymessageProperties, (const uint8_t *)"verifiedTelemetry", sizeof("verifiedTelemetry") - 1, (const uint8_t *)"Demo", sizeof("Demo") - 1);

    for( ; ; )
    {
        /* Attempt to establish TLS session with IoT Hub. If connection fails,
         * retry after a timeout. Timeout value will be exponentially increased
         * until  the maximum number of attempts are reached or the maximum timeout
         * value is reached. The function returns a failure status if the TCP
         * connection cannot be established to the IoT Hub after the configured
         * number of attempts. */
        ulStatus = prvConnectToServerWithBackoffRetries( ( const char * ) pucIotHubHostname,
                                                         democonfigIOTHUB_PORT,
                                                         &xNetworkCredentials, &xNetworkContext );
        configASSERT( ulStatus == 0 );

        /* Fill in Transport Interface send and receive function pointers. */
        xTransport.pxNetworkContext = &xNetworkContext;
        xTransport.xSend = TLS_Socket_Send;
        xTransport.xRecv = TLS_Socket_Recv;

        /* Init IoT Hub option */
        xResult = AzureIoTHubClient_OptionsInit( &xHubOptions );
        configASSERT( xResult == eAzureIoTSuccess );
 
        xHubOptions.pucModuleID = ( const uint8_t * ) democonfigMODULE_ID;
        xHubOptions.ulModuleIDLength = sizeof( democonfigMODULE_ID ) - 1;
        xHubOptions.pucModelID = (const uint8_t * )sampleazureiotMODEL_ID;
        xHubOptions.ulModelIDLength = strlen( sampleazureiotMODEL_ID );

        ////xHubOptions.pxComponentList=&(azureiothubCREATE_COMPONENT("componentSample"));
        //xHubOptions.ulComponentListLength=1;
//works on 1 and on strlen( "componentSample" ) as well 
        xHubOptions.pxComponentList = component_list; 
        xHubOptions.ulComponentListLength = NUM_COMPONENTS;


        xResult = AzureIoTHubClient_Init( &xAzureIoTHubClient,
                                          pucIotHubHostname, pulIothubHostnameLength,
                                          pucIotHubDeviceId, pulIothubDeviceIdLength,
                                          &xHubOptions,
                                          ucMQTTMessageBuffer, sizeof( ucMQTTMessageBuffer ),
                                          ullGetUnixTime,
                                          &xTransport );
        configASSERT( xResult == eAzureIoTSuccess );

        #ifdef democonfigDEVICE_SYMMETRIC_KEY
            xResult = AzureIoTHubClient_SetSymmetricKey( &xAzureIoTHubClient,
                                                         ( const uint8_t * ) democonfigDEVICE_SYMMETRIC_KEY,
                                                         sizeof( democonfigDEVICE_SYMMETRIC_KEY ) - 1,
                                                         Crypto_HMAC );
            configASSERT( xResult == eAzureIoTSuccess );
        #endif /* democonfigDEVICE_SYMMETRIC_KEY */

        /* Sends an MQTT Connect packet over the already established TLS connection,
         * and waits for connection acknowledgment (CONNACK) packet. */
        LogInfo( ( "Creating an MQTT connection to %s.\r\n", pucIotHubHostname ) );

        xResult = AzureIoTHubClient_Connect( &xAzureIoTHubClient,
                                             false, &xSessionPresent,
                                             sampleazureiotCONNACK_RECV_TIMEOUT_MS );
        configASSERT( xResult == eAzureIoTSuccess );

        xResult = AzureIoTHubClient_SubscribeCommand( &xAzureIoTHubClient, prvHandleCommand,
                                                      &xAzureIoTHubClient, sampleazureiotSUBSCRIBE_TIMEOUT );
        configASSERT( xResult == eAzureIoTSuccess );

        xResult = AzureIoTHubClient_SubscribeProperties( &xAzureIoTHubClient, prvHandleProperties,
                                                         &xAzureIoTHubClient, sampleazureiotSUBSCRIBE_TIMEOUT );
        configASSERT( xResult == eAzureIoTSuccess );

        /* Get property document after initial connection */
        xResult = AzureIoTHubClient_GetProperties( &xAzureIoTHubClient );
        configASSERT( xResult == eAzureIoTSuccess );

        xResult = AzureIoTHubClient_ProcessLoop( &xAzureIoTHubClient,
                                                 sampleazureiotPROCESS_LOOP_TIMEOUT_MS );
        configASSERT( xResult == eAzureIoTSuccess );

        xResult = prvReportedPropertiesSend();
        configASSERT( xResult == eAzureIoTSuccess );

        LogInfo( ( "Attempt to receive publish message from IoT Hub.\r\n" ) );
        xResult = AzureIoTHubClient_ProcessLoop( &xAzureIoTHubClient,
                                                     sampleazureiotPROCESS_LOOP_TIMEOUT_MS );
        configASSERT( xResult == eAzureIoTSuccess );

        /* Publish messages with QoS1, send and process Keep alive messages. */
        for( ulPublishCount = 0; ulPublishCount < ulMaxPublishCount; ulPublishCount++ )
        {

            //xResult = sendTelemetrySampleDevice(&xAzureIoTHubClient);
            //configASSERT( xResult == eAzureIoTSuccess );

            //send reported property
            sample_pnp_device_telemetry_send(&sample_device,&xAzureIoTHubClient);
            /*
            FreeRTOS_vt_verified_telemetry_message_create_send(verified_telemetry_DB,
                                                                &xAzureIoTHubClient,
                                                                (const UCHAR *)"sampleDevice",
                                                                strlen("sampleDevice"),
                                                                0,
                                                                (const UCHAR *)sampleazuretelemetryMESSAGE,
                                                                strlen(sampleazuretelemetryMESSAGE));
            */
            //xResult = prvReportedPropertiesSend();
            //configASSERT( xResult == eAzureIoTSuccess );

            FreeRTOS_vt_compute_evaluate_fingerprint_all_sensors(verified_telemetry_DB);

            FreeRTOS_vt_properties(verified_telemetry_DB,&xAzureIoTHubClient);

            LogInfo( ( "Attempt to receive publish message from IoT Hub.\r\n" ) );
            xResult = AzureIoTHubClient_ProcessLoop( &xAzureIoTHubClient,
                                                     sampleazureiotPROCESS_LOOP_TIMEOUT_MS );
            configASSERT( xResult == eAzureIoTSuccess );

            /* Leave Connection Idle for some time. */
            LogInfo( ( "Keeping Connection Idle...\r\n\r\n" ) );
            vTaskDelay( sampleazureiotDELAY_BETWEEN_PUBLISHES_TICKS );
        }

        xResult = AzureIoTHubClient_UnsubscribeProperties( &xAzureIoTHubClient );
        configASSERT( xResult == eAzureIoTSuccess );

        xResult = AzureIoTHubClient_UnsubscribeCommand( &xAzureIoTHubClient );
        configASSERT( xResult == eAzureIoTSuccess );

        /* Send an MQTT Disconnect packet over the already connected TLS over
         * TCP connection. There is no corresponding response for the disconnect
         * packet. After sending disconnect, client must close the network
         * connection. */
        xResult = AzureIoTHubClient_Disconnect( &xAzureIoTHubClient );
        configASSERT( xResult == eAzureIoTSuccess );

        /* Close the network connection.  */
        TLS_Socket_Disconnect( &xNetworkContext );

        /* Wait for some time between two iterations to ensure that we do not
         * bombard the IoT Hub. */
        LogInfo( ( "Demo completed successfully.\r\n" ) );
        LogInfo( ( "Short delay before starting the next iteration.... \r\n\r\n" ) );
        vTaskDelay( sampleazureiotDELAY_BETWEEN_DEMO_ITERATIONS_TICKS );
    }
}
/*-----------------------------------------------------------*/

#ifdef democonfigENABLE_DPS_SAMPLE

/**
 * @brief Get IoT Hub endpoint and device Id info, when Provisioning service is used.
 *   This function will block for Provisioning service for result or return failure.
 */
    static uint32_t prvIoTHubInfoGet( NetworkCredentials_t * pXNetworkCredentials,
                                      uint8_t ** ppucIothubHostname,
                                      uint32_t * pulIothubHostnameLength,
                                      uint8_t ** ppucIothubDeviceId,
                                      uint32_t * pulIothubDeviceIdLength )
    {
        NetworkContext_t xNetworkContext = { 0 };
        TlsTransportParams_t xTlsTransportParams = { 0 };
        AzureIoTResult_t xResult;
        AzureIoTTransportInterface_t xTransport;
        uint32_t ucSamplepIothubHostnameLength = sizeof( ucSampleIotHubHostname );
        uint32_t ucSamplepIothubDeviceIdLength = sizeof( ucSampleIotHubDeviceId );
        uint32_t ulStatus;

        /* Set the pParams member of the network context with desired transport. */
        xNetworkContext.pParams = &xTlsTransportParams;

        ulStatus = prvConnectToServerWithBackoffRetries( democonfigENDPOINT, democonfigIOTHUB_PORT,
                                                         pXNetworkCredentials, &xNetworkContext );
        configASSERT( ulStatus == 0 );

        /* Fill in Transport Interface send and receive function pointers. */
        xTransport.pxNetworkContext = &xNetworkContext;
        xTransport.xSend = TLS_Socket_Send;
        xTransport.xRecv = TLS_Socket_Recv;

        xResult = AzureIoTProvisioningClient_Init( &xAzureIoTProvisioningClient,
                                                   ( const uint8_t * ) democonfigENDPOINT,
                                                   sizeof( democonfigENDPOINT ) - 1,
                                                   ( const uint8_t * ) democonfigID_SCOPE,
                                                   sizeof( democonfigID_SCOPE ) - 1,
                                                   ( const uint8_t * ) democonfigREGISTRATION_ID,
                                                   sizeof( democonfigREGISTRATION_ID ) - 1,
                                                   NULL, ucMQTTMessageBuffer, sizeof( ucMQTTMessageBuffer ),
                                                   ullGetUnixTime,
                                                   &xTransport );
        configASSERT( xResult == eAzureIoTSuccess );

        #ifdef democonfigDEVICE_SYMMETRIC_KEY
            xResult = AzureIoTProvisioningClient_SetSymmetricKey( &xAzureIoTProvisioningClient,
                                                                  ( const uint8_t * ) democonfigDEVICE_SYMMETRIC_KEY,
                                                                  sizeof( democonfigDEVICE_SYMMETRIC_KEY ) - 1,
                                                                  Crypto_HMAC );
            configASSERT( xResult == eAzureIoTSuccess );
        #endif /* democonfigDEVICE_SYMMETRIC_KEY */

        xResult = AzureIoTProvisioningClient_SetRegistrationPayload( &xAzureIoTProvisioningClient,
                                                                     sampleazureiotPROVISIONING_PAYLOAD,
                                                                     strlen( sampleazureiotPROVISIONING_PAYLOAD ) );
        configASSERT( xResult == eAzureIoTSuccess );

        do
        {
            xResult = AzureIoTProvisioningClient_Register( &xAzureIoTProvisioningClient,
                                                           sampleazureiotProvisioning_Registration_TIMEOUT_MS );
        } while( xResult == eAzureIoTErrorPending );

        if( xResult == eAzureIoTSuccess )
        {
            LogInfo( ( "Successfully acquired IoT Hub name and Device ID" ) );
        }
        else
        {
            LogInfo( ( "Error geting IoT Hub name and Device ID: 0x%08", xResult ) );
        }

        configASSERT( xResult == eAzureIoTSuccess );

        xResult = AzureIoTProvisioningClient_GetDeviceAndHub( &xAzureIoTProvisioningClient,
                                                              ucSampleIotHubHostname, &ucSamplepIothubHostnameLength,
                                                              ucSampleIotHubDeviceId, &ucSamplepIothubDeviceIdLength );
        configASSERT( xResult == eAzureIoTSuccess );

        AzureIoTProvisioningClient_Deinit( &xAzureIoTProvisioningClient );

        /* Close the network connection.  */
        TLS_Socket_Disconnect( &xNetworkContext );

        *ppucIothubHostname = ucSampleIotHubHostname;
        *pulIothubHostnameLength = ucSamplepIothubHostnameLength;
        *ppucIothubDeviceId = ucSampleIotHubDeviceId;
        *pulIothubDeviceIdLength = ucSamplepIothubDeviceIdLength;

        return 0;
    }

#endif /* democonfigENABLE_DPS_SAMPLE */
/*-----------------------------------------------------------*/

/**
 * @brief Connect to server with backoff retries.
 */
static uint32_t prvConnectToServerWithBackoffRetries( const char * pcHostName,
                                                      uint32_t port,
                                                      NetworkCredentials_t * pxNetworkCredentials,
                                                      NetworkContext_t * pxNetworkContext )
{
    TlsTransportStatus_t xNetworkStatus;
    BackoffAlgorithmStatus_t xBackoffAlgStatus = BackoffAlgorithmSuccess;
    BackoffAlgorithmContext_t xReconnectParams;
    uint16_t usNextRetryBackOff = 0U;

    /* Initialize reconnect attempts and interval. */
    BackoffAlgorithm_InitializeParams( &xReconnectParams,
                                       sampleazureiotRETRY_BACKOFF_BASE_MS,
                                       sampleazureiotRETRY_MAX_BACKOFF_DELAY_MS,
                                       sampleazureiotRETRY_MAX_ATTEMPTS );

    /* Attempt to connect to IoT Hub. If connection fails, retry after
     * a timeout. Timeout value will exponentially increase till maximum
     * attempts are reached.
     */
    do
    {
        LogInfo( ( "Creating a TLS connection to %s:%u.\r\n", pcHostName, port ) );
        /* Attempt to create a mutually authenticated TLS connection. */
        xNetworkStatus = TLS_Socket_Connect( pxNetworkContext,
                                             pcHostName, port,
                                             pxNetworkCredentials,
                                             sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS,
                                             sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS );

        if( xNetworkStatus != eTLSTransportSuccess )
        {
            /* Generate a random number and calculate backoff value (in milliseconds) for
             * the next connection retry.
             * Note: It is recommended to seed the random number generator with a device-specific
             * entropy source so that possibility of multiple devices retrying failed network operations
             * at similar intervals can be avoided. */
            xBackoffAlgStatus = BackoffAlgorithm_GetNextBackoff( &xReconnectParams, configRAND32(), &usNextRetryBackOff );

            if( xBackoffAlgStatus == BackoffAlgorithmRetriesExhausted )
            {
                LogError( ( "Connection to the IoT Hub failed, all attempts exhausted." ) );
            }
            else if( xBackoffAlgStatus == BackoffAlgorithmSuccess )
            {
                LogWarn( ( "Connection to the IoT Hub failed [%d]. "
                           "Retrying connection with backoff and jitter [%d]ms.",
                           xNetworkStatus, usNextRetryBackOff ) );
                vTaskDelay( pdMS_TO_TICKS( usNextRetryBackOff ) );
            }
        }
    } while( ( xNetworkStatus != eTLSTransportSuccess ) && ( xBackoffAlgStatus == BackoffAlgorithmSuccess ) );

    return xNetworkStatus == eTLSTransportSuccess ? 0 : 1;
}
/*-----------------------------------------------------------*/

/*
 * @brief Create the task that demonstrates the AzureIoTHub demo
 */
void vStartDemoTask( void )
{
    /* This example uses a single application task, which in turn is used to
     * connect, subscribe, publish, unsubscribe and disconnect from the IoT Hub */
    xTaskCreate( prvAzureDemoTask,         /* Function that implements the task. */
                 "AzureDemoTask",          /* Text name for the task - only used for debugging. */
                 democonfigDEMO_STACKSIZE, /* Size of stack (in words, not bytes) to allocate for the task. */
                 NULL,                     /* Task parameter - not used in this case. */
                 tskIDLE_PRIORITY,         /* Task priority, must be between 0 and configMAX_PRIORITIES - 1. */
                 NULL );                   /* Used to pass out a handle to the created task - not used in this case. */
}
/*-----------------------------------------------------------*/
 