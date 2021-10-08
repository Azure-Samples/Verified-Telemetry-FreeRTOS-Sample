# Copyright (c) Microsoft Corporation. All rights reserved.
# SPDX-License-Identifier: MIT

set(COMPONENT_INCLUDE_DIRS
    .
    ../config
    ../verified-telemetry
)

list(APPEND COMPONENT_SOURCES

    "azure_iot_freertos_esp32_main.c"
    ../verified-telemetry/sample_pnp_device_component.c
    ../verified-telemetry/sample_vt_device_driver.c

)

idf_component_register(SRCS ${COMPONENT_SOURCES}
                    INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
                    REQUIRES esp_adc_cal freertos nvs_flash coreMQTT azure-sdk-for-c azure-iot-middleware-freertos sample-azure-iot)

