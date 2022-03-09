/*
 * FreeRTOS V1.4.7
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>


#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_tsensor.h"

#include "stm32l475e_iot01.h"
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Demo includes */
#include "demo_config.h"

/* WiFi driver includes. */
#include "es_wifi.h"
#include "wifi.h"

/* Define the default wifi ssid and password.
   User must override this in demo_config.h 
*/
#ifndef WIFI_SSID
#error "Symbol WIFI_SSID must be defined."
#endif /* WIFI_SSID  */

#ifndef WIFI_PASSWORD
#error "Symbol WIFI_PASSWORD must be defined."
#endif /* WIFI_PASSWORD  */

#ifndef WIFI_SECURITY_TYPE
#error "Symbol WIFI_SECURITY_TYPE must be defined."
#endif /* WIFI_SECURITY_TYPE  */

uint8_t  MAC_Addr[6];
uint8_t  IP_Addr[4];
uint8_t  Gateway_Addr[4];
uint8_t  DNS1_Addr[4];
uint8_t  DNS2_Addr[4];

/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void );

/* Defined in es_wifi_io.c. */
extern void SPI_WIFI_ISR( void );


/**********************
* Global Variables
**********************/

UART_HandleTypeDef UartHandle;
UART_HandleTypeDef UartHandle4; 

ADC_HandleTypeDef hadc1;
RTC_HandleTypeDef RtcHandle;
TIM_HandleTypeDef TimCCHandle;
extern SPI_HandleTypeDef hspi;
SPI_HandleTypeDef mcp3204;
TIM_HandleTypeDef HTIMx;
volatile uint32_t ButtonPressed = 0;
volatile uint32_t SendData      = 0;
uint32_t gu32_ticks;
static uint32_t t_TIM_CC1_Pulse;
RNG_HandleTypeDef xHrng;


#define TIMER  TIM4

#define DEFAULT_TIM_CC1_PULSE 4000

// Defines related to Clock configuration
#define RTC_ASYNCH_PREDIV 0x7F // LSE as RTC clock
#define RTC_SYNCH_PREDIV  0x00FF // LSE as RTC clock

#define REG32(x) (*(volatile unsigned int*)(x))

StaticSemaphore_t xSemaphoreBuffer;
xSemaphoreHandle xWifiSemaphoreHandle;

/* Private variables ---------------------------------------------------------*/
/* Use by the pseudo random number generator. */
static UBaseType_t ulNextRand;
static uint64_t ulGlobalEntryTime = 1639093301;

/* Private function prototypes -----------------------------------------------*/

/* Define RNG registers.  */
#define STM32_RNG    0x50060800
#define STM32_RNG_CR REG32(STM32_RNG + 0x00)
#define STM32_RNG_SR REG32(STM32_RNG + 0x04)
#define STM32_RNG_DR REG32(STM32_RNG + 0x08)

#define STM32_RNG_CR_RNGEN 0x00000004
#define STM32_RNG_CR_IE    0x00000008
#define STM32_RNG_CR_CED   0x00000020

#define STM32_RNG_SR_DRDY 0x00000001
#define STM32_RNG_SR_CECS 0x00000002
#define STM32_RNG_SR_SECS 0x00000004
#define STM32_RNG_SR_CEIS 0x00000020
#define STM32_RNG_SR_SEIS 0x00000040

/* UART settings */
#define CFG_HW_UART1_BAUDRATE       115200
#define CFG_HW_UART1_WORDLENGTH     UART_WORDLENGTH_8B
#define CFG_HW_UART1_STOPBITS       UART_STOPBITS_1
#define CFG_HW_UART1_PARITY         UART_PARITY_NONE
#define CFG_HW_UART1_HWFLOWCTL      UART_HWCONTROL_NONE
#define CFG_HW_UART1_MODE           UART_MODE_TX_RX
#define CFG_HW_UART1_ADVFEATUREINIT UART_ADVFEATURE_NO_INIT

#define CFG_HW_UART4_BAUDRATE       9600
#define CFG_HW_UART4_WORDLENGTH     UART_WORDLENGTH_8B
#define CFG_HW_UART4_STOPBITS       UART_STOPBITS_1
#define CFG_HW_UART4_PARITY         UART_PARITY_NONE
#define CFG_HW_UART4_HWFLOWCTL      UART_HWCONTROL_NONE
#define CFG_HW_UART4_MODE           UART_MODE_TX_RX
#define CFG_HW_UART4_ADVFEATUREINIT UART_ADVFEATURE_NO_INIT


//static void hardware_rand_initialize(void);
static void STM32_Error_Handler(void);
static void Init_MEM1_Sensors(void);
static void SystemClock_Config(void);
static void InitTimers(void);
static void InitRTC(void);
static void UART_Console_Init(void);
static void UART4_Console_Init(void);
// static void MX_SPI1_Init(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void SPI3_IRQHandler(void);
// void SPI1_IRQHandler(void);

static void MX_ADC1_Init(void);
static void MX_GPIO_Init(void);
void TimerDelay_Init(void);

/*
 * Prototypes for the demos that can be started from this project.
 */
extern void vStartDemoTask( void );

/*
 * Get board specific unix time.
 */
uint64_t ullGetUnixTime( void );

/**
 * @brief Initializes the STM32L475 IoT node board.
 *
 * Initialization of clock, LEDs, RNG, RTC, and WIFI module.
 */
static void prvMiscInitialization( void );

/**
 * @brief Initializes the FreeRTOS heap.
 *
 * Heap_5 is being used because the RAM is not contiguous, therefore the heap
 * needs to be initialized.  See http://www.freertos.org/a00111.html
 */
static void prvInitializeHeap( void );
/*-----------------------------------------------------------*/

static BaseType_t prvInitializeWifi( void );
/*-----------------------------------------------------------*/

void vLoggingPrintf( const char * pcFormat,
                     ... )
{
    va_list vargs;
    va_start( vargs, pcFormat );
    vprintf( pcFormat, vargs );
    va_end( vargs );
}

/**
 * @brief Application runtime entry point.
 */
int main( void )
{
    /* Perform any hardware initialization that does not require the RTOS to be
     * running.  */
    prvMiscInitialization();

    /* Start the scheduler.  Initialization that requires the OS to be running,
     * including the WiFi initialization, is performed in the RTOS daemon task
     * startup hook. */
    vTaskStartScheduler();

    return 0;
}
/*-----------------------------------------------------------*/


UBaseType_t uxRand( void )
{
    const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;

    /*
     * Utility function to generate a pseudo random number.
     *
     * !!!NOTE!!!
     * This is not a secure method of generating a random number.  Production
     * devices should use a True Random Number Generator (TRNG).
     */
    ulNextRand = ( ulMultiplier * ulNextRand ) + ulIncrement;
    return( ( int ) ( ulNextRand >> 16UL ) & 0x7fffUL );
}

void vApplicationDaemonTaskStartupHook( void )
{
	/**
	* Initialize wifi semaphore
	*/
	xWifiSemaphoreHandle = xSemaphoreCreateMutexStatic( &( xSemaphoreBuffer ) );
	/* Initialize semaphore. */
	xSemaphoreGive( xWifiSemaphoreHandle );

    /* Demos that use the network are created after the network is
    * up. */
    configPRINTF( ( "---------STARTING DEMO---------\r\n" ) );
    vStartDemoTask();
}
/*-----------------------------------------------------------*/

static BaseType_t prvInitializeWifi( void )
{
    uint32_t hal_version = HAL_GetHalVersion();
    uint32_t bsp_version = BSP_GetVersion();
    WIFI_Status_t xWifiStatus;
    BaseType_t ret;

    /* Turn on the WiFi before key provisioning. This is needed because
     * if we want to use offload SSL, device certificate and key is stored
     * on the WiFi module during key provisioning which requires the WiFi
     * module to be initialized. */
    xWifiStatus = WIFI_Init();

    if( xWifiStatus == WIFI_STATUS_OK )
    {
        configPRINTF( ( "WiFi module initialized.\r\n" ) );
        
        configPRINTF( ( "STM32L4XX Lib:\r\n") );
        configPRINTF( ( "> CMSIS Device Version: %d.%d.%d.%d.\r\n",
                        __STM32L4_CMSIS_VERSION_MAIN, __STM32L4_CMSIS_VERSION_SUB1,
                        __STM32L4_CMSIS_VERSION_SUB2, __STM32L4_CMSIS_VERSION_RC ) );
        configPRINTF( ( "> HAL Driver Version: %ld.%ld.%ld.%ld.\r\n",
                        ((hal_version >> 24U) & 0xFF), ((hal_version >> 16U) & 0xFF),
                        ((hal_version >> 8U) & 0xFF), (hal_version & 0xFF) ) );
        configPRINTF( ( "> BSP Driver Version: %ld.%ld.%ld.%ld.\r\n",
                        ((bsp_version >> 24U) & 0xFF), ((bsp_version >> 16U) & 0xFF),
                        ((bsp_version >> 8U) & 0xFF), (bsp_version & 0xFF)) );

        if( WIFI_GetMAC_Address( MAC_Addr ) != WIFI_STATUS_OK )
        {          
            configPRINTF( ( "!!!ERROR: ES-WIFI Get MAC Address Failed.\r\n") );
            ret = -1;
        }
        else if( WIFI_Connect( WIFI_SSID, WIFI_PASSWORD, WIFI_SECURITY_TYPE ) != WIFI_STATUS_OK )
        {
            configPRINTF( ("!!!ERROR: ES-WIFI NOT connected.\r\n") );
            ret = -1;
        }
        else
        {
            configPRINTF( ( "ES-WIFI MAC Address: %X:%X:%X:%X:%X:%X\r\n",     
                          MAC_Addr[0], MAC_Addr[1], MAC_Addr[2],
                          MAC_Addr[3], MAC_Addr[4], MAC_Addr[5] ) );
            
            configPRINTF( ( "ES-WIFI Connected.\r\n" ) );

            if( WIFI_GetIP_Address( IP_Addr ) == WIFI_STATUS_OK )
            {
                configPRINTF( ( "> ES-WIFI IP Address: %d.%d.%d.%d\r\n",     
                                IP_Addr[0],
                                IP_Addr[1],
                                IP_Addr[2],
                                IP_Addr[3] ) ); 
			}

            if( (IP_Addr[0] == 0)&& 
                (IP_Addr[1] == 0)&& 
                (IP_Addr[2] == 0)&&
                (IP_Addr[3] == 0) )
            {
                configPRINTF( ("!!!ERROR: ES-WIFI Get IP Address Failed.\r\n") );
                ret = -1;
            }
            else
            {
                ret = 0;
            }
        }
    }
    else
    {
        ret = -1;
    }

    return ret;
}

/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
 * function then they must be declared static - otherwise they will be allocated on
 * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetTimerTaskMemory() to provide the memory that is
 * used by the RTOS daemon/time task. */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
 * function then they must be declared static - otherwise they will be allocated on
 * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/

/**
 * @brief Publishes a character to the STM32L475 UART
 *
 * This is used to implement the tinyprintf created by Spare Time Labs
 * http://www.sparetimelabs.com/tinyprintf/tinyprintf.php
 *
 * @param pv    unused void pointer for compliance with tinyprintf
 * @param ch    character to be printed
 */
void vSTM32L475putc( void * pv,
                     char ch )
{
    while( HAL_OK != HAL_UART_Transmit( &UartHandle, ( uint8_t * ) &ch, 1, 30000 ) )
    {
    }
}

/*-----------------------------------------------------------*/

/**
 * @brief Publishes a character to the STM32L475 UART
 * 
 */
int __io_putchar(int ch)
{
    vSTM32L475putc(NULL, ch);

    return 0;
}

/*-----------------------------------------------------------*/

/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void )
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock. */
    SystemClock_Config();

    /* Heap_5 is being used because the RAM is not contiguous in memory, so the
     * heap must be initialized. */
    prvInitializeHeap();

    //    hardware_rand_initialize();
    // srand(hardware_rand());



    /* RNG init function. */
    xHrng.Instance = RNG;

    if( HAL_RNG_Init( &xHrng ) != HAL_OK )
    {
        Error_Handler();
    }

    // Initialize Real Time Clock
    InitRTC();

        // Initialize console
    UART_Console_Init();
    UART4_Console_Init();

    BSP_LED_Init( LED2 );
    BSP_PB_Init( BUTTON_USER, BUTTON_MODE_EXTI );



    if( prvInitializeWifi() != 0 )
    {
        Error_Handler();
    }

        // Initialize timers
    InitTimers();

    MX_GPIO_Init();

    MX_ADC1_Init();

    Init_MEM1_Sensors();

    TimerDelay_Init();

}

void TimerDelay_Init(void)
{
    gu32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
 
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
 
    HTIMx.Instance = TIMER;
    HTIMx.Init.Prescaler = gu32_ticks-1;
    HTIMx.Init.CounterMode = TIM_COUNTERMODE_UP;
    HTIMx.Init.Period = 65535;
    HTIMx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HTIMx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&HTIMx) != HAL_OK)
    {
      STM32_Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&HTIMx, &sClockSourceConfig) != HAL_OK)
    {
      STM32_Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&HTIMx, &sMasterConfig) != HAL_OK)
    {
      STM32_Error_Handler();
    }
 
    HAL_TIM_Base_Start(&HTIMx);
}

void delay_us(uint16_t au16_us)
{
    HTIMx.Instance->CNT = 0;
    while (HTIMx.Instance->CNT < au16_us);
}

int hardware_rand(void)
{
    /* Wait for data ready.  */
    while ((STM32_RNG_SR & STM32_RNG_SR_DRDY) == 0)
        ;

    /* Return the random number.  */
    return STM32_RNG_DR;
}

// static void hardware_rand_initialize(void)
// {
//     /* Enable clock for the RNG.  */
//     STM32L4_RCC_AHB2ENR |= STM32L4_RCC_AHB2ENR_RNGEN;

//     /* Enable the random number generator.  */
//     STM32_RNG_CR = STM32_RNG_CR_RNGEN;
// }
/*-----------------------------------------------------------*/

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void STM32_Error_Handler(void)
{
    printf("FATAL: STM32 Error Handler\r\n");

    // User may add here some code to deal with this error
    while (1)
    {
    }
}



/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
     */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion   = 1;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        STM32_Error_Handler();
    }
    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        STM32_Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        STM32_Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}



static void Init_MEM1_Sensors(void)
{
    // Accelero
    if (ACCELERO_OK != BSP_ACCELERO_Init())
    {
        printf("Error Accelero Sensor\r\n");
    }

    // Gyro
    if (GYRO_OK != BSP_GYRO_Init())
    {
        printf("Error Gyroscope Sensor\r\n");
    }

    // Mag
    if (MAGNETO_OK != BSP_MAGNETO_Init())
    {
        printf("Error Magneto Sensor\r\n");
    }

    // Humidity
    if (HSENSOR_OK != BSP_HSENSOR_Init())
    {
        printf("Error Humidity Sensor\r\n");
    }

    // Temperature
    if (TSENSOR_OK != BSP_TSENSOR_Init())
    {
        printf("Error Temperature Sensor\r\n");
    }

    // Pressure
    if (PSENSOR_OK != BSP_PSENSOR_Init())
    {
        printf("Error Pressure Sensor\r\n");
    }
}



/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

    /*Configure GPIO pin : PB8 */
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    /*Configure GPIO pin : PB8 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  Function for initializing timers for sending the Telemetry data to IoT hub
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
    uint32_t uwPrescalerValue;

    // Timer Output Compare Configuration Structure declaration
    TIM_OC_InitTypeDef sConfig;

    // Compute the prescaler value to have TIM3 counter clock equal to 2 KHz
    uwPrescalerValue = (uint32_t)((SystemCoreClock / 2000) - 1);

    // Set TIM1 instance (Motion)
    TimCCHandle.Instance           = TIM1;
    TimCCHandle.Init.Period        = 65535;
    TimCCHandle.Init.Prescaler     = uwPrescalerValue;
    TimCCHandle.Init.ClockDivision = 0;
    TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
    if (HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
    {
        STM32_Error_Handler();
    }

    // Configure the Output Compare channels
    // Common configuration for all channels
    sConfig.OCMode     = TIM_OCMODE_TOGGLE;
    sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

    // Output Compare Toggle Mode configuration: Channel1
    sConfig.Pulse = DEFAULT_TIM_CC1_PULSE;
    if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
    {
        STM32_Error_Handler();
    }
}


/**
 * @brief Initializes the system clock.
 */
static void SystemClock_Config( void )
{
    RCC_OscInitTypeDef xRCC_OscInitStruct;
    RCC_ClkInitTypeDef xRCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef xPeriphClkInit;

    xRCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    xRCC_OscInitStruct.LSEState = RCC_LSE_ON;
    xRCC_OscInitStruct.MSIState = RCC_MSI_ON;
    xRCC_OscInitStruct.MSICalibrationValue = 0;
    xRCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
    xRCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    xRCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    xRCC_OscInitStruct.PLL.PLLM = 6;
    xRCC_OscInitStruct.PLL.PLLN = 20;
    xRCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    xRCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    xRCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if( HAL_RCC_OscConfig( &xRCC_OscInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     * clocks dividers. */
    xRCC_ClkInitStruct.ClockType = ( RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 );
    xRCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    xRCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    xRCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    xRCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if( HAL_RCC_ClockConfig( &xRCC_ClkInitStruct, FLASH_LATENCY_4 ) != HAL_OK )
    {
        Error_Handler();
    }

    xPeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC
                                          | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C2
                                          | RCC_PERIPHCLK_RNG;
    xPeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    xPeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    xPeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    xPeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    xPeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_MSI;

    if( HAL_RCCEx_PeriphCLKConfig( &xPeriphClkInit ) != HAL_OK )
    {
        Error_Handler();
    }

    __HAL_RCC_PWR_CLK_ENABLE();

    if( HAL_PWREx_ControlVoltageScaling( PWR_REGULATOR_VOLTAGE_SCALE1 ) != HAL_OK )
    {
        Error_Handler();
    }

    /* Enable MSI PLL mode. */
    HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief  Output Compare callback in non blocking mode
 * @param  htim : TIM OC handle
 * @retval None
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
    uint32_t uhCapture = 0;

    // TIM1_CH1 toggling with frequency = 2Hz
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        // Set the Capture Compare Register value
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + t_TIM_CC1_Pulse));
        SendData = 1;
        BSP_LED_Toggle(LED2);
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief  Configures UART interface
 * @param  None
 * @retval None
 */
static void UART_Console_Init(void)
{
    UartHandle.Instance                    = USART1;
    UartHandle.Init.BaudRate               = CFG_HW_UART1_BAUDRATE;
    UartHandle.Init.WordLength             = CFG_HW_UART1_WORDLENGTH;
    UartHandle.Init.StopBits               = CFG_HW_UART1_STOPBITS;
    UartHandle.Init.Parity                 = CFG_HW_UART1_PARITY;
    UartHandle.Init.Mode                   = CFG_HW_UART1_MODE;
    UartHandle.Init.HwFlowCtl              = CFG_HW_UART1_HWFLOWCTL;
    UartHandle.AdvancedInit.AdvFeatureInit = CFG_HW_UART1_ADVFEATUREINIT;
    BSP_COM_Init(COM1, &UartHandle);

}

static void UART4_Console_Init(void)
{
    UartHandle4.Instance                    = UART4;
    UartHandle4.Init.BaudRate               = CFG_HW_UART4_BAUDRATE;
    UartHandle4.Init.WordLength             = CFG_HW_UART4_WORDLENGTH;
    UartHandle4.Init.StopBits               = CFG_HW_UART4_STOPBITS;
    UartHandle4.Init.Parity                 = CFG_HW_UART4_PARITY;
    UartHandle4.Init.Mode                   = CFG_HW_UART4_MODE;
    UartHandle4.Init.HwFlowCtl              = CFG_HW_UART4_HWFLOWCTL;
    UartHandle4.AdvancedInit.AdvFeatureInit = CFG_HW_UART4_ADVFEATUREINIT;
    BSP_COM2_Init(COM2, &UartHandle4);
}
/*-----------------------------------------------------------*/

/**
 * @brief RTC init function.
 */
static void InitRTC( void )
{
    // RTC_TimeTypeDef xsTime;
    // RTC_DateTypeDef xsDate;

    /* Initialize RTC Only. */
    RtcHandle.Instance            = RTC;
    RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
    RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

    if( HAL_RTC_Init( &RtcHandle ) != HAL_OK )
    {
        Error_Handler();
    }

    // /* Initialize RTC and set the Time and Date. */
    // xsTime.Hours = 0x12;
    // xsTime.Minutes = 0x0;
    // xsTime.Seconds = 0x0;
    // xsTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    // xsTime.StoreOperation = RTC_STOREOPERATION_RESET;

    // if( HAL_RTC_SetTime( &RtcHandle, &xsTime, RTC_FORMAT_BCD ) != HAL_OK )
    // {
    //     Error_Handler();
    // }

    // xsDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    // xsDate.Month = RTC_MONTH_JANUARY;
    // xsDate.Date = 0x24;
    // xsDate.Year = 0x17;

    // if( HAL_RTC_SetDate( &RtcHandle, &xsDate, RTC_FORMAT_BCD ) != HAL_OK )
    // {
    //     Error_Handler();
    // }
}
/*-----------------------------------------------------------*/

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler( void )
{
    while( 1 )
    {
        BSP_LED_Toggle( LED_GREEN );
        HAL_Delay( 200 );
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief Warn user if pvPortMalloc fails.
 *
 * Called if a call to pvPortMalloc() fails because there is insufficient
 * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
 * internally by FreeRTOS API functions that create tasks, queues, software
 * timers, and semaphores.  The size of the FreeRTOS heap is set by the
 * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
 *
 */
void vApplicationMallocFailedHook()
{
    taskDISABLE_INTERRUPTS();

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief Loop forever if stack overflow is detected.
 *
 * If configCHECK_FOR_STACK_OVERFLOW is set to 1,
 * this hook provides a location for applications to
 * define a response to a stack overflow.
 *
 * Use this hook to help identify that a stack overflow
 * has occurred.
 *
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char * pcTaskName )
{
    portDISABLE_INTERRUPTS();

    /* Loop forever */
    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

void prvGetRegistersFromStack( uint32_t * pulFaultStackAddress )
{
/* These are volatile to try and prevent the compiler/linker optimising them
 * away as the variables never actually get used.  If the debugger won't show the
 * values of the variables, make them global my moving their declaration outside
 * of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;  /* Link register. */
    volatile uint32_t pc;  /* Program counter. */
    volatile uint32_t psr; /* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* Remove compiler warnings about the variables not being used. */
    ( void ) r0;
    ( void ) r1;
    ( void ) r2;
    ( void ) r3;
    ( void ) r12;
    ( void ) lr;  /* Link register. */
    ( void ) pc;  /* Program counter. */
    ( void ) psr; /* Program status register. */

    /* When the following line is hit, the variables contain the register values. */
    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

/* Psuedo random number generator.  Just used by demos so does not need to be
 * secure.  Do not use the standard C library rand() function as it can cause
 * unexpected behaviour, such as calls to malloc(). */
int iMainRand32( void )
{
    static UBaseType_t uxlNextRand; /*_RB_ Not seeded. */
    const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;

    /* Utility function to generate a pseudo random number. */

    uxlNextRand = ( ulMultiplier * uxlNextRand ) + ulIncrement;

    return( ( int ) ( uxlNextRand >> 16UL ) & 0x7fffUL );
}
/*-----------------------------------------------------------*/

static void prvInitializeHeap( void )
{
    static uint8_t ucHeap1[ configTOTAL_HEAP_SIZE ];
    static uint8_t ucHeap2[ 25 * 1024 ] __attribute__( ( section( ".freertos_heap2" ) ) );

    HeapRegion_t xHeapRegions[] =
    {
        { ( unsigned char * ) ucHeap2, sizeof( ucHeap2 ) },
        { ( unsigned char * ) ucHeap1, sizeof( ucHeap1 ) },
        { NULL,                        0                 }
    };

    vPortDefineHeapRegions( xHeapRegions );
}

/*-----------------------------------------------------------*/


/**
 * @brief  EXTI line detection callback.
 *
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    switch( GPIO_Pin )
    {
        /* Pin number 1 is connected to Inventek Module Cmd-Data
         * ready pin. */
        case GPIO_PIN_13:
            ButtonPressed = 1;
            break;
        case ( GPIO_PIN_1 ):
            SPI_WIFI_ISR();
            break;

        default:
            break;
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief SPI Interrupt Handler.
 *
 * @note Inventek module is configured to use SPI3.
 */
void SPI3_IRQHandler( void )
{
    HAL_SPI_IRQHandler( &( hspi ) );
}
/*-----------------------------------------------------------*/

/**
 * @brief Period elapsed callback in non blocking mode
 *
 * @note This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 *
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef * htim )
{
    if( htim->Instance == TIM6 )
    {
        HAL_IncTick();
    }
}
/*-----------------------------------------------------------*/

int mbedtls_platform_entropy_poll( void * data,
                           unsigned char * output,
                           size_t len,
                           size_t * olen )
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t random_number = 0;
  
  status = HAL_RNG_GenerateRandomNumber(&xHrng, &random_number);
  ((void) data);
  *olen = 0;
  
  if ((len < sizeof(uint32_t)) || (HAL_OK != status))
  {
    return 0;
  }
  
  memcpy(output, &random_number, sizeof(uint32_t));
  *olen = sizeof(uint32_t);
  
  return 0;
}
/*-----------------------------------------------------------*/

uint64_t ullGetUnixTime( void )
{
    TickType_t xTickCount = 0;
    uint64_t ulTime = 0UL;

    /* Get the current tick count. */
    xTickCount = xTaskGetTickCount();

    /* Convert the ticks to milliseconds. */
    ulTime = ( uint64_t ) xTickCount / configTICK_RATE_HZ;

    /* Reduce ulGlobalEntryTimeMs from obtained time so as to always return the
     * elapsed time in the application. */
    ulTime = ( uint64_t ) ( ulTime + ulGlobalEntryTime );

    return ulTime;
}
/*-----------------------------------------------------------*/
