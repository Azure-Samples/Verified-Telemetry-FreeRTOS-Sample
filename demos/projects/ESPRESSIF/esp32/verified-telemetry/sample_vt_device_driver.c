#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"//cant seem to find #include "esp_adc_cal.h"
#include "driver/timer.h"
#include "driver/spi_master.h"

#include "sample_vt_device_driver.h"
#include<math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "semphr.h"
#include "soc/rtc.h"


#define TIMER_DIVIDER         (80)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define DEFAULT_VREF    1100        

#define TIMERG1    1
#define HW_TIMER_1    1
        
#define SAMPLE_INTERNAL_ADC_TYPE_ID  0x01
#define SAMPLE_INTERNAL_GPIO_TYPE_ID 0x01
#define SAMPLE_EXTERNAL_ADC_TYPE_ID  0x02

#define USINGESP 

static esp_adc_cal_characteristics_t adc_chars;

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

/* Sensor Hardware Definitions */

//sensor_1
uint16_t vt_gpio_id_sensor_1 = SAMPLE_INTERNAL_GPIO_TYPE_ID;
uint16_t vt_adc_id_sensor_1 = SAMPLE_INTERNAL_ADC_TYPE_ID;
adc_unit_t vt_adc_controller_sensor_1 = ADC_UNIT_1;
uint32_t vt_adc_channel_sensor_1 = ADC1_CHANNEL_4;
uint16_t* vt_gpio_port_sensor_1;
uint16_t vt_gpio_pin_sensor_1 = GPIO_NUM_18;

//sensor_2
uint16_t vt_gpio_id_sensor_2 = SAMPLE_INTERNAL_GPIO_TYPE_ID;
uint16_t vt_adc_id_sensor_2 = SAMPLE_INTERNAL_ADC_TYPE_ID;
adc_unit_t vt_adc_controller_sensor_2 = ADC_UNIT_1;
uint32_t vt_adc_channel_sensor_2 = ADC1_CHANNEL_5;
uint16_t* vt_gpio_port_sensor_2;
uint16_t vt_gpio_pin_sensor_2 = GPIO_NUM_19;

//sensor_3
uint16_t vt_adc_id_sensor_3 = SAMPLE_EXTERNAL_ADC_TYPE_ID;
spi_host_device_t vt_adc_controller_sensor_3 = SPI2_HOST;
uint32_t vt_adc_channel_sensor_3 = 0;

//sensor_4
uint16_t vt_adc_id_sensor_4 = SAMPLE_EXTERNAL_ADC_TYPE_ID;
spi_host_device_t vt_adc_controller_sensor_4 = SPI2_HOST;
uint32_t vt_adc_channel_sensor_4 = 1;



gpio_num_t ADC_GPIO_NUM ;

/* Variables needed for External ADC Buffer Read */
mcp320x_handle_t mcp320x_handle;

mcp320x_config_t mcp320x_cfg = {
        .host = SPI2_HOST,
        .device_model = MCP3204_MODEL,
        .reference_voltage = 5000,         // 5V
        .cs_io_num = GPIO_CS};

spi_transaction_t transaction = {
            .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
            .length = 24 // 24 bits.
        };

char ptrTaskList[250];
TaskStatus_t xTaskDetails;

//static bool init_called=false;
static bool once_called=false;

//uint8_t adc_mcp3204_TxData[3];
//uint8_t adc_mcp3204_RxData[3];
float value;
//void* adc_mcp3204_spi_handle;
float* adc_mcp3204_read_buffer_local;
uint16_t adc_mcp3204_read_buffer_length_local;
uint16_t adc_mcp3204_read_buffer_datapoints_stored = 0;
typedef void (*VT_ADC_BUFFER_READ_CALLBACK_FUNC)(void);
typedef uint16_t (*VT_ADC_BUFFER_READ_FULL_CALLBACK_FUNC)(uint16_t mode);
VT_ADC_BUFFER_READ_CALLBACK_FUNC adc_mcp3204_read_buffer_half_complete_callback;
VT_ADC_BUFFER_READ_FULL_CALLBACK_FUNC adc_mcp3204_read_buffer_full_complete_callback;

// void sample_mcp3204_read_start();
// void sample_mcp3204_read_stop();

TaskHandle_t TaskHandle_1;
TaskHandle_t TaskHandle_2;
TaskHandle_t azuretask;
SemaphoreHandle_t  sema_v=NULL; 
static bool oncerun=false;

int full=0;
bool stop_spi=false;
 gpio_config_t io_conf;


static void check_efuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

uint16_t vt_adc_single_read_init(
    uint16_t adc_id, void* adc_controller, void* adc_channel, uint16_t* adc_resolution, float* adc_ref_volt)
{
    printf("TIMER_BASE_CLK - %d",TIMER_BASE_CLK);
    adc_unit_t unit = *((adc_unit_t*)adc_controller);
    adc_channel_t channel = *((adc_channel_t*)adc_channel);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();
    
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc_gpio_init(ADC_UNIT_1,channel);
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
        adc1_pad_get_io_num(channel,&ADC_GPIO_NUM);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars);
    print_char_val_type(val_type);

        *adc_resolution = 12;
        *adc_ref_volt   = 3.6f;
        

    return 0;

}

uint16_t vt_adc_single_read(uint16_t adc_id, void* adc_controller, void* adc_channel)
{
    int adc_raw = 0;
    //int adc_raw_abs = 0;

    adc_unit_t unit = *((adc_unit_t*)adc_controller);
    adc_channel_t channel = *((adc_channel_t*)adc_channel);
    
    if (unit == ADC_UNIT_1)
    {
        adc_raw=adc1_get_raw(channel);
    }
    else
    {
        adc2_get_raw(channel, width, &adc_raw);
    }
    //adc_raw_abs=abs(adc_raw-4095);
    return (uint16_t)adc_raw;
}

uint16_t vt_gpio_on(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{

    //gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<(*((uint16_t*)gpio_pin)));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_pull_mode(ADC_GPIO_NUM,GPIO_FLOATING);

    gpio_set_level(*((uint16_t*)gpio_pin), 1);
    return 0;

}

uint16_t vt_gpio_off(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{

    //gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<(*((uint16_t*)gpio_pin)));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_pull_mode(ADC_GPIO_NUM,GPIO_PULLDOWN_ONLY);

    gpio_set_level(*((uint16_t*)gpio_pin), 0);
    return 0;

}

uint16_t vt_tick_init(uint16_t* max_value, uint16_t* resolution_usec)
{
    uint16_t default_max_tick        = 65535;
    uint16_t default_tick_resolution = 1;
    if (*max_value)
    {
        default_max_tick = *max_value;
    }
    else
    {
        *max_value = default_max_tick;
    }
    if (*resolution_usec)
    {
        default_tick_resolution = *resolution_usec;
    }
    else
    {
        *resolution_usec = default_tick_resolution;
    }

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    }; // default clock source is APB
    timer_init(TIMERG1, HW_TIMER_1, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMERG1, HW_TIMER_1, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMERG1, HW_TIMER_1, 65535);

    timer_start(TIMERG1, HW_TIMER_1);

    return 0;
}

unsigned long vt_tick_deinit()
{
    timer_pause(TIMERG1, HW_TIMER_1);
    timer_deinit(TIMERG1, HW_TIMER_1);

    return 0;
}

unsigned long vt_tick()
{
    uint64_t task_counter_value;
    timer_get_counter_value(TIMERG1, HW_TIMER_1, &task_counter_value);

    return task_counter_value;
}


void vt_interrupt_enable()
{
    esp_intr_noniram_enable();
}

void vt_interrupt_disable()
{
    esp_intr_noniram_disable();
}



static void read_task(){
    //   uint16_t decimal;
    // float frac_float;
    //  uint16_t frac;
    //  long stop;

    while(1){
         //long start = xTaskGetTickCount();
         //printf("\nstart read task\n");
        if(stop_spi){
           //printf("\nstuck\n");
            continue;
        }
           
   
    
        //printf("reading");
           gpio_set_level(GPIO_CS, 0);
       
        ESP_ERROR_CHECK(mcp320x_read_raw(mcp320x_handle, MCP320X_CHANNEL_0, MCP320X_READ_MODE_SINGLE, &value,&transaction));

        gpio_set_level(GPIO_CS, 1);

    adc_mcp3204_read_buffer_local[adc_mcp3204_read_buffer_datapoints_stored] =value;
        //(uint16_t)(adc_mcp3204_RxData[1] & 0x0F) << 8 | (uint16_t)adc_mcp3204_RxData[2];
    adc_mcp3204_read_buffer_datapoints_stored++;

    if (adc_mcp3204_read_buffer_datapoints_stored == adc_mcp3204_read_buffer_length_local / 2)
    {   
        full=0;
//      stop = xTaskGetTickCount() - start;

//    printf("\ntime= %lu\n",stop);
//         printf("half");
        
    //  printf("\nprinting rwa sig\n");
      

    //     for (uint16_t iter = 0; iter < adc_mcp3204_read_buffer_datapoints_stored; iter++)
    // {
    //     decimal    = adc_mcp3204_read_buffer_local[iter];
    //     frac_float = adc_mcp3204_read_buffer_local[iter] - (float)decimal;
    //     frac       = fabsf(frac_float) * 10000;
    //     printf("%d.%04d, ", decimal, frac);
    // }
    // printf("\n");
         xSemaphoreGive(sema_v);
    }
    else if (adc_mcp3204_read_buffer_datapoints_stored == adc_mcp3204_read_buffer_length_local)
    {adc_mcp3204_read_buffer_datapoints_stored=0;
        full=1;
             //stop = xTaskGetTickCount() - start;

   //printf("\ntime= %lu\n",stop);
        
        //printf("full");
        
    //  printf("\nprinting rwa sig\n");
      

    //     for (uint16_t iter = 0; iter < adc_mcp3204_read_buffer_datapoints_stored; iter++)
    // {
    //     decimal    = adc_mcp3204_read_buffer_local[iter];
    //     frac_float = adc_mcp3204_read_buffer_local[iter] - (float)decimal;
    //     frac       = fabsf(frac_float) * 10000;
    //     printf("%d.%04d, ", decimal, frac);
    // }
    // printf("\n");
    
         xSemaphoreGive(sema_v);
        
     }
    
    
    //             for (uint16_t iter = 0; iter < 128; iter++)
    // {
    //     decimal    = adc_mcp3204_read_buffer_local[iter];
    //     frac_float = adc_mcp3204_read_buffer_local[iter] - (float)decimal;
    //     frac       = fabsf(frac_float) * 10000;
    //     printf("%d.%04d , ", decimal, frac);
        
    // }
    // printf("\n\n");

    }

}

static void callback_task(){

    for(;;){

    xSemaphoreTake( sema_v,portMAX_DELAY );
    //long starts = xTaskGetTickCount();
    bool stop=false;
    //printf("\nin callback\n");
    //for(int i=0;i<1000000;i++){}

    if (full==0){
        adc_mcp3204_read_buffer_half_complete_callback();
        //for(int i=0;i<1000;i++)

    }
    else if (full==1){
       stop= adc_mcp3204_read_buffer_full_complete_callback(0);
       //printf("stop =%d",stop);
    }
    if (stop==true){
        stop_spi=true;
        // printf("\ntask deleted\n");
        //     vTaskGetInfo(  TaskHandle_1, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatusa=%d\n",xTaskDetails.eCurrentState);

        // printf("\n&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
        //vTaskDelay(100);
        
        vTaskSuspend(TaskHandle_1);
        vTaskDelay(2000/portTICK_RATE_MS);
        
        // vTaskGetInfo(  TaskHandle_1, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatusb=%d\n",xTaskDetails.eCurrentState);
        //         vTaskGetInfo(  TaskHandle_1, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatusc=%d\n",xTaskDetails.eCurrentState);


        // vTaskGetInfo(  TaskHandle_2, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatus2a=%d\n",xTaskDetails.eCurrentState);

        // printf("\n^^^^^^^^^^^^^^^^^^^^^^^\n");

        //ESP_ERROR_CHECK(mcp320x_release(mcp320x_handle));
        ESP_ERROR_CHECK(mcp320x_free(mcp320x_handle));
        stop_spi=false;
        //printf("\nspisptopped %d\n",stop_spi);
        vTaskSuspend(NULL);

        // vTaskGetInfo(  TaskHandle_1, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatusd=%d\n",xTaskDetails.eCurrentState);
        //             vTaskGetInfo(  TaskHandle_2, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatus2=%d\n",xTaskDetails.eCurrentState);
        
    }
//         long stops = xTaskGetTickCount() - starts;

//    printf("\ntime= %lu\n",stops);
    

    }
}






void vt_adc_buffer_read(uint16_t adc_id,
    void* adc_controller,
    void* adc_channel,
    float* adc_read_buffer,
    uint16_t buffer_length,
    float desired_sampling_frequency,
    float* set_sampling_frequency,
    void (*vt_adc_buffer_read_conv_half_cplt_callback)(),
    uint16_t (*vt_adc_buffer_read_conv_cplt_callback)(uint16_t mode))
{
    
    //uint16_t stop_reading=false;
    // azuretask=xTaskGetHandle("AzureDemoTask");
    // if(azuretask==NULL){
    //     printf("no aure task foud");
    // }
   
    // printf("\n in buffer read %d\n",stop_spi);
    //                 vTaskGetInfo(  TaskHandle_1, &xTaskDetails, pdTRUE, eInvalid );
    //     printf("\nstatusc=%d\n",xTaskDetails.eCurrentState);


        // vTaskGetInfo(  TaskHandle_2, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatus2a=%d\n",xTaskDetails.eCurrentState);


    if (adc_id == SAMPLE_EXTERNAL_ADC_TYPE_ID){
    // {   rtc_cpu_freq_config_t* currfreq=  (rtc_cpu_freq_config_t*)malloc(sizeof(rtc_cpu_freq_config_t));
    // rtc_cpu_freq_config_t* setfreq=  (rtc_cpu_freq_config_t*)malloc(sizeof(rtc_cpu_freq_config_t));
    //     rtc_clk_cpu_freq_mhz_to_config(240,setfreq);

    //     rtc_clk_cpu_freq_get_config(currfreq);
    //     printf("prev freq=%d",currfreq->freq_mhz);
    //     rtc_clk_cpu_freq_set_config(setfreq);
    //     rtc_clk_cpu_freq_get_config(currfreq);
    //     printf("after freq=%d",currfreq->freq_mhz);

        //adc_mcp3204_spi_handle                         = adc_controller;
        adc_mcp3204_read_buffer_half_complete_callback = vt_adc_buffer_read_conv_half_cplt_callback;
        adc_mcp3204_read_buffer_full_complete_callback = vt_adc_buffer_read_conv_cplt_callback;
        adc_mcp3204_read_buffer_local                  = adc_read_buffer;
        adc_mcp3204_read_buffer_length_local           = buffer_length;
        adc_mcp3204_read_buffer_datapoints_stored      = 0;

        /* SPI1 parameter configuration*/
        //spi_host_device_t* vt_adc_controller   = (spi_host_device_t*)adc_controller;
        // vt_adc_controller->Init.Mode           = SPI_MODE_MASTER;
        // vt_adc_controller->Init.Direction      = SPI_DIRECTION_2LINES;
        // vt_adc_controller->Init.DataSize       = SPI_DATASIZE_8BIT;
        // vt_adc_controller->Init.CLKPolarity    = SPI_POLARITY_LOW;
        // vt_adc_controller->Init.CLKPhase       = SPI_PHASE_1EDGE;
        // vt_adc_controller->Init.NSS            = SPI_NSS_SOFT;
        // vt_adc_controller->Init.FirstBit       = SPI_FIRSTBIT_MSB;
        // vt_adc_controller->Init.TIMode         = SPI_TIMODE_DISABLE;
        // vt_adc_controller->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        // vt_adc_controller->Init.CRCPolynomial  = 7;
        // vt_adc_controller->Init.CRCLength      = SPI_CRC_LENGTH_DATASIZE;
        // vt_adc_controller->Init.NSSPMode       = SPI_NSS_PULSE_DISABLE;

mcp320x_cfg.clock_speed_hz = 312500 ;
        // uint32_t baudrate_prescaler = 256;
        // if (desired_sampling_frequency > 1)
        // {
        //     baudrate_prescaler = (uint32_t)(
        //         (SystemCoreClock / (desired_sampling_frequency * SAMPLE_MCP3204_SPI_CLOCK_CYCLES_FOR_CONVERSION)) - 1);
        // } 
        // if (baudrate_prescaler < 2)
        // {
        //     vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        //     baudrate_prescaler                        = 2;
        // }
        // else if (baudrate_prescaler < 4)
        // {
        //     vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
        //     baudrate_prescaler                        = 4;
        // }
        // else if (baudrate_prescaler < 8)
        // {
        //     vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        //     baudrate_prescaler                        = 8;
        // }
        // else if (baudrate_prescaler < 16)
        // {
        //     vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        //     baudrate_prescaler                        = 16;
        // }
        // else if (baudrate_prescaler < 21)
        // {
        //     vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
        //     baudrate_prescaler                        = 32;
        // }
        // else if (baudrate_prescaler < 64)
        // {
        //     vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        //     baudrate_prescaler                        = 64;
        // }
        // else if (baudrate_prescaler < 128)
        // {
        //     vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        //     baudrate_prescaler                        = 128;
        // }
        // else
        // {
        //     vt_adc_controller->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
        //     baudrate_prescaler                        = 256;
        // }

        // *set_sampling_frequency = ((float)SystemCoreClock / ((float)(baudrate_prescaler + 1) *
        //                                                         (float)SAMPLE_MCP3204_SPI_CLOCK_CYCLES_FOR_CONVERSION));
*set_sampling_frequency=12970;//mcp320x_cfg.clock_speed_hz;
        // adc_mcp3204_TxData[0] = 0b110;
        // adc_mcp3204_TxData[1] = (*(uint8_t*)adc_channel) << 6;
        // adc_mcp3204_TxData[2] = 0b0;

        //transaction.tx_data[0] = (1 << 2) | (read_mode << 1) | ((channel & 4) >> 2);
        transaction.tx_data[0] =0b110;
        transaction.tx_data[1] = (*(uint8_t*)adc_channel) << 6;
        transaction.tx_data[2] = 0;

            io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_CS);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


        ESP_ERROR_CHECK(mcp320x_initialize(&mcp320x_cfg, &mcp320x_handle));
sema_v = xSemaphoreCreateBinary();

if(once_called==false){
       BaseType_t xReturned= xTaskCreate(read_task, "read", 2048, NULL, configMAX_PRIORITIES - 1, &TaskHandle_1);  
        if( xReturned == pdPASS )
    {
        //printf("task1 done");
    }
 xReturned=xTaskCreate(callback_task, "callback", 4096, NULL, configMAX_PRIORITIES - 1, &TaskHandle_2);
         if( xReturned == pdPASS )
    {
        //printf("task2 done");
    }
    once_called=true;
}

else{
   // printf("resuming");
    vTaskResume(TaskHandle_1);
    vTaskResume(TaskHandle_2);}

        //                 vTaskGetInfo(  TaskHandle_1, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatusc=%d\n",xTaskDetails.eCurrentState);


        // vTaskGetInfo(  TaskHandle_2, &xTaskDetails, pdTRUE, eInvalid );
        // printf("\nstatus2a=%d\n",xTaskDetails.eCurrentState);

    //      sample_mcp3204_read_stop();
       
    //  sample_mcp3204_read_start();

           //printf("MCP3204 read start \r\n");

   
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // io_conf.pin_bit_mask = (1ULL<<GPIO_CS);
    // io_conf.pull_down_en = 0;
    // io_conf.pull_up_en = 0;
    // gpio_config(&io_conf);

    //   gpio_set_level(GPIO_CS, 1);
    // //if (init_called==true){
    // //ESP_ERROR_CHECK(mcp320x_free(mcp320x_handle));
    // //init_called=false;}


    //     uint16_t decimal;
    // float frac_float;
    // uint16_t frac;

//     if(init_called==false){
//         ESP_ERROR_CHECK(mcp320x_initialize(&mcp320x_cfg, &mcp320x_handle,transmit_callback));
//     init_called=true;}
// //long start = xTaskGetTickCount();


//     while(stop_reading==false){
//         //for(int i=0;i<25;i++){
//         adc_mcp3204_read_buffer_datapoints_stored=0;

  
//     //long start = xTaskGetTickCount();
//     while (adc_mcp3204_read_buffer_datapoints_stored<128){
        
        

//         //gpio_config(&io_conf);
//         gpio_set_level(GPIO_CS, 0);
//        // ESP_ERROR_CHECK(mcp320x_initialize(&mcp320x_cfg, &mcp320x_handle,transmit_callback));
//         ESP_ERROR_CHECK(mcp320x_read_raw(mcp320x_handle, MCP320X_CHANNEL_0, MCP320X_READ_MODE_SINGLE, &value,&transaction));

//         gpio_set_level(GPIO_CS, 1);
//         //ESP_ERROR_CHECK(mcp320x_free(mcp320x_handle));

//          adc_mcp3204_read_buffer_local[adc_mcp3204_read_buffer_datapoints_stored] =value;
//         //(uint16_t)(adc_mcp3204_RxData[1] & 0x0F) << 8 | (uint16_t)adc_mcp3204_RxData[2];
//     adc_mcp3204_read_buffer_datapoints_stored++;
//     //printf("\npoint %d - value %f\n",adc_mcp3204_read_buffer_datapoints_stored, value);

//      if (adc_mcp3204_read_buffer_datapoints_stored == adc_mcp3204_read_buffer_length_local / 2)
//      {
//          //long start = xTaskGetTickCount();
//          adc_mcp3204_read_buffer_half_complete_callback();}
//     // long stop = xTaskGetTickCount() - start;

//     // printf("\nhalf time= %lu\n",stop);
//     // }
//      else if (adc_mcp3204_read_buffer_datapoints_stored == adc_mcp3204_read_buffer_length_local)
//      {
//        // long starts = xTaskGetTickCount();
//          stop_reading=adc_mcp3204_read_buffer_full_complete_callback();}
//     //         long stops = xTaskGetTickCount() - starts;

//     // printf("\full time= %lu\n",stops);
//     // }



//     }}
// //    long stop = xTaskGetTickCount() - start;

// //     printf("\ntime= %lu\n",stop);

//     //         for (uint16_t iter = 0; iter < 128; iter++)
//     // {
//     //     decimal    = adc_mcp3204_read_buffer_local[iter];
//     //     frac_float = adc_mcp3204_read_buffer_local[iter] - (float)decimal;
//     //     frac       = fabsf(frac_float) * 10000;
//     //     printf("%d.%04d , ", decimal, frac);
        
//     // }
//     // printf("\n\n");
     

//         ESP_ERROR_CHECK(mcp320x_free(mcp320x_handle));
//         init_called=false;

        oncerun=true;

    }
}