#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
//#include "esp_adc_cal.h"//cant seem to find #include "esp_adc_cal.h"
#include "driver/timer.h"

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define DEFAULT_VREF    1100        

#define TIMERG0    1
#define HW_TIMER_1    1
        
#define SAMPLE_INTERNAL_ADC_TYPE_ID  0x01
#define SAMPLE_INTERNAL_GPIO_TYPE_ID 0x01

//static esp_adc_cal_characteristics_t adc_chars;

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

uint16_t vt_gpio_id_sensor_1 = SAMPLE_INTERNAL_GPIO_TYPE_ID;
uint16_t vt_gpio_id_sensor_2 = SAMPLE_INTERNAL_GPIO_TYPE_ID;

/* ADC Definitions */
uint16_t vt_adc_id_sensor_1 = SAMPLE_INTERNAL_ADC_TYPE_ID;
uint16_t vt_adc_id_sensor_2 = SAMPLE_INTERNAL_ADC_TYPE_ID;

adc_unit_t vt_adc_controller_sensor_1 = ADC_UNIT_1;
adc_unit_t vt_adc_controller_sensor_2 = ADC_UNIT_1;

uint32_t vt_adc_channel_sensor_1 = ADC_CHANNEL_6;
uint32_t vt_adc_channel_sensor_2 = ADC_CHANNEL_7;

/* GPIO Definitions */
//uint16_t vt_gpio_id_sensor_1 = SAMPLE_INTERNAL_GPIO_TYPE_ID;
//uint16_t vt_gpio_id_sensor_2 = SAMPLE_INTERNAL_GPIO_TYPE_ID;

uint16_t* vt_gpio_port_sensor_1;
uint16_t* vt_gpio_port_sensor_2;

uint16_t vt_gpio_pin_sensor_1 = GPIO_NUM_18;
uint16_t vt_gpio_pin_sensor_2 = GPIO_NUM_19;
/*
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
*/
uint16_t vt_adc_single_read_init(
    uint16_t adc_id, void* adc_controller, void* adc_channel, uint16_t* adc_resolution, float* adc_ref_volt)
{

    adc_unit_t unit = *((adc_unit_t*)adc_controller);
    adc_channel_t channel = *((adc_channel_t*)adc_channel);

    //Check if Two Point or Vref are burned into eFuse
    //check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    //esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars);
    //print_char_val_type(val_type);

        *adc_resolution = 12;
        *adc_ref_volt   = 3.6f;
        
    return 0;

}

uint16_t vt_adc_single_read(uint16_t adc_id, void* adc_controller, void* adc_channel)
{
    int adc_raw = 0;
    int adc_raw_abs = 0;

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
    adc_raw_abs=abs(adc_raw-4095);
    return (uint16_t)adc_raw_abs;
}

uint16_t vt_gpio_on(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<(*((uint16_t*)gpio_pin)));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(*((uint16_t*)gpio_pin), 1);
    return 0;

}

uint16_t vt_gpio_off(uint16_t gpio_id, void* gpio_port, void* gpio_pin)
{

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<(*((uint16_t*)gpio_pin)));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

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
        .divider = ((80 * default_tick_resolution) - 1),
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    }; // default clock source is APB
    timer_init(TIMERG0, HW_TIMER_1, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMERG0, HW_TIMER_1, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMERG0, HW_TIMER_1, 65535);

    timer_start(TIMERG0, HW_TIMER_1);

    return 0;
}

unsigned long vt_tick_deinit()
{
    timer_pause(TIMERG0, HW_TIMER_1);
    timer_deinit(TIMERG0, HW_TIMER_1);

    return 0;
}

unsigned long vt_tick()
{
    uint64_t task_counter_value;
    timer_get_counter_value(TIMERG0, HW_TIMER_1, &task_counter_value);

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
