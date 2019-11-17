#include <stdint.h>
#include "driver/ledc.h"

#define LED_R     GPIO_NUM_19
#define LED_G     GPIO_NUM_18
#define LED_B     GPIO_NUM_21

ledc_timer_config_t timer_conf;
ledc_channel_config_t ledc_conf_r;
ledc_channel_config_t ledc_conf_g;
ledc_channel_config_t ledc_conf_b;

void signal_rgb_led_init( void );
void signal_rgb_led_set( uint8_t red, uint8_t green, uint8_t blue);
