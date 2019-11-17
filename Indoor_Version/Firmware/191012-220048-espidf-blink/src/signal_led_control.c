#include "signal_led_control.h"



void signal_rgb_led_init( void )
{
  timer_conf.bit_num = LEDC_TIMER_8_BIT;
  timer_conf.freq_hz = 500;
  timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  timer_conf.timer_num = LEDC_TIMER_0;
  ledc_timer_config(&timer_conf);

  ledc_conf_r.channel = LEDC_CHANNEL_0;
  ledc_conf_r.duty = 0;
  ledc_conf_r.gpio_num = LED_R;
  ledc_conf_r.intr_type = LEDC_INTR_DISABLE;
  ledc_conf_r.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_conf_r.timer_sel = LEDC_TIMER_0;
  ledc_channel_config(&ledc_conf_r);

  ledc_conf_g.channel = LEDC_CHANNEL_1;
  ledc_conf_g.duty = 0;
  ledc_conf_g.gpio_num = LED_G;
  ledc_conf_g.intr_type = LEDC_INTR_DISABLE;
  ledc_conf_g.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_conf_g.timer_sel = LEDC_TIMER_0;
  ledc_channel_config(&ledc_conf_g);

  ledc_conf_b.channel = LEDC_CHANNEL_2;
  ledc_conf_b.duty = 0;
  ledc_conf_b.gpio_num = LED_B;
  ledc_conf_b.intr_type = LEDC_INTR_DISABLE;
  ledc_conf_b.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_conf_b.timer_sel = LEDC_TIMER_0;
  ledc_channel_config(&ledc_conf_b);
}

void signal_rgb_led_set( uint8_t red, uint8_t green, uint8_t blue)
{
  ledc_set_duty(ledc_conf_r.speed_mode, ledc_conf_r.channel, red);
  ledc_update_duty(ledc_conf_r.speed_mode, ledc_conf_r.channel);

  ledc_set_duty(ledc_conf_g.speed_mode, ledc_conf_g.channel, green);
  ledc_update_duty(ledc_conf_g.speed_mode, ledc_conf_g.channel);

  ledc_set_duty(ledc_conf_b.speed_mode, ledc_conf_b.channel, blue);
  ledc_update_duty(ledc_conf_b.speed_mode, ledc_conf_b.channel);
}
