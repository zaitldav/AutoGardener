/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "signal_led_control.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define LED_R     GPIO_NUM_19
#define LED_G     GPIO_NUM_18
#define LED_B     GPIO_NUM_21

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(LED_R);
    gpio_pad_select_gpio(LED_G);
    gpio_pad_select_gpio(LED_B);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);
    while(1) {
    gpio_set_level(LED_R, 1);
    gpio_set_level(LED_G, 0);
    gpio_set_level(LED_B, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_R, 0);
    gpio_set_level(LED_G, 1);
    gpio_set_level(LED_B, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_R, 0);
    gpio_set_level(LED_G, 0);
    gpio_set_level(LED_B, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_R, 1);
    gpio_set_level(LED_G, 1);
    gpio_set_level(LED_B, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_R, 1);
    gpio_set_level(LED_G, 1);
    gpio_set_level(LED_B, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_R, 1);
    gpio_set_level(LED_G, 0);
    gpio_set_level(LED_B, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_R, 0);
    gpio_set_level(LED_G, 1);
    gpio_set_level(LED_B, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    }
}

void app_main()
{
    //xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    signal_rgb_led_init();
    while (1)
    {

      // RED
      signal_rgb_led_set(255, 0, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // GREEN
      signal_rgb_led_set(0, 255, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // BLUE
      signal_rgb_led_set(0, 0, 255);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // CYAN
      signal_rgb_led_set(0, 255, 255);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // PINK
      signal_rgb_led_set(255, 0, 255);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // PURPLE
      signal_rgb_led_set(100, 0, 255);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // YELLOW
      signal_rgb_led_set(255, 255, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // LIME
      signal_rgb_led_set(170, 255, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // ORANGE
      signal_rgb_led_set(255, 90, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      // WHITE
      signal_rgb_led_set(255, 255, 255);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}
