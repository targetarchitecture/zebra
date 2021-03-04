#include "esp_common.h"
#include "gpio.h"
#include "uart.h"
#include "queue.h"
#include "freertos/task.h"

// This task will execute forever and blink LED
// Note that internal high priority tasks still
// execute - such as WiFi stack routines
void LEDBlinkTask(void *pvParameters)
{
    while (1)
    {
        // Delay and turn on
        vTaskDelay(300 / portTICK_RATE_MS);
        GPIO_OUTPUT_SET(12, 1);

        // Delay and LED off
        vTaskDelay(300 / portTICK_RATE_MS);
        GPIO_OUTPUT_SET(12, 0);
    }
}

// User function
// All user code goes here.
// Note that the user function should exit and not block execution
void user_init(void)
{
    UART_SetBaudrate(0, 9600);
    printf("SDK version:%s\n", system_get_sdk_version());

    // Config pin as GPIO12
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);

    // This task blinks the LED continuously
    xTaskCreate(LEDBlinkTask, (signed char *)"Blink", 256, NULL, 2, NULL);
}

uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map)
    {
    case FLASH_SIZE_4M_MAP_256_256:
        rf_cal_sec = 128 - 8;
        break;

    case FLASH_SIZE_8M_MAP_512_512:
        rf_cal_sec = 256 - 5;
        break;

    case FLASH_SIZE_16M_MAP_512_512:
    case FLASH_SIZE_16M_MAP_1024_1024:
        rf_cal_sec = 512 - 5;
        break;

    case FLASH_SIZE_32M_MAP_512_512:
    case FLASH_SIZE_32M_MAP_1024_1024:
        rf_cal_sec = 1024 - 5;
        break;

    default:
        rf_cal_sec = 0;
        break;
    }

    return rf_cal_sec;
}