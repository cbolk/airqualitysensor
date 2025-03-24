#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pms5003.h"

void print_timestamp() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    printf("%04d-%02d-%02d %02d:%02d:%02d\n", 
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, 
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void delay_sec(int time)
{
    vTaskDelay(time * 1000 / portTICK_PERIOD_MS);
}

void app_main()
{
    pms5003_measurement_t reading;

    pms5003_config_t pms0 = {
        .rxd_pin = GPIO_NUM_16,
        .txd_pin = GPIO_NUM_17,
        .uart_instance = UART_NUM_2,
        .uart_buffer_size = 128
    };
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    printf("pms setup\n");
    uart_driver_install(pms0.uart_instance, pms0.uart_buffer_size * 2, 0, 0, NULL, 0);
    uart_param_config(pms0.uart_instance, &uart_config);
    uart_set_pin(pms0.uart_instance, pms0.txd_pin, pms0.rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    printf("pms setup concluded\n");

    while(1)
    {
        pms5003_make_measurement(&pms0, &reading, 1);
        print_timestamp();
        pms5003_print_measurement(&reading);
        delay_sec(60);
    }
}
