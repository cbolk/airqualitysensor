#include "pms5003.h"

void pms5003_setup(pms5003_config_t* inst)
{
    // Initializing UART interface
    uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(inst->uart_instance, &uart_config);
    uart_set_pin(inst->uart_instance, inst->txd_pin, inst->rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(inst->uart_instance, inst->uart_buffer_size * 2, 0, 0, NULL, 0);

    // Initializing GPIO pins
    gpio_config_t io_conf;
    io_conf.intr_type = 0;// GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << inst->set_pin) | (1ULL << inst->reset_pin) | (1ULL << inst->mode_pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(inst->set_pin, 0);
    gpio_set_level(inst->reset_pin, 1);
    gpio_set_level(inst->mode_pin, 1);
}

void pms5003_make_measurement(pms5003_config_t* inst, pms5003_measurement_t* reading, int num)
{
    uint8_t* data = (uint8_t*) malloc(inst->uart_buffer_size);
    int measurements_number = 0;
    pms5003_measurement_t current_reading;
    int len, res;

    gpio_set_level(inst->set_pin, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for a second...
    uart_flush(inst->uart_instance);

    pms5003_clear_measurement(reading);

    while (measurements_number < num)
    {
        len = uart_read_bytes(inst->uart_instance, data, inst->uart_buffer_size, 1);
        if (len >= PMS5003_MESSAGELENGTH)
        {
            res = pms5003_process_data(len, data, &current_reading);
            if(res){
                pms5003_add_measurements(reading, &current_reading);
                measurements_number++;
            }
        } 
    }
    pms5003_div_measurement(reading, measurements_number);
    free(data);
}

int pms5003_process_data(int len, uint8_t* data, pms5003_measurement_t* reading)
{
    // Length test
    if (len != PMS5003_MESSAGELENGTH){
        printf("frame length not correct (%d)\n", len);
        printf("received bytes:\n");
        for (int i = 0; i < len; i++) {
            printf("%02X ", data[i]);  // Print hex values
        }
        return 0;
    }

    // Start of frame delimiter test
    if (data[0] != 0x42 || data[1] != 0x4D || data[2] != 0x00 || data[3] != 0x1C){
        printf("header not correct\n");
        return 0;
    }

    // Checksum test
    int checksum = 0, checksum_h, checksum_l;
    for (int i = 0; i < 30; i++)
        checksum += data[i];
    checksum_h = (checksum >> 8) & 0xFF;
    checksum_l = checksum & 0xFF;
    if (data[30] != checksum_h || data[31] != checksum_l){
        printf("Checksum error\n");
        return 0;
    }

    // Reading data
    reading->pm1_0_std = (data[4] << 8) + data[5];
    reading->pm2_5_std = (data[6] << 8) + data[7];
    reading->pm10_std = (data[8] << 8) + data[9];
    reading->pm1_0_atm = (data[10] << 8) + data[11];
    reading->pm2_5_atm = (data[12] << 8) + data[13];
    reading->pm10_atm = (data[14] << 8) + data[15];

    return 1;
}

void pms5003_print_measurement(pms5003_measurement_t* reading)
{
    printf("PM1.0 concentration in standard material:       %d ug/m^3\r\n", reading->pm1_0_std);
    printf("PM2.5 concentration in standard material:       %d ug/m^3\r\n", reading->pm2_5_std);
    printf("PM10  concentration in standard material:       %d ug/m^3\r\n", reading->pm10_std);
    printf("PM1.0 concentration in atmospheric environment: %d ug/m^3\r\n", reading->pm1_0_atm);
    printf("PM2.5 concentration in atmospheric environment: %d ug/m^3\r\n", reading->pm2_5_atm);
    printf("PM10  concentration in atmospheric environment: %d ug/m^3\r\n", reading->pm10_atm);
    printf("\r\n");
}

void pms5003_clear_measurement(pms5003_measurement_t* m)
{
    m->pm1_0_std = 0;
    m->pm2_5_std = 0;
    m->pm10_std = 0;
    m->pm1_0_atm = 0;
    m->pm2_5_atm = 0;
    m->pm10_atm = 0;
}

void pms5003_add_measurements(pms5003_measurement_t* m1, pms5003_measurement_t* m2)
{
    m1->pm1_0_std += m2->pm1_0_std;
    m1->pm2_5_std += m2->pm2_5_std;
    m1->pm10_std += m2->pm10_std;
    m1->pm1_0_atm += m2->pm1_0_atm;
    m1->pm2_5_atm += m2->pm2_5_atm;
    m1->pm10_atm += m2->pm10_atm;
}

void pms5003_div_measurement(pms5003_measurement_t* m, int d)
{
    m->pm1_0_std /= d;
    m->pm2_5_std /= d;
    m->pm10_std /= d;
    m->pm1_0_atm /= d;
    m->pm2_5_atm /= d;
    m->pm10_atm /= d;
}
