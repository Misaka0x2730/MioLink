#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "device_i2c.h"

static SemaphoreHandle_t device_i2c_mutex = NULL;
static bool device_i2c_initialized = false;

void device_i2c_init(void)
{
    if (device_i2c_initialized == false)
    {
        i2c_init(DEVICE_I2C, DEVICE_I2C_FREQ);
        gpio_set_function(18, GPIO_FUNC_I2C);
        gpio_set_function(19, GPIO_FUNC_I2C);

        device_i2c_mutex = xSemaphoreCreateMutex();
        device_i2c_initialized = true;
    }
}

size_t device_i2c_read(const uint8_t slave_address,
                       const uint8_t memory_address,
                       uint8_t *p_data,
                       const size_t length)
{
    int32_t read_len = 0;
    if (xSemaphoreTake(device_i2c_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (i2c_write_blocking(DEVICE_I2C, slave_address, &memory_address, sizeof(memory_address), true) < PICO_OK)
        {
            xSemaphoreGive(device_i2c_mutex);
            return 0;
        }

        read_len = i2c_read_blocking(DEVICE_I2C, slave_address, p_data, length, false);

        xSemaphoreGive(device_i2c_mutex);

        if (read_len < PICO_OK)
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }

    return (size_t)read_len;
}

size_t device_i2c_write(const uint8_t slave_address,
                         const uint8_t memory_address,
                         const uint32_t max_write_size,
                         uint8_t *p_data,
                         const size_t length)
{
    if (length > (DEVICE_I2C_MAX_WRITE_SIZE - 1))
    {
        return 0;
    }

    uint8_t data_packet[DEVICE_I2C_MAX_WRITE_SIZE] = { 0 };
    if (xSemaphoreTake(device_i2c_mutex, portMAX_DELAY) == pdTRUE)
    {
        uint8_t attempt_count = 0;

        for (size_t i = 0; i < length; i += max_write_size)
        {
            const uint16_t write_size = (length - i) > max_write_size ? max_write_size : (length - i);

            data_packet[0] = memory_address + i; // Set address
            memcpy(data_packet + 1, p_data + i, write_size);
            attempt_count = 0;
            while (i2c_write_blocking(DEVICE_I2C, slave_address, data_packet, write_size + 1, false) < 0)
            {
                busy_wait_ms(DEVICE_I2C_WRITE_DELAY);
                if (++attempt_count >= DEVICE_I2C_WRITE_ATTEMPTS)
                {
                    break;
                }
            }

            if (attempt_count >= DEVICE_I2C_WRITE_ATTEMPTS)
            {
                xSemaphoreGive(device_i2c_mutex);
                return 0;
            }
        }

        xSemaphoreGive(device_i2c_mutex);
    }

    return (size_t)length;
}