#ifndef MIYAKOLINK_I2C_FUNCTIONS_H
#define MIYAKOLINK_I2C_FUNCTIONS_H

#include <stdint.h>
#include <string.h>

#define DEVICE_I2C                     (i2c1)
#define DEVICE_I2C_FREQ                (100000)
#define DEVICE_I2C_MAX_WRITE_SIZE      (64)

#define DEVICE_I2C_WRITE_DELAY         (5) // 5 ms before next writing attempt
#define DEVICE_I2C_WRITE_ATTEMPTS      (100) // 5*100 = 500 ms

void device_i2c_init(void);

size_t device_i2c_read(const uint8_t slave_address,
                       const uint8_t memory_address,
                       uint8_t *p_data,
                       const size_t length);

size_t device_i2c_write(const uint8_t slave_address,
                        const uint8_t memory_address,
                        const uint32_t max_write_size,
                        uint8_t *p_data,
                        const size_t length);

#endif //MIYAKOLINK_I2C_FUNCTIONS_H