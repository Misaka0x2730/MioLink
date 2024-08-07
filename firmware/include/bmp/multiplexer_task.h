#ifndef MIYAKOLINK_MULTIPLEXER_TASK_H
#define MIYAKOLINK_MULTIPLEXER_TASK_H

#include <stdint.h>

#define MULTIPLEXER_I2C_ADDR           (0x20) //TCA9535 address
#define MULTIPLEXER_REG_SIZE           (0x01)
#define MULTIPLEXER_MAX_WRITE_SIZE     (0x02)

#define MULTIPLEXER_PINS_DIRECTION_ALL_OUTPUTS   (0x0000) // 1 - input (default), 0 - output
#define MULTIPLEXER_PINS_DIRECTION               (0x0010) // 1 - input (default), 0 - output

#define MULTIPLEXER_TASK_UPDATE_PERIOD (5000) // milliseconds
#define MULTIPLEXER_TASK_EVENT_UPDATE  (0x01)

typedef enum
{
    I2C_MULTIPLEXER_REG_FIRST = 0,
    I2C_MULTIPLEXER_REG_INPUT_PORT_0 = I2C_MULTIPLEXER_REG_FIRST,
    I2C_MULTIPLEXER_REG_INPUT_PORT_1,
    I2C_MULTIPLEXER_REG_OUTPUT_PORT_0,
    I2C_MULTIPLEXER_REG_OUTPUT_PORT_1,
    I2C_MULTIPLEXER_REG_POLARITY_INVERSION_PORT_0,
    I2C_MULTIPLEXER_REG_POLARITY_INVERSION_PORT_1,
    I2C_MULTIPLEXER_REG_CONFIGURATION_PORT_0,
    I2C_MULTIPLEXER_REG_CONFIGURATION_PORT_1,
    I2C_MULTIPLEXER_REG_LAST = I2C_MULTIPLEXER_REG_CONFIGURATION_PORT_1,
    I2C_MULTIPLEXER_REG_COUNT
} i2c_multiplexer_reg_t;

void multiplexer_task_init(void);
void multiplexer_pin_set_value(uint16_t pin, bool value);
bool multiplexer_pin_get(uint16_t pin);

#endif //MIYAKOLINK_MULTIPLEXER_TASK_H