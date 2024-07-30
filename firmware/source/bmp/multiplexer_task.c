#include "FreeRTOS.h"
#include "task.h"
#include "atomic.h"
#include "platform.h"
#include "device_i2c.h"
#include "multiplexer_task.h"

static bool multiplexer_initialized = false;

static uint32_t shadow_pins_value = 0;
static uint32_t actual_pins_value = 0;
static TaskHandle_t multiplexer_task;

static void multiplexer_update(void);
static void multiplexer_directions_set(void);
_Noreturn static void multiplexer_thread_function(void *p_params);

static void multiplexer_update(void)
{
    uint32_t gpio_state = shadow_pins_value;

    uint8_t data[MULTIPLEXER_REG_SIZE*2] = { 0 };
    data[0] = gpio_state & 0xFF;
    data[1] = (gpio_state >> 8) & 0xFF;

    device_i2c_write(MULTIPLEXER_I2C_ADDR, I2C_MULTIPLEXER_REG_OUTPUT_PORT_0, MULTIPLEXER_MAX_WRITE_SIZE, data, sizeof(data));

#if (MULTIPLEXER_PINS_DIRECTION != MULTIPLEXER_PINS_DIRECTION_ALL_OUTPUTS)
    device_i2c_read(MULTIPLEXER_I2C_ADDR, I2C_MULTIPLEXER_REG_INPUT_PORT_0, data, sizeof(data));

    gpio_state = (((uint32_t)data[1]) << 8) + data[0];
#endif

    actual_pins_value = gpio_state;
}

static void multiplexer_directions_set(void)
{
    uint8_t data[MULTIPLEXER_REG_SIZE*2] = { 0 };
    data[0] = MULTIPLEXER_PINS_DIRECTION & 0xFF;
    data[1] = (MULTIPLEXER_PINS_DIRECTION >> 8) & 0xFF;

    if (device_i2c_write(MULTIPLEXER_I2C_ADDR, I2C_MULTIPLEXER_REG_CONFIGURATION_PORT_0, MULTIPLEXER_MAX_WRITE_SIZE, data, sizeof(data)) != sizeof(data))
    {

    }
}

void multiplexer_task_init(void)
{
    if (multiplexer_initialized == false)
    {
        device_i2c_init();

        BaseType_t status = xTaskCreate(multiplexer_thread_function,
                                        "multiplexer_task",
                                        128,
                                        NULL,
                                        PLATFORM_PRIORITY_LOWEST,
                                        &multiplexer_task);

        //vTaskCoreAffinitySet(multiplexer_task, 0x01);
        multiplexer_initialized = true;
    }
}

_Noreturn static void multiplexer_thread_function(void *p_params)
{
    multiplexer_directions_set();

    while (1)
    {
        multiplexer_update();
        xTaskNotifyWait(0, MULTIPLEXER_TASK_EVENT_UPDATE, NULL, pdMS_TO_TICKS(MULTIPLEXER_TASK_UPDATE_PERIOD));
    }
}

void multiplexer_pin_set_value(uint16_t pin, bool value)
{
    const uint32_t mask = (1 << pin);
    if (value)
    {
        Atomic_OR_u32(&shadow_pins_value, mask);
    }
    else
    {
        Atomic_AND_u32(&shadow_pins_value, ~mask);
    }

    if (portCHECK_IF_IN_ISR())
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t status = xTaskNotifyFromISR(multiplexer_task, MULTIPLEXER_TASK_EVENT_UPDATE, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        BaseType_t status = xTaskNotify(multiplexer_task, MULTIPLEXER_TASK_EVENT_UPDATE, eSetBits);
    }
}

bool multiplexer_pin_get(uint16_t pin)
{
    return (actual_pins_value & (1 << pin)) != 0;
}