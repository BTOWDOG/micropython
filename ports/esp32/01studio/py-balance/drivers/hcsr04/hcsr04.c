#include "hcsr04.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_timer.h"
#include "esp_rom_sys.h"

#define TRIG_GPIO 47
#define ECHO_GPIO 48

static volatile float g_distance_mm = 0;

static float hcsr04_read_cm(void)
{
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);

    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);

    gpio_set_level(TRIG_GPIO, 0);

    uint64_t timeout = esp_timer_get_time();

    // 等待高电平开始
    while (gpio_get_level(ECHO_GPIO) == 0)
    {
        if (esp_timer_get_time() - timeout > 30000)
        {
            return -1;
        }
    }

    uint64_t echo_start = esp_timer_get_time();

    // 等待高电平结束
    while (gpio_get_level(ECHO_GPIO) == 1)
    {
        if (esp_timer_get_time() - echo_start > 30000)
        {
            return -1;
        }
    }

    uint64_t echo_end = esp_timer_get_time();

    float duration_us =
        (float)(echo_end - echo_start);

    // us -> cm
    return duration_us / 5.8f;
}

static void ultrasonic_task(void *arg)
{
    while (1)
    {
        float dist = hcsr04_read_cm();

        if (dist > 0)
        {
            g_distance_mm = dist;
        }else{
            g_distance_mm = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void ultrasonicInit(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TRIG_GPIO,
    };

    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << ECHO_GPIO;


    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);

    xTaskCreate(
        ultrasonic_task,
        "ultrasonic_task",
        2048,
        NULL,
        3,
        NULL
    );
}

float ultrasonicGetDistance(void)
{
    return g_distance_mm;
}