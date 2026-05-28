#ifndef LIGHT_TASK_H
#define LIGHT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#define LIGHT_LED_COUNT 10U
#define LIGHT_RGB_BYTES (LIGHT_LED_COUNT * 3U)
#define LIGHT_UART_FRAME_BYTES (2U + LIGHT_RGB_BYTES + 2U)
#define LIGHT_TASK_INIT_TIME_MS 700U
#define LIGHT_TASK_PERIOD_MS 50U

typedef enum
{
    LIGHT_MODE_AUTO = 0,
    LIGHT_MODE_MANUAL,
} light_mode_t;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} light_rgb_t;

void LightTask_Init(void);
void light_task(void const *pvParameters);

void light_set_auto_mode(void);
void light_set_manual_mode(void);
void light_set_all(uint8_t r, uint8_t g, uint8_t b);
void light_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void light_clear(void);
void light_refresh_now(void);

#endif
