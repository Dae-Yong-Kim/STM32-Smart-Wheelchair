/*
#ifndef MAX30102_FOR_STM32_HAL_H
#define MAX30102_FOR_STM32_HAL_H

#include "main.h"
#include <stdint.h>
#include <stdlib.h>

#define MAX30102_I2C_ADDR 0xAE
#define MAX30102_I2C_TIMEOUT 1000

#define MAX30102_BYTES_PER_SAMPLE 6
#define MAX30102_SAMPLE_LEN_MAX 32

// Register Definitions (중략)

// MAX30102 Operation Modes
typedef enum max30102_mode_t
{
    max30102_heart_rate = 0x02,
    max30102_spo2 = 0x03,
    max30102_multi_led = 0x07
} max30102_mode_t;

// Sampling Averaging Modes (중략)

// MAX30102 Configuration Structure
typedef struct max30102_t
{
    I2C_HandleTypeDef *_ui2c;
    uint32_t _ir_samples[32];
    uint32_t _red_samples[32];
    uint8_t _interrupt_flag;
    uint32_t _last_heart_rate;  // 최근 심박수 값
} max30102_t;

// Function Declarations

void max30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c);
void max30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);
void max30102_read(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);

void max30102_reset(max30102_t *obj);
void max30102_set_mode(max30102_t *obj, max30102_mode_t mode);

void max30102_set_led_current_1(max30102_t *obj, float ma);
void max30102_set_fifo_config(max30102_t *obj, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full);
void max30102_clear_fifo(max30102_t *obj);

// 심박수 관련 함수 추가
uint32_t max30102_read_heart_rate(max30102_t *sensor);
uint8_t max30102_check_heart_rate(max30102_t *sensor);  // 심박수가 유효한지 체크하는 함수

#endif
*/
