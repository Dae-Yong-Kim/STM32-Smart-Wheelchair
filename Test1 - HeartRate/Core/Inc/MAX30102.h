#ifndef __MAX30102_H
#define __MAX30102_H

#include "stm32f4xx_hal.h"  // STM32 HAL 헤더

// MAX30102 I2C 주소 (기본적으로 0x57 또는 0xAE)
#define MAX30102_I2C_ADDR        (0xAE >> 1)  // 7-bit 주소로 설정

// MAX30102 레지스터 정의
#define MAX30102_REG_INT_STATUS       0x00
#define MAX30102_REG_INT_ENABLE       0x02
#define MAX30102_REG_FIFO_WR_PTR      0x04
#define MAX30102_REG_FIFO_RD_PTR      0x06
#define MAX30102_REG_FIFO_DATA        0x07
#define MAX30102_REG_MODE_CONFIG      0x09
#define MAX30102_REG_SPO2_CONFIG      0x0A
#define MAX30102_REG_LED1_PA          0x0C
#define MAX30102_REG_LED2_PA          0x0D
#define MAX30102_REG_FIFO_CONFIG      0x08
#define MAX30102_REG_TEMP_CONFIG      0x1F

// MAX30102 초기화 함수
HAL_StatusTypeDef MAX30102_Init(void);

// MAX30102 FIFO에서 데이터 읽기
HAL_StatusTypeDef MAX30102_ReadFIFO(uint32_t *red_sample, uint32_t *ir_sample);

// MAX30102 상태 체크 (데이터 준비 완료 여부)
HAL_StatusTypeDef MAX30102_Check(void);

// I2C로 데이터 읽기 및 쓰기
HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef MAX30102_ReadReg(uint8_t reg, uint8_t *value);

// 심박수 측정을 위한 함수

void calculate_heart_rate(uint32_t ir_sample, uint32_t *heart_rate);

#endif /* __MAX30102_H */
