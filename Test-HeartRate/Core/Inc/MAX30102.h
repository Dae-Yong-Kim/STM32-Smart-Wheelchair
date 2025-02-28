#ifndef __MAX30102_H
#define __MAX30102_H

#include "main.h"

// MAX30102 I2C 주소
#define MAX30102_I2C_ADDR         0xAE  // 7비트 주소 (0x57 << 1)

// 레지스터 주소들
#define MAX30102_REG_INT_STATUS_1 0x00
#define MAX30102_REG_INT_STATUS_2 0x01
#define MAX30102_REG_INT_ENABLE_1 0x02
#define MAX30102_REG_INT_ENABLE_2 0x03
#define MAX30102_REG_FIFO_WR_PTR  0x04
#define MAX30102_REG_FIFO_OVF_CNT 0x05
#define MAX30102_REG_FIFO_RD_PTR  0x06
#define MAX30102_REG_FIFO_DATA    0x07
#define MAX30102_REG_FIFO_CONFIG  0x08
#define MAX30102_REG_MODE_CONFIG  0x09
#define MAX30102_REG_SPO2_CONFIG  0x0A
#define MAX30102_REG_LED1_PA      0x0C
#define MAX30102_REG_LED2_PA      0x0D
#define MAX30102_REG_PILOT_PA     0x10
#define MAX30102_REG_MULTI_LED    0x11
#define MAX30102_REG_TEMP_INT     0x1F
#define MAX30102_REG_TEMP_FRAC    0x20
#define MAX30102_REG_TEMP_CONFIG  0x21
#define MAX30102_REG_PROX_INT_TH  0x30
#define MAX30102_REG_REV_ID       0xFE
#define MAX30102_REG_PART_ID      0xFF

// 모드 설정 값
#define MAX30102_MODE_HEART_RATE  0x02  // 심박수 모드 (IR LED만 사용)
#define MAX30102_MODE_SPO2        0x03  // SpO2 모드 (IR + Red LED 사용)
#define MAX30102_MODE_MULTI       0x07  // 멀티 LED 모드

// 샘플 평균 설정
#define MAX30102_SAMPLEAVG_1      0x00
#define MAX30102_SAMPLEAVG_2      0x20
#define MAX30102_SAMPLEAVG_4      0x40
#define MAX30102_SAMPLEAVG_8      0x60
#define MAX30102_SAMPLEAVG_16     0x80
#define MAX30102_SAMPLEAVG_32     0xA0

// FIFO 설정
#define MAX30102_FIFO_ROLLOVER_EN 0x10
#define MAX30102_FIFO_A_FULL_8    0x00
#define MAX30102_FIFO_A_FULL_16   0x01
#define MAX30102_FIFO_A_FULL_32   0x02
#define MAX30102_FIFO_A_FULL_24   0x03

// 함수 선언
void MAX30102_Init(void);
uint8_t MAX30102_Check(void);
uint32_t MAX30102_ReadFIFO(uint32_t *pun_red_led, uint32_t *pun_ir_led);
void MAX30102_ReadTemperature(int8_t *integer, uint8_t *frac);
void MAX30102_Reset(void);
uint8_t MAX30102_ReadPartID(void);
uint8_t MAX30102_ReadRegister(uint8_t reg);
void MAX30102_WriteRegister(uint8_t reg, uint8_t value);

#endif /* __MAX30102_H */
