#include "MAX30102.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

// printf 함수 구현

// MAX30102 초기화 함수
void MAX30102_Init(void) {
    uint8_t partId;

    // 센서 리셋
    MAX30102_Reset();
    HAL_Delay(100);

    // 센서 ID 확인
    partId = MAX30102_ReadPartID();
    if (partId != 0x15) {
        printf("MAX30102 센서 ID 오류: 0x%02X (예상: 0x15)\r\n", partId);
        return;
    }

    printf("MAX30102 센서 ID 확인: 0x%02X\r\n", partId);

    // 인터럽트 비활성화
    MAX30102_WriteRegister(MAX30102_REG_INT_ENABLE_1, 0x00);
    MAX30102_WriteRegister(MAX30102_REG_INT_ENABLE_2, 0x00);

    // FIFO 설정
    // 샘플 4개 평균, FIFO Rollover 활성화, Almost Full = 17
    MAX30102_WriteRegister(MAX30102_REG_FIFO_CONFIG, MAX30102_SAMPLEAVG_4 | MAX30102_FIFO_ROLLOVER_EN | MAX30102_FIFO_A_FULL_16);

    // 모드 설정 (SpO2 모드)
    MAX30102_WriteRegister(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2);

    // SPO2 설정 (ADC Range: 4096, 샘플링 속도: 100Hz, LED 펄스 폭: 411us)
    MAX30102_WriteRegister(MAX30102_REG_SPO2_CONFIG, 0x27);

    // LED 전류 설정
    MAX30102_WriteRegister(MAX30102_REG_LED1_PA, 0x24);  // IR LED 전류
    MAX30102_WriteRegister(MAX30102_REG_LED2_PA, 0x24);  // RED LED 전류

    // FIFO 포인터 초기화
    MAX30102_WriteRegister(MAX30102_REG_FIFO_WR_PTR, 0x00);
    MAX30102_WriteRegister(MAX30102_REG_FIFO_RD_PTR, 0x00);
    MAX30102_WriteRegister(MAX30102_REG_FIFO_OVF_CNT, 0x00);

    // 인터럽트 상태 초기화를 위해 읽기
    MAX30102_ReadRegister(MAX30102_REG_INT_STATUS_1);
    MAX30102_ReadRegister(MAX30102_REG_INT_STATUS_2);

    // 데이터 준비 인터럽트 활성화
    MAX30102_WriteRegister(MAX30102_REG_INT_ENABLE_1, 0x40);  // FIFO Almost Full 활성화

    printf("MAX30102 초기화 완료\r\n");
}

// MAX30102 리셋 함수
void MAX30102_Reset(void) {
    MAX30102_WriteRegister(MAX30102_REG_MODE_CONFIG, 0x40); // 리셋 비트 설정
}

// MAX30102 PART ID 읽기
uint8_t MAX30102_ReadPartID(void) {
    return MAX30102_ReadRegister(MAX30102_REG_PART_ID);
}

// MAX30102 레지스터에 값 쓰기
void MAX30102_WriteRegister(uint8_t reg, uint8_t value) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    if (status != HAL_OK) {
        printf("I2C 쓰기 오류: 0x%02X\r\n", status);
    }
}

// MAX30102 레지스터 값 읽기
uint8_t MAX30102_ReadRegister(uint8_t reg) {
    uint8_t value = 0;
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, MAX30102_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    if (status != HAL_OK) {
        printf("I2C 읽기 오류: 0x%02X\r\n", status);
    }
    return value;
}

// 센서 상태 확인 (데이터가 준비되었는지 확인)
uint8_t MAX30102_Check(void) {
    uint8_t status;

    status = MAX30102_ReadRegister(MAX30102_REG_INT_STATUS_1);

    // FIFO Almost Full 인터럽트 확인
    if (status & 0x40) {
        return 1;  // 데이터 준비됨
    }

    return 0;  // 데이터 준비 안됨
}

// FIFO에서 데이터 읽기 (SpO2 모드에서 IR과 RED 데이터 모두 읽기)
uint32_t MAX30102_ReadFIFO(uint32_t *pun_red_led, uint32_t *pun_ir_led) {
    uint8_t data[6];
    HAL_StatusTypeDef status;

    // FIFO에서 6바이트 데이터 읽기 (IR LED 3바이트 + RED LED 3바이트)
    status = HAL_I2C_Mem_Read(&hi2c1, MAX30102_I2C_ADDR, MAX30102_REG_FIFO_DATA,
                              I2C_MEMADD_SIZE_8BIT, data, 6, 100);

    if (status != HAL_OK) {
        printf("FIFO 읽기 오류: 0x%02X\r\n", status);
        return 0;
    }

    // RED 데이터 (18비트 샘플)
    *pun_red_led = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *pun_red_led &= 0x3FFFF;  // 18비트 마스크

    // IR 데이터 (18비트 샘플)
    *pun_ir_led = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *pun_ir_led &= 0x3FFFF;  // 18비트 마스크

    return 1;  // 성공
}


