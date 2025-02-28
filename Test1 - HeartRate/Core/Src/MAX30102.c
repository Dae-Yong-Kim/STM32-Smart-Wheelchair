#include "MAX30102.h"

// I2C 핸들러 정의 (hi2c1은 STM32CubeMX에서 설정한 I2C 핸들러입니다)
extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef MAX30102_Init(void) {
    HAL_StatusTypeDef status;

    // 센서 초기화 레지스터 설정
    status = MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG, 0x40);  // Reset
    if (status != HAL_OK) return status;

    HAL_Delay(100);  // Reset 후 100ms 대기

    status = MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG, 0x03);  // 정상 작동 모드 설정
    if (status != HAL_OK) return status;

    // SPO2 설정 (이 값은 측정 정확도에 영향을 미칩니다)
    status = MAX30102_WriteReg(MAX30102_REG_SPO2_CONFIG, 0x27); // SPO2 설정 (LED 강도 및 샘플링 설정)
    if (status != HAL_OK) return status;

    // FIFO 설정
    status = MAX30102_WriteReg(MAX30102_REG_FIFO_CONFIG, 0x0F);  // FIFO 설정 (샘플 데이터 읽기)
    return status;
}

HAL_StatusTypeDef MAX30102_ReadFIFO(uint32_t *red_sample, uint32_t *ir_sample) {
    uint8_t buffer[6];
    HAL_StatusTypeDef status;

    // FIFO 데이터 읽기
    status = HAL_I2C_Mem_Read(&hi2c1, MAX30102_I2C_ADDR, MAX30102_REG_FIFO_DATA, 1, buffer, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // IR 및 Red 샘플 값 읽기
    *red_sample = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
    *ir_sample = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | (uint32_t)buffer[5];

    return HAL_OK;
}

HAL_StatusTypeDef MAX30102_Check(void) {
    uint8_t status;
    HAL_StatusTypeDef result = MAX30102_ReadReg(MAX30102_REG_INT_STATUS, &status);
    if (result != HAL_OK) return result;

    // FIFO가 비어있지 않으면 데이터 준비 완료
    if (status & 0x40) {
        return HAL_OK;
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MAX30102_ReadReg(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c1, MAX30102_I2C_ADDR, reg, 1, value, 1, HAL_MAX_DELAY);
}

// 심박수 측정 로직 (예시 코드)
void calculate_heart_rate(uint32_t ir_sample, uint32_t *heart_rate) {
    static uint32_t beat_threshold = 0;
    static uint32_t beat_prev_sample = 0;
    static uint32_t beat_count = 0;
    static uint32_t beat_last_time = 0;
    static uint32_t samples_since_last_beat = 0;
    static uint8_t beat_detected = 0;

    samples_since_last_beat++;

    // 적응형 임계값 계산 (초기값 설정)
    if (beat_threshold == 0) {
        beat_threshold = ir_sample;
    }

    // 새로운 임계값 계산 (이동 평균)
    beat_threshold = (beat_threshold * 7 + ir_sample) / 8;

    // 심박수 검출 로직
    if (ir_sample > beat_prev_sample && ir_sample > beat_threshold && beat_detected == 0) {
        // 신호의 피크 검출됨
        if (samples_since_last_beat > 10) {  // 최소 간격 설정 (노이즈 방지)
            beat_count++;
            samples_since_last_beat = 0;
            beat_detected = 1;

            // 심박수 계산 (1초마다)
            uint32_t current_time = HAL_GetTick();
            if (current_time - beat_last_time > 1000) {
                *heart_rate = (beat_count * 60000) / (current_time - beat_last_time);
                beat_count = 0;
                beat_last_time = current_time;
            }
        }
    } else if (ir_sample < beat_threshold) {
        // 신호가 임계값 아래로 떨어지면 다음 비트 검출 준비
        beat_detected = 0;
    }

    beat_prev_sample = ir_sample;
}

