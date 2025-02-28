#include "main.h"
#include "stdio.h"  // fputc 사용을 위해 필요

// fputc 함수 정의 - STM32에서 UART로 데이터를 출력하기 위한 함수
int fputc(int ch, FILE *f) {
    // UART2를 사용하여 데이터를 전송
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
