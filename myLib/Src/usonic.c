/*
 * uSonic.c
 *
 *  Created on: Jan 16, 2025
 *      Author: user
 */
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

void microDelay(int us){
	/*int t1 = htim2.Instance->CNT;
	while((htim2.Instance->CNT - t1 < us));*/

	htim2.Instance->CNT = 0;
	while((htim2.Instance->CNT < us));
}

void Trigger() {
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
	microDelay(10);
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 1);
	microDelay(10);
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, 0);
}


double Distance() {
	int t1, t2;

	Trigger();

	htim2.Instance->CNT = 0;
	while((HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 0)) {
		if(htim2.Instance->CNT > 30000)	return -1;
	}
	t1 = htim2.Instance->CNT;
	while((HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 1)) {
		if(htim2.Instance->CNT > t1 + 60000)	return -1;
	}
	t2 = htim2.Instance->CNT;

	double ret = (t2 - t1) * 0.17;
	return ret;
}
