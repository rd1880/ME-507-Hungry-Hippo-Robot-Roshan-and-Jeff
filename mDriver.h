#ifndef MDRIVER_H
#define MDRIVER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct mDriver{
	int32_t channel_L;
	int32_t channel_R;
	TIM_HandleTypeDef* hTIM;
	int32_t ARR;
	int32_t currDuty;
} mDriver;

void init_driver(mDriver* motorPIN, int32_t channel_L, int32_t channel_R, TIM_HandleTypeDef* handle);

int32_t setMotorPIN(mDriver* motorPIN, int32_t level);

void enableMotor(mDriver* motorPIN);


#endif
