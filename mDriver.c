#include "mDriver.h"
#include <stdint.h>

void init_driver(mDriver* motorPIN, int32_t channel_L, int32_t channel_R, TIM_HandleTypeDef* handle){
	motorPIN->channel_L = channel_L;
	motorPIN->channel_R = channel_R;
	motorPIN->hTIM = handle;
	motorPIN->ARR = 4799;
	motorPIN->currDuty = 4799;
}

int32_t setMotorPIN(mDriver* motorPIN, int32_t level){

	int32_t channel_Left = 0;
	int32_t channel_Right = 4;
	if(motorPIN->channel_L == 8){
		channel_Left = 8;
		channel_Right = 12;
	}
	if(level > 5000 || level < -5000){
		level = 0;
	}
	if(level == 0){
		__HAL_TIM_SET_COMPARE(motorPIN->hTIM, channel_Left, 0);
		__HAL_TIM_SET_COMPARE(motorPIN->hTIM, channel_Right, 0);
	}
	else if(level > 0 && level <= 4800){
		__HAL_TIM_SET_COMPARE(motorPIN->hTIM, channel_Left, 0);
		__HAL_TIM_SET_COMPARE(motorPIN->hTIM, channel_Right, level);
	}
	else if(level < 0 && level >= -4800){
		__HAL_TIM_SET_COMPARE(motorPIN->hTIM, channel_Left, (-1*level));
		__HAL_TIM_SET_COMPARE(motorPIN->hTIM, channel_Right, 0);
	}

	return 1;
}

void enableMotor(mDriver* motorPIN){
	HAL_TIM_PWM_Start(motorPIN->hTIM, motorPIN->channel_L);
	HAL_TIM_PWM_Start(motorPIN->hTIM, motorPIN->channel_R);
}

void disableMotor(mDriver* motorPIN){
	HAL_TIM_PWM_Stop(motorPIN->hTIM, motorPIN->channel_L);
	HAL_TIM_PWM_Stop(motorPIN->hTIM, motorPIN->channel_R);
}



