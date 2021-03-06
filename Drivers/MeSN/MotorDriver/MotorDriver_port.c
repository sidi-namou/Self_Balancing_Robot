/**
  ******************************************************************************
  * @file    MotorDriver_port.c
  * @author  
  * @version V1.0.0
  * @date    
  * @brief   This file provides set of portable functions to manage low level
	*					 operations of the Motor Bridge.
  *          
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MotorDriver_port.h"		//check matching between prototype and implementation
#include "gpio.h"
#include "tim.h"
/* Private variables ---------------------------------------------------------*/

/* Private prototypes ---------------------------------------------------------*/

/* Public Functions ----------------------------------------------------------*/
/**
	* description : cf fichier header
  */
void MotorDriver_Port_GPIO_Init(void){
	
	//Peripheral initialization functions are generated by CubeMX software and
	//are note required anymore.
	//Review MX_GPIO_Init() function in file gpio.c for details.
	//
	//If needed, wrapper may be done by calling MX_GPIO_Init() below.
	//In such a case, MX_GPIO_Init() prototype must be exposed by including gpio.h
	// Initialisations des GPIOs
	MX_GPIO_Init(); //Uncomment if call is required
	
	//Pull the two dirPin low (motor in STOP configuration)
	MotorDriver_Port_SetPin_IN1(GPIO_DIRPIN_LOW);
	MotorDriver_Port_SetPin_IN2(GPIO_DIRPIN_LOW);
	
}

/**
	* description : cf fichier header
  */
void MotorDriver_Port_PWM_Init(void){
	
	//Peripheral initialization functions are generated by CubeMX software and
	//are note required anymore.
	//Review MX_TIM10_Init() function in file tim.c for details.
	//
	//If needed, wrapper may be done by calling MX_TIM10_Init() below.
	//In such a case, MX_TIM10_Init() prototype must be exposed by including tim.h
	MX_TIM10_Init();	//Uncomment if call is required
	
	//Start PWM generation
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	
}

/**
	* description : cf fichier header
  */
void MotorDriver_Port_SetPin_IN1(GPIO_DirPinState pinState){
	
	if (pinState == GPIO_DIRPIN_LOW){
		HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
	}
	
}

/**
	* description : cf fichier header
  */
void MotorDriver_Port_SetPin_IN2(GPIO_DirPinState pinState){
	
	if (pinState == GPIO_DIRPIN_LOW){
		HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
	}
	
}

/**
	* description : cf fichier header
  */
void MotorDriver_Port_SetPWM(uint32_t dutyCycle){
	
	if(dutyCycle <= PWM_DUTYCYCLE_FULL_SCALE){
		//Modifie le registre de comparaison (CCR1) a la volee (idem TIM10->CCR1 = dutyCycle)
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, dutyCycle);
	}
	
}
/* Private functions ----------------------------------------------------------*/

/**********END OF FILE****/
