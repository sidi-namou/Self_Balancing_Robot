/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "MotorDriver.h"								// Pilote du moteur
#include "lsm6ds3.h"										// Pilote de la centrale inertielle
#include "mesn_uart.h"									// Pilote de la liaison UART
#include "libSBR_autom_obs-corr.h"			// Algo observateur et correcteur
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BufferSize 100
/* USER CODE END PTD */

//Buffer circulaire partagée entre la tache d'enregistrement et la tache de synchro avec l'uart
typedef struct {
	int32_t buffer[BufferSize];// type int32 parceque il va contenir les angles qui sont bien de ce type
	uint32_t indexW;
	uint32_t indexR;
	uint32_t eltNb;
} EnregistrementAngleBuffer;
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle1,defaultTaskHandle2,defaultTaskHandle3;
QueueHandle_t queue12,queue23;// deux queue de messages pour la communication entre les 3 taches
SemaphoreHandle_t MutexGPIO,MutexTab;// deux mutex pour proteger le GPIO et le buffer
EnregistrementAngleBuffer Tab;// Tableau de type buffer circulaire
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */
// Tache asservissement d'equilibre
void StartDefaultTask(void const * argument);
// Tache enregistrement
void SaveTask(void const* argument);
// Tache synchronisation avec UART
void Synchro_Avec_UART(void const* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	MESN_UART_PutString_Poll((uint8_t*)"\r\nERROR : stack overflow from task");
	MESN_UART_PutString_Poll((uint8_t*)pcTaskName);
	while(1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	MESN_UART_PutString_Poll((uint8_t*)"\r\nERROR : Heap full!");
	while(1);
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
//	Initialisation des compteurs du buffer
       Tab.eltNb=0;// nombre d'elements qui sert pas à grande chause dans cette application
       Tab.indexR=0;// index de lecture
       Tab.indexW=0;// index d'ecriture
       for(int i=0;i<BufferSize;i++)Tab.buffer[i]=0;//initialisation du Tableau pour la commande dump pour eviter le probleme de l'appel de cette commande avant 1s
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  MutexGPIO=xSemaphoreCreateMutex();// creation d'un mutex qui protege le GPIO
  MutexTab=xSemaphoreCreateMutex();// creation d'un mutex qui protege le tableau
    /* start timers, add new ones */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  // Tache d'asservissement de priorité haute
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh , 0, 256);
  defaultTaskHandle1 = osThreadCreate(osThread(defaultTask), NULL);

  // Tache d'enregistrement de priorité normal
  osThreadDef(SaveTask, SaveTask, osPriorityNormal , 0, 256);
   defaultTaskHandle2 = osThreadCreate(osThread(SaveTask), NULL);

   // Tache de synchronisation avec UART de priorité basse
   osThreadDef(Synchro_Avec_UART, Synchro_Avec_UART, osPriorityLow , 0, 512);// 512 octets pour pouvoir declarer un tableau local de taille 100 de type int32_t
      defaultTaskHandle3 = osThreadCreate(osThread(Synchro_Avec_UART), NULL);


      /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  queue12 = xQueueCreate(60,sizeof(int32_t));//queue entre task1 et task2
  queue23 = xQueueCreate(1,sizeof(int32_t));//queue entre task2 et task3
  /* USER CODE END RTOS_QUEUES */


}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	uint8_t LSM6DS3_Res = 0;
	int32_t acc_mg,angle;
	 int32_t rotAng_mDegSec,rotAng_mDegSecY;
	 uint32_t PreviousWakeTime;

	 /*	Init de PreviousWakeTime Pour que cette tache soit periodique de periode 10ms	*/
	 PreviousWakeTime=osKernelSysTick();
	/* Init des periphs externes */
	MotorDriver_Init(); 	// GPIOs et TIM6 sont initialisé par cette fonction
	MESN_UART_Init();
	if(LSM6DS3_begin(&LSM6DS3_Res) != IMU_SUCCESS){
		MESN_UART_PutString_Poll((uint8_t*)"\r\nIMU Error !");
		while(1);
	}

  /* Infinite loop */
  for(;;)
  {
	  /*Calcul de l'angle*/

	  LSM6DS3_readMgAccelX(&acc_mg);
	  // la valeur de l'accelerometre suivant x est ecrit dans acc_mg
	  LSM6DS3_readMdpsGyroX(&rotAng_mDegSec);
	  // la valeur du gyro suivant x est ecrit dans la variable rotAng_mDegSec
	  LSM6DS3_readMdpsGyroY(&rotAng_mDegSecY);
	  //la valeur du gyro suivant y est ecrit dans la variable rotAng_mDegSecY

	  // On utilise un observateur pour estimer la valeur de l'angle
	  angle=autoAlgo_angleObs(acc_mg,rotAng_mDegSec);
	  // la valeur de l'angle est stocké dans la variable angle

	  //Envoie l'angle à la queue de message queue12
	  xQueueSend( queue12,&angle,portMAX_DELAY);

	  //prendre le mutex pour proteger le GPIO
	  xSemaphoreTake(MutexGPIO,portMAX_DELAY);

	  //appliquer la commande aux moteurs
	  MotorDriver_Move(autoAlgo_commandLaw(angle,rotAng_mDegSecY));

	  //rendre le mutex
	  xSemaphoreGive(MutexGPIO);
	  // Un delay pour la periodosation de cette tache
	  osDelayUntil(&PreviousWakeTime,10);

  }
  /* USER CODE END StartDefaultTask */
}
void SaveTask(void const* argument){

	int32_t Angle;
	uint32_t i=0;// a counter for managing LED activation
	for(;;)
	  {
		  /*Wait for angle*/
		  xQueueReceive( queue12,&Angle,portMAX_DELAY);

		  //Take mutex for protecting the circular buffer
		  xSemaphoreTake(MutexTab,portMAX_DELAY);

		  // insert angle value into the buffer
		  Tab.buffer[Tab.indexW]=Angle;
		  Tab.indexW++;
		  if(Tab.indexW >= BufferSize){
			  Tab.indexW = 0;
		  }

		  //Give mutex buffer
		  xSemaphoreGive(MutexTab);


		  // send angle value to the queue(queue23) for stream mode
		  xQueueSend(queue23,&Angle,0);

		  //Managing LED activation
		  // the counter i will incremenete every 10ms
		  // Because this task is synchronised with the first task by queue message


		  // Blinking Led when the angle is less than 25°

		  if(Angle<25000){


			  if(i<10){

				  // Take GPIO mutex for material protection
				  xSemaphoreTake(MutexGPIO,portMAX_DELAY);

				  	 // Led ON
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);

				  //Give GPIO mutex
				  xSemaphoreGive(MutexGPIO);

			  }
			  else{


				  // Take mutex for protecting GPIO
				  xSemaphoreTake(MutexGPIO,portMAX_DELAY);

				  // LED Off for 90*10ms=900ms
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);

				  //Give mutex
				  xSemaphoreGive(MutexGPIO);

				  if(i>99)i=0; // each second we initialise the counter by 0
			  }

		  }
		  // if not the case put permanently the led on
		  else{
			  // Take mutex for protecting GPIO
			  xSemaphoreTake(MutexGPIO,portMAX_DELAY);

			  //Led on
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);

			  //Give mutex
			  xSemaphoreGive(MutexGPIO);

		  }
		  // we incremente the counter
		  i++;
	  }

}
void Synchro_Avec_UART(void const* argument){
	char commande[10],text[10],ref[4];
	int32_t angle,AngleCourant;
	int32_t tableau[BufferSize];


	MESN_UART_PutString_Poll((uint8_t *)"Envoie help pour afficher un menu d'aide sur les 4 commandes");
	MESN_UART_PutString_Poll((uint8_t *)"\r\n");

	while(1){

		//Synchronisation de cette tache avec  l'UART
		MESN_UART_GetString((uint8_t *)commande,osWaitForever);

		// Test sur la commande

		if(strcmp(commande,"help")==0){

			// Si la commande est help alors on affiche un menu des commandes

			MESN_UART_PutString_Poll((uint8_t *)"\r\n");
			MESN_UART_PutString_Poll((uint8_t *)"read : retourne la dernière valeur de l'angle");
			MESN_UART_PutString_Poll((uint8_t *)"\r\n");

			MESN_UART_PutString_Poll((uint8_t *)"dump : retourne les 100 dernières valeurs de l'angle");
			MESN_UART_PutString_Poll((uint8_t *)"\r\n");


			MESN_UART_PutString_Poll((uint8_t *)"stream : retourne en continu la valeur de l'angle");
			MESN_UART_PutString_Poll((uint8_t *)"\r\n");


		}
		else if(strcmp(commande,"read")==0){

			// Si la commande est read alors on affiche la derniere valeur de l'angle

			//prendre le mutex pour proteger le Tableau
			xSemaphoreTake(MutexTab,portMAX_DELAY);

			//Lecture la derniere valeur de l'angle
			angle=Tab.buffer[Tab.indexW-1];

			//rendre le mutex
			xSemaphoreGive(MutexTab);

			// Changer le type de l'angle vers string
			sprintf(text,"%ld",angle);
			// Envoie à l'UART
			MESN_UART_PutString_Poll((uint8_t *)text);
			MESN_UART_PutString_Poll((uint8_t *)"\r\n");

		}
		else if(strcmp(commande,"dump")==0){
			// Si la commande est dump alors on affiche les 100 derniers valeurs de l'angle

			//On copie les 100 derniers valeurs de l'angle dans le tableau local
			// Pour ne pas monopoliser le microprocesseur pendant beaucoup du temps

			// prendre le mutex pour proteger le tableau
			xSemaphoreTake(MutexTab,portMAX_DELAY);

			// On fait une copie du plus ancien au plus recent

			Tab.indexR=Tab.indexW;
			for(int i=0;i<BufferSize;i++){
				tableau[i]=Tab.buffer[Tab.indexR];
				Tab.indexR++;
				if(Tab.indexR == BufferSize){
					Tab.indexR = 0;
				}
			}

			//rendre le mutex
			xSemaphoreGive(MutexTab);

			// On fait l'affichage du tableau sans le mutex comme ça on laisse les autres taches s'actualise
			for(int i=0;i<BufferSize;i++){
				sprintf(text,"%ld",tableau[i]);
				MESN_UART_PutString_Poll((uint8_t *)text);
				MESN_UART_PutString_Poll((uint8_t *)"\r\n");
			}
		}
		else if(strcmp(commande,"stream")==0){

			// Si la commande est stream alors on affiche en continu la valeur de l'angle

			do{
				// attente l'angle
			xQueueReceive( queue23,&AngleCourant,portMAX_DELAY);

			// changer le type de l'angle
			sprintf(text,"%ld",AngleCourant);
			//Envoie cette angle sur l'UART
			MESN_UART_PutString_Poll((uint8_t *)text);
			MESN_UART_PutString_Poll((uint8_t *)"\r\n");

			//attente 10ms pour la nouvelle valeur de l'angle car chaque 10ms on a une nouvelle valeur de l'angle
			MESN_UART_GetString((uint8_t *)ref,10);

			// On sort de ce mode si on appuie sur 'q'
			// cette boucle teste si on a recue un q on sort si non on reste dans ce mode
			}
			while(strcmp(ref,"q")!=0);
		}
		else{
			// Si la commande n'existe pas dans le menu on affiche commande n'existe pas
			MESN_UART_PutString_Poll((uint8_t *)"Commande n'exite pas essayer help pour plus d'information");
			MESN_UART_PutString_Poll((uint8_t *)"\r\n");
		}

	}
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
