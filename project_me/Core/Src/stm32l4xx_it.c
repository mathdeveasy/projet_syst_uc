/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20948.h"
#include "BME280_STM32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern axises my_gyro;
extern axises my_accel;
extern axises my_mag;
extern float Humidity, Pressure, Temperature;
extern float roll, pitch, yaw;

extern int newdata;
extern int ecran_type;

extern uint32_t mot_arinc;
extern int update_arinc;

// on déclare tout en static, pour garder la valeur en stock peut importe les appels

static uint32_t current_word = 0;
static int bit_index = 0; // stocke a quelle bit du mot on est
static int demi_bit = 0;  // si 1, pour envoyer le bit NULL
static int sending = 0;   // permet de savoir si on a fini l'envoi
static int attente= 0; 	  // pour temporiser 4 bits a l'etat NULL après l'envoi des 32bits du mot


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	// fonction pour anti-rebounce
	static uint32_t last_press = 0;
	uint32_t now = HAL_GetTick();

	if(now - last_press > 200) {  //
		ecran_type += 1;
		if(ecran_type >= 3) {
			ecran_type = 0;
		}
		last_press = now;
	}

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  //condition pour démarrer l'envoi
  if(update_arinc == 1 && sending == 0) {
         current_word = mot_arinc;
         bit_index = 0;
         demi_bit = 0;
         sending = 1;
         update_arinc = 0;
     }

  if(sending==1) {
      if(attente > 0) {

          // on veut temporiser pdt 4 bits entier (soit 8 demibits) ---- donc on met a et b a 0
    	  GPIOA->BSRR = GPIO_BSRR_BR8; // reset sur pa8 = A
    	  GPIOA->BSRR = GPIO_BSRR_BR11; // reset sur pa11 = B
          attente--;

          if(attente == 0) {
              sending = 0;  // mot + attente terminé
              bit_index = 0;
          }

      } else if(demi_bit == 0) {
          // première moitié — envoi de la valeur high ou low
          uint8_t bit = (current_word >> bit_index) & 1;
          if(bit == 1) {
        	  GPIOA->BSRR = GPIO_BSRR_BS8; // set sur pa8 = A
        	  GPIOA->BSRR = GPIO_BSRR_BR11; // reset sur pa11 = B
          } else {
        	  GPIOA->BSRR = GPIO_BSRR_BR8; // reset sur pa8 = A
        	  GPIOA->BSRR = GPIO_BSRR_BS11; // set sur pa11 = B
          }
          demi_bit = 1; // permet de passer lors du tick de timer suivant a l'autre condition

      } else {

          // deuxième moitié — dnvoi de la valeur null
    	  GPIOA->BSRR = GPIO_BSRR_BR8; // reset sur pa8 = A
    	  GPIOA->BSRR = GPIO_BSRR_BR11; // reset sur pa11 = B
          demi_bit = 0;
          bit_index++;

          if(bit_index >= 32) {
              bit_index = 0;
              attente = 8;  // on demarre les 8 demi périodes
          }
      }
  }

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); //toggle pin pour voir si ca marche bien visuellement

  icm20948_gyro_read_dps(&my_gyro); // mesures icm20948
  icm20948_accel_read_g(&my_accel);
  ak09916_mag_read_uT(&my_mag);

  BME280_Measure(&Temperature, &Pressure, &Humidity); //mesure temp

  newdata = 1;

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
