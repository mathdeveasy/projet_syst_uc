/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "BME280_STM32.h"
#include "stdio.h"
#include "fonts.h"
#include "ssd1306.h"
#include "icm20948.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
axises my_gyro;
axises my_accel;
axises my_mag;
float Humidity, Pressure, Temperature;
float roll, pitch, yaw;

int place_totale;
int place_dispo;

int newdata;
int update_arinc;
int ecran_type;
int last_ecran_type;


char mess[200];
char buffer_ecriture_ecran[50];

//gestion de l'envoi sur carteSD
uint16_t session_num = 0;
char log_filename[20];
char session_buf[10]={0};



void myprintf(const char *fmt, ...);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // ----------------- SEQUENCE D'INITIALISATION DES CAPTEURS ---------------------------

  int status_ecran = SSD1306_Init ();// demarrer ecran
    if (status_ecran==0){
  	  Error_Handler();
    }

  int status = BME280_Config(OSRS_2, OSRS_16, OSRS_OFF, MODE_NORMAL, T_SB_0p5, IIR_16); // demarrage bmp280
  if (status!= 0)
    {
  	  Error_Handler();
    }

  icm20948_init(); // initialiser gyro et accelero
  ak09916_init(); // initialiser magneto



  SSD1306_Clear();
  SSD1306_GotoXY (0,10); // va a la pos (0,10)
  SSD1306_Puts ("Initialisation en cours !", &Font_5x7, 1);
  SSD1306_GotoXY (0, 20);
  SSD1306_Puts ("Banc de test avionique.", &Font_5x7, 1);
  SSD1306_UpdateScreen(); // update screen



  HAL_Delay(3000); // delay pour calmer le jeu, surtout pour la carte sd

  // ----------------- FIN SEQUENCE D'INITIALISATION DES CAPTEURS ---------------------------


  // -------------------------- TEST SPI ------------------------------

  myprintf("TEST SPI EN COURS ...\r\n");

  SSD1306_GotoXY (0,40); // va a la pos (0,10)
  SSD1306_Puts ("TEST SPI EN COURS !", &Font_5x7, 1);
  SSD1306_UpdateScreen(); // update screen
  HAL_Delay(1000);


  //Force CS high
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  // Envoie 10 octets 0xFF et affiche ce qu'on reçoit
  uint8_t tx = 0xFF, rx = 0x00;
  myprintf("Dummy bytes received: ");
  for(int i = 0; i < 10; i++) {
	  HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100);
	  myprintf("%02X ", rx);
  }
  myprintf("\r\n");

  // Force CS low et envoie CMD0
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  uint8_t cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
  uint8_t resp[7] = {0};
  HAL_SPI_Transmit(&hspi1, cmd0, 6, 100);
  // Lire 7 octets de réponse
  for(int i = 0; i < 7; i++) {
	  HAL_SPI_TransmitReceive(&hspi1, &tx, &resp[i], 1, 100);
	  myprintf("resp[%d] = %02X\r\n", i, resp[i]);
  }
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  SSD1306_GotoXY (0,50); // va a la pos (0,50)
  SSD1306_Puts ("OK !", &Font_5x7, 1);
  SSD1306_UpdateScreen(); // update screen
  HAL_Delay(1000);

  SSD1306_GotoXY (0,50); // on fait ca supprimer le "ok" avant la routine d'init de la carte SD
  SSD1306_Puts ("     ", &Font_5x7, 1);

  // --------------------------- FIN DU TEST SPI ------------------------------

  // --------------------------- GESTION CARTE SD ------------------------------------


  myprintf("\r\n~ GESTION CARTE SD ~\r\n\r\n");

  SSD1306_GotoXY (0,40); // va a la pos (0,10)
  SSD1306_Puts ("TEST CARTE SD EN COURS !", &Font_5x7, 1);
  SSD1306_UpdateScreen(); // update screen
  HAL_Delay(1000);

  //some variables for FatFs
    FATFS FatFs; 	//Fatfs handle
    FIL fil; 		//File handle
    FRESULT fres; //Result after operations


  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
	  myprintf("f_mount error (%i)\r\n", fres);
	  while(1);
  }

  //   //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;
  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
	  myprintf("f_getfree error (%i)\r\n", fres);
	  while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  place_totale=total_sectors/2;
  place_dispo=free_sectors/2;

  //Now let's try to open file "write.txt"
  fres = f_open(&fil, "write.txt", FA_READ);
  if (fres != FR_OK) {
	  myprintf("f_open error (%i)\r\n", fres);
	  while(1);
  }
  myprintf("I was able to open 'write.txt' for reading!\r\n");

  //Read 30 bytes from "write.txt" on the SD card
  BYTE readBuf[30];

  //We can either use f_read OR f_gets to get data out of files
  //f_gets is a wrapper on f_read that does some string formatting for us
  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
  if(rres != 0) {
	  myprintf("Read string from 'write.txt' contents: %s\r\n", readBuf);
  } else {
	  myprintf("f_gets error (%i)\r\n", fres);
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  //Now let's try and write a file "testf.txt"
  fres = f_open(&fil, "testf.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres == FR_OK) {
	  myprintf("I was able to open 'testf.txt' for writing\r\n");
  } else {
	  myprintf("f_open error (%i)\r\n", fres);
  }

  //Copy in a string
  strncpy((char*)readBuf, "ca marche tres bien hugo cuck", 30);
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 30, &bytesWrote);
  if(fres == FR_OK) {
	  myprintf("Wrote %i bytes to 'testf.txt'!\r\n", bytesWrote);
  } else {
	  myprintf("f_write error (%i)\r\n", fres);
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  //We're done, so de-mount the drive
  f_mount(NULL, "", 0);

  // -------------- ICI nous allons créer un fichier de gestion de version pour les fichiers de données

  FIL fil_session;

  f_mount(&FatFs, "", 1);

  //ouvre le fichier et chope le numéro sur lequel on est
  fres = f_open(&fil_session, "session.txt", FA_READ);
  if (fres == FR_OK) {
      f_gets(session_buf, sizeof(session_buf), &fil_session);
      session_num = (uint16_t)atoi(session_buf);
      f_close(&fil_session);
  }

  // incrémente et ecriture de la nouvelle valeur dans le fichier
  session_num++;
  fres = f_open(&fil_session, "session.txt", FA_WRITE | FA_CREATE_ALWAYS);
  if (fres == FR_OK) {
      f_printf(&fil_session, "%u", session_num);
      f_close(&fil_session);
  }

  // construction du fichier de log avec le numero de session actuel
  snprintf(log_filename, sizeof(log_filename), "log_%03u.csv", session_num);

  // ecriture de l'entete du fichier
  fres = f_open(&fil, log_filename, FA_WRITE | FA_CREATE_ALWAYS);
  if (fres == FR_OK) {
	  f_printf(&fil, "roll,pitch,yaw,temperature,pressure\n");
	  f_close(&fil);
  }

  f_mount(NULL, "", 0);



  SSD1306_GotoXY(0,50); // va a la pos (0,50)
  SSD1306_Puts("OK !", &Font_5x7, 1);
  SSD1306_UpdateScreen(); // update screen
  HAL_Delay(1000);

  SSD1306_Clear();
  SSD1306_GotoXY(0,0); // va a la pos (0,0)
  SSD1306_Puts("STATS CARTE SD :", &Font_5x7, 1);

  SSD1306_GotoXY(0,20); // va a la pos (0,20)
  sprintf(buffer_ecriture_ecran, "TOTAL BITS : %d bits",place_totale);
  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1);

  SSD1306_GotoXY(0,30); // va a la pos (0,30)
  sprintf(buffer_ecriture_ecran, "BITS DISPO: %d bits ",place_dispo);
  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1);

  SSD1306_UpdateScreen(); // update screen
  HAL_Delay(5000);



  // ---------------------------  FIN GESTION CARTE SD ------------------------------------





  SSD1306_Clear();
  HAL_TIM_Base_Start_IT(&htim2); //démarrage du timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  // calcul des angles pitch et roll et yaw avec la trigo
	  // marche mais le capteur doit etre initialisé dans le bon sens@	@
	  roll = atan2(my_accel.y, my_accel.z) * 180.0 / M_PI;
	  pitch = atan2(my_accel.x, my_accel.z) * 180.0 / M_PI;
	  yaw = atan2(my_mag.y, my_mag.x) * 180.0 / M_PI;


	  if(newdata == 1){

		  // -------------------- ENVOI DES DONNES --------------------------------
		  int len = snprintf(mess, sizeof(mess), "Angles -> Roll: %.2f° | Pitch: %.2f° | Yaw: %.2f°\r\n BMP280-> Temp: %.2f | Pressure: %.2f \r\n", roll, pitch, yaw, Temperature, Pressure);
		  HAL_UART_Transmit(&huart2, (uint8_t*)mess, len, 100);


		  f_mount(&FatFs, "", 1);
		  f_open(&fil, log_filename, FA_WRITE | FA_OPEN_ALWAYS);
		  f_lseek(&fil, f_size(&fil)); // aller à la fin du fichier

		  len = snprintf(mess, sizeof(mess), "%.2f,%.2f,%.2f,%.2f,%.2f\n",roll, pitch, yaw, Temperature, Pressure);
		  UINT bw;
		  f_write(&fil, mess, len, &bw);
		  f_close(&fil);
		  f_mount(NULL, "", 0);





		  // -------------------- AFFICHAGE DES DONNES SUR ECRAN ----------------------------
		  int page_changed = (ecran_type != last_ecran_type);
		  if(page_changed) {
		      SSD1306_Clear();
		      last_ecran_type = ecran_type;
		  }

		  switch(ecran_type)
		  {
		  case 0:
			  SSD1306_GotoXY(0,0); // goto 0, 0
			  SSD1306_Puts("CENTRALE INERTIELLE", &Font_7x10, 1);

			  SSD1306_GotoXY(0,20); // goto 0, 10
			  sprintf(buffer_ecriture_ecran, "ROLL: %.2f ", roll); // on stocke la donnée dans un buffer car la fonction attend un str en argument
			  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1); // afficher angle

			  SSD1306_GotoXY(0, 30);
			  sprintf(buffer_ecriture_ecran, "PITCH: %.2f", pitch);
			  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1);

			  SSD1306_GotoXY(0, 40);
			  sprintf(buffer_ecriture_ecran, "YAW: %.2f ", yaw);
			  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1);

			  SSD1306_GotoXY(0, 50);
			  SSD1306_Puts("Cliquez pour + d'infos ", &Font_5x7, 1);

			  SSD1306_UpdateScreen(); // update screen
			  break;

		  case 1 :
			  SSD1306_GotoXY(0,0); // va a la pos (0,0)
			  SSD1306_Puts("VALEURS BMP 280 :", &Font_5x7, 1);

			  SSD1306_GotoXY(0,20); // va a la pos (0,20)
			  sprintf(buffer_ecriture_ecran, "Temp : %.1f degC",Temperature);
			  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1);

			  SSD1306_GotoXY(0,30); // va a la pos (0,30)
			  sprintf(buffer_ecriture_ecran, "Press: %.2f hPa ",Pressure/100);
			  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1);

			  SSD1306_UpdateScreen(); // update screen
			  break;

		  case 2 :
			  SSD1306_GotoXY(0,0); // va a la pos (0,0)
			  SSD1306_Puts("STATS CARTE SD :", &Font_5x7, 1);

			  SSD1306_GotoXY(0,20); // va a la pos (0,20)
			  sprintf(buffer_ecriture_ecran, "TOTAL BITS : %d bits",place_totale);
			  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1);

			  SSD1306_GotoXY(0,30); // va a la pos (0,30)
			  sprintf(buffer_ecriture_ecran, "BITS DISPO: %d bits ",place_dispo);
			  SSD1306_Puts(buffer_ecriture_ecran, &Font_5x7, 1);

			  SSD1306_UpdateScreen(); // update screen
			  break;

		  }

		  newdata = 0;







		  /* USER CODE END WHILE */

		  /* USER CODE BEGIN 3 */
	  }
	  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void){

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0060112F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin PB4 */
  GPIO_InitStruct.Pin = LD3_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
