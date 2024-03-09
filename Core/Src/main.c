/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
//#include <audio_sd.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_CHANNELS 2U
#define AUDIO_SAMPLING_FREQUENCY 48000U
#define N_MS_PER_INTERRUPT (1U)
#define CHANNEL_DEMUX_MASK  0x55U
FRESULT res;                                          /* FatFs function common result code */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */

volatile uint8_t button_flag = 0;

char newLine[] = "\r\n";
volatile uint8_t SD_Log_Enabled = 0;
char SDPath[4]; /* SD card logical drive path */
static volatile uint8_t BUTTONInterrupt, audio_in_recording_state = 0;
int index_n=0;

volatile int  index_buff=0;
static uint16_t I2S_InternalBuffer[768]; //TODO:CHECK
uint16_t PCM_Buffer[AUDIO_CHANNELS * AUDIO_SAMPLING_FREQUENCY / 1000];
uint16_t PDM_Buffer[AUDIO_CHANNELS * AUDIO_SAMPLING_FREQUENCY / 1000 * 64 / 8];

#define WRITE_EACH 16//ms
#define SIZE_BUFF (AUDIO_SAMPLING_FREQUENCY / 1000 * AUDIO_CHANNELS * (WRITE_EACH*2))

int16_t Audio_OUT_Buff[SIZE_BUFF];
uint8_t pHeader[44];

/* PDM filters params */
static PDM_Filter_Handler_t  PDM_FilterHandler[2];
static PDM_Filter_Config_t   PDM_FilterConfig[2];

static uint8_t Channel_Demux[128] = {
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f,
  0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f
};


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void sd_card_init_new(void);
void PDMToPCMInit(void);
void myprintf(const char *fmt, ...);
void set_wav_header(void);
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

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderInit()
{
  uint16_t   BitPerSample=16;
  uint16_t   NbrChannels=AUDIO_CHANNELS;
  uint32_t   ByteRate=AUDIO_SAMPLING_FREQUENCY*(BitPerSample/8);

  uint32_t   SampleRate=AUDIO_SAMPLING_FREQUENCY;
  uint16_t   BlockAlign= NbrChannels * (BitPerSample/8);

  /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
  pHeader[0] = 'R';
  pHeader[1] = 'I';
  pHeader[2] = 'F';
  pHeader[3] = 'F';

  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
     recording opearation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = 0x00;
  pHeader[5] = 0x4C;
  pHeader[6] = 0x1D;
  pHeader[7] = 0x00;

  /* Write the file format, must be 'WAVE' -----------------------------------*/
  pHeader[8]  = 'W';
  pHeader[9]  = 'A';
  pHeader[10] = 'V';
  pHeader[11] = 'E';

  /* Write the format chunk, must be'fmt ' -----------------------------------*/
  pHeader[12]  = 'f';
  pHeader[13]  = 'm';
  pHeader[14]  = 't';
  pHeader[15]  = ' ';

  /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
  pHeader[16]  = 0x10;
  pHeader[17]  = 0x00;
  pHeader[18]  = 0x00;
  pHeader[19]  = 0x00;

  /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
  pHeader[20]  = 0x01;
  pHeader[21]  = 0x00;

  /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
  pHeader[22]  = NbrChannels;
  pHeader[23]  = 0x00;

  /* Write the Sample Rate in Hz ---------------------------------------------*/
  /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
  pHeader[24]  = (uint8_t)((SampleRate & 0xFF));
  pHeader[25]  = (uint8_t)((SampleRate >> 8) & 0xFF);
  pHeader[26]  = (uint8_t)((SampleRate >> 16) & 0xFF);
  pHeader[27]  = (uint8_t)((SampleRate >> 24) & 0xFF);

  /* Write the Byte Rate -----------------------------------------------------*/
  pHeader[28]  = (uint8_t)(( ByteRate & 0xFF));
  pHeader[29]  = (uint8_t)(( ByteRate >> 8) & 0xFF);
  pHeader[30]  = (uint8_t)(( ByteRate >> 16) & 0xFF);
  pHeader[31]  = (uint8_t)(( ByteRate >> 24) & 0xFF);

  /* Write the block alignment -----------------------------------------------*/
  pHeader[32]  = BlockAlign;
  pHeader[33]  = 0x00;

  /* Write the number of bits per sample -------------------------------------*/
  pHeader[34]  = BitPerSample;
  pHeader[35]  = 0x00;

  /* Write the Data chunk, must be 'data' ------------------------------------*/
  pHeader[36]  = 'd';
  pHeader[37]  = 'a';
  pHeader[38]  = 't';
  pHeader[39]  = 'a';

  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  pHeader[40]  = 0x00;
  pHeader[41]  = 0x4C;
  pHeader[42]  = 0x1D;
  pHeader[43]  = 0x00;

  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderUpdate(uint32_t len)
{
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
     recording opearation.  Example: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = (uint8_t)(len);
  pHeader[5] = (uint8_t)(len >> 8);
  pHeader[6] = (uint8_t)(len >> 16);
  pHeader[7] = (uint8_t)(len >> 24);
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  len -=44;
  pHeader[40] = (uint8_t)(len);
  pHeader[41] = (uint8_t)(len >> 8);
  pHeader[42] = (uint8_t)(len >> 16);
  pHeader[43] = (uint8_t)(len >> 24);
  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_Log_Enable(void)
{
  static uint16_t sdcard_file_counter = 0;

  uint32_t byteswritten; /* written byte count */
//  char file_name[30] = {0};
	static char file_name[] = "w_000.wav";
	static uint8_t file_counter = 1; //TODO: check if 10
	int file_number_digits = file_counter;
//	uint32_t byte_rate = frequency * 2 * 2;
//	wav_file_header[24] = (uint8_t)frequency;
//	wav_file_header[25] = (uint8_t)(frequency >> 8);
//	wav_file_header[26] = (uint8_t)(frequency >> 16);
//	wav_file_header[27] = (uint8_t)(frequency >> 24);
//	wav_file_header[28] = (uint8_t)byte_rate;
//	wav_file_header[29] = (uint8_t)(byte_rate >> 8);
//	wav_file_header[30] = (uint8_t)(byte_rate >> 16);
//	wav_file_header[31] = (uint8_t)(byte_rate >> 24);

	// defining a wave file name
	file_name[4] = file_number_digits%10 + 48; //48 is digit 0
	file_number_digits /= 10;
	file_name[3] = file_number_digits%10 + 48;
	file_number_digits /= 10;
	file_name[2] = file_number_digits%10 + 48;
	printf("file name %s \n", file_name);
	myprintf("file name %s \n", file_name);
	file_counter++;

  WavProcess_HeaderInit();

//  sprintf(file_name, "%s%.3d%s", "BC_Log_N", sdcard_file_counter, ".wav");
//  sdcard_file_counter++;

//  if(f_open(&MyFile, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
//  {
//	myprintf("Testing file create failed! \r\n");
//    return 0;
//  }
	// creating a file
    FRESULT fres = f_open(&MyFile ,file_name, FA_WRITE|FA_CREATE_ALWAYS);
	if(fres != 0)
	{
		myprintf("error in creating a file: %d \n", fres);
	}

  if(f_write(&MyFile, pHeader, sizeof(pHeader), (void *)&byteswritten) != FR_OK)
  {
	myprintf("Testing file header write failed! \r\n");
    return 0;
  }
  return 1;
}

/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_Log_Disable(void)
{
  uint32_t len;
  uint32_t byteswritten;                     /* File write/read counts */


  len = f_size(&MyFile);
  WavProcess_HeaderUpdate(len);

    /* Update the data length in the header of the recorded Wave */
  f_lseek(&MyFile, 0);

  /* Parse the wav file header and extract required information */

  f_write(&MyFile, pHeader, 44, (void*)&byteswritten);

  /* Close file and unmount MyFilesystem */

  f_close(&MyFile);
}

static HAL_StatusTypeDef AUDIO_IN_Timer_Start(void) {
	HAL_StatusTypeDef ret = HAL_OK;
	if (HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
		ret = HAL_ERROR;
	}
	if (HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {
		ret = HAL_ERROR;
	}
//	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {;
//		ret = HAL_ERROR;
//	}
	return ret;
}

void init_recording_state(uint8_t* pBuf, uint32_t NbrOfBytes) {
	if(AUDIO_IN_Timer_Start() != HAL_OK) {
		myprintf("Error in AUDIO IN TIMER START!!!! \r\n");
	}
	if(HAL_I2S_Receive_DMA(&hi2s2, I2S_InternalBuffer, (uint16_t)768/2U) != HAL_OK) {
		int status_for_hal = HAL_I2S_Receive_DMA(&hi2s2, I2S_InternalBuffer, (uint16_t)768/2U);
		myprintf("I2S FAILEDDD, %i\r\n", status_for_hal);
	}
	audio_in_recording_state = 0;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  MX_FATFS_Init();
  MX_PDM2PCM_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority((IRQn_Type)EXTI2_IRQn, 0xFF, 0);
   //HAL_NVIC_EnableIRQ((IRGB1_Bluebutton_EXTI_IRQn);
	HAL_NVIC_EnableIRQ((IRQn_Type)EXTI2_IRQn);
	HAL_Delay(1000);
	/*BELOW ADDED FOR SD_CARD*/
	sd_card_init_new();
	HAL_Delay(1000);
	//sd_demo();
	PDMToPCMInit();
	init_recording_state((uint8_t *) PDM_Buffer, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (button_flag) {
	 		  button_flag = 0;

	 		  if (audio_in_recording_state) {
	 			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	 			  myprintf("STOPPED RECORDING, DUMPING!!!");
	 			  audio_in_recording_state = 0;
	 			  DATALOG_SD_Log_Disable();
	 		  }
	 		  else {
	 			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	 			  myprintf("Started in Recording!!!");
	 			  DATALOG_SD_Log_Enable();
	 			  audio_in_recording_state = 1;
	 			  }
	 		  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 7;
  RCC_OscInitStruct.PLL.PLLN = 344;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLLR;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_Button_Pin */
  GPIO_InitStruct.Pin = Blue_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Blue_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sd_card_init_new(void)
{
	res = f_mount(&SDFatFs, "", 1);
	if(res != FR_OK)
	{
		myprintf("error in mounting an sd card: %d \n", res);
		while (res != FR_OK) {
			res = f_mount(&SDFatFs, "", 1);
		}
	}
	else
	{
		myprintf("succeded in mounting an sd card \n");
	}
}

/**
* @brief Throws Highest priority interrupt
* @param Noned
* @retval None
*/
void SW_Task1_Start(void)
{
	//myprintf("In SW_Task1_star \r\n");
	HAL_NVIC_SetPendingIRQ(EXTI2_IRQn);
}


/**
* @brief Highest priority interrupt handler routine
* @param None
* @retval None
*/
void SW_Task1_Callback(void)
{
  FRESULT s;
  uint32_t byteswritten;                     /* File write/read counts */


  /* Check Push Button Event  */
  if (audio_in_recording_state)
  {
    s=f_write(&MyFile, &(((uint8_t *)Audio_OUT_Buff)[index_buff]), SIZE_BUFF, (void *)&byteswritten);
    if(s != FR_OK){
    	myprintf("Error in TASK1 \r\n");
    }
    myprintf("interrupt storing \r\n");
    myprintf("%i", Audio_OUT_Buff[0]);
  }
}

void PDMToPCMInit(void) {

	uint32_t index;
	for(index = 0; index < AUDIO_CHANNELS; index++)
		{
		  volatile uint32_t error = 0;
		  /* Init PDM filters */
		  PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
		  PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
		  PDM_FilterHandler[index].high_pass_tap = 2122358088;
		  PDM_FilterHandler[index].out_ptr_channels = (uint16_t)AUDIO_CHANNELS;
		  PDM_FilterHandler[index].in_ptr_channels  = (uint16_t)AUDIO_CHANNELS;

	      /* PDM lib config phase */
	      PDM_FilterConfig[index].output_samples_number = (uint16_t) ((AUDIO_SAMPLING_FREQUENCY/1000U) * N_MS_PER_INTERRUPT);
	      PDM_FilterConfig[index].mic_gain = 24;
	      PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;

	      error = PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));
	      if (error) {
	    	  myprintf("Error in initializing PDM");
	      }
	      error = PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
	      if (error) {
	    	  myprintf("Error in setPDMConfig, %d", error);
	      }
		}
}

int32_t BSP_AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf) {
	int32_t ret = 0;
	uint32_t index;
	for(index = 0; index < AUDIO_CHANNELS; index++) {
		//(void)PDM_Filter(&((uint8_t)(PDMBuf))[index], (uint16_t)&(PCMBuf[index]), &PDM_FilterHandler[index]);
		(void)PDM_Filter(&((uint8_t*)(PDMBuf))[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
	}
	return ret;
}

void AudioProcess(void)
{
  /*for L4 PDM to PCM conversion is performed in hardware by DFSDM peripheral*/
  uint16_t index = 0;
  static uint16_t OUT_Buff_lvl = 0;
  uint32_t msTick, msTickPrev = 0;

  BSP_AUDIO_IN_PDMToPCM((uint16_t * )PDM_Buffer, (uint16_t *)&PCM_Buffer[0]);
  //myprintf("%x \r\n", PDM_Buffer[0]);


  //for(int i = 0; i < 30; i++){
  //  myprintf("\r\n******** PCM Data: %x ********", PCM_Buffer[i]);
  //}

  //for(int i = 0; i < 32; i++){
  //  myprintf("\r\n-------- PDM Data: %x --------", PDM_Buffer[i]);
  //}

  for (index = 0; index < AUDIO_SAMPLING_FREQUENCY/1000 ; index++)
  {

    Audio_OUT_Buff[OUT_Buff_lvl + 0] = PCM_Buffer[index * AUDIO_CHANNELS + 0];// + PCM_Buffer[index * AUDIO_CHANNELS + 3];
    Audio_OUT_Buff[OUT_Buff_lvl + 1] = PCM_Buffer[index * AUDIO_CHANNELS + 1];//+ PCM_Buffer[index * AUDIO_CHANNELS + 2];

//    Audio_OUT_Buff[OUT_Buff_lvl + 2] = PCM_Buffer[index * AUDIO_CHANNELS + 2];// + PCM_Buffer[index * AUDIO_CHANNELS + 3];
//    Audio_OUT_Buff[OUT_Buff_lvl + 3] = PCM_Buffer[index * AUDIO_CHANNELS + 3];//+ PCM_Buffer[index * AUDIO_CHANNELS + 2];

    OUT_Buff_lvl = (OUT_Buff_lvl + 2)%SIZE_BUFF;
  }

  if (audio_in_recording_state)
  {
    //led toggle
//    msTick = HAL_GetTick();
//    if(msTick % 20 == 0 && msTickPrev != msTick)
//    {
//      msTickPrev = msTick;
//
//      BSP_LED_Toggle(LED4);
//    }

    //for (int i = 0; i < 30; i++) {
    //  myprintf("%x \r\n", Audio_OUT_Buff[i]);
    //}
    //first half
    if(OUT_Buff_lvl == SIZE_BUFF/2)
    {
      index_buff=0;
      SW_Task1_Start();

    }
    //second half
    else if (OUT_Buff_lvl == 0)
    {
      index_buff= SIZE_BUFF;
      SW_Task1_Start();
    }
  }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	UNUSED(hi2s);
	uint32_t index;


  //for(int i = 0; i < 32; i++){
  //  myprintf("\r\n I2S Internal Buffer Content: %x full", I2S_InternalBuffer[i]);
  //}

	uint16_t * DataTempI2S = &(I2S_InternalBuffer[768/2U]);

//  for(int i = 0; i < 32; i++){
//    myprintf("\r\n I2S Internal Buffer Content: %x full", DataTempI2S[i]);
//  }

	  uint8_t a,b;
	  for(index=0; index<(768/2U); index++)
	  {
		  a = ((uint8_t *)(DataTempI2S))[(index*2U)];
		  b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];

      //myprintf("\r\nA: %x", a);
      //myprintf("\r\nB: %x", b);

		 ((uint8_t *)(PDM_Buffer))[(index*2U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] | (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
		 ((uint8_t *)(PDM_Buffer))[(index*2U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] | (Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
	  }
	  // for(int i = 0; i < sizeof(PDM_Buffer)/sizeof(PDM_Buffer[0]); i++){
	  //     myprintf("\r\n PDM Content full: %x full", PDM_Buffer[i]);
	  //  }
	  AudioProcess();
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	UNUSED(hi2s);
	uint32_t index;

//	  for(int i = 0; i < 32; i++){
//	    myprintf("\r\n I2S Internal Buffer Content: %x half", I2S_InternalBuffer[i]);
//	  }


	uint16_t * DataTempI2S = I2S_InternalBuffer;

	  uint8_t a,b;
	  for(index=0; index<384; index++)
	  {
		a = ((uint8_t *)(DataTempI2S))[(index*2U)];
		b = ((uint8_t *)(DataTempI2S))[(index*2U)+1U];
		((uint8_t *)(PDM_Buffer))[(index*2U)] = Channel_Demux[a & CHANNEL_DEMUX_MASK] |
		  (Channel_Demux[b & CHANNEL_DEMUX_MASK] << 4);;
		  ((uint8_t *)(PDM_Buffer))[(index*2U)+1U] = Channel_Demux[(a>>1) & CHANNEL_DEMUX_MASK] |
			(Channel_Demux[(b>>1) & CHANNEL_DEMUX_MASK] << 4);
	  }
	  // for(int i = 0; i < 32; i++){
	  // 	      myprintf("\r\n PDM Content full: %x half", PDM_Buffer[i]);
	  // 	   }
	  AudioProcess();
}

int _write(int file, char *ptr, int len) {

	for (int DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == Blue_Button_Pin) {
		button_flag = 1;
	}
}
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

#ifdef  USE_FULL_ASSERT
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
