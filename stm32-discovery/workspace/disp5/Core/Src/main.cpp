/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "dma2d.h"
#include "ltdc.h"
#include "quadspi.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "lidar_driver.h"
#include "lidar.h"

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

/* USER CODE BEGIN PV */
volatile bool received_flag = false;
volatile bool parseData = false;
uint8_t buff[3*BLOCK_SIZE] = {0};
lidar_map map;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//DMA data-packet interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	static uint8_t k = 0;
	static uint8_t previous_k = 2;
	static uint8_t next_k = 1;

	if(parseData)
	{
		//Set up DMA to receive next packet.
		HAL_UART_Receive_DMA(&huart6, buff + next_k * BLOCK_SIZE, BLOCK_SIZE);


		//Calculate start angles of current and last packet, compare them -> calculate step, convert to Q16
		int current_start_angle_q8 = (((uint16_t)(*(buff + k * BLOCK_SIZE + ANGLE_OFFSET_14_8)) & 0x7f) << 10);
		current_start_angle_q8 |= (*(buff + k * BLOCK_SIZE + ANGLE_OFFSET_07_0)) << 2;
		int previous_start_angle_q8 = (((uint16_t)(*(buff + previous_k * BLOCK_SIZE + ANGLE_OFFSET_14_8)) & 0x7f) << 10);
		previous_start_angle_q8 |= (*(buff + previous_k * BLOCK_SIZE + ANGLE_OFFSET_07_0)) << 2;

		int start_angle_diff_q8 = current_start_angle_q8 - previous_start_angle_q8;
		if(previous_start_angle_q8 > current_start_angle_q8) start_angle_diff_q8 += (360 << 8);
		int angle_step_q16 = (start_angle_diff_q8 << 3) / 3;
		int current_raw_angle_q16 = (previous_start_angle_q8 << 8);


		if(previous_start_angle_q8 > 0 &&  map.recv_status != RECV_STATUS_SYNCED_TWICE_MAP_READY)
		{
			for(uint8_t i = 0; i < ULTRA_CABINS_IN_RESPONSE ; i++)
			{
				//Each ultra_cabin consists of data for three points.
				int dist_q2[3];
				int angle_q6[3];
				int syncBit[3];

				//Get one ultra-cabin FROM PREVIOUS BUFFER and save it as uint32_t
				uint32_t combined = (*((uint32_t*)(buff + previous_k * BLOCK_SIZE + ULTRA_CABIN_0_OFFSET + CABIN_SIZE * i)));

				//"Magic shift", 'DO NOT TOUCH', from SDK.
				int dist_major = (combined & 0xfff);
				int dist_predict1 = (((int)(combined << 10)) >> 22);
				int dist_predict2 = (((int)combined) >> 22);

				int dist_major2;
				uint32_t scale1, scale2;

				//Fetch data from next ultra_cabin, from the right packet.
				if(i == ULTRA_CABINS_IN_RESPONSE - 1){
					//last ultra cabin in packet, get data from first ultra_cabin in "current" buffer
					dist_major2 = (*((uint32_t*)(buff + k * BLOCK_SIZE + ULTRA_CABIN_0_OFFSET + CABIN_SIZE * 0))) & 0xfff;
				}else{
					dist_major2 = (*((uint32_t*)(buff + previous_k * BLOCK_SIZE + ULTRA_CABIN_0_OFFSET + CABIN_SIZE * (i + 1)))) & 0xfff;
				}


				//### SDK MATH ###
				dist_major = _varbitscale_decode(dist_major, scale1);
				dist_major2 = _varbitscale_decode(dist_major2, scale2);

				int dist_base1 = dist_major;
				int dist_base2 = dist_major2;

				if((!dist_major) && dist_major2){
					dist_base1 = dist_major2;
					scale1 = scale2;
				}


				//Fetch data for THREE points
				dist_q2[0] = (dist_major << 2);

				//For -Wsign-compare warnings: The following line was copied from SDK and probably should not be changed
				if((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)){
					dist_q2[1] = 0;
				} else {
					dist_predict1 = (dist_predict1 << scale1);
					dist_q2[1] = (dist_predict1 + dist_base1) << 2;
				}

				//For -Wsign-compare warnings: The following line was copied from SDK and probably should not be changed
				if((dist_predict1 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)){
					dist_q2[2] = 0;
				} else {
					dist_predict2 =  (dist_predict2 << scale2);
					dist_q2[2] = (dist_predict2 + dist_base2) << 2;
				}

				//## ENDOF SDK MATH ###
				for(int j = 0; j < 3; j++)
				{

					syncBit[j] = (((current_raw_angle_q16 + angle_step_q16) % (360 << 16)) < angle_step_q16) ? 1 : 0;

					//Check if a full 360 map is already saved (two syncs <=> twice passed 0 angle)
					if(syncBit[j]){
						if(map.recv_status == RECV_STATUS_NOT_SYNCED){
							map.recv_status = RECV_STATUS_SYNCED_ONCE;

						}else if(map.recv_status == RECV_STATUS_SYNCED_ONCE){
							map.recv_status = RECV_STATUS_SYNCED_TWICE_MAP_READY;
							break;
						}
					}

					//### SDK MATH ###
					int offset_angle_mean_q16 = (int)(7.5 * 3.1415926535 * (1<<16) / 180.0);

					if(dist_q2[j] >= (50*4)){
						const int k1 = 98361;
						const int k2 = (int)(k1/dist_q2[j]);
						offset_angle_mean_q16 = (int)((8 * 3.1415926535 * (1<<16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304);
					}

					angle_q6[j] = ((current_raw_angle_q16 - int(offset_angle_mean_q16 * 180 / 3.14159265)) >> 10);
					current_raw_angle_q16 += angle_step_q16;

					if(angle_q6[j] < 0) angle_q6[j] += (360 << 6);
					else if(angle_q6[j] >= (360 << 6)) angle_q6[j] -= (360 << 6);

					//### ENDOF SDK MATH ###

					float f_angle = ((float) angle_q6[j])/ 64.0f;
					float f_distance = ((float) dist_q2[j]) / 4.0f;

					uint16_t quality = dist_q2[j] ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;

					if(f_angle >1.0 && f_angle < 360.0 && quality != 0  && map.cnt < POINTS  && map.recv_status == RECV_STATUS_SYNCED_ONCE)
					{

					  f_angle = 180.0 - f_angle;
					  if(f_angle < 0) f_angle += 360.0;

					  //Add received data to the arrays
					  map.angles[map.cnt] = f_angle;
					  map.distances[map.cnt] = f_distance;

					  //If this is the greatest distance, save it
					  if(f_distance > map.dmax){
						  map.dmax = f_distance;
						  map.amax = f_angle;
					  }
					  else if(f_distance < map.dmin && f_distance > 0.0){
						  map.dmin = f_distance;
						  map.amin = f_angle;
					  }
					  map.cnt++;
					}
				}
				if(map.recv_status == RECV_STATUS_SYNCED_TWICE_MAP_READY) break;
			}
		}
		k++;
		previous_k++;
		next_k++;
		if(k >= 3) k = 0;
		if(previous_k >= 3) previous_k = 0;
		if(next_k >= 3) next_k = 0;
	}
	//In main, this flag is used only once.
	//Can be used to indicate that DMA has finished receiving data.
	received_flag = true;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  //Initialize LCD layers (in separate 4MB memory sectors)
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS + 1024 * 1024 * 4);
  BSP_LCD_DisplayOn();
  BSP_LCD_SelectLayer(0);

  //Pulse 500, Period 1000. Duty 50% by default
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

  //DMA transmits 2-byte RESET command over UART6
  HAL_UART_Transmit_DMA(&huart6, (uint8_t*) rplidar_reset, 2);

  //Clear both layers and wait  for the LIDAR to boot
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(0);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_Clear(0);
  HAL_Delay(1200);

  //Send the ultra capsuled mode scan request over DMA
  HAL_UART_Transmit_DMA(&huart6, (uint8_t*) rplidar_scan_ultra, 9);

  //Clear UART receiving flags
  __HAL_UART_CLEAR_IT(&huart6, UART_CLEAR_NEF|UART_CLEAR_OREF);

  //Set DMA to receive 8-byte response descriptor, do parse it(parseData==false)
  HAL_UART_Receive_DMA(&huart6, buff, 8);

  //Wait until the descriptor is received.
  while(!received_flag);

  //With parseData flag on, the DMA "received" interrupt from now will now parse the data packets
  //received. The interrupt will also set DMA to pend for another 132b block, continuously.
  parseData = true;

  //Let the DMA receive blocks continuously
  HAL_UART_Receive_DMA(&huart6, buff, BLOCK_SIZE);

  //LCD layer identifier
  uint8_t k = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
		k^=1; //Swap layers. Visible layer: k^1. Edited layer: k;

		//Wait for data to be gathered...


		BSP_LCD_SetLayerVisible((k^1), ENABLE);
		HAL_Delay(5); //Reduces flickering due to layer swap
		BSP_LCD_SetLayerVisible((k), DISABLE);
		BSP_LCD_SelectLayer(k);
		BSP_LCD_Clear(k);
		draw_grid(COLOR_GRID);
		draw_point(ORIGIN_X, ORIGIN_Y, make_color(255, 255, 255), 1.0);
		while(map.recv_status != RECV_STATUS_SYNCED_TWICE_MAP_READY);
		draw_connected_cloud_from_map(map, 0, 0, 1.0, true);

		//Clear received data.
		map = lidar_map();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
