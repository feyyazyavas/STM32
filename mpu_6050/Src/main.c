#include "main.h"
#include "math.h"

//register address
#define mpu6050Address 0xD0
#define pwr_mgmt_1 0x6B

//gyro - accel LSB sensitivity and pi number
#define gyroSens 131
#define accelSens 16384
#define PI 3.14159265359f

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

uint8_t i2cBuf[8];
uint32_t timer, previousTime;	
float passingTime;
float roll, pitch;

//gyro variables
int gyro_error=0;
int16_t gyro_X_out, gyro_Y_out, gyro_Z_out;
float gyro_X, gyro_Y, gyro_Z;
float gyro_angle_X, gyro_angle_Y, gyro_angle_Z;
float gyro_error_X, gyro_error_Y, gyro_error_Z;

//acc variables
int acc_error=0;
int16_t acc_X_out, acc_Y_out, acc_Z_out;
float acc_X, acc_Y, acc_Z;
float acc_angle_X, acc_angle_Y;
float acc_error_X, acc_error_Y;

//temp variables
int16_t temp_out;
float temp;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

	for(uint8_t i=0; i<255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) == HAL_OK)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		}
	}
	
	//mpu6050 wake up
	i2cBuf[0]=0x6B;
	i2cBuf[1]=0x00;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuf, 2, 10);
	
	//mpu6050 gyroscope config	(+/- 250 degree/second)
	i2cBuf[0]=0x1B;
	i2cBuf[1]=0x00;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuf, 2, 10);
	
	//mpu6050 accelerometer config (+/- 2g)
	i2cBuf[0]=0x1C;
	i2cBuf[1]=0x00;
	HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuf, 2, 10);
	
	timer = HAL_GetTick();
	
	//calculate gyro data error
	if(gyro_error==0)
	{
		for(int i=0; i<200; i++)
		{
			//gyroscope read 
			i2cBuf[0]=0x43;
			HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuf, 1, 10);
		
			i2cBuf[1]=0x00;
			HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuf[1], 6, 10);
		
			gyro_X = (i2cBuf[1]<<8 | i2cBuf[2]);
			gyro_Y = (i2cBuf[3]<<8 | i2cBuf[4]);
			gyro_Z = (i2cBuf[5]<<8 | i2cBuf[6]);
			
			//gyroscope error measurements
			gyro_error_X = gyro_error_X + ((float)gyro_X / gyroSens);
			gyro_error_Y = gyro_error_Y + ((float)gyro_Y / gyroSens);
			gyro_error_Z = gyro_error_Z + ((float)gyro_Z / gyroSens);
			
			if(i==199)
			{
				gyro_error_X = gyro_error_X / 200;
				gyro_error_Y = gyro_error_Y / 200;
				gyro_error_Z = gyro_error_Z / 200;
				gyro_error = 1;
			}
		}
	}
	
	//calculate acc data error
	if(acc_error==0)
	{
		for(int j=0; j<200; j++)
		{
			//accelerometer read
			i2cBuf[0]=0x3B;
			HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuf, 1, 10);
		
			i2cBuf[1]=0x00;
			HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuf[1], 6, 10);
			
			acc_X_out = (i2cBuf[1]<<8 | i2cBuf[2]);
			acc_Y_out = (i2cBuf[3]<<8 | i2cBuf[4]);
			acc_Z_out = (i2cBuf[5]<<8 | i2cBuf[6]);
			
			//accelerometer error measurements
			acc_X = (float)acc_X_out / accelSens;
			acc_Y = (float)acc_Y_out / accelSens;
			acc_Z = (float)acc_Z_out / accelSens;
			
			acc_error_X = acc_error_X + ((atan((acc_Y) / sqrt(pow((acc_X),2) + pow((acc_Z),2))) * 180 / PI));
			acc_error_Y = acc_error_Y + ((atan(-1 * (acc_X) / sqrt(pow((acc_Y),2) + pow((acc_Z),2))) * 180 / PI));
			
			if(j==199)
			{
				acc_error_X = acc_error_X / 200;
				acc_error_Y = acc_error_Y / 200;
				acc_error = 1;
			}
		}
	}
  while (1)
  {
		/*tempreture read*/
		i2cBuf[0]=0x41;
		HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuf, 1, 10);
		
		i2cBuf[1]=0x00;
		HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuf[1], 2, 10);
		
		temp_out = (i2cBuf[1]<<8 | i2cBuf[2]);
		temp = (float)(temp_out/340 + 36.53f);
		
		
		previousTime = timer;
		timer = HAL_GetTick();
		passingTime = ((float)timer - (float)previousTime)/1000;
		
		
		/*gyroscope read*/
		i2cBuf[0]=0x43;
		HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuf, 1, 10);
		
		i2cBuf[1]=0x00;
		HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuf[1], 6, 10);
		
		gyro_X_out = (i2cBuf[1]<<8 | i2cBuf[2]);
		gyro_Y_out = (i2cBuf[3]<<8 | i2cBuf[4]);
		gyro_Z_out = (i2cBuf[5]<<8 | i2cBuf[6]);
		
		//gyroscope measurements
		gyro_X = ((float)gyro_X_out / gyroSens) - gyro_error_X;
		gyro_Y = ((float)gyro_Y_out / gyroSens) - gyro_error_Y;
		gyro_Z = ((float)gyro_Z_out / gyroSens) - gyro_error_Z;
		
		gyro_angle_X = gyro_X * passingTime;
		gyro_angle_Y = gyro_Y * passingTime;
		
		
		
		/*accelerometer read*/
		i2cBuf[0]=0x3B;
		HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuf, 1, 10);
		
		i2cBuf[1]=0x00;
		HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuf[1], 6, 10);
		
		acc_X_out = (i2cBuf[1]<<8 | i2cBuf[2]);
		acc_Y_out = (i2cBuf[3]<<8 | i2cBuf[4]);
		acc_Z_out = (i2cBuf[5]<<8 | i2cBuf[6]);
		
		//accelerometer measurements
		acc_X = (float)acc_X_out / accelSens;
		acc_Y = (float)acc_Y_out / accelSens;
		acc_Z = (float)acc_Z_out / accelSens;
		
		acc_angle_X = (atan((acc_Y) / sqrt(pow((acc_X),2) + pow((acc_Z),2))) * 180 / PI) - acc_error_X;
		acc_angle_Y = (atan(-1 * (acc_X) / sqrt(pow((acc_Y),2) + pow((acc_Z),2))) * 180 / PI) - acc_error_Y;
		
		roll = (0.9f * (roll + gyro_angle_X) + 0.1f * acc_angle_X);
		pitch = (0.9f * (pitch + gyro_angle_Y) + 0.1f * acc_angle_Y);
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{

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
