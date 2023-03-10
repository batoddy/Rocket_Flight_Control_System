/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "stdio.h"
#include "BMP180.h"
#include "stdlib.h"
#include "time.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void dataFilter(float data[9],float filteredData);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BMP180_Start();
_config();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float altitudeRaw[9]={};
	float accelRaw[3]={};
	float accelZ[9]={};
	int i,j,t;
	int tick = 10;
	int altCounter,veloCounter;
	float fltrAlt,fltrAccelZ1=0,fltrAccelZ2=0;
	float velocity_,area;
	float Altitude[10];
	float Velocity[10];

	clock_t before,difference;
	int msec=0;
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	altCounter=0;
	veloCounter=0;
	for(i=0;i<10;i++){ // tetikleyici parametrenin do??ruluk y??zdesi ??l????lmek i??in veriler 10 elemanl?? diziler halinde tutulur.
	fltrAccelZ2 = fltrAccelZ1; //son veri de??i??tirilmeden ??nce kaybedilmemek i??in kaydediliyor.
	for(j=0;j<9;j++){ //filtre her 9 say??da 1 filtrelenmi?? veri ??retti??i i??in veriler 9 elemanl?? dizide saklan??r.
		bno055_get_Accel_XYZ(accelRaw[3]);
		altitudeRaw[j] = BMP180_GetAlt(0);
		accelRawZ[j] = accelRaw[2]; // yaln??zca dikey h??za bak??ld?????? i??in sadece z ekseninin ivmesi tutulur.
		before = clock();
		do{
			clock_t difference = clock() - before;		//her veri 10 ms'de bir ??ekilir, filtrelenmi?? her veri 9 veride bir elde edildi??i i??in
			msec = difference * 1000 / CLOCKS_PER_SEC;  // toplam 90 ms'de bir sens??r verisi elde edilir.
		}while(msec < tick);
	}

	dataFilter(altitudeRaw,&fltrAlt);   // imu verileri ??nce medyan sonra ise hareketli ortalama filtrelerinden ge??irilerek
	dataFilter(accelZ,&fltrAccelZ1); //  ??nce ani ve abart??l?? g??r??lt??lerden ar??nd??r??l??r sonra ise keskin k??r??lmalar yumu??at??l??r.
	area = ((fltrAccelZ1+fltrAccelZ2)/2)*10000; //ivme verisinin integrali al??narak h??z verisine ula????l??r.
	velocity_ += area;
	Altitude[i] = fltrAlt;
	Velocity[i] = velocity_;
	}
	for(t=0;t<9;t++){
		if(Altitude[t] > Altitude[t+1]) // bu 10 elemanl?? dizide ka?? eleman aras??nda y??kseklik d????meye ba??lam???? diye kontrol edilir. Y??kseklik azalmaya ba??lad?????? anda roket zirve noktaya ula??m????
			altCounter++;				// ve geri d??????yor anlam??na gelmektedir.
		if(Velocity[t+1]<0) // dikey h??z??n y??n?? de??i??ti??i zaman (-) de??er ald?????? zaman roket zirve noktaya ula??m???? ve geri d????meye ba??lam????t??r anlam??na gelmektedir.
			veloCounter++;  // hem veride g??r??lt?? riski olabilece??inden hem de 90ms'de bir ??l????m yap??labildi??i i??in tetikleme parametresi h??z??n 0 olamas?? de??il - de??er almas?? al??nm????t??r.
	}
	if(altCounter>=7 && veloCounter>=7){ // sens??rlerde %77 do??ruluk sa??land?????? anda para????t a????l??r. (her ne kadar veri filtrelenmi?? olsa da verilerde yine de g??r??lt?? ????kmas?? durumunda i??i garantiye almak i??in eklenmi?? bir do??ruluk ??l??eridir.)
		/*--------------------------------Para????t?? A??--------------------------------*/
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void dataFilter(float data[9],float *filteredData){
	int i,j;
	float dummy;
	float dumArry[3];
	for (i=0;i<3;i++){
		for(j=0;j<3;j++){
			if (data[i]>data[j]){
				dummy =  data[i];
				data[i] = data[j];
				data[j] = dummy;
			}
			if (data[i+3]>data[j+3]){
			  dummy =  data[i];
			  data[i+3] = data[j+3];
			  data[j+3] = dummy;
			}
			if (data[i+6]>data[j+6]){
			  dummy =  data[i];
			  data[i+6] = data[j+6];
			  data[j+6] = dummy;
			}
		}
	}
	*filteredData = (data[1]+data[4]+data[7])/3;
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
