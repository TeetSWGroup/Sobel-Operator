#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "lcd.h"
#include "camera.h"
#include <math.h>
const int sobel_operator_x[3][3] = {{1,0,-1},
				    {2,0,-2},
			            {1,0,-1}};
const int sobel_operator_y[3][3] = {{1,2,1},
				    {0,0,0},
				    {-1,-2,-1}};
uint8_t Sobel_operation_x(uint8_t top_left,uint8_t top,uint8_t top_right,uint8_t left,uint8_t mid,uint8_t right,uint8_t bottom_left,uint8_t bottom,uint8_t bottom_right){
	uint8_t output = top_left * sobel_operator_x[0][0]+top * sobel_operator_x[0][1]+top_right * sobel_operator_x[0][2]+left * sobel_operator_x[1][0]+mid * sobel_operator_x[1][1]+right * sobel_operator_x[1][2]+bottom_left * sobel_operator_x[2][0]+bottom * sobel_operator_x[2][1]+bottom_right * sobel_operator_x[2][2];
	return output;
}
uint8_t Sobel_operation_y(uint8_t top_left,uint8_t top,uint8_t top_right,uint8_t left,uint8_t mid,uint8_t right,uint8_t bottom_left,uint8_t bottom,uint8_t bottom_right){
	uint8_t output = top_left * sobel_operator_y[0][0]+top * sobel_operator_y[0][1]+top_right * sobel_operator_y[0][2]+left * sobel_operator_y[1][0]+mid * sobel_operator_y[1][1]+right * sobel_operator_y[1][2]+bottom_left * sobel_operator_y[2][0]+bottom * sobel_operator_y[2][1]+bottom_right * sobel_operator_y[2][2];
	return output;
}
void SystemClock_Config(void);
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();     //Initialize most GPIOs on board
  MX_DMA_Init();      //Initialize DMA
  tft_init(0, WHITE, BLACK, RED,BLUE);
  CAM_Init();
  uint8_t grey_image[CAM_FrameHeight()][CAM_FrameWidth()];
  uint16_t rgb_image[CAM_FrameHeight()][CAM_FrameWidth()];
  uint8_t grey_image_sobel_x[CAM_FrameHeight()-2][CAM_FrameWidth()-2];
  uint8_t grey_image_sobel_y[CAM_FrameHeight()-2][CAM_FrameWidth()-2];
  uint8_t grey_image_sobel[CAM_FrameHeight()-2][CAM_FrameWidth()-2];
  uint8_t median_image[CAM_FrameHeight()][CAM_FrameWidth()];

  uint8_t find_median_in3x3(uint8_t matrix[3][3])
  {
	  uint8_t temp[9];
	  uint8_t temp_num=0;
	  for (uint8_t i=0; i<3; i++)
	  {
		  temp[i]=matrix[0][i];
		  temp[i+3]=matrix[1][i];
		  temp[i+6]=matrix[2][i];
	  }
	  for(uint8_t j=0; j<8; j++)
	  {
		  if(temp[j]>temp[j+1]){temp_num=temp[j];temp[j]=temp[j+1];temp[j+1]=temp_num;}
	  }
	  return temp[4];
  }

  uint8_t median_filter(uint8_t grey_image[][CAM_FrameWidth()-2])
  {
	  uint8_t temp[3][3];
	  for (uint8_t y=0; y<CAM_FrameHeight();y++)
	  {
		  for(uint8_t x=0; x<CAM_FrameWidth();x++)
		  {
			   for(uint8_t i=0; i<3; i++)
			   {
				   for(uint8_t j=0; j<3; j++)
				   {
					   temp[i][j]=grey_image[y-1+i][x-1+j];
				   }
			   }
			   median_image[y][x]=find_median_in3x3(temp);
		  }
	  }
  }

  while(1) {
	  if (CAM_FrameReady()==1){
	  CAM_GetGrayscale(grey_image);
	  }
	  for(uint8_t y= 0;(y < CAM_FrameHeight()-2);y++){
		  for(uint8_t x = 0; (x < CAM_FrameWidth() - 2); x++){
			  grey_image_sobel_x[y][x] = Sobel_operation_x(grey_image[y][x],grey_image[y][x+1],grey_image[y][x+2],grey_image[y+1][x],grey_image[y+1][x+1],grey_image[y+1][x+2],grey_image[y+2][x],grey_image[y+2][x+1],grey_image[y+2][x+2]);
			  grey_image_sobel_y[y][x] = Sobel_operation_y(grey_image[y][x],grey_image[y][x+1],grey_image[y][x+2],grey_image[y+1][x],grey_image[y+1][x+1],grey_image[y+1][x+2],grey_image[y+2][x],grey_image[y+2][x+1],grey_image[y+2][x+2]);
			  grey_image_sobel[y][x] = sqrt(grey_image_sobel_x[y][x]*grey_image_sobel_x[y][x]+grey_image_sobel_y[y][x]*grey_image_sobel_y[y][x]);
			  if(grey_image_sobel[y][x]>123){
				  grey_image_sobel[y][x] = 255;
			  }
			  else{
				  grey_image_sobel[y][x] = 0;
			  }
		  }
	  }
	  CAM_GreyToRGB565(grey_image_sobel, rgb_image);
	  median_filter(grey_image_sobel);
	  tft_print_image(median_image, 0, 0, CAM_FrameWidth()-2, CAM_FrameHeight()-2);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {}
