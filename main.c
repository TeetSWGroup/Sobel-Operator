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

uint16_t servo_location = 375;
//sobel values
//const int sobel_operator_x[3][3] = {{3,0,-3},
//				    				{10,0,-10},
//									{3,0,-3}};
//const int sobel_operator_y[3][3] = {{3,10,3},
//				    				{0,0,0},
//									{-3,-10,-3}};
//const int sobel_operator_x[3][3] = {{47,0,-47},
//				    				{162,0,-162},
//									{47,0,-47}};
//const int sobel_operator_y[3][3] = {{47,162,47},
//				    				{0,0,0},
//									{-47,-162,-47}};
//const int sobel_operator_x[3][3] = {{1,0,-1},
//				    				{1,0,-1},
//									{1,0,-1}};
//const int sobel_operator_y[3][3] = {{1,1,1},
//				    				{0,0,0},
//									{-1,-1,-1}};
//const int sobel_operator_x[3][3] = {{1,0,-1},
//				    				{2,0,-2},
//									{1,0,-1}};
//const int sobel_operator_y[3][3] = {{1,2,1},
//				    				{0,0,0},
//									{-1,-2,-1}};
//const int random_kernal[3][3] = {{1,0,1},
//				    			{0,1,0},
//								{1,0,1}};
const int random_kernal[3][3] = {{1,0,1},
				    			{0,1,0},
								{1,0,1}};

//to multiply the image array by the sobel values with respect to x and y
//uint8_t Sobel_operation_x(uint8_t top_left,uint8_t top,uint8_t top_right,uint8_t left,uint8_t mid,uint8_t right,uint8_t bottom_left,uint8_t bottom,uint8_t bottom_right){
//	uint8_t output = top_left * sobel_operator_x[0][0]+top * sobel_operator_x[0][1]+top_right * sobel_operator_x[0][2]+left * sobel_operator_x[1][0]+mid * sobel_operator_x[1][1]+right * sobel_operator_x[1][2]+bottom_left * sobel_operator_x[2][0]+bottom * sobel_operator_x[2][1]+bottom_right * sobel_operator_x[2][2];
//	return output;
//}
//uint8_t Sobel_operation_y(uint8_t top_left,uint8_t top,uint8_t top_right,uint8_t left,uint8_t mid,uint8_t right,uint8_t bottom_left,uint8_t bottom,uint8_t bottom_right){
//	uint8_t output = top_left * sobel_operator_y[0][0]+top * sobel_operator_y[0][1]+top_right * sobel_operator_y[0][2]+left * sobel_operator_y[1][0]+mid * sobel_operator_y[1][1]+right * sobel_operator_y[1][2]+bottom_left * sobel_operator_y[2][0]+bottom * sobel_operator_y[2][1]+bottom_right * sobel_operator_y[2][2];
//	return output;
//}
uint8_t kernal_operation(uint8_t top_left,uint8_t top,uint8_t top_right,uint8_t left,uint8_t mid,uint8_t right,uint8_t bottom_left,uint8_t bottom,uint8_t bottom_right){
	uint8_t output = top_left * random_kernal[0][0]+top * random_kernal[0][1]+top_right * random_kernal[0][2]+left * random_kernal[1][0]+mid * random_kernal[1][1]+right * random_kernal[1][2]+bottom_left * random_kernal[2][0]+bottom * random_kernal[2][1]+bottom_right * random_kernal[2][2];
	return output;
}
//centre_position is the centre of the image array
uint8_t Left_Centre_distance(uint8_t centre_position,uint8_t image[][CAM_FrameWidth()-4]){
	for(uint8_t distance = centre_position; distance>= 0;distance--){
		if(image[59][distance]==0){
			distance = centre_position - distance;
			return distance;
		}
	}
}
uint8_t Right_Centre_distance(uint8_t centre_position,uint8_t image[][CAM_FrameWidth()-4]){
	for(uint8_t distance = centre_position; distance< CAM_FrameHeight();distance++){
		if(image[59][distance]==0){
			distance = distance = centre_position;
			return distance;
		}
	}
}
uint16_t difference_of_magenetic_sensor_values(){
//not sure about if it's ADC_Values[0][4] or not, might need to tell the hardware guys to check it
	uint16_t difference = ADC_Values[0][2]-ADC_Values[0][4];
	return difference;
}
void SystemClock_Config(void);
void main(void)
{
//inits that I don't know what are the uses for
  HAL_Init();
  SystemClock_Config();
  TIM1_PWM_Init();
  TIM5_PWM_Init();
  MX_GPIO_Init();     //Initialize most GPIOs on board
  MX_DMA_Init();
  HAL_TIM_ConfigTimer(SERVO_TIM, 287, 4999);
  HAL_TIM_ConfigTimer(MOTOR_TIM, 287, 4999);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  tft_init(0, WHITE, BLACK, RED,BLUE);
  CAM_Init();
  ADC_Start();
//  HAL_TIM_ConfigTimer(&htim5, 287, 4999);
//  HAL_TIM_PWM_Start(SERVO_TIM, TIM_CHANNEL_1);
  uint8_t Height = CAM_FrameHeight();
  uint8_t Width = CAM_FrameWidth();
  uint8_t left_distance;
  uint8_t right_distance;
  uint16_t left_magnetic_sensor;
  uint16_t right_magnetic_sensor;
  uint8_t difference_of_magnetic_values;
//defining variables for sobel_operator and median filter
  uint8_t grey_image[Height][Width];
  uint16_t rgb_image[Height][Width];
  uint8_t grey_image_sobel_x[Height-2][Width-2];
  uint8_t grey_image_sobel_y[Height-2][Width-2];
  uint8_t grey_image_sobel[Height-2][Width-2];
  uint8_t final_image[Height-4][Width-4];
  uint8_t grey_kernal_image[Height-2][Width-2];
  CAM_SetBrightness(20);
  CAM_SetContrast(100);
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
	  for (uint8_t k=0;k<8;k++)
	  {
		 for(uint8_t j=0; j<8; j++)
		  {
			if(temp[j]>temp[j+1]){temp_num=temp[j];temp[j]=temp[j+1];temp[j+1]=temp_num;}
		  }
	  }

	  return temp[4];
  }
  uint8_t median_filter_remastered(uint8_t image[][Width-2]){
	  uint8_t temp[3][3];
	  for(uint8_t y = 1;y<(Height-3);y++){
		  for(uint8_t x = 1;x < (Width-3);x++){
			  temp[0][0]=image[y-1][x-1];
			  temp[0][1]=image[y-1][x];
			  temp[0][2]=image[y-1][x+1];
			  temp[1][0]=image[y][x-1];
			  temp[1][1]=image[y][x];
			  temp[1][2]=image[y][x+1];
			  temp[2][0]=image[y+1][x-1];
			  temp[2][1]=image[y+1][x];
			  temp[2][2]=image[y+1][x-2];
			  final_image[y-1][x-1]=find_median_in3x3(temp);
		  }
	  }
  }
  while(1) {
//set the wheels facing to the centre
	  HAL_TIM_PWM_SetCompare(SERVO_TIM, TIM_CHANNEL_1, servo_location);
//To make sure the Camera is on when capturing the grey image
	  left_magnetic_sensor = ADC_Values[0][2];
	  right_magnetic_sensor = ADC_Values[0][3];
	  if (CAM_FrameReady()==1){
	  CAM_GetGrayscale(grey_image);
	  }
	  for(uint8_t y= 1;(y < CAM_FrameHeight() - 1);y++){
		  for(uint8_t x = 1; (x < CAM_FrameWidth() - 1); x++){
//			  grey_image_sobel_x[y][x] = Sobel_operation_x(grey_image[y-1][x-1],grey_image[y-1][x],grey_image[y-1][x+1],grey_image[y][x-1],grey_image[y][x],grey_image[y][x+1],grey_image[y-1][x-1],grey_image[y-1][x],grey_image[y-1][x+1]);
//			  grey_image_sobel_y[y][x] = Sobel_operation_y(grey_image[y-1][x-1],grey_image[y-1][x],grey_image[y-1][x+1],grey_image[y][x-1],grey_image[y][x],grey_image[y][x+1],grey_image[y-1][x-1],grey_image[y-1][x],grey_image[y-1][x+1]);
//			  grey_image_sobel[y][x] = sqrt(grey_image_sobel_x[y][x]*grey_image_sobel_x[y][x]+grey_image_sobel_y[y][x]*grey_image_sobel_y[y][x]);
			  grey_kernal_image[y][x] = kernal_operation(grey_image[y-1][x-1],grey_image[y-1][x],grey_image[y-1][x+1],grey_image[y][x-1],grey_image[y][x],grey_image[y][x+1],grey_image[y-1][x-1],grey_image[y-1][x],grey_image[y-1][x+1]);
			  if(grey_kernal_image[y][x]>120){
				  grey_kernal_image[y][x] = 255;
			  }
			  else{
				  grey_kernal_image[y][x] = 0;
			  }
		  }
	  }
	  CAM_GreyToRGB565(grey_kernal_image, rgb_image);
	  tft_print_image(rgb_image, 0, 0, CAM_FrameWidth()-2, CAM_FrameHeight()-2 );
	  if(tft_update(50)==0){
		  tft_prints(0, 5, "%d",ADC_Values[0][0]);
		  tft_prints(0, 6, "%d",ADC_Values[0][1]);
		  tft_prints(0, 7, "%d",ADC_Values[0][2]);
		  tft_prints(0, 8, "%d",ADC_Values[0][3]);
	  }
	  //FInished producing image, now getting the distance of border from centre to left and right
	  left_distance=Left_Centre_distance(Width/2,final_image);
	  right_distance=Right_Centre_distance(Width/2,final_image);
	  if(abs(right_distance - left_distance)>5){
		  if(right_distance > left_distance){
			  servo_location = 575;
		  }
		  else{
			  servo_location = 225;
		  }
	  }
	  difference_of_magnetic_values = difference_of_magenetic_sensor_values();
	  if(difference_of_magnetic_values > 20){
		  if(left_magnetic_sensor > right_magnetic_sensor){
			  servo_location = 225;
		  }
		  else{
			  servo_location = 575;
		  }
	  }
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
