/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"
#include "math.h"

#include "remote_control.h"
#include "bsp_usart.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "bsp_fric.h"
#include "BMI088driver.h"

#include "chassis.h"
#include "turret.h"
#include "pid.h"
#include "constants.h"
//#include "referee.h"

#include "time_manip.h"

#include <stdio.h>
#include <stdarg.h>
#include "string.h"

//#include <stdio.h>


//struct __FILE {int handle;/* Add whatever you need here */};
/*
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
  ITM_SendChar(ch);
  return(ch);
}
*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

const RC_ctrl_t *local_rc_ctrl;

void masterLoop(void);

void usart_printf(const char *fmt,...) {
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    //·µ»Ø×Ö·û´®³¤¶È
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

    usart1_tx_dma_enable(tx_buf, len);

}

/**
  * @brief  The application entry point.
  * @retval int
  */
	
void muestra_cosas() {
	        usart_printf(
"**********\r\n\
ch0:%d\r\n\
ch1:%d\r\n\
ch2:%d\r\n\
ch3:%d\r\n\
ch4:%d\r\n\
s1:%d\r\n\
s2:%d\r\n\
mouse_x:%d\r\n\
mouse_y:%d\r\n\
press_l:%d\r\n\
press_r:%d\r\n\
key:%d\r\n\
**********\r\n",
            local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3], local_rc_ctrl->rc.ch[4],
            local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1],
            local_rc_ctrl->mouse.x, local_rc_ctrl->mouse.y,local_rc_ctrl->mouse.z, local_rc_ctrl->mouse.press_l, local_rc_ctrl->mouse.press_r,
            local_rc_ctrl->key.v);

        HAL_Delay(1000);
}
	

uint32_t prevTick;

int main(void) {
  // Essential Setup
	//printf("Hola");
  HAL_Init();
  SystemClock_Config();

  // Lib inits
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
	MX_DMA_Init();
  //MX_SPI1_Init();
	MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  //MX_TIM10_Init();
	//BMI088_init();
  
  // Configure PWM and CAN communication
	usart1_tx_dma_init();
	can_filter_init();


  
  // Start lib processes
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	
	remote_control_init();
	fric_off();
	
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1000);	
	
  // Start remote control process
	
	//usart1_tx_dma_init();
	local_rc_ctrl = get_remote_control_point();
	
  /* Init functions */
  //chassisInit();
  //turretInit();
  //refereeInit();

  /* Run master loop function on a timer */
  //while (1) { function_with_interval(&masterLoop, NULL, M_MASTER_LOOP_INTERVAL); };
	CAN_cmd_gimbal(1000, 1000, 1000, 1000);
	
	
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, FRIC_OFF);
	uint32_t ptik = 0;
	usart_printf("Hola hola!");
	while (1) {
		muestra_cosas();
	}
	
	
	//printf("Hola");
	while (1) {
		float rotation = 0;
		float algo2 = 0;
		float ch0 = 0;
		float ch1 = 0;
		float ch2 = 0;
		float ch3 = 0;
		float ch4 = 0;
		float sw0 = 0;
		float sw1 = 0;
		
		if (HAL_GetTick() - ptik > 2) { //Cada 2ms
			ptik = HAL_GetTick();
			//masterLoop();
			
			#define AMPLI 10
			#define RC_MOVE 1000
			#define MOVE_WHEEL 1000
			#define TURN_OFFSET 200
			
			ch0 = local_rc_ctrl->rc.ch[0] * AMPLI;
			ch1 = local_rc_ctrl->rc.ch[1] * AMPLI;
			ch2 = local_rc_ctrl->rc.ch[2] * AMPLI;
			ch3 = local_rc_ctrl->rc.ch[3] * AMPLI;
			ch4 = local_rc_ctrl->rc.ch[4] * AMPLI; //Disco
			
			sw0 = local_rc_ctrl->rc.s[0] * AMPLI;   //Switch left
			sw1 = local_rc_ctrl->rc.s[1] * AMPLI;   //Switch right
			
			//CAN_cmd_gimbal(Trasera izquierda, Frontal derecha, Frontal Izquierda, Trasera derecha);
			
			//Movimiento hacia adelante. LEFT JOY
			int wbl = 0;
			int wbr = 0;
			int wfl = 0;
			int wfr = 0;
			
			if (ch3 > RC_MOVE && ch1 == 0) { // 
				//Solo para adelante
				wfl = MOVE_WHEEL;
				wfr = MOVE_WHEEL;
				wbl = MOVE_WHEEL;
				wbr = MOVE_WHEEL;
				CAN_cmd_gimbal(wbl, wfr*-1, wfl, wbr*-1);
			}
			if (ch3 < -RC_MOVE && ch2 == 0) {  //
				//Solo para atras
				wfl = MOVE_WHEEL*-1;
				wfr = MOVE_WHEEL*-1;
				wbl = MOVE_WHEEL*-1;
				wbr = MOVE_WHEEL*-1;
				CAN_cmd_gimbal(wbl, wfr*-1, wfl, wbr*-1);
			}
			
			if (ch3 > RC_MOVE && ch2 > RC_MOVE) { // 
				//Para adelante a la derecha
				wfl = MOVE_WHEEL;
				wfr = MOVE_WHEEL - TURN_OFFSET;
				wbl = MOVE_WHEEL;
				wbr = MOVE_WHEEL - TURN_OFFSET;
				CAN_cmd_gimbal(wbl, wfr*-1, wfl, wbr*-1);
			}			

			if (ch3 > RC_MOVE && ch2 < -RC_MOVE) { // 
				//Para adelante a la izquierda
				wfl = MOVE_WHEEL - TURN_OFFSET;
				wfr = MOVE_WHEEL;
				wbl = MOVE_WHEEL - TURN_OFFSET;
				wbr = MOVE_WHEEL;
				CAN_cmd_gimbal(wbl, wfr*-1, wfl, wbr*-1);
			}		
			
			if (ch3 == 0 && ch2 > RC_MOVE) { // 
				//Giro a la derecha
				wfl = MOVE_WHEEL;
				wfr = MOVE_WHEEL*-1;
				wbl = MOVE_WHEEL;
				wbr = MOVE_WHEEL*-1;
				CAN_cmd_gimbal(wbl, wfr*-1, wfl, wbr*-1);
			}	
			
			if (ch3 == 0 && ch2 < -RC_MOVE) { // 
				//Giro a la izquierda
				wfl = MOVE_WHEEL*-1;
				wfr = MOVE_WHEEL*1;
				wbl = MOVE_WHEEL*-1;
				wbr = MOVE_WHEEL*1;
				CAN_cmd_gimbal(wbl, wfr*-1, wfl, wbr*-1);
			}	
			
			//Prueba de mover una sola llanta del chasis
			/*
			if (ch0 > 500) {
				CAN_cmd_gimbal(MOVE_WHEEL, 000, 000, 0);
			}
			if (ch0 < -500) {
				CAN_cmd_gimbal(MOVE_WHEEL*-1, 000, 000, 0);
			}			
			if (ch1 > 500) {
				CAN_cmd_gimbal(000, MOVE_WHEEL, 000, 0);
			}				
			if (ch2 > 500) {
				CAN_cmd_gimbal(000, 000, MOVE_WHEEL, 0);
			}
			if (ch3 > 500) {
				CAN_cmd_gimbal(000, 000, 000, MOVE_WHEEL);
			}		
			*/
			if (ch4 > 500) { //Disco
				CAN_cmd_gimbal(1000, 1000, 1000, 1000);
			}		

			/*
			rotation = local_rc_ctrl->rc.ch[4] * 1000;  //Channel 4 es el disco 
			algo2 = local_rc_ctrl->rc.ch[0] * 1000; //Channel 0 es el joystick 
			if (rotation > 500) {
				//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1100);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1300); //1000 es detenido
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1300);
				//fric_on(1200); //<<- Este es el que sirve
				//CAN_cmd_gimbal(1000, 1000, 1000, 1000);
				//printf("Rotacion");
			} else if (rotation < 500) {
				fric_off();
				//CAN_cmd_gimbal(-1000, -1000, -1000, -1000);
			}
			
			if (algo2 > 500) {
				//fric_on((uint16_t) (1100));
				CAN_cmd_gimbal(1000, 1000, 1000, 1000);
				//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, FRIC_OFF);
			}
			*/
			
			
			
			//printf("ch0 %d ch1 %d ch2 %d ch3 %d",local_rc_ctrl->rc.ch[0],local_rc_ctrl->rc.ch[1],local_rc_ctrl->rc.ch[2],local_rc_ctrl->rc.ch[3]);
			
		}
	}
}

/* Loop functions */
void masterLoop(void) {
  uint32_t currentTick = HAL_GetTick();
  int deltaTime = currentTick - prevTick;

  chassisLoop(local_rc_ctrl, deltaTime);
  turretLoop(local_rc_ctrl, deltaTime);
	//refereeLoop();
	
  prevTick = currentTick;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) { 

}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
