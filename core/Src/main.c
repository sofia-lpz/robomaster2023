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
#include <stdlib.h>
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

long mi_map(long x, long in_min, long in_max, long out_min, long out_max);

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
	

uint32_t prevTick;


#define AMPLI 1
#define RC_MOVE 1000
#define RC_MIN_MOVE 50
#define CH_VALUE_MIN 0
#define CH_VALUE_MAX 660
#define MIN_MOVE_WHEEL 500
#define MAX_MOVE_WHEEL 2000
#define TURN_FACTOR 0.5
#define MOVE_WHEEL 1000
#define TURN_OFFSET 300
#define INICIAL_TORRETA1 1500
#define MAX_TORRETA1 2000
#define MIN_TORRETA1 1000
#define PASO_TORRETA1 0.5
#define INICIAL_TORRETA2 1400
#define MAX_TORRETA2 1600
#define MIN_TORRETA2 1300
#define PASO_TORRETA2 0.5
#define FEEDER_SPEED 700
#define FEEDER_REVERSE_SPEED 2000
#define FEEDER_CHAKA_TIME 275
#define FEEDER_SPEED_FACTOR 3
#define FEEDER_SLOW_SPEED 300
#define SNAIL_WAIT_FEEDER 1500
#define SNAIL_SPEED  1350

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
	
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000); //SNAIL 1
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1000); //SNAIL 2
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 1000);

	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, INICIAL_TORRETA1); //TORRETA 1
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, INICIAL_TORRETA2); //TORRETA 2
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1000);
	
  // Start remote control process
	
	//usart1_tx_dma_init();
	local_rc_ctrl = get_remote_control_point();
	
	
  /* Init functions */
  //chassisInit();
  //turretInit();
  //refereeInit();

  /* Run master loop function on a timer */
  //while (1) { function_with_interval(&masterLoop, NULL, M_MASTER_LOOP_INTERVAL); };

	//Arrancon de prueba
	CAN_cmd_chassis_good(1000, 1000, 1000, 1000);
	

	
	
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, FRIC_OFF);
	uint32_t ptik = 0;
	uint32_t tik_start_snail = 0;
	uint32_t tik_start_feeder = 0;
	
	int feeder_phase = 0;
	
	float pos_torreta1 = INICIAL_TORRETA1; //1000 es ??
	float pos_torreta2 = INICIAL_TORRETA2; //1000 es ??

	while (1) {
		float rotation = 0;
		float algo2 = 0;
		int16_t ch0 = 0;
		int ch1 = 0;
		int ch2 = 0;
		int ch3 = 0;
		int ch4 = 0;
		int sw0 = 0;
		int sw1 = 0;
		int factor_velocidad = 1;
		
		if (HAL_GetTick() - ptik > 2) { //Cada 2ms
			ptik = HAL_GetTick();

	
			

			
			//En los ejes x y y de los joysticks y el disco rangos: -660,660
			//switch 1 izq  en medio 3 arriba 1 abajo 2
			//switch 0 der
			
			if (local_rc_ctrl->rc.s[1] == 2) {
				factor_velocidad = 1;
			}
			if (local_rc_ctrl->rc.s[1] == 3) {
				factor_velocidad = 2;
			}
			ch0 = local_rc_ctrl->rc.ch[0] * AMPLI;
			ch1 = local_rc_ctrl->rc.ch[1] * AMPLI;
			ch2 = local_rc_ctrl->rc.ch[2] * AMPLI;
			ch3 = local_rc_ctrl->rc.ch[3] * AMPLI;
			ch4 = local_rc_ctrl->rc.ch[4] * AMPLI; //Disco
			
			sw0 = local_rc_ctrl->rc.s[0];   //Switch left
			sw1 = local_rc_ctrl->rc.s[1];   //Switch right
			
			/*
			* Joystick izquierdo
			* Para mover hacia adelanta, atras y derecha e izquierda
			*/
			
			//CAN_cmd_chassis_good(Frontal Derecha, Trasera Derecha, Trasera Izquierda, Delantera Izquierda);
			
			//Movimiento hacia adelante. LEFT JOY
			int wbl = 0;
			int wbr = 0;
			int wfl = 0;
			int wfr = 0;
			
			if (ch3 > RC_MIN_MOVE && ch1 == 0) { // 
				//Solo para adelante
				/*
				wfl = MOVE_WHEEL;
				wfr = MOVE_WHEEL;
				wbl = MOVE_WHEEL;
				wbr = MOVE_WHEEL;
				*/
				long fmove = mi_map(ch3, CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL);
				wfl = fmove;
				wfr = -fmove;
				wbl = fmove;
				wbr = -fmove;				
				CAN_cmd_chassis_good(wfr, wbr, wbl, wfl);

			}
			if (ch3 < -RC_MIN_MOVE && ch2 == 0) {  //
				//Solo para atras
				long fmove = mi_map(abs(ch3), CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL);
				wfl = -fmove;
				wfr = fmove;
				wbl = -fmove;
				wbr = fmove;
				CAN_cmd_chassis_good(wfr, wbr, wbl, wfl);
			}
			
			if (ch3 > RC_MIN_MOVE && ch2 > RC_MIN_MOVE) { //
				//Para adelante a la derecha
				long fmove = mi_map(ch3, CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL);
				long fmove_r = mi_map(ch2, CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL)*TURN_FACTOR;
				wfl = fmove;
				wfr = -fmove + fmove_r;
				wbl = fmove;
				wbr = -fmove + fmove_r;					
				
				CAN_cmd_chassis_good(wfr, wbr, wbl, wfl);
			}			

			if (ch3 > RC_MIN_MOVE && ch2 < -RC_MIN_MOVE) { // 
				//Para adelante a la izquierda
				long fmove = mi_map(ch3, CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL);
				long fmove_l = mi_map(abs(ch2), CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL)*TURN_FACTOR;				
				wfl = fmove - fmove_l;
				wfr = -fmove;
				wbl = fmove - fmove_l;
				wbr = -fmove;
				CAN_cmd_chassis_good(wfr, wbr, wbl, wfl);
			}	

			if (ch3 < -RC_MIN_MOVE && ch2 > RC_MIN_MOVE) { // 
				//Para atras a la derecha
				long fmove = mi_map(abs(ch3), CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL);
				long fmove_r = mi_map(ch2, CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL)*TURN_FACTOR;				
				wfl = -fmove;
				wfr = fmove - fmove_r;
				wbl = -fmove;
				wbr = fmove - fmove_r;
				CAN_cmd_chassis_good(wfr, wbr, wbl, wfl);
			}	

			if (ch3 < -RC_MIN_MOVE && ch2 < -RC_MIN_MOVE) { // 
				//Para atras a la izquierda
				

				long fmove = mi_map(abs(ch3), CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL);
				long fmove_r = mi_map(abs(ch2), CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL)*TURN_FACTOR;				
				wfl = -fmove + fmove_r;
				wfr = fmove;
				wbl = -fmove + fmove_r;
				wbr = fmove;
				CAN_cmd_chassis_good(wfr, wbr, wbl, wfl);				
			}	
			
			
			if (ch3 == 0 && ch2 > RC_MIN_MOVE) { // 
				long fmove = mi_map(ch2, CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL);
				if (sw1 == 2) {  //Abajo
					//Giro a la derecha en su propio eje
					
					wfl = fmove/2;
					wfr = fmove/2;
					wbl = fmove/2;
					wbr = fmove/2;					
				}
				if (sw1 == 3) {   //En medio
					//Movimiento lateral a la derecha sin avanzar
					wfl = fmove;
					wfr = fmove;
					wbl = -fmove;
					wbr = -fmove;					
				}
				CAN_cmd_chassis_good(wfr*1, wbr*1, wbl*1, wfl*1);
			}	
			
			if (ch3 == 0 && ch2 < -RC_MIN_MOVE) { // 
				long fmove = mi_map(abs(ch2), CH_VALUE_MIN, CH_VALUE_MAX, MIN_MOVE_WHEEL, MAX_MOVE_WHEEL);
				if (sw1 == 2) {  //Abajo
					//Giro a la izquierda
					wfl = -fmove/2;
					wfr = -fmove/2;
					wbl = -fmove/2;
					wbr = -fmove/2;						
				}
				if (sw1 == 3) {  //En medio
					//Movimiento lateral a la izquierda sin avanzar
					wfl = -fmove;
					wfr = -fmove;
					wbl = fmove;
					wbr = fmove;						
				}		
				CAN_cmd_chassis_good(wfr*1, wbr*1, wbl*1, wfl*1);
			}	
			
			/*
			* Joystick derecho
			*/
			//Motor 5
			if (ch0 < -RC_MIN_MOVE) {
				pos_torreta1 = pos_torreta1 + PASO_TORRETA1;
				if (pos_torreta1 > MAX_TORRETA1) {
					pos_torreta1 = MAX_TORRETA1;
				}
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pos_torreta1); //Torreta 1
			}
			if (ch0 > RC_MIN_MOVE) {
				pos_torreta1 = pos_torreta1 - PASO_TORRETA1;
				if (pos_torreta1 < MIN_TORRETA1) {
					pos_torreta1 = MIN_TORRETA1;
				}				
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pos_torreta1); //Torreta 1
			}

			//Motor 6
			if (ch1 > RC_MIN_MOVE) {
				pos_torreta2 = pos_torreta2 + PASO_TORRETA2;
				if (pos_torreta2 > MAX_TORRETA2) {
					pos_torreta2 = MAX_TORRETA2;
				}				
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pos_torreta2); //Torreta 2	

			} else if (ch1 < RC_MOVE && ch1 > -RC_MOVE) {
				//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1000); //Torreta 2
			}
			if (ch1 < -RC_MIN_MOVE) {
				pos_torreta2 = pos_torreta2 - PASO_TORRETA2;
				if (pos_torreta2 < MIN_TORRETA2) {
					pos_torreta2 = MIN_TORRETA2;
				}					
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pos_torreta2); //Torreta 2
			}				

			if (ch4 > 500) { //Disco
				//SNAIL 1000 es detenido
				if (tik_start_snail == 0) {
					tik_start_snail = HAL_GetTick();
					tik_start_feeder = HAL_GetTick();
					feeder_phase = 0;
				}
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, SNAIL_SPEED); //Snail 1
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, SNAIL_SPEED); //Snail 2	
				
				if (HAL_GetTick() - tik_start_feeder > FEEDER_CHAKA_TIME && (feeder_phase < 3 || sw0 == 1) ) {
					tik_start_feeder = HAL_GetTick();
					if (feeder_phase == 0) {
						feeder_phase = 1;
					} else if (feeder_phase == 1) {
						feeder_phase = 0;
					}
					if (HAL_GetTick() - tik_start_snail > SNAIL_WAIT_FEEDER && sw0 != 1) {
						feeder_phase = 3;
					}						
				}
				if (feeder_phase == 0) {
					CAN_cmd_feeder(FEEDER_SPEED*FEEDER_SPEED_FACTOR);
				}
				if (feeder_phase == 1) {
					CAN_cmd_feeder(-FEEDER_SPEED*FEEDER_SPEED_FACTOR);
				}
				
				if (HAL_GetTick() - tik_start_snail > SNAIL_WAIT_FEEDER && feeder_phase == 3) {
					//CAN_cmd_feeder(FEEDER_SPEED*3);
					//HAL_Delay(500);
					if (sw0 == 2) {  //Abajo
						CAN_cmd_feeder(FEEDER_SPEED);
					}
					if (sw0 == 3) {  //En medio
						CAN_cmd_feeder(FEEDER_SLOW_SPEED);
					}					
				}
			
			} else if (ch4 < 500 && ch4 > -500) {
				CAN_cmd_feeder(00);
				tik_start_snail = 0;
				feeder_phase = 0;
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000); //1000 es detenido
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1000);				
			}
			if (ch4 < -500) { //Disco		
				CAN_cmd_feeder(-FEEDER_REVERSE_SPEED);
				feeder_phase = 0;
			}				
		}
	}
}



long mi_map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
