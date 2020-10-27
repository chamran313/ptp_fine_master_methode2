
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "udp.h"


#define LWIP_PTP
#define ptp_port		1234

#define synq_interval  500
#define f_f_packet_prescaler  50
#define addend0   3314017975
#define tim4_period_reg		107   // (863+1) * 9.26us = 8ms-- 107->1ms
#define synq_pkt_number		5
//#define PTP_timeout
#define ptp_tout_ver2
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
ip_addr_t local_ip;
ip_addr_t remote_ip;
extern struct netif gnetif;
struct udp_pcb *udpc;
uint8_t rcv_buf[4];
//int32_t buf[14];
//char buf[256];
uint16_t udp_rcv_cnt;
uint32_t tsh, sytt, rsytl, rsyth, rxtsl, rxtsh;
int32_t  tsl;
uint16_t i, istop;
uint8_t coarse_flag, fine_flag, c_state, f_state;
uint8_t  udp_serr, ptp_timeout, fine_pkt_prescaler, ptp_to_flag;
int32_t ptphdr;

uint16_t ptp_synq_interval;
uint8_t tim4_period;
//int8_t uerr = 5;
//uint8_t synq_state, synq_state_cop;
//extern struct ETH_TimeStamp tx_ts;
//extern ETH_TimeStamp tx_ts2;
extern ETH_TimeStamp tx_ts;
extern ETH_TimeStamp rx_ts;
ETH_TimeStamp tx_ts_array[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ptp_start(void);
void udp_receive_callback(void *arg, struct udp_pcb *udpc, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void ip_asign(void);
void udp_master_init(void);
//void udp_send1( ETH_TimeStamp* timestamp, int32_t ptp_hdr);
void udp_send1( ETH_TimeStamp* timestamp, int32_t ptp_hdr, uint16_t syn_interval);

//uint8_t Master_synq_process( uint8_t state);
void ETH_SetPTPTimeStampAddend(uint32_t Value);
void ETH_EnablePTPTimeStampAddend(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance == TIM4)
	{
		i++;
		if(i>synq_interval)
			{
				
			if(coarse_flag==1)
				{
				 ptphdr = ++c_state;	
			   udp_send1(&tx_ts, ptphdr, ptp_synq_interval);
				 //c_state++;
				 if(c_state==2)		
						{	
							coarse_flag = 0;
							c_state = 0;
							#ifdef ptp_tout_ver2
									ptp_to_flag = 1;
									ptp_timeout = 0;
							#endif
							
						}
				}
				
				
				
			else if(fine_flag==1)
				{
				 
				 if(fine_pkt_prescaler==0)
					{
					 ptphdr = f_state + 4;
					 //udp_send1(&tx_ts, ptphdr);
					 
					 udp_send1(&tx_ts_array[f_state], ptphdr, ptp_synq_interval);
					 f_state++;
					 tx_ts_array[f_state] = tx_ts;
					 if(f_state!= (synq_pkt_number-1) )		fine_pkt_prescaler =  f_f_packet_prescaler;
					 else		fine_pkt_prescaler = 1; 
					}
					fine_pkt_prescaler--;
					
					if(f_state==synq_pkt_number)
						{
							ptp_to_flag = 1;
						}
				}
				
				
			#ifdef PTP_timeout
			ptp_timeout++;
				if(ptp_timeout==29)  // should timeout > (nfp - 1)*4 + 3
					{
						if(fine_flag) 	coarse_flag = 0;
						else 						coarse_flag = 1;
						//coarse_flag = 1;
						ptp_timeout = 0;
						c_state = 0;
						//fine_flag = 0;
						f_state = 0;
						i = synq_interval - 1; //i=0 bood
						fine_pkt_prescaler = 0;
					}
			#endif
			 	
			
			}
			
			#ifdef ptp_tout_ver2 
					if(ptp_to_flag)
						{
							ptp_timeout++;
							if(ptp_timeout==4)
								{
									if(fine_flag) 	
										{
										  coarse_flag = 0;
											f_state--;
										}
									else 		coarse_flag = 1;
									//coarse_flag = 1;
									ptp_timeout = 0;
									ptp_to_flag = 0;
									c_state = 0;
									//fine_flag = 0;
									//if(f_state==5)	f_state = 4;
									//else  f_state = 0;
									i = synq_interval - 1; //i=0 bood
									fine_pkt_prescaler = 0;
								}	
						}
			#endif
	
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_LWIP_Init();
	#ifdef LWIP_PTP
	ptp_start();
	ip_asign();
  udp_master_init();
	#endif
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
	
	//synq_interval = 1;
	coarse_flag = 1;
	tim4_period = (tim4_period_reg+1) * 0.00926;
	//ptp_synq_interval = (synq_interval + 2 + 1 + (3*f_f_packet_prescaler) + 1+ 2)* tim4_period; // 2 akhar baraye pkt loss
	ptp_synq_interval = (2+(synq_pkt_number-2)*f_f_packet_prescaler+4) * tim4_period; //4 for pkt loss
	
	//ptp_synq_interval = (synq_interval + 2 + 1 + (3*f_f_packet_prescaler) + 1+ 1) * 8; // 1 akhar baraye pkt loss
	
  /* USER CODE END 2 */
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
  MX_LWIP_Process();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = tim4_period_reg; //emad add --- first 1000 = 9.26ms  -- 863->8ms
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */   // is ext pull down
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ptp_start(void)
{
	
	//assert_param(IS_ETH_PTP_UPDATE(UpdateMethod));
	ETH->MACIMR |=  0x200;  // disable timestamp trigger interrupt generation
  ETH-> PTPTSCR |= 0x00000001;   // enable ptp time stamping
	//ETH-> PTPSSIR = 8;      // sub_second increment register  0XA:FOR 96MHZ
													// : 5 FOR 216MHZ
	ETH-> PTPSSIR = 6;
	
	ETH_SetPTPTimeStampAddend( addend0 ); // hclk = 216M  --- Ts accuracy = 1/120M = 8.33nS
	ETH_EnablePTPTimeStampAddend();
	while(  ETH->PTPTSCR & 0x00000020 );
	
	ETH->PTPTSCR |= 0x00000002;  // fine correction method
	
	ETH->PTPTSCR |= 0x00000200; // sub_second reg overflow when recieve 999 999 999
	
	//ETH->PTPTSCR |= 0x00000c00;
  ETH->PTPTSCR |= 0x00000100; //set TSSARFE bit -> timestamp enable for all rcv frames
	
	ETH->PTPTSHUR = 0;
	ETH->PTPTSLUR = 0;
	ETH-> PTPTSCR |= 0x00000004;  // set bit2 = tssti  time stamp system time initialize
	while(ETH->PTPTSCR & 0x4){};
	
	//Enable enhanced descriptor structure 
    ETH->DMABMR |= ETH_DMABMR_EDE;	
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ip_asign(void)
{
local_ip = gnetif.ip_addr;
 ip4addr_aton("192.168.1.50", &remote_ip); // .100 for stm    .50 for pc 
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++

void udp_master_init(void)
{
	
	err_t err;
	
	 /* Create a new UDP control block  */
  udpc = udp_new();
  
  if (udpc!=NULL)
  {
    /*assign destination IP address */
    //IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
    udp_bind(udpc, &local_ip, ptp_port);
    /* configure destination IP address and port */
    err= udp_connect(udpc, &remote_ip, ptp_port);
    
    if (err == ERR_OK)
    {
      /* Set a receive callback for the upcb */
      udp_recv(udpc, udp_receive_callback, NULL);  
    }
		else udp_remove(udpc);
  }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//void udp_send1(void )
void udp_send1( ETH_TimeStamp* timestamp, int32_t ptp_hdr, uint16_t syn_interval)
{
  struct pbuf *pb;
	uint8_t len;
	int32_t buf[3];
	err_t err;
	uint32_t syn_int_copy = 0;
	//uint8_t udp_serc = 0;
	
	//len = sprintf(buf, "a");
	buf[0] = timestamp->TimeStampLow;
	buf[1] = timestamp->TimeStampHigh;
	if(ptphdr==1)	
		{
		syn_int_copy = syn_interval << 8;
		}
	
	buf[2] =  syn_int_copy | ptp_hdr ;
	
	len = 12;
	
	pb = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
	err = pbuf_take(pb,buf,len);
	if(err == ERR_OK )
	 {
		//udp_connect(udpc, &remote_ip, 320);
		 
		/*erru = udp_send(udpc, pb);*/
		udp_send(udpc, pb);
		 
		//udp_disconnect(udpc);
		  
		//test_send_cnt++;
		 
		 //pbuf_free(pb);
		 //udp_remove(udpc);
	 }
	//else 	{udp_serc = 1;}
	
	pbuf_free(pb);
	
	//return erru;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
 
	   pbuf_copy_partial( p, rcv_buf, p->len, 0); 
	   if(port== ptp_port)
		 {
	   if(rcv_buf[0]== 0x12)   // delay_req recieved
				{
				 ptphdr = 3;
				 udp_send1(&rx_ts, ptphdr, ptp_synq_interval);
				 //udp_send1(&rx_ts);
				 coarse_flag = 0;
				 fine_flag = 1;
				 fine_pkt_prescaler = 0;
				 f_state = 0;
				 #ifdef ptp_tout_ver2	
						ptp_to_flag = 0;
					  ptp_timeout = 0;
				 #endif
				 //i = 107;
				}
				
		else if(rcv_buf[0] & 0x80)   // unmatch hdr bw master & slave (coarse ok) recieved
			{
				f_state = (rcv_buf[0] & 0xf) - 4 ;// 0x0f === 15
				
				coarse_flag = 0;
				fine_flag = 1;
				//f_state = 0;
				i = synq_interval -1 ;
				fine_pkt_prescaler = 0;
				
				ptp_to_flag = 0;
				ptp_timeout = 0;
			}
			
		else if(rcv_buf[0]== 0x66)   // unmatch hdr bw master & slave (coarse nok) recieved
			{
				coarse_flag = 1;
				c_state = 0;
				fine_flag = 0;
				f_state = 0;
				i = synq_interval - 1;
				
			  ptp_to_flag = 0;
				ptp_timeout = 0;
			}
			
	  else if(rcv_buf[0]== 0x77)   // ok end of fine process
			{
				//if(f_state==5) 
				//		{
					fine_flag = 0;
					f_state = 0;
					i = 0;
					coarse_flag = 1;
					fine_pkt_prescaler = 0;
					
				  #ifdef ptp_tout_ver2
				  ptp_to_flag = 0;
				  #endif
				
					ptp_timeout = 0;
				//		}
			}
		}
		 /*else if(rcv_buf[0]== 0x23) // after 2 synq - rcv
			{
				udp_send1(&tx_ts);
				f_state = 0;
				coarse_flag = 1;
				i = 0;
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
			} */
	   
	   /*udp_send1( &rx_ts);
	   c_state++;*/
		 //synq_state = 0;
	   //i = 0;
	   //synq_state = 3;
	   //fine_flag = 1;
		 //i = 6;
	   //htim4.Init.Period = 50000; // 460ms
		 //synq_interval = 6;
	   //i = 5;
		 
	//	}
	
	//rxtsh = rx_ts.TimeStampHigh;
	//rxtsl = rx_ts.TimeStampLow;
	
	//rsyth = ETH->PTPTSHR;
	//rsytl = ETH->PTPTSLR;
	
  /* Free receive pbuf */
  pbuf_free(p);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*uint8_t Master_synq_process( uint8_t state)
{
	
uint8_t state_copy = 0;
	
if( state < 2)
	{
		udp_send1( &tx_ts); // in SYNQ : can send evry thing
		state_copy = state++;
	}
 return state_copy;


}*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ETH_SetPTPTimeStampAddend(uint32_t Value)
{
/* Set the PTP Time Stamp Addend Register */
  ETH->PTPTSAR = Value; 
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ETH_EnablePTPTimeStampAddend(void)
{
  /* Enable the PTP block update with the Time Stamp Addend register value */
  ETH->PTPTSCR |= ETH_PTPTSCR_TSARU;    
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
