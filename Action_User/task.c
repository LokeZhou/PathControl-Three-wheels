#include  <includes.h>
#include  <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "usart.h"
#include "walk.h"
#include "can.h"
#include "elmo.h"
#include "ucos_ii.h"
//////////////////Area of defining semaphore////////////////////////
OS_EVENT 		  			*PeriodSem;
OS_EVENT 						*CalculateSem;

int8_t status						   = 0 ;  
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err; /* prevent warning... */
	
	/******************Create Semaphore***********************/
    PeriodSem				=	OSSemCreate(0);
    CalculateSem    = OSSemCreate(0);

  /******************Create Task**************************/	
	 os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,					//Initial Task
	                      	(void          * ) 0,							
													(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],		
													(INT8U           ) Config_TASK_START_PRIO  );	
						
													
	 os_err = OSTaskCreate(	(void (*)(void *)) WalkTask,					
	                      	(void          * ) 0,							
													(OS_STK        * )&WalkTaskStk[Walk_TASK_STK_SIZE-1],		
													(INT8U           ) Walk_TASK_PRIO  );
	 os_err = OSTaskCreate(	(void (*)(void *)) CalTask,					
	                      	(void          * ) 0,							
													(OS_STK        * )&CalTaskStk[Cal_TASK_STK_SIZE-1],		
													(INT8U           ) Cal_TASK_PRIO  );
												
}

FPIDGather_TypeDef          fpid_gather;
void ConfigTask(void)
{	
	  CPU_INT08U  os_err;
	  os_err = os_err;
	  /*********************电机初始化***********************************/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);  //1 1

		ElmoInit(5000000,5000000,3);
		/*********************GYRO_USART Init*********/
	  TIM_Delayms(TIM5,20000);
	  GYRO_USART3_Init(115200);  //1 1
	
	  FPIDValSet(0.15,	0.01 ,  0.15,  0.02 ,	0.002,	0.03,   &(fpid_gather.FPosition));
	  FPIDValSet(60  ,	1.5  ,	120 ,  10.00,	0.2  ,  30  ,   &(fpid_gather.FAngle));
	  FPIDValSet(0.15,	0.010,	0.10,  0.04 ,	0.003,	0.03,   &(fpid_gather.FActvel));
 		/*********************TIMER INIT*******************************/
	  TIM_Init(TIM2,999,839,3,3);//3 3
	
  	OSSemSet(PeriodSem,0,&os_err);	
		OSTaskSuspend(OS_PRIO_SELF);
}

void WalkTask(void)
{
	CPU_INT08U  os_err;

  while(DEF_TRUE)
	{
	  OSSemPend(PeriodSem,0,&os_err);     //release once semaphore per 5 millisecond
		

		switch(status)
		{
			case 0:
			  
				break;
			

			default:
				break;
		 }
	}
}

/****************计算坐标*****************/
/*****************************************/
void CalTask(void)
{
	  CPU_INT08U  os_err;
	  while(1)
		{
			 	  OSSemPend(CalculateSem,0,&os_err);
				  CoordsConversion();
		}
}

