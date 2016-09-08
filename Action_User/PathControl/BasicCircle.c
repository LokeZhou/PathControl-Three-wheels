/**
  ******************************************************************************
  * @file    此文件实现了闭环画弧的功能
  * @author  tzaiyang
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "Walk.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  闭环画弧,画完返回值1，没画完返回值0
  * @param  Vel:车的线速度（正数 Vel>0）
  * @param  WardInit：开始的方向
  * @param  WardEnd：画完的方向
  * @param  Radius：弧的半径（正数为顺时针，负数为逆时针）
  * @param  IsRotate：IsRotate==1,要自转，IsRotate==0，不自转

  * @retval none
  * @author tzy
  */
int8_t BasicCircle(int Vel,float WardInit,float WardEnd,float Refangle,float Radius,int8_t IsRotate,FPIDGather_TypeDef *fpid)
{
	float ActRadius,Pos_Ox,Pos_Oy,angle,WardAdd;
	
	Pos_Ox=get_origin_x(Radius,WardInit);//坐标原点
	Pos_Oy=get_origin_y(Radius,WardInit);
	ActRadius=sqrt(pow((Get_POS_X()-Pos_Ox),2)+pow((Get_POS_Y()-Pos_Oy),2));//实际半径
	angle=((90-WardInit)-Acos(get_cos(Radius,WardInit)));//画圆的角度 

	WardAdd=WardInit+angle;	
	
	if(Radius>0)//顺时针
	{
	  if(IsRotate==1)FuzPidLine(Vel,WardAdd,WardAdd,Radius,ActRadius,1,fpid);
    if(IsRotate==0)FuzPidLine(Vel,WardAdd,Refangle,Radius,ActRadius,1,fpid);
	  if(WardAdd<=WardEnd)return CIRCLE_END;
	  if(WardAdd>WardEnd) return 0;		
	}
	else if(Radius<0)//逆时针
	{
		if(IsRotate==1) FuzPidLine(Vel,WardAdd,WardAdd,-Radius,ActRadius,-1,fpid);
    if(IsRotate==0)	FuzPidLine(Vel,WardAdd,Refangle,-Radius,ActRadius,-1,fpid); 
		if(WardAdd>=WardEnd)return CIRCLE_END;
		if(WardAdd<WardEnd) return 0;
	}
  return  2;	
}

/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

