/**
******************************************************************************
* @file     定位系统原始坐标系变换到世界坐标系
* @author   tzaiyang
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
* @brief  定位系统原始坐标系变换到世界坐标系
* @param  none
* @param
* @retval none
*/

void CoordsConversion(void)
{
	    float posX=0,posY=0;
	 		float act_x=Get_Original_POS_X();
			float act_y=Get_Original_POS_Y();
			float angle=Get_Original_Angle();
	
			 if(angle>180)
			 angle=angle-360;
		   if(angle<-180)
		   angle=angle+360;
			 
			 Set_Angle(angle);
       #ifdef BLUE_FIELD
			 posX= (act_x*Cos(60.0)-act_y*Sin(60.0))/*坐标轴变换*/+147*(Cos(angle)-Cos(-30));
		   posY=-(act_x*Sin(60.0)+act_y*Cos(60.0))/*坐标轴变换*/-147*(Sin(angle)-Sin(-30));
			 #elif  defined RED_FIELD
			 posX= (act_x*Cos(120.0)-act_y*Sin(120.0))/*坐标轴变换*/+147*(Cos(angle)-Cos(-30));  //120为编码器坐标系x与世界坐标系x的夹角，-30为初始角度
		   posY= (act_x*Sin(120.0)+act_y*Cos(120.0))/*坐标轴变换*/-147*(Sin(angle)-Sin(-30));
			 #endif
			 Set_POS_X(posX);
			 Set_POS_Y(posY); 
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/


