/**
******************************************************************************
* @file   实现开环直线加上旋转的功能
* @author tzaiyang
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
* @brief  电机抱死
* @param  none
* @param
* @retval none
*/
void LockWheel()  
{
	VelCrl(1,0);
	VelCrl(2,0);
	VelCrl(3,0);	
}

/**
  * @brief  开环直线加上旋转
  * @param  Vel:车的和速度（正数 Vel>0）
  * @param  ward:车的行进方向
                   -180到+180
  * @param  Rotate:车自身的旋转速度（正数时右旋 俯视图）
  * @param  selfAngle:车自身的姿态角度
  * @retval none
  * @author lxy
  */
void BasicLine(int Vel,float ward,float Rotate)
{ 
	static  int tarV[3];
	        int V_sum;	
					int RotateVal;
					int maxV=0;
	        int i=0;
	        float reduceP;

	/*计算自转脉冲数*/
	RotateVal=VelTransform(Rotate);
  /*计算前进总车速的脉冲数*/
	V_sum=VelTransform(Vel);
	
/*三轮速度计算*/
	#ifdef    BLUE_FIELD
	tarV[2]= (V_sum*Cos(ward-Get_Angle()+ 60))+RotateVal;
	tarV[1]=-(V_sum*Cos(ward-Get_Angle()+  0))+RotateVal;
	tarV[0]=-(V_sum*Cos(ward-Get_Angle()+120))+RotateVal;
	#elif	    defined	RED_FIELD
	tarV[0]=-(V_sum*Cos(ward-Get_Angle()+ 60))-RotateVal;//换蓝场时将tarV[0]各自都加个负号，并把0号与2号对调
	tarV[1]= (V_sum*Cos(ward-Get_Angle()+  0))-RotateVal;
	tarV[2]= (V_sum*Cos(ward-Get_Angle()+120))-RotateVal;
  #endif
	
//最大速度限制
	for(i=0;i<3;i++)
	{
		if(fabs(tarV[i])>maxV)
			  maxV=fabs(tarV[i]);
	}
	
	if(maxV>420000)
	{
		for(i=0;i<3;i++)
		{
			reduceP=(float)tarV[i]/(float)maxV;
			tarV[i]=420000*reduceP;
		}
	}

	/*三轮速度给定*/
	VelCrl(1,tarV[0]);
	VelCrl(2,tarV[1]);
	VelCrl(3,tarV[2]);
	
}



/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/



