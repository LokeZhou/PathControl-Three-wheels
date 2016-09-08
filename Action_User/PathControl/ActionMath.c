/**
  ******************************************************************************
  * @file    此文件中包含了走形函数需要的三角函数，圆心计算，结束角度计算的功能函数
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
#define PI 3.1415926
/* Private  variables ---------------------------------------------------------*/
//static float Pos_x;
//static float Pos_y;
//static float Angle;
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief 三角函数转换
  * @param Angle 角度（不是弧度值）
  * @param 
  * @retval 
  */

double Sin(double Angle)
{
	return sin(Angle/180*PI);
}
double Cos(double Angle)
{
	return cos(Angle/180*PI);
}
double Tan(double Angle)
{
	return tan(Angle/180*PI);
}

/**
  * @brief   安全反三角函数
  * @param   
  * @param 
  * @retval  返回角度值（不是弧度值）
  */
double Acos(double Cos)
{
	if(Cos>1)Cos=1;
	if(Cos<-1)Cos=-1;
	return ((180/PI)*acos(Cos));
}

double Asin(double Sin)
{
	if(Sin>1)Sin=1;
	if(Sin<-1)Sin=-1;
	return ((180/PI)*asin(Sin));
}

/**
  * @brief   实际速度转脉冲速度 
  * @param   vel实际速度
  * @param 
  * @retval  返回脉冲速度
  */
int VelTransform(float vel)
{
	return ((vel*2000*Reduce_Rate)/(PI*2*R_Wheel));//2000线（2000脉转一圈），减速比，直径
}

/**
  * @brief   获得圆弧圆心坐标
  * @param   Circle_Radius圆半径
  * @param   angle_init圆起点的方向角
  * @retval  返回脉冲速度
  */
float get_origin_x(float Circle_Radius,float angle_init)
{
	return (Get_POS_Xtemp()-(Circle_Radius*Cos(angle_init)));
}
float get_origin_y(float Circle_Radius,float angle_init)
{
	return (Get_POS_Ytemp()+(Circle_Radius*Sin(angle_init)));
}

/**
  * @brief   计算圆弧结束条件
  * @param   Circle_Radius圆半径
  * @param   angle_init圆起点的方向角
  * @retval  返回余弦值 
  */
float get_cos(float Circle_Radius,float angle_init)
{
	return ((get_origin_y(Circle_Radius,angle_init)-Get_POS_Y())/Circle_Radius);
}
/**
  * @brief   计算圆弧结束条件
  * @param   Circle_Radius圆半径
  * @param   angle_init圆起点的方向角
  * @retval  返回正弦值 
  */
float get_sin(float Circle_Radius,float angle_init)
{
	return ((Get_POS_X()-get_origin_x(Circle_Radius,angle_init))/Circle_Radius);
}

/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
