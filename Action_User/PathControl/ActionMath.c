/**
  ******************************************************************************
  * @file    ���ļ��а��������κ�����Ҫ�����Ǻ�����Բ�ļ��㣬�����Ƕȼ���Ĺ��ܺ���
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
  * @brief ���Ǻ���ת��
  * @param Angle �Ƕȣ����ǻ���ֵ��
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
  * @brief   ��ȫ�����Ǻ���
  * @param   
  * @param 
  * @retval  ���ؽǶ�ֵ�����ǻ���ֵ��
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
  * @brief   ʵ���ٶ�ת�����ٶ� 
  * @param   velʵ���ٶ�
  * @param 
  * @retval  ���������ٶ�
  */
int VelTransform(float vel)
{
	return ((vel*2000*Reduce_Rate)/(PI*2*R_Wheel));//2000�ߣ�2000��תһȦ�������ٱȣ�ֱ��
}

/**
  * @brief   ���Բ��Բ������
  * @param   Circle_RadiusԲ�뾶
  * @param   angle_initԲ���ķ����
  * @retval  ���������ٶ�
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
  * @brief   ����Բ����������
  * @param   Circle_RadiusԲ�뾶
  * @param   angle_initԲ���ķ����
  * @retval  ��������ֵ 
  */
float get_cos(float Circle_Radius,float angle_init)
{
	return ((get_origin_y(Circle_Radius,angle_init)-Get_POS_Y())/Circle_Radius);
}
/**
  * @brief   ����Բ����������
  * @param   Circle_RadiusԲ�뾶
  * @param   angle_initԲ���ķ����
  * @retval  ��������ֵ 
  */
float get_sin(float Circle_Radius,float angle_init)
{
	return ((Get_POS_X()-get_origin_x(Circle_Radius,angle_init))/Circle_Radius);
}

/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/