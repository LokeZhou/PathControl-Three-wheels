/**
  ******************************************************************************
  * @file    走形函数的公共头文件
  * @author  tzaiyang
  * @version 
  * @date   
  * @brief   This file contains the headers of walk.c
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __WALK_H__
#define  __WALK_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include <math.h>
#include "elmo.h"

/* Exported types ------------------------------------------------------------*/ 
 typedef struct{
	 float p_base;
	 float i_base;
	 float d_base;
	 
	 float p_adj;
	 float i_adj;
	 float d_adj;
 }FPID_TypeDef;
 
  typedef struct{
	 float pParam;
	 float iParam;
	 float dParam;
 }PID_TypeDef;
 
  typedef struct{
	 PID_TypeDef Position;
	 PID_TypeDef Angle;
	 PID_TypeDef Actvel;
 }PIDGather_TypeDef;
	
  typedef struct{
	 FPID_TypeDef FPosition;
	 FPID_TypeDef FAngle;
	 FPID_TypeDef FActvel;
 }FPIDGather_TypeDef;
/* Exported macro ------------------------------------------------------------*/
 
#define Reduce_Rate 299/14
#define R_Wheel 70

#define d_circle                 145         //Distance between Encoder and Centrality of Automaton
#define d_angle                  60          //Angle between Initial coordinate and Word coordinate

// #define  RED_FIELD
#define BLUE_FIELD	

 //以下宏定义一般不用改动
 #define ISROTATE            1   //要自转
 #define NOROTATE            0   //不自转
 
 #define PLEFT              -1   //与左边保持距离
 #define PRIGHT              1   //与右边保持距离
 
 #define CIRCLE_END          1   //画完圆
 
 #define XDIRECTION         90  //X方向
 #define YDIRECTION          0  //Y方向

 #define PID_TAN            ((Get_POS_Xtemp()-Get_POS_X())/(Get_POS_Y()-Get_POS_Ytemp()))
 
 

//差量限制
#define ERRVALUE_MAX    250
#define ERRVALUE_MIN   -250
#define ERRANGLE_MAX    15
#define ERRANGLE_MIN   -15
#define ERRACTVEL_MAX   600
#define ERRACTVEL_MIN  -600
//差量微分限制
#define ERRCVALUE_MAX   20
#define ERRCVALUE_MIN  -20
#define ERRCANGLE_MAX   0.8f
#define ERRCANGLE_MIN  -0.8f
#define ERRCACTVEL_MAX  60
#define ERRCACTVEL_MIN -60
//积分限制
#define ERRSUMVALUE_MAX  500
#define ERRSUMVALUE_MIN -500
#define ERRSUMANGLE_MAX  30
#define ERRSUMANGLE_MIN -30
#define ERRSUMACTVEL_MAX  1000
#define ERRSUMACTVEL_MIN -1000

//模糊语言定义
#define NB     -3
#define NM     -2
#define NS     -1
#define ZO      0
#define PS      1
#define PM      2
#define PB      3


#define VALUE_PID_SET   1
#define ANGLE_PID_SET   2
#define ACVEL_PID_SET   3
 
/* Exported functions ------------------------------------------------------- */
/*****action_math.c*****/
double   Sin(double Angle);
double   Cos(double Angle);
double   Tan(double Angle);
double   Asin(double Sin);
double   Acos(double Cos);
 
 
int      VelTransform(float vel);
float    get_origin_x(float Circle_Radius,float angle_init);
float    get_origin_y(float Circle_Radius,float angle_init);
float    get_cos(float Circle_Radius,float angle_init);
float    get_sin(float Circle_Radius,float angle_init);


void 		 Set_Original_POS_X(float val);
void 		 Set_Original_POS_Y(float val);
void 		 Set_Original_Angle(float val);
float 	 Get_Original_POS_X(void);
float		 Get_Original_POS_Y(void);
float		 Get_Original_Angle(void);


void		 Set_POS_X(float val);
float		 Get_POS_X(void);
void		 Set_POS_Y(float val);
float		 Get_POS_Y(void);
void		 Set_Angle(float val);
float		 Get_Angle(void);

void		 Set_Vel(int vel,int num);
int   	 Get_Vel(int num);

void		 Set_POS_Xtemp(float val);
void		 Set_POS_Ytemp(float val);
float		 Get_POS_Xtemp(void);
float		 Get_POS_Ytemp(void);

void		 Set_ActVel_X(int val);
void		 Set_ActVel_Y(int val);
int			 Get_ActVel_X(void);
int			 Get_ActVel_Y(void);
void     SetActualVel(void);

//对走形函数作支持的函数
void     ClearFirst(void);
void     SetFirst(void);
void     PIDValSet(float pid_p,float pid_i,float pid_d,PID_TypeDef *PIDVal,FPID_TypeDef *FPIDVal);
void     FuzPidTable(int8_t Flag,int Err,int Errc,PIDGather_TypeDef *PIDGather,FPIDGather_TypeDef *FPIDGather);
void     CoordsConversion(void);

//走形函数
uint8_t  ElmoInit(uint32_t acc,uint32_t dcc,uint8_t n);
void 		 LockWheel(void);
void 		 BasicLine(int Vel,float ward,float Rotate);
void     FPIDValSet(float p_base,float i_base,float d_base,float p_adj,float i_adj,float d_adj,FPID_TypeDef *FPIDVal);
void     FuzPidLine (float Vel,float Ward,float Refangle,float Refvalue,float Curvalue,int8_t Flag,FPIDGather_TypeDef *FPIDGather);
int8_t	 BasicCircle(int Vel,float WardInit,float WardEnd,float Refangle,float Radius,int8_t IsRotate,FPIDGather_TypeDef *fpid);

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

