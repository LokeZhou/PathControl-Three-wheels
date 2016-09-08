/**
******************************************************************************
* @file      基于模糊PID控制算法的PID参数设定
* @author    tzaiyang
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
  * @brief  基于模糊PID控制算法的PID参数自调整
  * @param  pid_p:p参数调整度
  * @param  pid_i:i参数调整度
  * @param  pid_d:d参数调整度
  * @param  PIDGathe:三个闭环PID参数的结构体
  * @param  FPIDVal:PID参数的基数与pid的调整度范围参数相关结构体

  * @retval none
  */
void FPIDValSet(float p_base,float i_base,float d_base,float p_adj,float i_adj,float d_adj,FPID_TypeDef *FPIDVal)
{
	 FPIDVal->p_base=p_base;
	 FPIDVal->i_base=p_base;
	 FPIDVal->d_base=p_base;
	
	 FPIDVal->p_adj =p_adj;
	 FPIDVal->i_adj =i_adj;
	 FPIDVal->d_adj =d_adj;
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/



