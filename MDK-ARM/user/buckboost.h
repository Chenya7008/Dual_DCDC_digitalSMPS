#ifndef _buckboost_h
#define _buckboost_h

#include "main.h"

//#define targetOutputVotage 12.0f
//#define targetChargeCurrent 5.0f
#define adc1_channalNum 3

#define adc2_channalNum 1
//float ratio = 20;
//pwm
#define DP_PWM_PER   HRTIM1->sMasterRegs.MPER
#define MAX_PWM_CMP   (uint32_t)(0.90F * HRTIM1->sMasterRegs.MPER) //PWM最大比较值
#define MIN_PWM_CMP   (uint32_t)(0.04F * HRTIM1->sMasterRegs.MPER) //PWM最小比较值
//F334用


extern uint16_t adc1_in_buff[adc1_channalNum];
extern uint16_t adc2_in_buff[adc2_channalNum];



//	adc1开启
typedef struct
{

    float Votage_ratio;
		float Current_ratio;
    float targetCapVoatge;
    float targetCapCurrent;
    float targetPower;

}ELEC_value_struct;


extern ELEC_value_struct Fuck;
//extern uint32_t	 adc2_in_buff[adc2_channalNum];
extern float V_in,V_out,Curr_in,Curr_out,Chassis_Curr,Output_Power;

extern void buckboost_ctrl(void);//buckboost环路控制
extern void pwm_update(uint32_t compareValue);//buckboost_PWM更新函数
extern void power_start();
extern void buckboost_init(void);
extern void math(void);
#endif