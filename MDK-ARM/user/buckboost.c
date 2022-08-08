#include "buckboost.h"
//#include "adc.h"
#include "pid.h"
#include "Filter.h"
#include "math.h"
float V_in,V_out,Curr_in,Curr_out,Chassis_Curr;
float buff_1,buff_2,buff_3,buff_4,buff_5;

float Output_Power;

ELEC_value_struct Fuck;
uint32_t fuck;
PID_STRUCT Vin_loop;
PID_STRUCT Vout_loop;
PID_STRUCT Cout_loop;
PID_STRUCT Cin_loop;
PID_STRUCT DCDC_Cin_loop;

float capcurr;
/*无论使用MCU如何
	本程序均使用DMA方式传输ADC收到的数据
	无论ADC配置数量如何
	均以当前配置的ADC标号从小到大开始数通道
	前两个通道依次为测量输入电压，电容电压（输出电压）
	剩下的通道依次分别测量输入电流，电容充电电流（输出电流），
	底盘电流。
*/
 //F334
uint16_t adc1_in_buff[adc1_channalNum] = {0,0,0};
uint16_t adc2_in_buff[adc2_channalNum] = {0};


//输入电气参数初始化
void buckboost_init(void)
{
		//设定值整定
	
		Fuck.targetPower = 80.0f;
		Fuck.targetCapCurrent = 3.0f;
		Fuck.Votage_ratio = 20.0f;
		Fuck.Current_ratio = 4.0f ;
		Fuck.targetCapVoatge = 22.0f;
		
	 //pid
		pid_func.reset(&Vout_loop);
    Vout_loop.T       = 0.50f;//PID控制周期，单位100us
    Vout_loop.Kp      = 18.70f;      
    Vout_loop.Ti      = 3.00f;
    Vout_loop.Td      = 0.01f;
    Vout_loop.Ek_Dead = 0.01f;
    Vout_loop.OutMin  = 0.04f * DP_PWM_PER;//最小占空比
    Vout_loop.OutMax  = 1.70f * DP_PWM_PER;//最大占空比
    pid_func.init(&Vout_loop);
		
	  pid_func.reset(&Cout_loop);
    Cout_loop.T       = 0.40f;//PID控制周期，单位100us
    Cout_loop.Kp      = 12.0f;
    Cout_loop.Ti      = 1.50f;
    Cout_loop.Td      = 0.01f;
    Cout_loop.Ek_Dead = 0.01f;
    Cout_loop.OutMin  = 0.04f * DP_PWM_PER;//最小占空比
    Cout_loop.OutMax  = 1.70f * DP_PWM_PER;//最大占空比
    pid_func.init(&Cout_loop);
		
		pid_func.reset(&DCDC_Cin_loop);
    DCDC_Cin_loop.T       = 0.40f;//PID控制周期，单位100us
    DCDC_Cin_loop.Kp      = 13.5f;
    DCDC_Cin_loop.Ti      = 2.42f;
    DCDC_Cin_loop.Td      = 0.01f;
    DCDC_Cin_loop.Ek_Dead = 0.01f;
    DCDC_Cin_loop.OutMin  = 0.04f * DP_PWM_PER;//最小占空比
    DCDC_Cin_loop.OutMax  = 1.70f * DP_PWM_PER;//最大占空比
    pid_func.init(&DCDC_Cin_loop);
		
		pid_func.reset(&Cin_loop);
    Cin_loop.T       = 0.40f;//PID控制周期，单位100us
    Cin_loop.Kp      = 10.5f;
    Cin_loop.Ti      = 2.42f;
    Cin_loop.Td      = 0.01f;
    Cin_loop.Ek_Dead = 0.01f;
    Cin_loop.OutMin  = 0.04f * DP_PWM_PER;//最小占空比
    Cin_loop.OutMax  = 1.70f * DP_PWM_PER;//最大占空比
    pid_func.init(&Cin_loop);
		
		
}
/*void adc_liear_calc()
{
	
}
*/
void pwm_update(uint32_t pwm_cmp_value)
{

  uint32_t buck_duty  = 0;
  uint32_t boost_duty = 0;

  /* 
   * 升降压的PWM方案：
   * 1.由于MOS采用自举电容驱动方式，故上管不能100%占空比导通，这里我们设为90%。
   * 2.本函数的输入参数为总的占空比，需要分别算出两个半桥的占空比。
   * 3.降压工作时，BOOST的半桥的占空比是固定的为90%，通过改变BUCK半桥占空比实现稳压。
   * 3.升压或等压工作时，BUCK半桥的占空比是固定的为90%，通过改变BOOST半桥占空比实现稳压。
   */  

  if( pwm_cmp_value >= MAX_PWM_CMP)
  {
    boost_duty  = pwm_cmp_value - MAX_PWM_CMP + 0.06F * HRTIM1->sMasterRegs.MPER;//BOOST半桥占空比计算
  }
  else
  {
    boost_duty  = 0.06F * HRTIM1->sMasterRegs.MPER;  //BUCK模式,boost_duty给固定占空比
  }

  if( pwm_cmp_value >= MAX_PWM_CMP)
  {
    buck_duty  = MAX_PWM_CMP;   //BOOST模式,buck_duty给固定占空比
  }
  else
  {
    buck_duty  = pwm_cmp_value;//BUCK半桥占空比设定
  }

  /*
   *  占空比限制
   */
  if( boost_duty > MAX_PWM_CMP )
		boost_duty = MAX_PWM_CMP;
  if( boost_duty < MIN_PWM_CMP )
		boost_duty = MIN_PWM_CMP;
  if( buck_duty  > MAX_PWM_CMP )
		buck_duty  = MAX_PWM_CMP;
  if( buck_duty  < MIN_PWM_CMP )
		buck_duty  = MIN_PWM_CMP;

  /*
   *  更新比较寄存器
   */
  HRTIM1->sMasterRegs.MCMP1R  = ( DP_PWM_PER - buck_duty  ) >> 1;
  HRTIM1->sMasterRegs.MCMP2R  = ( DP_PWM_PER + buck_duty  ) >> 1;
  HRTIM1->sMasterRegs.MCMP3R  = ( DP_PWM_PER + boost_duty ) >> 1;
  HRTIM1->sMasterRegs.MCMP4R  = ( DP_PWM_PER - boost_duty ) >> 1;

}
void power_start()
{
	  float m_pwm_cmp;
	  //static float CinLoop_out;
	 //adc读入值滤波处理
		buff_1 = GetAverage(adc1_in_buff,0);
		
		//fuck = adc2_in_buff[0];
    buff_2 = GetAverage(adc1_in_buff,1);
		
		buff_3 = GetAverage(adc1_in_buff,2);
	 // buff_4 = GetAverage(adc1_in_buff,2);
	  //buff_5 = GetAverage(adc2_in_buff,0);
		//电气实际值整定
		V_in = buff_1 /4095.0 *3.3f *  Fuck.Votage_ratio;
		V_out = buff_2 /4095.0 *3.3f * Fuck.Votage_ratio;
		//Curr_in = buff_3 /4095.0 *3.3f *  Fuck.Current_ratio;
		Curr_out = (buff_3 /4095.0 *3.3f)*3.7636f - 0.0263f;
		//Chassis_Curr = buff_5 /4095.0 *3.3f *  Fuck.Current_ratio;
	  
	/*
   *  输出电压环PID计算
   */
    Vout_loop.Ref = Fuck.targetCapVoatge;
    Vout_loop.Fdb = V_out;
    pid_func.calc( &Vout_loop );  // 输出电压环PID计算

  /*
   *  输出电流环PID计算
   */
	 capcurr = Fuck.targetPower / V_out;
	 
	 if(capcurr > 5)
	 {
		 capcurr = 5;
	 }
	 else if(V_out > 21)
	 {
		 capcurr = 0.1;
	 }
	 
	 capcurr *= 0.9;
	 
   Cout_loop.Ref = capcurr;
   Cout_loop.Fdb = Curr_out;
   pid_func.calc( &Cout_loop );  // 电流环PID计算
  
   m_pwm_cmp = fminf( Vout_loop.Output, Cout_loop.Output);// 取两个环路中较小的输出值
	 //m_pwm_cmp = Vout_loop.Output;
	 Vout_loop.Output = m_pwm_cmp;// 保存本次PWM比较值
   Cout_loop.Output = m_pwm_cmp;// 保存本次PWM比较值
		
    pwm_update( m_pwm_cmp );         // 更新PWM
	
	/*
	//分割线
	//Cin闭环指将底盘电流与dcdc变换器的电流相加，与目标电流（裁判系统设定电流闭环）
	Cin_loop.Ref = Fuck.targetPower / V_in;
	Cin_loop.Fdb = Chassis_Curr + Curr_in;
	pid_func.calc(&Cin_loop);
	CinLoop_out = Cin_loop.Output;
	if(CinLoop_out > 0)//充电
	{
		Vout_loop.Ref = Fuck.targetCapVoatge;
    Vout_loop.Fdb = V_out;
    pid_func.calc( &Vout_loop );
		
		Cout_loop.Ref = Fuck.targetCapCurrent;
    Cout_loop.Fdb = Curr_out;
    pid_func.calc( &Cout_loop );
		
		m_pwm_cmp = fminf( Vout_loop.Output, Cout_loop.Output);
		Vout_loop.Output = m_pwm_cmp;// 保存本次PWM比较值
    Cout_loop.Output = m_pwm_cmp;// 保存本次PWM比较值
		pwm_update( m_pwm_cmp ); 
		
	}
	else if(CinLoop_out < 0)//放电
	{
		
		DCDC_Cin_loop.Ref = Fuck.targetCapCurrent;
		DCDC_Cin_loop.Fdb = Curr_in;
		pid_func.calc(&DCDC_Cin_loop);
		m_pwm_cmp = DCDC_Cin_loop.Output;
		pwm_update(m_pwm_cmp);
	}
	*/
	
}
void math(void)
{
	
	Output_Power = V_out * 	Curr_out;

}
	