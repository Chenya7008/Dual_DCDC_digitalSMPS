#include "software_i2c.h"

static void SET_SDA_Output_Mode(void);
static void SET_SDA_Input_Mode(void);
static void SET_SCL_Output_Mode(void);

/**
 * @function: void Delay_us(uint32_t xus)
 * @description: 基于HAL库的微秒延时函数
 * @param {uint32_t} xus待延时的时间(单位:us)
 * @return {*}
 */
void Delay_us(uint32_t xus)
{
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000000);
  HAL_Delay(xus - 1);
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
}

/**
 * @function: void Software_I2C_GPIO_Init(void)
 * @description: 软件I2C的GPIO初始化配置
 * @param {*}
 * @return {*}
 */
void Software_I2C_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_SOFTWARE_I2C_GPIO_PORT_CLK_ENABLE();
  SET_SCL_Output_Mode();
  SET_SDA_Output_Mode();
  I2C_SCL_WritePin(1);
  I2C_SDA_WritePin(1);
}

/**
 * @function: static void SET_SDA_Output_Mode(void)
 * @description: 设置I2C的SDA线为输出模式
 * @param {*}
 * @return {*}
 */
static void SET_SDA_Output_Mode(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = SOFTWARE_I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SOFTWARE_I2C_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @function: static void SET_SCL_Output_Mode(void)
 * @description: 设置I2C的SCL线为输出模式
 * @param {*}
 * @return {*}
 */
static void SET_SCL_Output_Mode(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = SOFTWARE_I2C_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SOFTWARE_I2C_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @function: static void SET_SDA_Input_Mode(void)
 * @description: 设置I2C的SDA线为输入模式
 * @param {*}
 * @return {*}
 */
static void SET_SDA_Input_Mode(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = SOFTWARE_I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SOFTWARE_I2C_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @function: void Software_I2C_Start(void)
 * @description: 软件I2C起始信号
 * @param {*}
 * @return {*}
 */
void Software_I2C_Start(void)
{
  SET_SDA_Output_Mode();
  I2C_SDA_WritePin(1);
  I2C_SCL_WritePin(1);
  Delay_us(5);
  I2C_SDA_WritePin(0);
  Delay_us(5);
  I2C_SCL_WritePin(0);
  Delay_us(2);
}

/**
 * @function: void Software_I2C_Stop(void)
 * @description: 软件I2C结束信号
 * @param {*}
 * @return {*}
 */
void Software_I2C_Stop(void)
{
  SET_SDA_Output_Mode();
  I2C_SCL_WritePin(1);
  I2C_SDA_WritePin(0);
  Delay_us(5);
  I2C_SDA_WritePin(1);
  Delay_us(2);
}

/**
 * @function: uint8_t Software_I2C_WaitACK(void)
 * @description: 等待应答信号
 * @param {*}
 * @return {1}没有应答
 * @return {0}产生应答
 */
uint8_t Software_I2C_WaitACK(void)
{
  uint8_t ErrorTime = 0;
  SET_SDA_Input_Mode();
  I2C_SDA_WritePin(1);
  I2C_SCL_WritePin(1);
  while (I2C_SDA_ReadPin())
  {
    ErrorTime++;
    if (ErrorTime > 250)
    {
      Software_I2C_Stop();
      return 1;
    }
  }
  Delay_us(4);
  I2C_SCL_WritePin(0);
  Delay_us(5);
  return 0;
}

/**
 * @function: void Software_I2C_ACK(void)
 * @description: 产生应答信号
 * @param {*}
 * @return {*}
 */
void Software_I2C_ACK(void)
{
  I2C_SCL_WritePin(0);
  SET_SDA_Output_Mode();
  I2C_SDA_WritePin(0);
  Delay_us(2);
  I2C_SCL_WritePin(1);
  Delay_us(5);
  I2C_SCL_WritePin(0);
}

/**
 * @function: void Software_I2C_NACK(void)
 * @description: 产生非应答信号
 * @param {*}
 * @return {*}
 */
void Software_I2C_NACK(void)
{
  I2C_SCL_WritePin(0);
  SET_SDA_Output_Mode();
  I2C_SDA_WritePin(1);
  Delay_us(2);
  I2C_SCL_WritePin(1);
  Delay_us(5);
  I2C_SCL_WritePin(0);
}

/**
 * @function: void Software_I2C_WriteByte(uint8_t WriteData)
 * @description: 写入一个字节数据
 * @param {uint8_t} WriteData待写入的一个字节数据
 * @return {*}
 */
void Software_I2C_WriteByte(uint8_t WriteData)
{
  SET_SDA_Output_Mode();
  I2C_SCL_WritePin(0);
  for (uint8_t i = 0; i < 8; i++)
  {
    I2C_SDA_WritePin((WriteData & 0x80) >> 7); //高位先写
    WriteData <<= 1;
    Delay_us(4);
    I2C_SCL_WritePin(1);
    Delay_us(5);
    I2C_SCL_WritePin(0);
    Delay_us(5);
  }
}

/**
 * @function: uint8_t Software_I2C_ReadByte(void)
 * @description: 读取一个字节数据
 * @param {uint8_t} ReadData待写入的一个字节数据
 * @return {*}
 */
uint8_t Software_I2C_ReadByte(void)
{
  uint8_t ReadData = 0;
  SET_SDA_Input_Mode();
  for (uint8_t i = 0; i < 8; i++)
  {
    I2C_SCL_WritePin(0);
    Delay_us(5);
    I2C_SCL_WritePin(1);
    ReadData <<= 1;
    if(I2C_SDA_ReadPin()) //读取到1，则最低位置一
      ReadData |= 0x01;
    Delay_us(5);
  }
  return ReadData;
}
