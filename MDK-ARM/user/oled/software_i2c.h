#ifndef __SOFTWARE_I2C_H__
#define __SOFTWARE_I2C_H__

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"

/** Software_I2C GPIO Ports Clock Definition 
 * GPIOG ------> SOFTWARE_I2C_GPIO_PORT
**/
#define __HAL_RCC_CLK_ENABLE(_GPIO_PORT_) __HAL_RCC_##_GPIO_PORT_##_CLK_ENABLE()
#define __HAL_RCC_SOFTWARE_I2C_GPIO_PORT_CLK_ENABLE() __HAL_RCC_CLK_ENABLE(GPIOB)

/** Software_I2C GPIO Definition
 * PB2 ------> Software_I2C_SCL
 * PB1 ------> Software_I2C_SDA 
**/
#define SOFTWARE_I2C_GPIO_PORT GPIOB
#define SOFTWARE_I2C_SCL_PIN GPIO_PIN_2
#define SOFTWARE_I2C_SDA_PIN GPIO_PIN_1

/* Software_I2C_Pin Read and Write */
#define I2C_SCL_WritePin(_x) HAL_GPIO_WritePin(SOFTWARE_I2C_GPIO_PORT, SOFTWARE_I2C_SCL_PIN, ((_x) ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define I2C_SDA_WritePin(_x) HAL_GPIO_WritePin(SOFTWARE_I2C_GPIO_PORT, SOFTWARE_I2C_SDA_PIN, ((_x) ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define I2C_SDA_ReadPin() HAL_GPIO_ReadPin(SOFTWARE_I2C_GPIO_PORT, SOFTWARE_I2C_SDA_PIN)

void Delay_us(uint32_t xus);
void Software_I2C_GPIO_Init(void);
void Software_I2C_Start(void);
void Software_I2C_Stop(void);
uint8_t Software_I2C_WaitACK(void);
void Software_I2C_ACK(void);
void Software_I2C_NACK(void);
void Software_I2C_WriteByte(uint8_t WriteData);
uint8_t Software_I2C_ReadByte(void);

#endif //__SOFTWARE_I2C_H__
