# STM32的HAL库实现OLED_SSD1306显示
### 已通过STM32F407ZGT6+STM32Cube_FW_F4_V1.25.2实现

> 通过 `OLED_SSD1306.h` 的宏定义实现软/硬件i2c的切换
---
> 硬件I2C
> - 基于hal库实现，直接通过CubeMX配置i2c即可
> - 默认使用i2c1，课直接在 `OLED_SSD1306.h` 中的 `#define I2Cx hi2c1` 更改i2c句柄，如下代码部分
> ```C
> #ifdef USING_HARDWARE_I2C
> #include "i2c.h"
> #include "FontLib.h"
> #define I2Cx hi2c1
> #endif
> ```
> 注：需要将i2c中断打开，优先级最高
---
> 软件I2C
> - 基于HAL库 `software_i2c.h` 中更改相关GPIO端口和引脚即可([git@gitee.com:KANG-H/STM32_HAL_Software_I2C.git](https://gitee.com/KANG-H/STM32_HAL_Software_I2C.git))
> - 默认使用GPIOG端口，可在 `Software_I2C/software_i2c.h` 中更改相关GPIO引脚
> PG13 ------> Software_I2C_SCL
> PG14 ------> Software_I2C_SDA
---
> main
> ```C
> int main(void)
>{
>  /* USER CODE BEGIN 1 */
>
>  /* USER CODE END 1 */
>
>  /* MCU Configuration--------------------------------------------------------*/
>
>  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
>  HAL_Init();
>
>  /* USER CODE BEGIN Init */
>
>  /* USER CODE END Init */
>
>  /* Configure the system clock */
>  SystemClock_Config();
>
>  /* USER CODE BEGIN SysInit */
>
>  /* USER CODE END SysInit */
>
>  /* Initialize all configured peripherals */
>  MX_GPIO_Init();
>  MX_I2C1_Init();
>  MX_USART1_UART_Init();
>  /* USER CODE BEGIN 2 */
>
>  OLED_Init();
>  OLED_ShowChar(0, 0, 'A', FontSize8x16, 1);
>  OLED_ShowStr(0, 2, (uint8_t *)"123abc!@#$^", FontSize6x8, 0);
>  OLED_ShowCN(0, 6, 1, 1);
>  OLED_LocalFill(70, 3, 80, 4, Bright);
>  // OLED_ShowBMP(0, 0, 127, 7, (uint8_t *)TestBMP);
>
>  /* USER CODE END 2 */
>
>  /* Infinite loop */
>  /* USER CODE BEGIN WHILE */
>  while (1)
>  {
>    /* USER CODE END WHILE */
>
>    /* USER CODE BEGIN 3 */
>    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
>    HAL_Delay(500);
>    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
>    HAL_Delay(500);
>  }
>  /* USER CODE END 3 */
>}
>```