
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/* vim: set ai et ts=4 sw=4: */
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include "r820t2.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff),
                      HAL_MAX_DELAY);
    va_end(args);
}

HAL_StatusTypeDef UART_ReceiveString(
        UART_HandleTypeDef *huart, uint8_t *pData,
        uint16_t Size, uint32_t Timeout) {
    const char newline[] = "\r\n";
    const char delete[] = "\x08 \x08";
    HAL_StatusTypeDef status;

    if(Size == 0)
        return HAL_ERROR;

    int i = 0;
    for(;;) {
        status = HAL_UART_Receive(huart, &pData[i], 1, Timeout);
        if(status != HAL_OK)
            return status;

        if((pData[i] == '\x08')||(pData[i] == '\x7F')) { // backspace
            if(i > 0) {
                status = HAL_UART_Transmit(huart, (uint8_t*)delete,
                                           sizeof(delete)-1, Timeout);
                if(status != HAL_OK)
                    return status;
                i--;
            }
            continue;
        }

        if((pData[i] == '\r') || (pData[i] == '\n')) {
            pData[i] = '\0';
            status = HAL_UART_Transmit(huart, (uint8_t*)newline,
                                       sizeof(newline)-1, Timeout);
            if(status != HAL_OK)
                return status;
            break;
        }

        // last character is reserved for '\0'
        if(i == (Size-1)) {
            continue; // buffer is full, ignore any input
        }

        status = HAL_UART_Transmit(huart, &pData[i], 1, Timeout);
        if(status != HAL_OK)
            return status;
        i++;
    }

    return HAL_OK;
}

void init() {
    /* do nothing */
}

void cmd_help() {
    UART_Printf("help              - show this message\r\n");
    UART_Printf("scan              - perform I2C scan\r\n");
    UART_Printf("dump              - read all registers\r\n");
    UART_Printf("read <reg>        - read given register value\r\n"
                "                    (e.g `read 0A`)\r\n");
    UART_Printf("write <reg> <val> - write <val> to register <reg>\r\n"
                "                    (e.g. `write 0A E1`)\r\n");
    UART_Printf("init              - initialize R820T2\r\n");
    UART_Printf("calibrate         - calibrate R820T2\r\n");
    UART_Printf("frequency <val>   - set frequency to <val>\r\n"
                "                    (e.g. `frequency 144000000`)\r\n");
    UART_Printf("bandwidth <val>   - Set IF bandwidth [0-15]\r\n");
    UART_Printf("lna_gain <val>    - Set gain of LNA [0-15]\r\n");
    UART_Printf("vga_gain <val>    - Set gain of VGA [0-15]\r\n");
    UART_Printf("mixer_gain <val>  - Set gain Mixer [0-15]\r\n");
    UART_Printf("lna_agc <val>     - Enable/disable LNA AGC [0-1]\r\n");
    UART_Printf("mixer_agc <val>   - Enable/disable Mixer AGC [0-1]\r\n");
}

void cmd_scan() {
    HAL_StatusTypeDef res;
    for(uint16_t i = 0; i < 128; i++) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
        if(res == HAL_OK) {
            UART_Printf("0x%02X ", i);
        } else {
            UART_Printf(".");
        }
    }  
    UART_Printf("\r\n");
}

void cmd_dump() {
    uint8_t regs[R820T2_NUM_REGS];
    R820T2_read(0x00, regs, sizeof(regs));
    for(uint8_t i = 0; i < R820T2_NUM_REGS; i++) {
        UART_Printf("%02X ", regs[i]);
        if((i & 0x7) == 0x7) {
            UART_Printf("  ");
        }
        if((i & 0xF) == 0xF) {
            UART_Printf("\r\n");
        }
    }
}

void cmd_read(uint8_t reg) {
    if(reg >= R820T2_NUM_REGS) {
        UART_Printf("Out of bound: 0x00-0x%02X\r\n", R820T2_NUM_REGS);
        return;
    }

    uint8_t val = R820T2_read_reg(reg);
    UART_Printf("%02X\r\n", val);
}

void cmd_write(uint8_t reg, uint8_t val) {
    if(reg >= R820T2_NUM_REGS) {
        UART_Printf("Out of bound: 0x00-0x%02X\r\n", R820T2_NUM_REGS);
        return;
    }

    R820T2_write_reg(reg, val);
}

void cmd_init() {
    R820T2_init(); 
}

void cmd_calibrate() {
    int32_t res = R820T2_calibrate(); 
    if(res != 0) {
        UART_Printf("Calibration failed, res = %d\r\n", res);
    }
}

void cmd_frequency(uint32_t val) {
    if((val < 24000000) || (val > 1766000000)) {
        UART_Printf("Out of bound: 24000000-1766000000\r\n");
        return;
    }
    R820T2_set_frequency(val); 
}

void cmd_bandwidth(uint32_t val) {
    if(val > 15) {
        UART_Printf("Out of bound: 0-15\r\n");
        return;
    }

    R820T2_set_bandwidth(val); 
}

void cmd_lna_gain(uint32_t val) {
    if(val > 15) {
        UART_Printf("Out of bound: 0-15\r\n");
        return;
    }

    R820T2_set_lna_gain(val); 
}

void cmd_vga_gain(uint32_t val) {
    if(val > 15) {
        UART_Printf("Out of bound: 0-15\r\n");
        return;
    }

    R820T2_set_vga_gain(val); 
}

void cmd_mixer_gain(uint32_t val) {
    if(val > 15) {
        UART_Printf("Out of bound: 0-15\r\n");
        return;
    }

    R820T2_set_mixer_gain(val); 
}

void cmd_lna_agc(uint32_t val) {
    if(val > 1) {
        UART_Printf("Out of bound: 0-15\r\n");
        return;
    }

    R820T2_set_lna_agc(val); 
}

void cmd_mixer_agc(uint32_t val) {
    if(val > 1) {
        UART_Printf("Out of bound: 0-1\r\n");
        return;
    }

    R820T2_set_mixer_agc(val); 
}

void loop() {
    unsigned int uint1, uint2;
    char cmd[128];
    UART_Printf("r820t2> ");
    UART_ReceiveString(&huart1, (uint8_t*)cmd, sizeof(cmd), HAL_MAX_DELAY);

    if(strcmp(cmd, "") == 0) {
        /* empty command - do nothing */
    } else if(strcmp(cmd, "help") == 0) {
        cmd_help();
    } else if(strcmp(cmd, "scan") == 0) {
        cmd_scan();
    } else if(strcmp(cmd, "init") == 0) {
        cmd_init();
    } else if(strcmp(cmd, "calibrate") == 0) {
        cmd_calibrate();
    } else if(strcmp(cmd, "dump") == 0) {
        cmd_dump();
    } else if(sscanf(cmd, "frequency %u", &uint1) == 1) {
        cmd_frequency((uint32_t)uint1);
    } else if(sscanf(cmd, "bandwidth %u", &uint1) == 1) {
        cmd_bandwidth((uint32_t)uint1);
    } else if(sscanf(cmd, "lna_gain %u", &uint1) == 1) {
        cmd_lna_gain((uint32_t)uint1);
    } else if(sscanf(cmd, "vga_gain %u", &uint1) == 1) {
        cmd_vga_gain((uint32_t)uint1);
    } else if(sscanf(cmd, "mixer_gain %u", &uint1) == 1) {
        cmd_mixer_gain((uint32_t)uint1);
    } else if(sscanf(cmd, "lna_agc %u", &uint1) == 1) {
        cmd_lna_agc((uint32_t)uint1);
    } else if(sscanf(cmd, "mixer_agc %u", &uint1) == 1) {
        cmd_mixer_agc((uint32_t)uint1);
    } else if(sscanf(cmd, "read %02x", &uint1) == 1) {
        cmd_read((uint8_t)uint1);
    } else if(sscanf(cmd, "write %02x %02x", &uint1, &uint2) == 2) {
        cmd_write((uint8_t)uint1, (uint8_t)uint2);
    } else {
        UART_Printf("Unknown command, try `help`\r\n");
    }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  init();
  while (1)
  {
    loop();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ULED_GPIO_Port, ULED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ULED_Pin */
  GPIO_InitStruct.Pin = ULED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ULED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
