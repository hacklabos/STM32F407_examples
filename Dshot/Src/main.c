#include "main.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_update;


#define MOTOR_PWM_BIT_1   14 //3
#define MOTOR_PWM_BIT_0   7  // 2
#define MOTOR_PWM_BIT_WIDTH 20 //4
#define BUFFER_SIZE 18

uint32_t dmaBuffer[MOTORS_NUMBER][BUFFER_SIZE] = {0};
uint32_t dmaBurstBuffer[MOTORS_NUMBER*BUFFER_SIZE] = {0};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

void dataPrepare(uint16_t throttle, uint32_t* dmaBuffer);
void dshotBufferPrepare(uint32_t bufferIn[MOTORS_NUMBER][BUFFER_SIZE], uint32_t* bufferOut);

int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();

    HAL_TIM_Base_Start_IT(&htim2);

    uint16_t throttle;

    throttle = 0;
    dataPrepare(throttle, &(dmaBuffer[MOTOR1][0]));
    throttle = 0;
    dataPrepare(throttle, &(dmaBuffer[MOTOR2][0]));
    throttle = 0;
    dataPrepare(throttle, &(dmaBuffer[MOTOR3][0]));
    throttle = 0;
    dataPrepare(throttle, &(dmaBuffer[MOTOR4][0]));
    dshotBufferPrepare(dmaBuffer, dmaBurstBuffer);

    HAL_Delay(1000);
    throttle = 7;
    dataPrepare(throttle, &(dmaBuffer[MOTOR2][0]));
    dshotBufferPrepare(dmaBuffer, dmaBurstBuffer);

    HAL_Delay(1000);
    throttle = 100;
    dataPrepare(throttle, &(dmaBuffer[MOTOR2][0]));
    dshotBufferPrepare(dmaBuffer, dmaBurstBuffer);

    HAL_Delay(1000);
    throttle = 0;
    dataPrepare(throttle, &(dmaBuffer[MOTOR2][0]));
    dshotBufferPrepare(dmaBuffer, dmaBurstBuffer);

    while (1)
    {
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
    Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
    Error_Handler();
    }
}

static void MX_TIM1_Init(void)
{
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 8; //32
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = MOTOR_PWM_BIT_WIDTH;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
    Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
    Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
    Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);

}

static void MX_TIM2_Init(void)
{
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84.;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
    Error_Handler();
    }
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
    {
    Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
    Error_Handler();
    }
}


//0-47 are reserved, 48-2047 give 2000 steps of throttle resolution
void dataPrepare(uint16_t throttle, uint32_t* dmaBuffer)
{
    // 1-11    - throttle
    // 12      - telemetry
    // 13-16   - checksum
    uint16_t packet = 0;
    uint16_t telemetry = 0;
    uint16_t checksum = 0;

    packet |= throttle << 5;
    packet |= telemetry << 4;

    uint32_t i;
    uint16_t csum_data = packet;

    csum_data >>= 4;
    for (i = 0; i < 3; i++) {
     checksum ^= (csum_data & 0x0F); // xor data by nibbles
     csum_data >>= 4;
    }

    packet |= (checksum & 0x0F);

    for(i = 0; i < 16; i++) {
        dmaBuffer[i] = (packet & 0x8000) ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;  // MSB first
        packet <<= 1;
    }

    dmaBuffer[16] = 0;
    dmaBuffer[17] = 0;
}

void dshotBufferPrepare(uint32_t bufferIn[MOTORS_NUMBER][BUFFER_SIZE], uint32_t* bufferOut)
{
    for(uint32_t i = 0; i < BUFFER_SIZE ; i++)
    {
        for(uint32_t j = 0; j < MOTORS_NUMBER; j++)
        {
            bufferOut[i*MOTORS_NUMBER+j] = bufferIn[j][i];
        }
    }
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    }

static void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);

    /*
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *) &(dmaBuffer[MOTOR4][0]), BUFFER_SIZE);
    while(HAL_DMA_STATE_BUSY == htim1.hdma[TIM_DMA_ID_CC1]->State) {}
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *) &(dmaBuffer[MOTOR4][0]), BUFFER_SIZE);
    while(HAL_DMA_STATE_BUSY == htim1.hdma[TIM_DMA_ID_CC2]->State) {}
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *) &(dmaBuffer[MOTOR4][0]), BUFFER_SIZE);
    while(HAL_DMA_STATE_BUSY == htim1.hdma[TIM_DMA_ID_CC3]->State) {}
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t *) &(dmaBuffer[MOTOR4][0]), BUFFER_SIZE);
    while(HAL_DMA_STATE_BUSY == htim1.hdma[TIM_DMA_ID_CC4]->State) {}
    */

    //HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t *) &(dmaBuffer[MOTOR4][0]), BUFFER_SIZE);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_DMABurst_WriteStart(&htim1, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, dmaBurstBuffer, TIM_DMABURSTLENGTH_4TRANSFERS, MOTORS_NUMBER*BUFFER_SIZE);
    //while(HAL_DMA_STATE_BUSY == htim1.hdma[TIM_DMA_ID_CC4]->State) {}
}

#ifdef  USE_FULL_ASSERT
    /**
    * @brief  Reports the name of the source file and the source line number
    *         where the assert_param error has occurred.
    * @param  file: pointer to the source file name
    * @param  line: assert_param error line source number
    * @retval None
    */
void assert_failed(uint8_t *file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
