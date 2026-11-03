/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Roller Shutter Control System for STM32G474RE
  *
  * 【專案設計提示詞與硬體規格摘要】
  * 1. 系統時脈：HSI 配置 PLL，核心主頻 170 MHz (FLASH_LATENCY_4, 電壓調節 Scale 1)。
  * 2. PWM 輸出 (漸亮呼吸燈)：
     - PA5 (TIM2_CH1)：綠燈，上升動作 0%~99% 漸亮 (Prescaler = 170-1, Period = 99)。
     - PA6 (TIM3_CH1)：黃燈，下降動作 0%~99% 漸亮 (Prescaler = 170-1, Period = 99)。
  * 3. GPIO 輸出：
     - PA7 (Red_LED)：推挽輸出，停止到位告示燈，停機時點亮 3 秒。
     - PB1 (Emergency_LED)：推挽輸出，緊急警報燈，急停時以 250ms 間隔規律閃爍。
  * 4. GPIO 輸入 (皆啟用內部 Pull-up，10ms 防彈跳)：
     - PC6 (Btn_Up)：上升按鈕。
     - PC7 (Btn_Down)：下降按鈕。
     - PC8 (Btn_Stop)：停止按鈕。
  * 5. 外部中斷 (EXTI)：
     - PC9 (Btn_Emergency)：緊急按鈕 (下降沿觸發，EXTI9_5_IRQn 最高優先級，旗標法處理)。
  * 6. 序列埠通訊 (USART2)：
     - PA2 (TX) / PA3 (RX)，115200 8N1，啟用中斷接收，支援終端機快捷指令。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
  * @brief 智慧鐵捲門有限狀態機 (FSM) 狀態定義
  */
typedef enum {
    STATE_IDLE = 0,       /* 0: 完全待機（所有燈號關閉，等待按鈕或指令） */
    STATE_RISING,         /* 1: 上升中（啟動 TIM2 PWM 綠燈 5 秒內漸亮至 99%） */
    STATE_FALLING,        /* 2: 下降中（啟動 TIM3 PWM 黃燈 5 秒內漸亮至 99%） */
    STATE_EMERGENCY,      /* 3: 緊急互鎖/急停（切斷 PWM，紅燈滅，PB1 警報燈 250ms 閃爍） */
    STATE_PAUSE           /* 4: 暫停保持（紅燈點亮維持 3 秒後自動歸零回待機） */
} SystemState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY_MS    10     /* 按鈕防彈跳間隔 (ms) */
#define TRAVEL_DURATION_MS   5000   /* 鐵捲門完整行程時間 (5秒) */
#define EMERGENCY_FLASH_MS   250    /* 緊急警報燈閃爍間隔 (250ms) */
#define PAUSE_DURATION_MS    3000   /* 暫停紅燈維持時間 (3秒) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* 系統狀態與計時變數 */
static SystemState_t current_state = STATE_IDLE;
static uint32_t state_timer = 0;
static uint32_t flash_timer = 0;
static uint32_t debounce_timer = 0;

/* PLC 聯網日誌統計計數器 */
static uint32_t total_ops = 0;
static uint32_t count_up = 0;
static uint32_t count_down = 0;
static uint32_t count_stop = 0;
static uint32_t count_emergency = 0;

/* 硬體與通訊變數 */
static uint8_t uart_rx_byte;
static volatile uint8_t emergency_flag = 0;
static GPIO_PinState last_btn_up = GPIO_PIN_SET;
static GPIO_PinState last_btn_down = GPIO_PIN_SET;
static GPIO_PinState last_btn_stop = GPIO_PIN_SET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Log_System_Status(const char* cmd_name);
void Process_Command(char cmd);
void Handle_Buttons(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  PLC 聯網日誌 (PLC Logger)
  * @note   每當有動作被觸發時，透過 USART2 傳送 Uptime 運行秒數以及各項操作累計次數
  */
void Log_System_Status(const char* cmd_name) {
    char tx_buf[128];
    uint32_t uptime_sec = HAL_GetTick() / 1000;

    int len = snprintf(tx_buf, sizeof(tx_buf),
        "[PLC LOG] Cmd: %s | Uptime: %lu s | Total: %lu | Up: %lu | Down: %lu | Stop: %lu | Emergency: %lu\r\n",
        cmd_name, uptime_sec, total_ops, count_up, count_down, count_stop, count_emergency);

    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, len, HAL_MAX_DELAY);
}

/**
  * @brief  終端機指令與按鈕事件處理器
  * @note   支援對應指令：
  *         - u / U -> 觸發上升
  *         - d / D -> 觸發下降
  *         - s / S -> 觸發停止
  *         - e / E / c / C -> 切換/解除緊急互鎖
  */
void Process_Command(char cmd) {
    switch (cmd) {
        case 'u':
        case 'U':
            if (current_state == STATE_IDLE) {
                current_state = STATE_RISING;
                state_timer = HAL_GetTick();
                count_up++;
                total_ops++;
                Log_System_Status("UP");
            }
            break;

        case 'd':
        case 'D':
            if (current_state == STATE_IDLE) {
                current_state = STATE_FALLING;
                state_timer = HAL_GetTick();
                count_down++;
                total_ops++;
                Log_System_Status("DOWN");
            }
            break;

        case 's':
        case 'S':
            if (current_state == STATE_RISING || current_state == STATE_FALLING) {
                current_state = STATE_PAUSE;
                state_timer = HAL_GetTick();
                // 立即關閉 PWM 輸出
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                // 點亮停止到位告示紅燈 (PA7)
                HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
                count_stop++;
                total_ops++;
                Log_System_Status("STOP");
            }
            break;

        case 'e':
        case 'E':
        case 'c':
        case 'C':
            if (current_state != STATE_EMERGENCY) {
                current_state = STATE_EMERGENCY;
                flash_timer = HAL_GetTick();
                // 立即切斷所有 PWM 與紅燈
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);

                count_emergency++;
                total_ops++;
                Log_System_Status("EMERGENCY_LOCK");
            } else {
                // 再次觸發則解除鎖定，回到完全待機狀態
                current_state = STATE_IDLE;
                HAL_GPIO_WritePin(Emergency_LED_GPIO_Port, Emergency_LED_Pin, GPIO_PIN_RESET);
                Log_System_Status("EMERGENCY_RELEASE");
            }
            break;

        default:
            break;
    }
}

/**
  * @brief  按鈕防彈跳與實體按鈕輪詢處理 (以 10ms 間隔執行)
  */
void Handle_Buttons(void) {
    if (HAL_GetTick() - debounce_timer >= DEBOUNCE_DELAY_MS) {
        debounce_timer = HAL_GetTick();

        // PC6: Btn_Up (內部 Pull-up，按下為 LOW)
        GPIO_PinState current_up = HAL_GPIO_ReadPin(Btn_Up_GPIO_Port, Btn_Up_Pin);
        if (current_up == GPIO_PIN_RESET && last_btn_up == GPIO_PIN_SET) {
            Process_Command('u');
        }
        last_btn_up = current_up;

        // PC7: Btn_Down (內部 Pull-up)
        GPIO_PinState current_down = HAL_GPIO_ReadPin(Btn_Down_GPIO_Port, Btn_Down_Pin);
        if (current_down == GPIO_PIN_RESET && last_btn_down == GPIO_PIN_SET) {
            Process_Command('d');
        }
        last_btn_down = current_down;

        // PC8: Btn_Stop (內部 Pull-up)
        GPIO_PinState current_stop = HAL_GPIO_ReadPin(Btn_Stop_GPIO_Port, Btn_Stop_Pin);
        if (current_stop == GPIO_PIN_RESET && last_btn_stop == GPIO_PIN_SET) {
            Process_Command('s');
        }
        last_btn_stop = current_stop;
    }
}

/**
  * @brief  UART 非同步接收完成回呼函式
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        Process_Command((char)uart_rx_byte);
        // 重新啟動中斷接收以持續聆聽終端機指令
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

/**
  * @brief  GPIO 外部中斷回呼函式 (處理 PC9 緊急按鈕觸發)
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Btn_Emergency_Pin) {
        emergency_flag = 1; // 設定旗標交由主迴圈安全處理
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // 強制手動初始化 PA5 (TIM2_CH1) 與 PA6 (TIM3_CH1) 為復用推挽輸出，確保綠燈與黃燈 PWM 輸出正常
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 啟動序列埠中斷接收並發送開機日誌
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
  Log_System_Status("SYSTEM_BOOT");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. 處理外部中斷旗標 (安全交由主迴圈處理緊急按鈕)
      if (emergency_flag) {
          emergency_flag = 0;
          Process_Command('e');
      }

      // 2. 實體按鈕輪詢防彈跳
      Handle_Buttons();

      // 3. 有限狀態機 (FSM) 核心運作邏輯
      switch (current_state) {
          case STATE_IDLE:
              // 狀態 0：完全待機，關閉所有燈號
              HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(Emergency_LED_GPIO_Port, Emergency_LED_Pin, GPIO_PIN_RESET);
              break;

          case STATE_RISING: {
              // 狀態 1：上升中，綠燈 (TIM2_CH1) 5 秒內線性從 0% 漸亮至 99%
              uint32_t elapsed = HAL_GetTick() - state_timer;
              if (elapsed < TRAVEL_DURATION_MS) {
                  uint32_t duty = (elapsed * 99) / TRAVEL_DURATION_MS;
                  if (duty > 99) duty = 99;

                  if (HAL_TIM_PWM_GetState(&htim2) == HAL_TIM_STATE_READY) {
                      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
                  }
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
              } else {
                  // 行程超過 5 秒自動切換至暫停保持狀態
                  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
                  current_state = STATE_PAUSE;
                  state_timer = HAL_GetTick();
                  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
                  count_stop++;
                  total_ops++;
                  Log_System_Status("TRAVEL_COMPLETE_UP");
              }
              break;
          }

          case STATE_FALLING: {
              // 狀態 2：下降中，黃燈 (TIM3_CH1) 5 秒內線性從 0% 漸亮至 99%
              uint32_t elapsed = HAL_GetTick() - state_timer;
              if (elapsed < TRAVEL_DURATION_MS) {
                  uint32_t duty = (elapsed * 99) / TRAVEL_DURATION_MS;
                  if (duty > 99) duty = 99;

                  if (HAL_TIM_PWM_GetState(&htim3) == HAL_TIM_STATE_READY) {
                      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                  }
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
              } else {
                  // 行程超過 5 秒自動切換至暫停保持狀態
                  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                  current_state = STATE_PAUSE;
                  state_timer = HAL_GetTick();
                  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
                  count_stop++;
                  total_ops++;
                  Log_System_Status("TRAVEL_COMPLETE_DOWN");
              }
              break;
          }

          case STATE_EMERGENCY: {
              // 狀態 3：緊急互鎖/急停，PB1 緊急警報燈每 250ms 規律閃爍
              if (HAL_GetTick() - flash_timer >= EMERGENCY_FLASH_MS) {
                  flash_timer = HAL_GetTick();
                  HAL_GPIO_TogglePin(Emergency_LED_GPIO_Port, Emergency_LED_Pin);
              }
              break;
          }

          case STATE_PAUSE: {
              // 狀態 4：暫停保持，PA7 紅燈點亮維持 3 秒後自動歸零回到待機
              if (HAL_GetTick() - state_timer >= PAUSE_DURATION_MS) {
                  HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
                  current_state = STATE_IDLE;
                  Log_System_Status("PAUSE_TIMEOUT_IDLE");
              }
              break;
          }

          default:
              current_state = STATE_IDLE;
              break;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  * @note   配置 HSI 透過 PLL 跑 170 MHz，設定 FLASH_LATENCY_4 與電壓調節 Scale 1
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function (綠燈上升動作 PWM, PA5)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 170 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_SLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function (黃燈下降動作 PWM, PA6)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 170 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_SLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function (序列埠通訊 115200 8N1)
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function (設定 LED 輸出與按鈕輸入/中斷)
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Red_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Emergency_LED_GPIO_Port, Emergency_LED_Pin, GPIO_PIN_RESET);

  /* Configure GPIO pin : Red_LED_Pin (PA7) - 停止到位告示燈 */
  GPIO_InitStruct.Pin = Red_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Red_LED_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pin : Emergency_LED_Pin (PB1) - 緊急警報燈 */
  GPIO_InitStruct.Pin = Emergency_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Emergency_LED_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pins : Btn_Up_Pin (PC6) Btn_Down_Pin (PC7) Btn_Stop_Pin (PC8) - 啟用內部 Pull-up */
  GPIO_InitStruct.Pin = Btn_Up_Pin | Btn_Down_Pin | Btn_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure GPIO pin : Btn_Emergency_Pin (PC9) - 外部中斷下降沿觸發 */
  GPIO_InitStruct.Pin = Btn_Emergency_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Btn_Emergency_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init-> NVIC Priority (設定 EXTI9_5_IRQn 為最高優先級 0,0) */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error is required.
  * @param  file: pointer to source file
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t *line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
