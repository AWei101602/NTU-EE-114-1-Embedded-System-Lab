/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    // --- 中斷與計時變數 (Volatile 因為在中斷內修改) ---
    volatile uint32_t start_tick; // Echo 上升緣時間點 (Timer Count)
    volatile uint32_t end_tick;   // Echo 下降緣時間點
    volatile bool is_measuring;   // 標記：是否正在等待回波

    // --- 輸出結果變數 ---
    volatile float distance;      // 最新計算距離 (cm)
    volatile uint8_t valid;       // 數據狀態 (1:有效, 0:更新中/無效)
} Ultrasonic_t;
typedef enum {
    ZONE_NONE = 0,
    ZONE_LEFT,
    ZONE_CENTER,
    ZONE_RIGHT
} Zone_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 1. 左側 (Left) - Trig:D2(PD14), Echo:D3(PB0->EXTI0)
#define L_TRIG_PORT GPIOD
#define L_TRIG_PIN  GPIO_PIN_14
#define L_ECHO_PORT GPIOB
#define L_ECHO_PIN  GPIO_PIN_0

// 2. 中間 (Center) - Trig:D4(PA3), Echo:D5(PB4->EXTI4)
#define C_TRIG_PORT GPIOA
#define C_TRIG_PIN  GPIO_PIN_3
#define C_ECHO_PORT GPIOB
#define C_ECHO_PIN  GPIO_PIN_4

// 3. 右側 (Right) - Trig:D6(PB1), Echo:D8(PB2->EXTI2) [關鍵修改]
#define R_TRIG_PORT GPIOB
#define R_TRIG_PIN  GPIO_PIN_1
#define R_ECHO_PORT GPIOB
#define R_ECHO_PIN  GPIO_PIN_2

// --- 邏輯參數 ---
#define DETECT_THRESHOLD  80.0f  // 偵測門檻 (cm)，超過這個距離視為沒人
#define MOVE_HYSTERESIS   5.0f   // 移動判斷的遲滯/最小變化量 (cm)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
Ultrasonic_t us_L = {0};
Ultrasonic_t us_C = {0};
Ultrasonic_t us_R = {0};

// 任務排程器變數
uint8_t current_sensor_idx = 0; // 目前輪到的感測器 (0:L, 1:C, 2:R)
uint8_t measure_step = 0;       // 狀態機步驟 (0:發射, 1:等待, 2:切換)
uint32_t last_process_tick = 0; // 用於控制測量間隔與超時

Zone_t current_zone = ZONE_NONE;
Zone_t last_zone = ZONE_NONE;
float last_min_distance = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//int _write(int file, char *ptr, int len)
//{
//    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);
//    return len;
//}
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

void delay_us_short(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

// --- 發送 Trigger 脈衝訊號 ---
void US_Trigger(GPIO_TypeDef* port, uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    delay_us_short(10); // 維持 10us High
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/**
 * @brief 讀取 HC-SR04 距離 (含完整除錯機制)
 * @return float: 距離 (cm), 錯誤代碼: -1.0 (等不到回波), -2.0 (回波過長/卡死), -3.0 (觸發前線路異常)
 */
//float HCSR04_Read(void) {
//    uint32_t pMillis;
//    uint32_t val1 = 0;
//    uint32_t val2 = 0;
//    uint32_t pWidth = 0;
//
//    // --- 步驟 0: 檢查感測器狀態 (Pre-check) ---
//    // 在發送訊號前，Echo 應該要是 Low。如果是 High，代表線路浮接、接錯或感測器當機。
//    if (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET) {
//        printf("Error: Echo is already HIGH before trigger! (Check Wiring or Pull-down)\r\n");
//        return -3.0;
//    }
//
//    // --- 步驟 1: 發送 Trigger 訊號 ---
//    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
//    delay_us(10);
//    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
//
//    // --- 步驟 2: 等待 Echo 腳位變高 (開始接收回波) ---
//    pMillis = HAL_GetTick();
//    while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))) {
//        // 超時 10ms 未收到回波 (代表感測器沒反應)
//        if (HAL_GetTick() - pMillis > 10) {
//            printf("NO ECHO (Timeout waiting for HIGH) !! \r\n");
//            return -1.0;
//        }
//    }
//
//    // 記錄 Echo 變高的時刻
//    val1 = __HAL_TIM_GET_COUNTER(&htim2);
//
//    // --- 步驟 3: 等待 Echo 腳位變低 (回波結束) ---
//    pMillis = HAL_GetTick();
//    while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))) {
//        // 超時 50ms (約 8.5公尺，超出規格)
//        // 如果卡在這裡，通常是上面步驟 0 沒過，或者感測器誤判
//        if (HAL_GetTick() - pMillis > 50) {
//            printf("ECHO STUCK HIGH (Timeout waiting for LOW) !! \r\n");
//            return -2.0;
//        }
//    }
//
//    // 記錄 Echo 變低的時刻
//    val2 = __HAL_TIM_GET_COUNTER(&htim2);
//
//    // --- 步驟 4: 計算 ---
//    if (val2 >= val1) {
//        pWidth = val2 - val1;
//    } else {
//        // 處理 Timer 溢位 (假設 32-bit Timer)
//        pWidth = (0xFFFFFFFF - val1) + val2;
//    }
//
//    // 距離 = 時間 * 0.034 / 2
//    float distance = pWidth * 0.034f / 2.0f;
//
//    return distance;
//}

// --- [關鍵] 外部中斷回調函式 (Interrupt Callback) ---
// 當 Echo 腳位電位發生變化時，CPU 會自動跳轉至此
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // 立即讀取當前時間 (Timer計數值)
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);

    Ultrasonic_t *target = NULL;
    GPIO_TypeDef *target_port = NULL;

    // 1. 辨識是哪一個感測器觸發中斷
    if (GPIO_Pin == L_ECHO_PIN) {
        target = &us_L;
        target_port = L_ECHO_PORT;
    }
    else if (GPIO_Pin == C_ECHO_PIN) {
        target = &us_C;
        target_port = C_ECHO_PORT;
    }
    else if (GPIO_Pin == R_ECHO_PIN) {
        target = &us_R;
        target_port = R_ECHO_PORT;
    }

    // 2. 處理時間記錄
    if (target != NULL) {
        // 讀取該腳位目前的電位狀態
        if (HAL_GPIO_ReadPin(target_port, GPIO_Pin) == GPIO_PIN_SET) {
            // [上升緣 RISING]: 回波開始
            target->start_tick = now;
            target->is_measuring = true;
        } else {
            // [下降緣 FALLING]: 回波結束
            if (target->is_measuring) {
                target->end_tick = now;
                target->is_measuring = false;

                // 計算脈衝寬度 (處理 Timer 溢位狀況)
                uint32_t duration;
                if (target->end_tick >= target->start_tick) {
                    duration = target->end_tick - target->start_tick;
                } else {
                    // 若 Timer 在測量期間溢位 (0xFFFFFFFF -> 0)
                    duration = (0xFFFFFFFF - target->start_tick) + target->end_tick;
                }

                // 計算距離: 距離(cm) = 時間(us) * 0.034 / 2
                // 簡化計算: duration * 0.017
                target->distance = (float)duration * 0.017f;
                target->valid = 1; // 標記數據有效
            }
        }
    }
}

// --- [任務] 超音波排程器 (Scheduler Task) ---
// 負責輪詢三個感測器，並處理超時。請在主迴圈中持續呼叫。
void Task_Ultrasonic_Update(void) {
    uint32_t current_time = HAL_GetTick(); // 取得系統毫秒 (ms)

    switch (measure_step) {
        case 0: // [準備/發射階段] Trigger
            // 每次測量間隔 30ms，避免上一輪的聲波殘留干擾
            if (current_time - last_process_tick > 10) {

                // 根據當前索引觸發對應的感測器
                if (current_sensor_idx == 0) US_Trigger(L_TRIG_PORT, L_TRIG_PIN);
                else if (current_sensor_idx == 1) US_Trigger(C_TRIG_PORT, C_TRIG_PIN);
                else if (current_sensor_idx == 2) US_Trigger(R_TRIG_PORT, R_TRIG_PIN);

                // 將該感測器的數據標記為「更新中/無效」
                if(current_sensor_idx == 0) us_L.valid = 0;
                else if(current_sensor_idx == 1) us_C.valid = 0;
                else if(current_sensor_idx == 2) us_R.valid = 0;

                measure_step = 1; // 進入等待階段
                last_process_tick = current_time;
            }
            break;

        case 1: // [等待階段] Wait for Echo
            // 檢查中斷是否已經完成測量 (valid變為1)
            bool done = false;
            if (current_sensor_idx == 0 && us_L.valid) done = true;
            else if (current_sensor_idx == 1 && us_C.valid) done = true;
            else if (current_sensor_idx == 2 && us_R.valid) done = true;

            if (done) {
                measure_step = 2; // 收到回波，準備換下一個
            }
            else if (current_time - last_process_tick > 20) {
                // [超時處理]
                // 如果過了 35ms 還沒收到完整回波 (約 6公尺距離)
                // 強制結束等待，標示為 999 (代表無限遠或遺失)
                if(current_sensor_idx == 0) us_L.distance = 999.0;
                else if(current_sensor_idx == 1) us_C.distance = 999.0;
                else if(current_sensor_idx == 2) us_R.distance = 999.0;

                measure_step = 2; // 強制進下一階段
            }
            break;

        case 2: // [切換階段] Next Sensor
            // 切換索引: 0->1->2->0
            current_sensor_idx++;
            if (current_sensor_idx > 2) current_sensor_idx = 0;

            measure_step = 0; // 回到發射狀態
            // 注意：這裡不更新 last_process_tick，利用 case 0 的檢查來產生間隔
            break;
    }
}

void Task_Movement_Logic(void) {
    static uint32_t logic_timer = 0;
    // 每 200ms 進行一次判斷，避免數值跳動太快看不清
    if (HAL_GetTick() - logic_timer < 400) return;
    logic_timer = HAL_GetTick();

    // 1. 找出最近的物體距離與位置 (Filter Logic)
    float dL = us_L.distance;
    float dC = us_C.distance;
    float dR = us_R.distance;

    // 過濾無效值 (0 或 999)
    if (dL <= 0) dL = 999;
    if (dC <= 0) dC = 999;
    if (dR <= 0) dR = 999;

    float min_dist = 999.0f;
    Zone_t detected_zone = ZONE_NONE;

    // 簡單的最小值比較，找出物體主要在哪一區
    if (dL < DETECT_THRESHOLD) { min_dist = dL; detected_zone = ZONE_LEFT; }
    if (dC < DETECT_THRESHOLD && dC < min_dist) { min_dist = dC; detected_zone = ZONE_CENTER; }
    if (dR < DETECT_THRESHOLD && dR < min_dist) { min_dist = dR; detected_zone = ZONE_RIGHT; }

    // 2. 判斷移動方向 (Movement Analysis)
    char status_msg[64] = "Waiting...";

    if (detected_zone == ZONE_NONE) {
        sprintf(status_msg, "No Target");
        last_zone = ZONE_NONE; // 重置狀態
    }
    else {
        // --- 橫向移動判斷 (Left <-> Right) ---
        if (last_zone != ZONE_NONE && last_zone != detected_zone) {
            if (last_zone == ZONE_LEFT && detected_zone == ZONE_CENTER) sprintf(status_msg, "Moving RIGHT (L->C) >>>");
            else if (last_zone == ZONE_CENTER && detected_zone == ZONE_RIGHT) sprintf(status_msg, "Moving RIGHT (C->R) >>>");
            else if (last_zone == ZONE_RIGHT && detected_zone == ZONE_CENTER) sprintf(status_msg, "Moving LEFT (R->C) <<<");
            else if (last_zone == ZONE_CENTER && detected_zone == ZONE_LEFT) sprintf(status_msg, "Moving LEFT (C->L) <<<");
            else sprintf(status_msg, "Position Changed"); // 例如直接從 L 跳到 R (速度很快)
        }
        // --- 徑向移動判斷 (靠近/遠離) ---
        else if (last_zone == detected_zone) {
            float diff = min_dist - last_min_distance;

            if (diff < -MOVE_HYSTERESIS) sprintf(status_msg, "Approaching (%.0fcm)", min_dist);
            else if (diff > MOVE_HYSTERESIS) sprintf(status_msg, "Leaving (%.0fcm)", min_dist);
            else {
                 // 靜止狀態或變化不大
                 if(detected_zone == ZONE_LEFT) sprintf(status_msg, "Stationary [LEFT]");
                 else if(detected_zone == ZONE_CENTER) sprintf(status_msg, "Stationary [CENTER]");
                 else if(detected_zone == ZONE_RIGHT) sprintf(status_msg, "Stationary [RIGHT]");
            }
        }
        else {
             // 剛偵測到目標的第一個瞬間
             sprintf(status_msg, "Target Detected!");
        }

        // 更新歷史狀態
        last_zone = detected_zone;
        last_min_distance = min_dist;
    }

    // 3. 輸出結果
    printf("[L:%3.0f C:%3.0f R:%3.0f] -> %s\r\n", dL, dC, dR, status_msg);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  printf("\r\n=== STM32 3-Way Ultrasonic System (Non-Blocking) ===\r\n");
  printf("Configuration Checked:\r\n");
  printf("  [L] Trig:PD14, Echo:PB0\r\n");
  printf("  [C] Trig:PA3,  Echo:PB4\r\n");
  printf("  [R] Trig:PB1,  Echo:PB2\r\n");
  printf("System Started.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	Task_Ultrasonic_Update();
	// --- 應用任務：每 200ms 顯示一次結果 ---
	Task_Movement_Logic();
//	static uint32_t print_timer = 0;
//	if (HAL_GetTick() - print_timer > 200) {
//		printf("L:%5.1f | C:%5.1f | R:%5.1f (cm)", us_L.distance, us_C.distance, us_R.distance);
//
//		// 簡單的測試邏輯: 顯示目標方向
//		if (us_C.distance > 0 && us_C.distance < 30) printf("  [Target Center]");
//		else if (us_L.distance > 0 && us_L.distance < 30) printf("  [Target Left]");
//		else if (us_R.distance > 0 && us_R.distance < 30) printf("  [Target Right]");
//
//		printf("\r\n");
//		print_timer = HAL_GetTick();
//	}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|C_TRIG_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R_TRIG_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|L_TRIG_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin C_TRIG_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|C_TRIG_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L_ECHO_Pin R_ECHO_Pin C_ECHO_Pin */
  GPIO_InitStruct.Pin = L_ECHO_Pin|R_ECHO_Pin|C_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R_TRIG_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = R_TRIG_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin L_TRIG_Pin PMOD_RESET_Pin
                           STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|L_TRIG_Pin|PMOD_RESET_Pin
                          |STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
