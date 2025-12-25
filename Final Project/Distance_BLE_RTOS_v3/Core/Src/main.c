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
#include "cmsis_os.h"
#include "app_bluenrg_ms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_tim.h"
#include "queue.h"
#include"stdbool.h"
#include"stdio.h"
#include "../../BlueNRG_MS/App/sensor.h"
#include"gatt_db.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
typedef struct {
    volatile uint32_t start_tick;
    volatile uint32_t end_tick;
    volatile bool is_measuring;
    volatile float distance;
    volatile uint8_t valid;
    volatile float last_distance;
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
// (Left) - Trig:D2(PD14), Echo:D3(PB0->EXTI0)
#define L_TRIG_PORT GPIOD
#define L_TRIG_PIN  GPIO_PIN_14
#define L_ECHO_PORT GPIOB
#define L_ECHO_PIN  GPIO_PIN_0

// (Center) - Trig:D4(PA3), Echo:D5(PB4->EXTI4)
#define C_TRIG_PORT GPIOA
#define C_TRIG_PIN  GPIO_PIN_3
#define C_ECHO_PORT GPIOB
#define C_ECHO_PIN  GPIO_PIN_4

// (Right) - Trig:D6(PB1), Echo:D8(PB2->EXTI2)
#define R_TRIG_PORT GPIOB
#define R_TRIG_PIN  GPIO_PIN_1
#define R_ECHO_PORT GPIOB
#define R_ECHO_PIN  GPIO_PIN_2

#define DETECT_THRESHOLD  50.0f
//#define MOVE_HYSTERESIS   5.0f
//#define NEAR_THRESHOLD    0.3f

#define MAX7219_REG_NOOP        0x00
#define MAX7219_REG_DIGIT0      0x01
#define MAX7219_REG_DECODEMODE  0x09
#define MAX7219_REG_INTENSITY   0x0A
#define MAX7219_REG_SCANLIMIT   0x0B
#define MAX7219_REG_SHUTDOWN    0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BLETask */
osThreadId_t BLETaskHandle;
uint32_t BLETaskBuffer[ 1024 ];
osStaticThreadDef_t BLETaskControlBlock;
const osThreadAttr_t BLETask_attributes = {
  .name = "BLETask",
  .cb_mem = &BLETaskControlBlock,
  .cb_size = sizeof(BLETaskControlBlock),
  .stack_mem = &BLETaskBuffer[0],
  .stack_size = sizeof(BLETaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
uint32_t SensorTaskBuffer[ 512 ];
osStaticThreadDef_t SensorTaskControlBlock;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .cb_mem = &SensorTaskControlBlock,
  .cb_size = sizeof(SensorTaskControlBlock),
  .stack_mem = &SensorTaskBuffer[0],
  .stack_size = sizeof(SensorTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for LogicTask */
osThreadId_t LogicTaskHandle;
uint32_t LogicTaskBuffer[ 512 ];
osStaticThreadDef_t LogicTaskControlBlock;
const osThreadAttr_t LogicTask_attributes = {
  .name = "LogicTask",
  .cb_mem = &LogicTaskControlBlock,
  .cb_size = sizeof(LogicTaskControlBlock),
  .stack_mem = &LogicTaskBuffer[0],
  .stack_size = sizeof(LogicTaskBuffer),
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
uint32_t MotorTaskBuffer[ 512 ];
osStaticThreadDef_t MotorTaskControlBlock;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .cb_mem = &MotorTaskControlBlock,
  .cb_size = sizeof(MotorTaskControlBlock),
  .stack_mem = &MotorTaskBuffer[0],
  .stack_size = sizeof(MotorTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for SensorQueue */
osMessageQueueId_t SensorQueueHandle;
const osMessageQueueAttr_t SensorQueue_attributes = {
  .name = "SensorQueue"
};
/* Definitions for MotorQueue */
osMessageQueueId_t MotorQueueHandle;
const osMessageQueueAttr_t MotorQueue_attributes = {
  .name = "MotorQueue"
};
/* Definitions for DataReadySem */
osSemaphoreId_t DataReadySemHandle;
const osSemaphoreAttr_t DataReadySem_attributes = {
  .name = "DataReadySem"
};
/* USER CODE BEGIN PV */
Ultrasonic_t us_L = {0};
Ultrasonic_t us_C = {0};
Ultrasonic_t us_R = {0};

uint8_t current_sensor_idx = 0;
uint8_t measure_step = 0;
uint32_t last_process_tick = 0;

Zone_t current_zone = ZONE_NONE;
Zone_t last_zone = ZONE_NONE;
float last_min_distance = 0.0f;

volatile uint8_t sentry_mode_enabled = 0;
uint8_t last_alert_status = 0;

const uint8_t ICON_ALL[8] = { // 笑臉
  0b11111111, 0b11111111, 0b11111111, 0b11111111,
  0b11111111, 0b11111111, 0b11111111, 0b11111111
};
const uint8_t ICON_ALERT[8] = { // 驚嘆號
  0b00011000, 0b00011000, 0b00011000, 0b00011000,
  0b00011000, 0b00000000, 0b00011000, 0b00000000
};
const uint8_t ICON_NONE[8] = { // 叉叉
  0b00000000, 0b00000000, 0b00000000, 0b00000000,
  0b00000000, 0b00000000, 0b00000000, 0b00000000
};

uint8_t display_buffer[8] = {0};
uint8_t last_buffer[8] = {0};
uint8_t temp_buffer[8] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void StartBLETask(void *argument);
void StartSensorTask(void *argument);
void StartLogicTask(void *argument);
void StartMotorTask(void *argument);

/* USER CODE BEGIN PFP */
#ifdef __GNUC__
int __io_putchar(int ch) // other Def in b-l475e-iot01a1.c
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


void US_Trigger(GPIO_TypeDef* port, uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    delay_us_short(10); // 維持 10us High
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
    Ultrasonic_t *target = NULL;
    GPIO_TypeDef *target_port = NULL;

    // 請確保 main.h 中有定義 L_ECHO_Pin 等
    if (GPIO_Pin == L_ECHO_Pin) { target = &us_L; target_port = L_ECHO_GPIO_Port; }
    else if (GPIO_Pin == C_ECHO_Pin) { target = &us_C; target_port = C_ECHO_GPIO_Port; }
    else if (GPIO_Pin == R_ECHO_Pin) { target = &us_R; target_port = R_ECHO_GPIO_Port; }

    if (target != NULL) {
        if (HAL_GPIO_ReadPin(target_port, GPIO_Pin) == GPIO_PIN_SET) {
            target->start_tick = now;
            target->is_measuring = true;
        } else {
            if (target->is_measuring) {
                target->end_tick = now;
                target->is_measuring = false;
                uint32_t duration = (target->end_tick >= target->start_tick) ?
                                    (target->end_tick - target->start_tick) :
                                    ((0xFFFFFFFF - target->start_tick) + target->end_tick);
                target->distance = (float)duration * 0.017f;
                target->valid = 1;
            }
        }
    }
}
void Servo_SetAngle(uint8_t angle)
{
    uint16_t pulse;
    // 0° → 500us, 180° → 2500us
    pulse = 500 + (angle * 2000) / 180;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
}


void MAX7219_Write(uint8_t addr, uint8_t data) {
    // 1. CS Low (Start)
    HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_RESET);

    // 2. Send Address
    for(int i=0; i<8; i++) {
        HAL_GPIO_WritePin(LED_CLK_GPIO_Port, LED_CLK_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_DIN_GPIO_Port, LED_DIN_Pin, (addr & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        addr <<= 1;
        HAL_GPIO_WritePin(LED_CLK_GPIO_Port, LED_CLK_Pin, GPIO_PIN_SET);
    }

    // 3. Send Data
    for(int i=0; i<8; i++) {
        HAL_GPIO_WritePin(LED_CLK_GPIO_Port, LED_CLK_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_DIN_GPIO_Port, LED_DIN_Pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1;
        HAL_GPIO_WritePin(LED_CLK_GPIO_Port, LED_CLK_Pin, GPIO_PIN_SET);
    }

    // 4. CS High (End)
    HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_SET);
}

void MAX7219_Init(void) {
    MAX7219_Write(MAX7219_REG_SHUTDOWN, 0x01);    // Wake up
    MAX7219_Write(MAX7219_REG_DECODEMODE, 0x00);  // Matrix mode
    MAX7219_Write(MAX7219_REG_SCANLIMIT, 0x07);   // Scan all digits
    MAX7219_Write(MAX7219_REG_INTENSITY, 0x03);   // Brightness
    MAX7219_Write(MAX7219_REG_DISPLAYTEST, 0x00); // Normal
    for(int i=1; i<=8; i++) MAX7219_Write(i, 0x00); // Clear
}

float min(float a,float b){
	if (a>b || a==b) return b;
	else return a;

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  printf("\r\n=== STM32 3-Way Ultrasonic System (Non-Blocking) ===\r\n");
  printf("System Started.\r\n");
  MAX7219_Init();
  memcpy(display_buffer, ICON_ALL, 8);
  for(int i=0; i<8; i++) MAX7219_Write(i+1, display_buffer[i]);
  HAL_Delay(1000);
  memcpy(display_buffer, ICON_NONE, 8);
  for(int i=0; i<8; i++) MAX7219_Write(i+1, display_buffer[i]);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of DataReadySem */
  DataReadySemHandle = osSemaphoreNew(1, 1, &DataReadySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SensorQueue */
  SensorQueueHandle = osMessageQueueNew (16, sizeof(uint64_t), &SensorQueue_attributes);

  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew (4, sizeof(uint8_t), &MotorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of BLETask */
  BLETaskHandle = osThreadNew(StartBLETask, NULL, &BLETask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of LogicTask */
  LogicTaskHandle = osThreadNew(StartLogicTask, NULL, &LogicTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
//  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 79;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|C_TRIG_Pin|LED_DIN_Pin|BUZZER_Pin
                          |BLE_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R_TRIG_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|SPSGRF_915_SDN_Pin
                          |LED_CLK_Pin|LED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|L_TRIG_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin BLE_INT_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|BLE_INT_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(ARD_D0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin C_TRIG_Pin BUZZER_Pin BLE_RST_Pin
                           ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|C_TRIG_Pin|BUZZER_Pin|BLE_RST_Pin
                          |ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_DIN_Pin */
  GPIO_InitStruct.Pin = LED_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_DIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D12_Pin */
  GPIO_InitStruct.Pin = ARD_D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(ARD_D12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L_ECHO_Pin R_ECHO_Pin C_ECHO_Pin */
  GPIO_InitStruct.Pin = L_ECHO_Pin|R_ECHO_Pin|C_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R_TRIG_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin SPSGRF_915_SDN_Pin
                           SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = R_TRIG_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|SPSGRF_915_SDN_Pin
                          |SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin L_TRIG_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|L_TRIG_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : LED_CLK_Pin LED_CS_Pin */
  GPIO_InitStruct.Pin = LED_CLK_Pin|LED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBLETask */
/**
* @brief Function implementing the BLETask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBLETask */
void StartBLETask(void *argument)
{
  /* USER CODE BEGIN StartBLETask */
  /* Infinite loop */
  for(;;)
  {
	MX_BlueNRG_MS_Process();
	osDelay(50);
  }
  /* USER CODE END StartBLETask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
	us_L.distance = 999.0f;
	us_L.last_distance = 999.0f;
	us_C.distance = 999.0f;
	us_C.last_distance = 999.0f;
	us_R.distance = 999.0f;
	us_R.last_distance = 999.0f;
	for(;;)
	{
		// --- L ---
		us_L.valid = 0; // 標記為更新中，但"不"清空 distance
		US_Trigger(L_TRIG_GPIO_Port, L_TRIG_Pin);
		osDelay(12);
		if(us_L.valid == 0) us_L.distance = us_L.last_distance;
		us_L.last_distance = us_L.distance;
		// --- C (同理) ---
		us_C.valid = 0;
		US_Trigger(C_TRIG_GPIO_Port, C_TRIG_Pin);
		osDelay(12);
		if(us_C.valid == 0) us_C.distance = us_C.last_distance;
		us_C.last_distance = us_C.distance;

		// --- R (同理) ---
		us_R.valid = 0;
		US_Trigger(R_TRIG_GPIO_Port, R_TRIG_Pin);
		osDelay(12);
		if(us_R.valid == 0) us_R.distance = us_R.last_distance;
		us_R.last_distance = us_R.distance;
		osSemaphoreRelease(DataReadySemHandle);
		osDelay(14);
	}
//	osDelay();
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartLogicTask */
/**
* @brief Function implementing the LogicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogicTask */
void StartLogicTask(void *argument)
{
  /* USER CODE BEGIN StartLogicTask */
  static uint8_t beep_toggle = 0;
//  static uint8_t last_wifi_status = 255;

  uint8_t target_angle = 90;
  uint8_t last_target_angle = 90;
  uint8_t loop_counter = 0;
  uint8_t current_state = 0;
  uint8_t last_state = 0;

	// 定義遲滯區間
	const float DIST_ENTER_ALERT = 50.0f; // 小於 50 進入警戒
	const float DIST_EXIT_ALERT  = 65.0f; // 大於 65 才解除警戒 (防閃爍)

  for(;;)
  {
    osStatus_t status = osSemaphoreAcquire(DataReadySemHandle, osWaitForever);
    if (status == osOK)
    {
		float dL=999, dC=999, dR=999;

		if(us_L.valid) dL = us_L.distance;
		else   		   dL = us_L.last_distance;
		if(us_C.valid) dC = us_C.distance;
		else		   dC = us_C.last_distance;
		if(us_R.valid) dR = us_R.distance;
		else		   dR = us_R.last_distance;
		float min_dist = 999.0f;
		min_dist = min(dL,dC);
		min_dist = min(min_dist,dR);
		Zone_t zone = ZONE_NONE;
		if (dL == min_dist) {
			zone = ZONE_LEFT;
		}
		else if (dC == min_dist) {
			zone = ZONE_CENTER;
		}
		else if (dR == min_dist) {
			zone = ZONE_RIGHT;
		}
		if (min_dist < DIST_ENTER_ALERT) {
			current_state = 1; // Alert
		}
		else if (min_dist > DIST_EXIT_ALERT) {
			current_state = 2; // Safe
		}
		if (sentry_mode_enabled)
		{
			if (min_dist < (DETECT_THRESHOLD ))
			{
				MAX7219_Init();
				switch(zone) {
					case ZONE_CENTER:
						target_angle = 90;
						break;
					case ZONE_LEFT:
						target_angle = 45;
						break;
					case ZONE_RIGHT:
						target_angle = 135;
						break;

					default:
						target_angle = 90;
				 }
				// Alert
				memcpy(display_buffer, ICON_ALL, 8);
//				for(int i=0; i<8; i++) MAX7219_Write(i+1, display_buffer[i]);
				// Buzzer
				beep_toggle = !beep_toggle;
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, beep_toggle ? GPIO_PIN_SET : GPIO_PIN_RESET);
				// BLE Notify
				BLE_Send_Alert_Status(1);
				last_alert_status = 1;
			}
			else
			{
				if (last_state != 2) {
					memcpy(display_buffer, ICON_NONE, 8);
//					for(int i=0; i<8; i++) MAX7219_Write(i+1, display_buffer[i]);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
					BLE_Send_Alert_Status(0);
					last_alert_status = 0;
					target_angle = 90;
				}
			}
		}
		else
		{
			// Inactive
			current_state = 0; // Reset state
			if (last_state != 0) {
				 memcpy(display_buffer, ICON_NONE, 8);
					for(int i=0; i<8; i++) MAX7219_Write(i+1, display_buffer[i]);
				 HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			}
		}
		last_state = current_state;
		if (target_angle != last_target_angle) {
			osMessageQueuePut(MotorQueueHandle, &target_angle, 0, 0);
			last_target_angle = target_angle;
		}
		loop_counter++;
		if (loop_counter >= 5)
		{
			loop_counter = 0;
			// --- 處理 Printf (這是最慢的部分) ---
			printf("[L:%3.0f C:%3.0f R:%3.0f] \r\n", dL, dC, dR);
			printf("Target: %d | Dist: %3.0f | %s\r\n",
				   target_angle, min_dist, sentry_mode_enabled ? "Active" : "Inactive");
			printf("================== \r\n");
		}
		memcpy(last_buffer, display_buffer, 8);
//		MAX7219_Write(MAX7219_REG_SHUTDOWN, 0x01);    // Wake up
//		if (last_buffer[0] != display_buffer[0])
			for(int i=0; i<8; i++) MAX7219_Write(i+1, display_buffer[i]);
    }

  }
  /* USER CODE END StartLogicTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */

  uint8_t target = 90;
  Servo_SetAngle(90);
  osDelay(20);
  /* Infinite loop */
  for(;;)
  {
    osStatus_t status = osMessageQueueGet(MotorQueueHandle, &target, NULL, osWaitForever);
    if (status == osOK) {
	  Servo_SetAngle(target);
	}
	  // 這裡不需要 osDelay，因為 osWaitForever 已經控制了節奏
  }

  /* USER CODE END StartMotorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
