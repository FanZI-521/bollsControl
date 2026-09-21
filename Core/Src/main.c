/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "Emm_V5.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  TASK3_IDLE = 0,
  TASK3_MOVE_TO_NEGATIVE,
  TASK3_MOVE_TO_POSITIVE,
  TASK3_HOLD_POSITIVE
} Task3State_t;

typedef enum
{
  MOTOR_STARTUP_WAIT_POWER = 0,
  MOTOR_STARTUP_WAIT_SETTLE,
  MOTOR_STARTUP_WAIT_TEST,
  MOTOR_STARTUP_READY
} MotorStartupState_t;

typedef struct
{
  bool armed;
  bool pressDebounceActive;
  bool releaseDebounceActive;
  uint32_t debounceTick;
} StartFeedforwardInputState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_ADDR                    1U
#define MOTOR_POS_SPEED_RPM           600U//最大速度限制
#define MOTOR_POS_ACC                 5U//加速度
#define MOTOR_PULSES_PER_REV          3200L
#define MOTOR_ANGLE_LIMIT_DEG_X10     80L//限制角度。
/* Change to -1L if positive vision offset drives the beam in the wrong direction. */
#define MOTOR_DIR_SIGN                -1L
#define VISION_FRAME_TIMEOUT_MS       500U
#define CONTROL_TASK_PERIOD_MS        5U
#define MOTOR_POWER_ON_DELAY_MS       1000U
#define MOTOR_ENABLE_SETTLE_MS        200U
#define MOTOR_ENABLE_REFRESH_MS       1000U
#define MOTOR_STARTUP_TEST_ENABLE     0U
#define MOTOR_STARTUP_TEST_PULSE      320U
#define VISION_OFFSET_DEADBAND_X10    3L
#define VISION_OFFSET_MID_X10         30L
#define VISION_OFFSET_HIGH_X10        70L
#define MOTOR_ANGLE_SMALL_GAIN_NUM    1L
#define MOTOR_ANGLE_SMALL_GAIN_DEN    2L
#define MOTOR_ANGLE_MID_GAIN_NUM      3L
#define MOTOR_ANGLE_MID_GAIN_DEN      2L
#define MOTOR_ANGLE_HIGH_GAIN_NUM     3L
#define MOTOR_ANGLE_HIGH_GAIN_DEN     1L
#define MOTOR_MIN_ACTIVE_PULSE        4L
#define MOTOR_MAX_STEP_PULSE          300L//每次变化最大速度
#define MOTOR_CMD_MIN_DELTA_PULSE     3L
#define MOTOR_CMD_INTERVAL_MS         5U
#define PSEUDO_PID_KI_DEN             1000L
#define BALL_PID_DT_MIN_MS            20U
#define BALL_PID_DT_MAX_MS            200U
#define STICK_OFFSET_START_X10        8L
#define STICK_OFFSET_IMPROVE_X10      3L
#define STICK_RAMP_DELAY_MS           50U
#define STICK_RAMP_INTERVAL_MS        35U
#define STICK_RAMP_STEP_ANGLE_X10     5L
#define STICK_MAX_BOOST_ANGLE_X10     60L
#undef MOTOR_POS_SPEED_RPM
#define MOTOR_POS_SPEED_RPM           120U
#undef MOTOR_POS_ACC
#define MOTOR_POS_ACC                4U//0: direct start for minimum command latency
#undef MOTOR_MAX_STEP_PULSE
#define MOTOR_MAX_STEP_PULSE          24L
#undef MOTOR_CMD_INTERVAL_MS
#define MOTOR_CMD_INTERVAL_MS         5U
#define VISION_OFFSET_MAX_X10         120L
#define VISION_OFFSET_MAX_X100        (VISION_OFFSET_MAX_X10 * 10L)
#define BALL_TRACK_LENGTH_MM          250L
#define BALL_TRACK_HALF_MM_X10        ((BALL_TRACK_LENGTH_MM * 10L) / 2L)
#define BALL_TARGET_MAX_ERROR_MM_X10  50L
#define BALL_TARGET_MAX_ERROR_CM_X100 50L
#define BALL_TARGET_MAX_ERROR_VISION_X10 \
                                        ((BALL_TARGET_MAX_ERROR_MM_X10 * VISION_OFFSET_MAX_X10 + (BALL_TRACK_HALF_MM_X10 / 2L)) / BALL_TRACK_HALF_MM_X10)
#define BALL_POSITION_DEADBAND_MM_X10 1L
#define BALL_CENTER_BRAKE_WINDOW_MM_X10 120L
#define BALL_SPEED_LIMIT_MM_X10_PER_SEC                 800L
#define PSEUDO_PID_INTEGRAL_LIMIT                    500000L
#define BALL_PID_BASE_KP_PERMILLE                       26L
#define BALL_PID_BASE_KI_PERMILLE                        0L
#define BALL_PID_BASE_KD_PERMILLE                       12L
#define PID_DRIVE_SPEED_MIN_RPM                       120U
#define PID_DRIVE_SPEED_TARGET_VEL_DEN                50L//冲-大（200）
#define PID_DRIVE_SPEED_ERROR_DEN                     100L//（400）越小角度导致的 RPM 越高。震动就调大
#define PID_DRIVE_ACC_MIN                             1U
#define PID_DRIVE_ACC_ANGLE_DEN                       200L//（1200）越小加速度越猛。震动就调大
#define PID_DRIVE_ACC_ERROR_DEN                       400L//（2400）
#define VISION_FILTER_OLD_NUM         1L
#define VISION_FILTER_NEW_NUM         1L
#define VISION_FILTER_DEN             2L
#define VISION_VELOCITY_FILTER_OLD_NUM 3L
#define VISION_VELOCITY_FILTER_NEW_NUM 1L
#define VISION_VELOCITY_FILTER_DEN     4L
#define VISION_VELOCITY_VALID_MS       80U
#define VISION_PREDICT_MAX_MS         0U
#undef BALL_PID_DT_MIN_MS
#define BALL_PID_DT_MIN_MS            5U
#define STICK_OFFSET_START_MM_X10     60L
#define STICK_OFFSET_IMPROVE_MM_X10   20L
#undef STICK_RAMP_STEP_ANGLE_X10
#define STICK_RAMP_STEP_ANGLE_X10     3L
#undef STICK_MAX_BOOST_ANGLE_X10
#define STICK_MAX_BOOST_ANGLE_X10     24L
#define TASK3_NEGATIVE_DRIVE_VISION_X100 (-1200L)
#define TASK3_NEGATIVE_TURN_VISION_X100  (-450L)
#define TASK3_POSITIVE_TARGET_VISION_X100 950L
#define TASK3_ARRIVAL_TOLERANCE_VISION_X100 20L
#define TASK3_ARRIVAL_HOLD_MS          200U
#define TASK3_BUTTON_DEBOUNCE_MS       30U
#define TASK6_BUTTON_DEBOUNCE_MS       30U
#define TASK3_POSITIVE_DRIVE_LIMIT_ANGLE_X10 50L
#define TASK3_POSITIVE_NEAR_LIMIT_ANGLE_X10  20L
#define TASK3_POSITIVE_NEAR_WINDOW_VISION_X100 150L
#define TASK3_POSITIVE_STALL_ERROR_VISION_X100 170L
#define TASK3_POSITIVE_STALL_VELOCITY_MM_X10_PER_SEC 100L
#define TASK3_POSITIVE_STALL_ANGLE_X10 60L
#define VISION_UART_BYTES_PER_POLL     32U
#define START_FEEDFORWARD_ANGLE_DEG_X10 (-160L)
#define START_FEEDFORWARD_ANGLE_LIMIT_DEG_X10 160L
#define START_FEEDFORWARD_DURATION_MS   500U
#define START_FEEDFORWARD_EXIT_VISION_X10 (-4L)
#define START_FEEDFORWARD_EXIT_POSITION_MM_X10 \
                                        ((START_FEEDFORWARD_EXIT_VISION_X10 * BALL_TRACK_HALF_MM_X10) / VISION_OFFSET_MAX_X10)
#define START_FEEDFORWARD_BUTTON_DEBOUNCE_MS 10U
#define FUZZY_ERROR_FINE_MM_X10         30L
#define FUZZY_ERROR_NEAR_MM_X10         100L
#define FUZZY_ERROR_FAR_MM_X10          300L

#define FUZZY_KP_FINE                   26L
#define FUZZY_KP_NEAR                   36L
#define FUZZY_KP_MID                    62L
#define FUZZY_KP_FAR                    65L

#define FUZZY_KI_FINE                   0L
#define FUZZY_KI_NEAR                   0L
#define FUZZY_KI_MID                    0L
#define FUZZY_KI_FAR                    0L

#define FUZZY_KD_FINE                   10L
#define FUZZY_KD_NEAR                   18L
#define FUZZY_KD_MID                    55L
#define FUZZY_KD_FAR                    48L
#define FUZZY_KD_VELOCITY_BOOST_MAX      8L

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int32_t g_motorTargetPulse = 0;
static int32_t g_motorCommandPulse = 0;
static int32_t g_lastSentPulse = 0;
static uint32_t g_lastVisionTick = 0;
static uint32_t g_lastMotorCmdTick = 0;
static uint32_t g_lastMotorEnableTick = 0;
static uint32_t g_lastControlTick = 0;
static bool g_motorStopped = false;
static bool g_hasSentMotorCmd = false;
static bool g_motorEnabled = false;
static int32_t g_ballPositionMmX10 = 0;
static int32_t g_visionOffsetX100 = 0;
static int32_t g_filteredVisionPositionMmX10 = 0;
static int32_t g_lastVisionPositionMmX10 = 0;
static int32_t g_visionVelocityMmX10PerSec = 0;
static int32_t g_basePidKpPermille = BALL_PID_BASE_KP_PERMILLE;
static int32_t g_basePidKiPermille = BALL_PID_BASE_KI_PERMILLE;
static int32_t g_basePidKdPermille = BALL_PID_BASE_KD_PERMILLE;
static int32_t g_activePidKpPermille = BALL_PID_BASE_KP_PERMILLE;
static int32_t g_activePidKiPermille = BALL_PID_BASE_KI_PERMILLE;
static int32_t g_activePidKdPermille = BALL_PID_BASE_KD_PERMILLE;
static int32_t g_pseudoPidIntegral = 0;
static int32_t g_pseudoPidLastErrorMmX10 = 0;
static int32_t g_pseudoPidVelocityMmX10PerSec = 0;
static int32_t g_pidOutputAngleX10 = 0;
static uint16_t g_pidDriveSpeedRpm = MOTOR_POS_SPEED_RPM;
static uint8_t g_pidDriveAcc = MOTOR_POS_ACC;
static uint32_t g_pidLastTick = 0;
static bool g_pidReady = false;
static int32_t g_lastControlPositionMmX10 = 0;
static int32_t g_stickBoostAngleX10 = 0;
static int8_t g_stickDir = 0;
static uint32_t g_stickHoldStartTick = 0;
static uint32_t g_lastStickRampTick = 0;
static uint32_t g_lastVisionSampleTick = 0;
static bool g_hasVisionSample = false;
static uint8_t g_uart3VisionBuf[CMD_LEN] = {0};
static uint8_t g_uart3VisionLen = 0;
static int32_t g_targetPositionMmX10 = 0;
static uint8_t g_motorAcc = MOTOR_POS_ACC;
static int32_t g_normalReferencePositionMmX10 = 0;
static Task3State_t g_task3State = TASK3_IDLE;
static bool g_task3ButtonCandidatePressed = false;
static bool g_task3ButtonStablePressed = false;
static bool g_task3ArrivalPending = false;
static uint32_t g_task3ButtonChangeTick = 0;
static uint32_t g_task3ArrivalTick = 0;
static uint32_t g_task3LastArrivalVisionTick = 0;
static int32_t g_task6ReferencePositionMmX10 = 0;
static bool g_task6ReferenceActive = false;
static bool g_task6ButtonCandidatePressed = false;
static bool g_task6ButtonStablePressed = false;
static bool g_task6ButtonArmed = false;
static uint32_t g_task6ButtonChangeTick = 0;
static MotorStartupState_t g_motorStartupState = MOTOR_STARTUP_WAIT_POWER;
static uint32_t g_motorStartupTick = 0;
static bool g_motorReady = false;
static uint32_t g_startFeedforwardTick = 0;
static bool g_startFeedforwardActive = false;
static int32_t g_startFeedforwardReferencePositionMmX10 = 0;
static volatile bool g_startFeedforwardButtonIrqPending = false;
static StartFeedforwardInputState_t g_startFeedforwardButtonState = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static bool Vision_ParseOffsetX100(const uint8_t *data, uint8_t len, int32_t *offsetX100);
static int32_t ClampInt32(int32_t value, int32_t minValue, int32_t maxValue);
static int32_t VisionOffsetX100ToBallMmX10(int32_t visionOffsetX100);
static int32_t AngleX10ToPulse(int32_t angleX10);
static int32_t FuzzyPID_Interpolate(int32_t value, int32_t lowValue, int32_t highValue,
                                    int32_t lowGain, int32_t highGain);
static void FuzzyPID_SelectGains(int32_t absErrorMmX10, int32_t absVelocityMmX10PerSec,
                                 int32_t *kpPermille, int32_t *kiPermille, int32_t *kdPermille);
static int32_t BallPidAngleX10(int32_t positionMmX10, uint32_t now);
static int32_t ApplyStictionBoost(int32_t positionMmX10, int32_t angleX10, uint32_t now);
static uint16_t PidDrive_SelectSpeedRpm(int32_t pseudoVelocityMmX10PerSec, int32_t angleX10);
static uint8_t PidDrive_SelectAcc(int32_t angleX10, int32_t pseudoVelocityMmX10PerSec);
static void Vision_UpdateMeasurement(int32_t positionMmX10, uint32_t now);
static int32_t Vision_GetControlPositionMmX10(uint32_t now);
static void Control_ResetState(void);
static void Control_Task(uint32_t now);
static void Motor_EnableEnsure(void);
static void Motor_StartupTask(uint32_t now);
static void Motor_GotoPulse(int32_t targetPulse, uint16_t speedRpm, uint8_t acc);
static void Motor_StopOnce(void);
static void Vision_HandleFrame(const uint8_t *frame, uint8_t frameLen, const char *source);
static void Vision_PollUsart3Input(void);
static void Task3_ResetControllerForTarget(void);
static void Task3_SetTarget(int32_t targetPositionMmX10);
static bool Task3_ButtonPressedEvent(uint32_t now);
static void Task3_Update(uint32_t now);
static int32_t Reference_GetActivePositionMmX10(void);
static void Task6_ButtonInit(uint32_t now);
static bool Task6_ButtonPressedEvent(uint32_t now);
static void Task6_Update(uint32_t now);
static void StartFeedforward_ButtonInit(uint32_t now);
static bool StartFeedforward_ButtonPressedEvent(uint32_t now);
static void StartFeedforward_Begin(uint32_t now);
static void StartFeedforward_Update(uint32_t now);

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  g_motorStartupTick = HAL_GetTick();
  g_lastVisionTick = g_motorStartupTick;
  g_lastControlTick = g_lastVisionTick;
  g_normalReferencePositionMmX10 = g_targetPositionMmX10;
  Task6_ButtonInit(g_motorStartupTick);
  StartFeedforward_ButtonInit(g_motorStartupTick);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Motor_StartupTask(now);
    Vision_PollUsart3Input();
    Task6_Update(now);
    if (g_motorReady && StartFeedforward_ButtonPressedEvent(now))
    {
      StartFeedforward_Begin(now);
    }
    StartFeedforward_Update(now);
    Task3_Update(now);

    if ((now - g_lastControlTick) >= CONTROL_TASK_PERIOD_MS)
    {
      g_lastControlTick = now;
      Control_Task(g_lastControlTick);
    }

    if (g_motorReady && ((now - g_lastVisionTick) > VISION_FRAME_TIMEOUT_MS))
    {
      Control_ResetState();
      Motor_StopOnce();
    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = TASK3_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TASK3_BUTTON_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TASK6_BUTTON_Pin;
  HAL_GPIO_Init(TASK6_BUTTON_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = START_FEEDFORWARD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(START_FEEDFORWARD_BUTTON_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static bool Vision_ParseOffsetX100(const uint8_t *data, uint8_t len, int32_t *offsetX100)
{
  uint8_t i = 0;
  int32_t sign = 1;
  int32_t integer = 0;
  int32_t decimal = 0;
  uint8_t decimalCount = 0;
  bool hasDigit = false;

  while (i < len)
  {
    if ((data[i] == 'n') && ((i + 3U) < len) && (data[i + 1U] == 'o') && (data[i + 2U] == 'n') && (data[i + 3U] == 'e'))
    {
      return false;
    }
    if ((data[i] == '-') || (data[i] == '+') || (data[i] == '.') || ((data[i] >= '0') && (data[i] <= '9')))
    {
      break;
    }
    i++;
  }

  if (i >= len)
  {
    return false;
  }

  if ((i < len) && ((data[i] == '-') || (data[i] == '+')))
  {
    sign = (data[i] == '-') ? -1 : 1;
    i++;
  }

  while ((i < len) && (data[i] >= '0') && (data[i] <= '9'))
  {
    integer = integer * 10 + (data[i] - '0');
    hasDigit = true;
    i++;
  }

  if ((i < len) && (data[i] == '.'))
  {
    i++;
    while ((i < len) && (data[i] >= '0') && (data[i] <= '9') && (decimalCount < 2U))
    {
      decimal = decimal * 10 + (data[i] - '0');
      decimalCount++;
      hasDigit = true;
      i++;
    }
  }

  if (!hasDigit)
  {
    return false;
  }

  while (decimalCount < 2U)
  {
    decimal *= 10;
    decimalCount++;
  }

  *offsetX100 = sign * (integer * 100 + decimal);
  return true;
}

static int32_t ClampInt32(int32_t value, int32_t minValue, int32_t maxValue)
{
  if (value < minValue)
  {
    return minValue;
  }
  if (value > maxValue)
  {
    return maxValue;
  }
  return value;
}

static int32_t VisionOffsetX100ToBallMmX10(int32_t visionOffsetX100)
{
  return (visionOffsetX100 * BALL_TRACK_HALF_MM_X10) / VISION_OFFSET_MAX_X100;
}

static int32_t FuzzyPID_Interpolate(int32_t value, int32_t lowValue, int32_t highValue,
                                    int32_t lowGain, int32_t highGain)
{
  if (value <= lowValue)
  {
    return lowGain;
  }
  if (value >= highValue)
  {
    return highGain;
  }

  return lowGain + ((highGain - lowGain) * (value - lowValue)) /
                   (highValue - lowValue);
}

static void FuzzyPID_SelectGains(int32_t absErrorMmX10, int32_t absVelocityMmX10PerSec,
                                 int32_t *kpPermille, int32_t *kiPermille, int32_t *kdPermille)
{
  int32_t kp = FUZZY_KP_FAR;
  int32_t ki = FUZZY_KI_FAR;
  int32_t kd = FUZZY_KD_FAR;

  if (absErrorMmX10 <= FUZZY_ERROR_FINE_MM_X10)
  {
    kp = FuzzyPID_Interpolate(absErrorMmX10, 0L, FUZZY_ERROR_FINE_MM_X10,
                              FUZZY_KP_FINE, FUZZY_KP_NEAR);
    ki = FuzzyPID_Interpolate(absErrorMmX10, 0L, FUZZY_ERROR_FINE_MM_X10,
                              FUZZY_KI_FINE, FUZZY_KI_NEAR);
    kd = FuzzyPID_Interpolate(absErrorMmX10, 0L, FUZZY_ERROR_FINE_MM_X10,
                              FUZZY_KD_FINE, FUZZY_KD_NEAR);
  }
  else if (absErrorMmX10 <= FUZZY_ERROR_NEAR_MM_X10)
  {
    kp = FuzzyPID_Interpolate(absErrorMmX10, FUZZY_ERROR_FINE_MM_X10, FUZZY_ERROR_NEAR_MM_X10,
                              FUZZY_KP_NEAR, FUZZY_KP_MID);
    ki = FuzzyPID_Interpolate(absErrorMmX10, FUZZY_ERROR_FINE_MM_X10, FUZZY_ERROR_NEAR_MM_X10,
                              FUZZY_KI_NEAR, FUZZY_KI_MID);
    kd = FuzzyPID_Interpolate(absErrorMmX10, FUZZY_ERROR_FINE_MM_X10, FUZZY_ERROR_NEAR_MM_X10,
                              FUZZY_KD_NEAR, FUZZY_KD_MID);
  }
  else if (absErrorMmX10 <= FUZZY_ERROR_FAR_MM_X10)
  {
    kp = FuzzyPID_Interpolate(absErrorMmX10, FUZZY_ERROR_NEAR_MM_X10, FUZZY_ERROR_FAR_MM_X10,
                              FUZZY_KP_MID, FUZZY_KP_FAR);
    ki = FuzzyPID_Interpolate(absErrorMmX10, FUZZY_ERROR_NEAR_MM_X10, FUZZY_ERROR_FAR_MM_X10,
                              FUZZY_KI_MID, FUZZY_KI_FAR);
    kd = FuzzyPID_Interpolate(absErrorMmX10, FUZZY_ERROR_NEAR_MM_X10, FUZZY_ERROR_FAR_MM_X10,
                              FUZZY_KD_MID, FUZZY_KD_FAR);
  }

  absVelocityMmX10PerSec = ClampInt32(absVelocityMmX10PerSec,
                                      0L,
                                      BALL_SPEED_LIMIT_MM_X10_PER_SEC);
  kd += (FUZZY_KD_VELOCITY_BOOST_MAX * absVelocityMmX10PerSec) /
        BALL_SPEED_LIMIT_MM_X10_PER_SEC;

  *kpPermille = kp;
  *kiPermille = ki;
  *kdPermille = kd;
}

static int32_t AngleX10ToPulse(int32_t angleX10)
{
  return (angleX10 * MOTOR_PULSES_PER_REV) / 3600L;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == START_FEEDFORWARD_BUTTON_Pin)
  {
    g_startFeedforwardButtonIrqPending = true;
  }
}

static void StartFeedforward_InputInit(StartFeedforwardInputState_t *state,
                                       bool pressed,
                                       uint32_t now)
{
  state->armed = !pressed;
  state->pressDebounceActive = false;
  state->releaseDebounceActive = false;
  state->debounceTick = now;
}

static bool StartFeedforward_InputPressedEvent(StartFeedforwardInputState_t *state,
                                               bool pressed,
                                               bool fallingPending,
                                               uint32_t now)
{
  if (!pressed)
  {
    state->pressDebounceActive = false;
    if (!state->releaseDebounceActive)
    {
      state->releaseDebounceActive = true;
      state->debounceTick = now;
    }
    else if ((now - state->debounceTick) >= START_FEEDFORWARD_BUTTON_DEBOUNCE_MS)
    {
      state->armed = true;
    }
    return false;
  }

  state->releaseDebounceActive = false;
  if (fallingPending && state->armed)
  {
    state->pressDebounceActive = true;
    state->debounceTick = now;
  }

  if (state->pressDebounceActive &&
      ((now - state->debounceTick) >= START_FEEDFORWARD_BUTTON_DEBOUNCE_MS))
  {
    state->pressDebounceActive = false;
    state->armed = false;
    return true;
  }

  return false;
}

static void StartFeedforward_ButtonInit(uint32_t now)
{
  bool pressed = (HAL_GPIO_ReadPin(START_FEEDFORWARD_BUTTON_GPIO_Port,
                                  START_FEEDFORWARD_BUTTON_Pin) == GPIO_PIN_RESET);

  g_startFeedforwardButtonIrqPending = false;
  StartFeedforward_InputInit(&g_startFeedforwardButtonState, pressed, now);
}

static bool StartFeedforward_ButtonPressedEvent(uint32_t now)
{
  bool fallingPending = false;
  bool pressed = false;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  fallingPending = g_startFeedforwardButtonIrqPending;
  g_startFeedforwardButtonIrqPending = false;
  if (primask == 0U)
  {
    __enable_irq();
  }

  pressed = (HAL_GPIO_ReadPin(START_FEEDFORWARD_BUTTON_GPIO_Port,
                             START_FEEDFORWARD_BUTTON_Pin) == GPIO_PIN_RESET);

  return StartFeedforward_InputPressedEvent(&g_startFeedforwardButtonState,
                                             pressed,
                                             fallingPending,
                                             now);
}

static void StartFeedforward_Begin(uint32_t now)
{
  g_task3State = TASK3_IDLE;
  g_startFeedforwardReferencePositionMmX10 = Reference_GetActivePositionMmX10();
  Task3_SetTarget(g_startFeedforwardReferencePositionMmX10);
  g_startFeedforwardTick = now;
  g_startFeedforwardActive = true;
}

static void StartFeedforward_Update(uint32_t now)
{
  int32_t positionFromReferenceMmX10 = 0;

  if (!g_startFeedforwardActive)
  {
    return;
  }

  if ((now - g_startFeedforwardTick) >= START_FEEDFORWARD_DURATION_MS)
  {
    g_startFeedforwardActive = false;
    return;
  }

  if (g_hasVisionSample)
  {
    positionFromReferenceMmX10 = g_ballPositionMmX10 -
                                 g_startFeedforwardReferencePositionMmX10;
    if (positionFromReferenceMmX10 <= START_FEEDFORWARD_EXIT_POSITION_MM_X10)
    {
      g_startFeedforwardActive = false;
    }
  }
}

static void Vision_UpdateMeasurement(int32_t positionMmX10, uint32_t now)
{
  int32_t dtMs = 0;
  int32_t measuredVelocityMmX10PerSec = 0;

  if (g_hasVisionSample)
  {
    dtMs = (int32_t)(now - g_lastVisionSampleTick);
    dtMs = ClampInt32(dtMs, 1L, (int32_t)BALL_PID_DT_MAX_MS);
    measuredVelocityMmX10PerSec = ((positionMmX10 - g_lastVisionPositionMmX10) * 1000L) / dtMs;
    measuredVelocityMmX10PerSec = ClampInt32(measuredVelocityMmX10PerSec,
                                             -BALL_SPEED_LIMIT_MM_X10_PER_SEC,
                                             BALL_SPEED_LIMIT_MM_X10_PER_SEC);
    g_visionVelocityMmX10PerSec =
        ((VISION_VELOCITY_FILTER_OLD_NUM * g_visionVelocityMmX10PerSec) +
         (VISION_VELOCITY_FILTER_NEW_NUM * measuredVelocityMmX10PerSec)) /
        VISION_VELOCITY_FILTER_DEN;
    g_filteredVisionPositionMmX10 =
        ((VISION_FILTER_OLD_NUM * g_filteredVisionPositionMmX10) +
         (VISION_FILTER_NEW_NUM * positionMmX10)) / VISION_FILTER_DEN;
  }
  else
  {
    g_visionVelocityMmX10PerSec = 0;
    g_filteredVisionPositionMmX10 = positionMmX10;
  }

  g_lastVisionPositionMmX10 = positionMmX10;
  g_ballPositionMmX10 = positionMmX10;
  g_lastVisionSampleTick = now;
  g_lastVisionTick = now;
  g_hasVisionSample = true;
}

static int32_t Vision_GetControlPositionMmX10(uint32_t now)
{
  int32_t ageMs = 0;
  int32_t predictMs = 0;
  int32_t controlPositionMmX10 = g_filteredVisionPositionMmX10;

  if (!g_hasVisionSample)
  {
    return 0;
  }

  ageMs = (int32_t)(now - g_lastVisionSampleTick);
  predictMs = ClampInt32(ageMs, 0L, (int32_t)VISION_PREDICT_MAX_MS);
  controlPositionMmX10 += (g_visionVelocityMmX10PerSec * predictMs) / 1000L;
  return controlPositionMmX10;
}

static int32_t BallPidAngleX10(int32_t positionMmX10, uint32_t now)
{
  int32_t absPositionMmX10 = (positionMmX10 >= 0) ? positionMmX10 : -positionMmX10;
  int32_t absPseudoVelocityMmX10PerSec = 0;
  int32_t fuzzyKpPermille = FUZZY_KP_FINE;
  int32_t fuzzyKiPermille = FUZZY_KI_FINE;
  int32_t fuzzyKdPermille = FUZZY_KD_FINE;
  int32_t dtMs = 0;
  int32_t pseudoVelocityMmX10PerSec = 0;
  int32_t rawAngleX10 = 0;

  if (absPositionMmX10 <= BALL_POSITION_DEADBAND_MM_X10)
  {
    g_pseudoPidIntegral = 0;
    g_pseudoPidLastErrorMmX10 = positionMmX10;
    g_pseudoPidVelocityMmX10PerSec = 0;
    g_pidOutputAngleX10 = 0;
    g_pidDriveSpeedRpm = PID_DRIVE_SPEED_MIN_RPM;
    g_pidDriveAcc = PID_DRIVE_ACC_MIN;
    g_pidLastTick = now;
    g_pidReady = true;
    return 0;
  }

  if (!g_pidReady)
  {
    g_pseudoPidLastErrorMmX10 = positionMmX10;
    g_pseudoPidVelocityMmX10PerSec = 0;
    g_pidLastTick = now;
    g_pidReady = true;
  }

  dtMs = (int32_t)(now - g_pidLastTick);
  dtMs = ClampInt32(dtMs, (int32_t)BALL_PID_DT_MIN_MS, (int32_t)BALL_PID_DT_MAX_MS);
  if ((now - g_lastVisionSampleTick) <= VISION_VELOCITY_VALID_MS)
  {
    pseudoVelocityMmX10PerSec = g_visionVelocityMmX10PerSec;
  }

  absPseudoVelocityMmX10PerSec = (pseudoVelocityMmX10PerSec >= 0) ?
                                 pseudoVelocityMmX10PerSec : -pseudoVelocityMmX10PerSec;
  FuzzyPID_SelectGains(absPositionMmX10,
                       absPseudoVelocityMmX10PerSec,
                       &fuzzyKpPermille,
                       &fuzzyKiPermille,
                       &fuzzyKdPermille);

  g_activePidKpPermille = ClampInt32(g_basePidKpPermille +
                                     fuzzyKpPermille - FUZZY_KP_FINE,
                                     0L, 20000L);
  g_activePidKiPermille = ClampInt32(g_basePidKiPermille +
                                     fuzzyKiPermille - FUZZY_KI_FINE,
                                     0L, 20000L);
  g_activePidKdPermille = ClampInt32(g_basePidKdPermille +
                                     fuzzyKdPermille - FUZZY_KD_FINE,
                                     0L, 5000L);

  if (absPositionMmX10 > FUZZY_ERROR_NEAR_MM_X10)
  {
    g_pseudoPidIntegral = 0;
  }
  else
  {
    g_pseudoPidIntegral += positionMmX10 * dtMs;
  }
  g_pseudoPidIntegral = ClampInt32(g_pseudoPidIntegral,
                                   -PSEUDO_PID_INTEGRAL_LIMIT,
                                   PSEUDO_PID_INTEGRAL_LIMIT);

  rawAngleX10 += (g_activePidKpPermille * positionMmX10) / 1000L;
  rawAngleX10 += (g_activePidKiPermille * g_pseudoPidIntegral) / (1000L * PSEUDO_PID_KI_DEN);
  rawAngleX10 += (g_activePidKdPermille * pseudoVelocityMmX10PerSec) / 1000L;

  rawAngleX10 = MOTOR_DIR_SIGN * rawAngleX10;
  rawAngleX10 = ClampInt32(rawAngleX10, -MOTOR_ANGLE_LIMIT_DEG_X10, MOTOR_ANGLE_LIMIT_DEG_X10);

  g_pseudoPidVelocityMmX10PerSec = pseudoVelocityMmX10PerSec;
  g_pidOutputAngleX10 = rawAngleX10;
  g_pidDriveSpeedRpm = PidDrive_SelectSpeedRpm(pseudoVelocityMmX10PerSec, rawAngleX10);
  g_pidDriveAcc = PidDrive_SelectAcc(rawAngleX10, pseudoVelocityMmX10PerSec);

  g_pseudoPidLastErrorMmX10 = positionMmX10;
  g_pidLastTick = now;
  return rawAngleX10;
}

static void Control_ResetState(void)
{
  g_pseudoPidIntegral = 0;
  g_pseudoPidLastErrorMmX10 = 0;
  g_pseudoPidVelocityMmX10PerSec = 0;
  g_pidLastTick = 0;
  g_pidReady = false;
  g_lastControlPositionMmX10 = 0;
  g_stickBoostAngleX10 = 0;
  g_stickDir = 0;
  g_stickHoldStartTick = 0;
  g_lastStickRampTick = 0;
  g_motorTargetPulse = 0;
  g_motorCommandPulse = 0;
  g_lastSentPulse = 0;
  g_hasSentMotorCmd = false;
  g_visionVelocityMmX10PerSec = 0;
  g_hasVisionSample = false;
}

static int32_t ApplyStictionBoost(int32_t positionMmX10, int32_t angleX10, uint32_t now)
{
  int32_t absPositionMmX10 = (positionMmX10 >= 0) ? positionMmX10 : -positionMmX10;
  int32_t lastAbsPositionMmX10 = (g_lastControlPositionMmX10 >= 0) ? g_lastControlPositionMmX10 : -g_lastControlPositionMmX10;
  int8_t positionDir = 0;
  int8_t angleDir = 0;
  bool positionImproving = false;

  if ((absPositionMmX10 < STICK_OFFSET_START_MM_X10) || (angleX10 == 0))
  {
    g_stickBoostAngleX10 = 0;
    g_stickDir = 0;
    g_stickHoldStartTick = now;
    g_lastStickRampTick = now;
    g_lastControlPositionMmX10 = positionMmX10;
    return angleX10;
  }

  positionDir = (positionMmX10 > 0) ? 1 : -1;
  angleDir = (angleX10 > 0) ? 1 : -1;
  positionImproving = ((g_stickDir == positionDir) &&
                       ((absPositionMmX10 + STICK_OFFSET_IMPROVE_MM_X10) < lastAbsPositionMmX10));

  if ((g_stickDir != positionDir) || positionImproving)
  {
    g_stickBoostAngleX10 = 0;
    g_stickDir = positionDir;
    g_stickHoldStartTick = now;
    g_lastStickRampTick = now;
  }
  else if ((now - g_stickHoldStartTick) >= STICK_RAMP_DELAY_MS)
  {
    if ((now - g_lastStickRampTick) >= STICK_RAMP_INTERVAL_MS)
    {
      g_stickBoostAngleX10 += STICK_RAMP_STEP_ANGLE_X10;
      g_stickBoostAngleX10 = ClampInt32(g_stickBoostAngleX10, 0, STICK_MAX_BOOST_ANGLE_X10);
      g_lastStickRampTick = now;
    }
  }

  g_lastControlPositionMmX10 = positionMmX10;
  angleX10 += angleDir * g_stickBoostAngleX10;
  return ClampInt32(angleX10, -MOTOR_ANGLE_LIMIT_DEG_X10, MOTOR_ANGLE_LIMIT_DEG_X10);
}

static uint16_t PidDrive_SelectSpeedRpm(int32_t pseudoVelocityMmX10PerSec, int32_t angleX10)
{
  (void)pseudoVelocityMmX10PerSec;
  (void)angleX10;
  return (uint16_t)PID_DRIVE_SPEED_MIN_RPM;
}

static uint8_t PidDrive_SelectAcc(int32_t angleX10, int32_t pseudoVelocityMmX10PerSec)
{
  int32_t absAngle = (angleX10 >= 0) ? angleX10 : -angleX10;
  int32_t absPseudoVelocity = (pseudoVelocityMmX10PerSec >= 0) ? pseudoVelocityMmX10PerSec : -pseudoVelocityMmX10PerSec;
  int32_t acc = (int32_t)PID_DRIVE_ACC_MIN;

  acc += absAngle / PID_DRIVE_ACC_ANGLE_DEN;
  acc += absPseudoVelocity / PID_DRIVE_ACC_ERROR_DEN;
  acc = ClampInt32(acc, (int32_t)PID_DRIVE_ACC_MIN, (int32_t)g_motorAcc);
  return (uint8_t)acc;
}

static void Motor_EnableEnsure(void)
{
  uint32_t now = HAL_GetTick();

  if ((!g_motorEnabled) || ((now - g_lastMotorEnableTick) >= MOTOR_ENABLE_REFRESH_MS))
  {
    Emm_V5_En_Control(MOTOR_ADDR, true, false);
    g_motorEnabled = true;
    g_lastMotorEnableTick = now;
  }
}

static void Motor_StartupTask(uint32_t now)
{
  if (g_motorStartupState == MOTOR_STARTUP_WAIT_POWER)
  {
    if ((now - g_motorStartupTick) >= MOTOR_POWER_ON_DELAY_MS)
    {
      Motor_EnableEnsure();
      g_motorStartupState = MOTOR_STARTUP_WAIT_SETTLE;
      g_motorStartupTick = now;
    }
    return;
  }

  if (g_motorStartupState == MOTOR_STARTUP_WAIT_SETTLE)
  {
    if ((now - g_motorStartupTick) >= MOTOR_ENABLE_SETTLE_MS)
    {
#if MOTOR_STARTUP_TEST_ENABLE
      Emm_V5_Pos_Control(MOTOR_ADDR, 0, 1000, 0, MOTOR_STARTUP_TEST_PULSE, 0, false);
      g_motorStartupState = MOTOR_STARTUP_WAIT_TEST;
      g_motorStartupTick = now;
#else
      g_motorStartupState = MOTOR_STARTUP_READY;
      g_motorReady = true;
#endif
    }
    return;
  }

  if (g_motorStartupState == MOTOR_STARTUP_WAIT_TEST)
  {
    if ((now - g_motorStartupTick) >= 800U)
    {
      g_motorStartupState = MOTOR_STARTUP_READY;
      g_motorReady = true;
    }
  }
}

static void Control_Task(uint32_t now)
{
  int32_t controlPositionMmX10 = 0;
  int32_t targetAngleX10 = 0;
  int32_t commandDeltaPulse = 0;
  int32_t remainingVisionX100 = 0;
  int32_t absVisionVelocityMmX10PerSec = 0;
  int32_t positiveDriveLimitAngleX10 = TASK3_POSITIVE_DRIVE_LIMIT_ANGLE_X10;

  if (!g_motorReady || !g_hasVisionSample)
  {
    return;
  }

  if ((now - g_lastVisionTick) > VISION_FRAME_TIMEOUT_MS)
  {
    return;
  }

  controlPositionMmX10 = Vision_GetControlPositionMmX10(now);
  controlPositionMmX10 -= g_targetPositionMmX10;

  targetAngleX10 = BallPidAngleX10(controlPositionMmX10, now);
  targetAngleX10 = ApplyStictionBoost(controlPositionMmX10, targetAngleX10, now);
  if (g_task3State == TASK3_MOVE_TO_POSITIVE)
  {
    remainingVisionX100 = TASK3_POSITIVE_TARGET_VISION_X100 - g_visionOffsetX100;
    absVisionVelocityMmX10PerSec = (g_visionVelocityMmX10PerSec >= 0) ?
                                   g_visionVelocityMmX10PerSec :
                                  -g_visionVelocityMmX10PerSec;

    if (remainingVisionX100 <= TASK3_POSITIVE_NEAR_WINDOW_VISION_X100)
    {
      positiveDriveLimitAngleX10 = TASK3_POSITIVE_NEAR_LIMIT_ANGLE_X10;
    }
    if (targetAngleX10 > positiveDriveLimitAngleX10)
    {
      targetAngleX10 = positiveDriveLimitAngleX10;
    }

    if ((remainingVisionX100 > TASK3_POSITIVE_STALL_ERROR_VISION_X100) &&
        (absVisionVelocityMmX10PerSec <=
         TASK3_POSITIVE_STALL_VELOCITY_MM_X10_PER_SEC) &&
        (targetAngleX10 > 0L) &&
        (targetAngleX10 < TASK3_POSITIVE_STALL_ANGLE_X10))
    {
      targetAngleX10 = TASK3_POSITIVE_STALL_ANGLE_X10;
    }
  }
  if (g_startFeedforwardActive)
  {
    targetAngleX10 = ClampInt32(targetAngleX10 + START_FEEDFORWARD_ANGLE_DEG_X10,
                                -START_FEEDFORWARD_ANGLE_LIMIT_DEG_X10,
                                START_FEEDFORWARD_ANGLE_LIMIT_DEG_X10);
    g_pidDriveAcc = g_motorAcc;
  }
  g_motorTargetPulse = AngleX10ToPulse(targetAngleX10);

  /* Keep each absolute-position update short while retaining a fixed origin. */
  commandDeltaPulse = g_motorTargetPulse - g_motorCommandPulse;
  commandDeltaPulse = ClampInt32(commandDeltaPulse,
                                 -MOTOR_MAX_STEP_PULSE,
                                 MOTOR_MAX_STEP_PULSE);
  g_motorCommandPulse += commandDeltaPulse;

  if (((now - g_lastMotorCmdTick) >= MOTOR_CMD_INTERVAL_MS) &&
      ((!g_hasSentMotorCmd) ||
       (g_motorCommandPulse - g_lastSentPulse >= MOTOR_CMD_MIN_DELTA_PULSE) ||
         (g_lastSentPulse - g_motorCommandPulse >= MOTOR_CMD_MIN_DELTA_PULSE)))
  {
    Motor_GotoPulse(g_motorCommandPulse, g_pidDriveSpeedRpm, g_pidDriveAcc);
    g_lastSentPulse = g_motorCommandPulse;
    g_hasSentMotorCmd = true;
    g_lastMotorCmdTick = now;
  }

  g_motorStopped = false;
}

static void Vision_HandleFrame(const uint8_t *frame, uint8_t frameLen, const char *source)
{
  int32_t visionOffsetX100 = 0;
  int32_t positionMmX10 = 0;
  uint32_t now = HAL_GetTick();
  (void)source;

  if (Vision_ParseOffsetX100(frame, frameLen, &visionOffsetX100))
  {
    g_visionOffsetX100 = visionOffsetX100;
    positionMmX10 = VisionOffsetX100ToBallMmX10(visionOffsetX100);
    Vision_UpdateMeasurement(positionMmX10, now);
  }
}

static void Vision_PollUsart3Input(void)
{
  uint8_t byte = 0;
  uint8_t bytesRead = 0;

  while ((bytesRead < VISION_UART_BYTES_PER_POLL) &&
         (HAL_UART_Receive(&huart3, &byte, 1, 0) == HAL_OK))
  {
    bytesRead++;
    if ((byte == '\n') || (byte == '\r') || (g_uart3VisionLen >= (CMD_LEN - 1U)))
    {
      if (g_uart3VisionLen > 0U)
      {
        Vision_HandleFrame(g_uart3VisionBuf, g_uart3VisionLen, "USART3");
        g_uart3VisionLen = 0;
        memset(g_uart3VisionBuf, 0, sizeof(g_uart3VisionBuf));
      }
    }
    else
    {
      g_uart3VisionBuf[g_uart3VisionLen] = byte;
      g_uart3VisionLen++;
    }
  }
}

static void Motor_GotoPulse(int32_t targetPulse, uint16_t speedRpm, uint8_t acc)
{
  uint8_t dir = 0;
  uint32_t pulse = 0;

  Motor_EnableEnsure();

  /* Emm_V5 position mode uses dir + unsigned pulse: 0=CW, 1=CCW. */
  if (targetPulse < 0)
  {
    dir = 1;
    pulse = (uint32_t)(-targetPulse);
  }
  else
  {
    dir = 0;
    pulse = (uint32_t)targetPulse;
  }

  Emm_V5_Pos_Control(MOTOR_ADDR, dir, speedRpm, acc, pulse, 1, false);
}

static void Motor_StopOnce(void)
{
  if (!g_motorStopped)
  {
    Emm_V5_Stop_Now(MOTOR_ADDR, false);
    g_motorStopped = true;
    g_pidReady = false;
    g_pseudoPidIntegral = 0;
    g_pseudoPidLastErrorMmX10 = 0;
    g_pseudoPidVelocityMmX10PerSec = 0;
    g_pidOutputAngleX10 = 0;
    g_pidDriveSpeedRpm = PID_DRIVE_SPEED_MIN_RPM;
    g_pidDriveAcc = PID_DRIVE_ACC_MIN;
    g_pidLastTick = 0;
    g_lastControlPositionMmX10 = 0;
    g_stickBoostAngleX10 = 0;
    g_stickDir = 0;
    g_stickHoldStartTick = HAL_GetTick();
    g_lastStickRampTick = g_stickHoldStartTick;
  }
}

static void Task3_ResetControllerForTarget(void)
{
  g_pseudoPidIntegral = 0;
  g_pseudoPidVelocityMmX10PerSec = 0;
  g_pidReady = false;
  g_stickBoostAngleX10 = 0;
  g_stickDir = 0;
  g_stickHoldStartTick = HAL_GetTick();
  g_lastStickRampTick = g_stickHoldStartTick;
}

static void Task3_SetTarget(int32_t targetPositionMmX10)
{
  g_targetPositionMmX10 = ClampInt32(targetPositionMmX10,
                                     -BALL_TRACK_HALF_MM_X10,
                                     BALL_TRACK_HALF_MM_X10);
  g_task3ArrivalPending = false;
  Task3_ResetControllerForTarget();
}

static bool Task3_ButtonPressedEvent(uint32_t now)
{
  bool rawPressed = (HAL_GPIO_ReadPin(TASK3_BUTTON_GPIO_Port,
                                     TASK3_BUTTON_Pin) == GPIO_PIN_RESET);

  if (rawPressed != g_task3ButtonCandidatePressed)
  {
    g_task3ButtonCandidatePressed = rawPressed;
    g_task3ButtonChangeTick = now;
    return false;
  }

  if ((rawPressed != g_task3ButtonStablePressed) &&
      ((now - g_task3ButtonChangeTick) >= TASK3_BUTTON_DEBOUNCE_MS))
  {
    g_task3ButtonStablePressed = rawPressed;
    return g_task3ButtonStablePressed;
  }

  return false;
}

static int32_t Reference_GetActivePositionMmX10(void)
{
  if (g_task6ReferenceActive)
  {
    return g_task6ReferencePositionMmX10;
  }

  return g_normalReferencePositionMmX10;
}

static void Task6_ButtonInit(uint32_t now)
{
  bool rawPressed = (HAL_GPIO_ReadPin(TASK6_BUTTON_GPIO_Port,
                                     TASK6_BUTTON_Pin) == GPIO_PIN_RESET);

  g_task6ButtonCandidatePressed = rawPressed;
  g_task6ButtonStablePressed = rawPressed;
  g_task6ButtonArmed = !rawPressed;
  g_task6ButtonChangeTick = now;
}

static bool Task6_ButtonPressedEvent(uint32_t now)
{
  bool rawPressed = (HAL_GPIO_ReadPin(TASK6_BUTTON_GPIO_Port,
                                     TASK6_BUTTON_Pin) == GPIO_PIN_RESET);

  if (rawPressed != g_task6ButtonCandidatePressed)
  {
    g_task6ButtonCandidatePressed = rawPressed;
    g_task6ButtonChangeTick = now;
    return false;
  }

  if ((rawPressed != g_task6ButtonStablePressed) &&
      ((now - g_task6ButtonChangeTick) >= TASK6_BUTTON_DEBOUNCE_MS))
  {
    g_task6ButtonStablePressed = rawPressed;
    if (!g_task6ButtonStablePressed)
    {
      g_task6ButtonArmed = true;
      return false;
    }

    if (g_task6ButtonArmed)
    {
      g_task6ButtonArmed = false;
      return true;
    }
  }

  return false;
}

static void Task6_Update(uint32_t now)
{
  if (!Task6_ButtonPressedEvent(now))
  {
    return;
  }

  if (g_task6ReferenceActive)
  {
    g_task3State = TASK3_IDLE;
    g_startFeedforwardActive = false;
    g_task6ReferenceActive = false;
    Task3_SetTarget(g_normalReferencePositionMmX10);
    return;
  }

  if ((!g_hasVisionSample) ||
      ((now - g_lastVisionTick) > VISION_FRAME_TIMEOUT_MS))
  {
    return;
  }

  g_task3State = TASK3_IDLE;
  g_startFeedforwardActive = false;
  g_task6ReferencePositionMmX10 = ClampInt32(g_filteredVisionPositionMmX10,
                                              -BALL_TRACK_HALF_MM_X10,
                                              BALL_TRACK_HALF_MM_X10);
  g_task6ReferenceActive = true;
  Task3_SetTarget(g_task6ReferencePositionMmX10);
}

static void Task3_Update(uint32_t now)
{
  int32_t targetErrorVisionX100 = 0;

  if (Task3_ButtonPressedEvent(now))
  {
    if (g_task3State == TASK3_IDLE)
    {
      g_task3State = TASK3_MOVE_TO_NEGATIVE;
      Task3_SetTarget(VisionOffsetX100ToBallMmX10(TASK3_NEGATIVE_DRIVE_VISION_X100));
    }
    else
    {
      g_task3State = TASK3_IDLE;
      Task3_SetTarget(Reference_GetActivePositionMmX10());
    }
  }

  if (!g_hasVisionSample)
  {
    g_task3ArrivalPending = false;
    return;
  }

  if ((g_task3State != TASK3_MOVE_TO_NEGATIVE) &&
      (g_task3State != TASK3_MOVE_TO_POSITIVE))
  {
    g_task3ArrivalPending = false;
    return;
  }

  if (g_lastVisionSampleTick == g_task3LastArrivalVisionTick)
  {
    return;
  }
  g_task3LastArrivalVisionTick = g_lastVisionSampleTick;

  if (g_task3State == TASK3_MOVE_TO_NEGATIVE)
  {
    if (g_visionOffsetX100 <= TASK3_NEGATIVE_TURN_VISION_X100)
    {
      g_task3State = TASK3_MOVE_TO_POSITIVE;
      Task3_SetTarget(VisionOffsetX100ToBallMmX10(TASK3_POSITIVE_TARGET_VISION_X100));
    }
    return;
  }

  targetErrorVisionX100 = g_visionOffsetX100 - TASK3_POSITIVE_TARGET_VISION_X100;
  if ((targetErrorVisionX100 < -TASK3_ARRIVAL_TOLERANCE_VISION_X100) ||
      (targetErrorVisionX100 > TASK3_ARRIVAL_TOLERANCE_VISION_X100))
  {
    g_task3ArrivalPending = false;
    return;
  }

  if (!g_task3ArrivalPending)
  {
    g_task3ArrivalPending = true;
    g_task3ArrivalTick = now;
    return;
  }

  if ((now - g_task3ArrivalTick) < TASK3_ARRIVAL_HOLD_MS)
  {
    return;
  }

  g_task3State = TASK3_HOLD_POSITIVE;
  g_task3ArrivalPending = false;
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
