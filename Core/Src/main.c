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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max31865.h"
#include "tm1637.h"
#include "button_input.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t temperatureTenthsC;
  uint16_t sterilizeMinutes;
  uint16_t dryMinutes;
} ProgramConfig;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} GpioOutput;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PROGRAM_COUNT 6U
#define PROGRAM_NONE 0xffU
#define PROGRAM_DEBOUNCE_MS 30U
#define PROGRAM_LONG_PRESS_MS 1000U
#define PROGRAM_REPEAT_MS 500U
#define PROGRAM_TIME_ALTERNATE_MS 2000U
#define TEMPERATURE_REFRESH_MS 300U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
Max31865Handle gMax31865;
TM1637Handle gDisplay1;
TM1637Handle gDisplay2;
ButtonInput gProgramButtons[PROGRAM_COUNT];
int16_t gTemperatureTenthsC = 0;
uint8_t gSensorReady = 0U;
uint8_t gSelectedProgram = PROGRAM_NONE;
uint8_t gProgramTimeDisplayPhase = 0xffU;
uint32_t gProgramSelectedTick = 0U;
uint32_t gLastTemperatureReadTick = 0U;

static const ProgramConfig programPresets[PROGRAM_COUNT] = {
  {1210U, 15U, 0U}, {1210U, 20U, 15U}, {1320U, 7U, 10U},
  {1340U, 7U, 10U}, {1340U, 10U, 20U}, {1340U, 5U, 5U}
};

static const GpioOutput programLeds[PROGRAM_COUNT] = {
  {LD_P1_GPIO_Port, LD_P1_Pin}, {LD_P2_GPIO_Port, LD_P2_Pin},
  {LD_P3_GPIO_Port, LD_P3_Pin}, {LD_P4_GPIO_Port, LD_P4_Pin},
  {LD_P5_GPIO_Port, LD_P5_Pin}, {LD_P6_GPIO_Port, LD_P6_Pin}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void DisplayErrorOnDisplay2(uint8_t code);
static void ProgramButtons_Init(void);
static void ProgramButtons_Process(uint32_t now);
static void Program_Select(uint8_t programIndex, uint32_t now);
static void ProgramLeds_Set(uint8_t programIndex);
static void ProgramDisplay_Update(uint32_t now);
static void DisplayProgramTime(TM1637Handle *display, const char prefix[2], uint16_t minutes);
static void TemperatureDisplay_Process(uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DisplayErrorOnDisplay2(uint8_t code)
{
  /* E r <tens><ones> (e.g. Er01 for PT100 read failure). */
  uint8_t segments[4] = {0x79U, 0x50U, 0x00U, 0x00U};
  static const uint8_t digitMap[10] = {0x3fU, 0x06U, 0x5bU, 0x4fU, 0x66U, 0x6dU, 0x7dU, 0x07U, 0x7fU, 0x6fU};
  uint8_t normalizedCode = (uint8_t)(code % 100U);
  segments[2] = digitMap[normalizedCode / 10U];
  segments[3] = digitMap[normalizedCode % 10U];
  tm1637DisplaySegments(&gDisplay2, segments);
}

static uint8_t SegmentForCharacter(char character)
{
  switch (character) {
    case '0': return 0x3fU;
    case '1': return 0x06U;
    case '2': return 0x5bU;
    case '3': return 0x4fU;
    case '4': return 0x66U;
    case '5': return 0x6dU;
    case '6': return 0x7dU;
    case '7': return 0x07U;
    case '8': return 0x7fU;
    case '9': return 0x6fU;
    case 'D': return 0x5eU;
    case 'S': return 0x6dU;
    case 'r': return 0x50U;
    case 't': return 0x78U;
    default: return 0x00U;
  }
}

static void ProgramButtons_Init(void)
{
  ButtonInput_Init(&gProgramButtons[0], B_P1_GPIO_Port, B_P1_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&gProgramButtons[1], B_P2_GPIO_Port, B_P2_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&gProgramButtons[2], B_P3_GPIO_Port, B_P3_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&gProgramButtons[3], B_P4_GPIO_Port, B_P4_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&gProgramButtons[4], B_P5_GPIO_Port, B_P5_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&gProgramButtons[5], B_P6_GPIO_Port, B_P6_Pin, GPIO_PIN_SET);
}

static void ProgramButtons_Process(uint32_t now)
{
  for (uint8_t i = 0U; i < PROGRAM_COUNT; ++i) {
    ButtonInput_Update(&gProgramButtons[i], now, PROGRAM_DEBOUNCE_MS, PROGRAM_LONG_PRESS_MS, PROGRAM_REPEAT_MS);
    if (ButtonInput_ConsumePressed(&gProgramButtons[i]) != 0U) {
      Program_Select(i, now);
    }
  }
}

static void Program_Select(uint8_t programIndex, uint32_t now)
{
  if (programIndex >= PROGRAM_COUNT) {
    return;
  }

  gSelectedProgram = programIndex;
  gProgramSelectedTick = now;
  gProgramTimeDisplayPhase = 0xffU;
  ProgramLeds_Set(programIndex);
  ProgramDisplay_Update(now);
  tm1637DisplayDecimalTenths(&gDisplay2, programPresets[programIndex].temperatureTenthsC);
}

static void ProgramLeds_Set(uint8_t programIndex)
{
  for (uint8_t i = 0U; i < PROGRAM_COUNT; ++i) {
    HAL_GPIO_WritePin(programLeds[i].port, programLeds[i].pin, GPIO_PIN_RESET);
  }

  if (programIndex < PROGRAM_COUNT) {
    HAL_GPIO_WritePin(programLeds[programIndex].port, programLeds[programIndex].pin, GPIO_PIN_SET);
  }
}

static void ProgramDisplay_Update(uint32_t now)
{
  uint8_t phase;
  const ProgramConfig *program;

  if (gSelectedProgram >= PROGRAM_COUNT) {
    return;
  }

  phase = (uint8_t)(((now - gProgramSelectedTick) / PROGRAM_TIME_ALTERNATE_MS) & 0x01U);
  if (phase == gProgramTimeDisplayPhase) {
    return;
  }

  gProgramTimeDisplayPhase = phase;
  program = &programPresets[gSelectedProgram];

  if (phase == 0U) {
    DisplayProgramTime(&gDisplay1, "St", program->sterilizeMinutes);
  }
  else {
    DisplayProgramTime(&gDisplay1, "Dr", program->dryMinutes);
  }
}

static void DisplayProgramTime(TM1637Handle *display, const char prefix[2], uint16_t minutes)
{
  uint8_t segments[4];

  if (minutes > 99U) {
    minutes = 99U;
  }

  segments[0] = SegmentForCharacter(prefix[0]);
  segments[1] = SegmentForCharacter(prefix[1]) | (1U << 7);
  segments[2] = SegmentForCharacter((char)('0' + ((minutes / 10U) % 10U)));
  segments[3] = SegmentForCharacter((char)('0' + (minutes % 10U)));
  tm1637DisplaySegments(display, segments);
}

static void TemperatureDisplay_Process(uint32_t now)
{
  if ((now - gLastTemperatureReadTick) < TEMPERATURE_REFRESH_MS) {
    return;
  }

  gLastTemperatureReadTick = now;
  if (gSelectedProgram != PROGRAM_NONE) {
    return;
  }

  if (gSensorReady != 0U) {
    if (Max31865_ReadTemperatureTenthsC(&gMax31865, &gTemperatureTenthsC) != 0U) {
      tm1637DisplayDecimalTenths(&gDisplay2, gTemperatureTenthsC);
    }
    else {
      DisplayErrorOnDisplay2(Max31865_ReadFault(&gMax31865, MAX31865_FAULT_NONE));
    }
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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_HOST_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  tm1637Init(&gDisplay1, TM1637_DISPLAY_1);
  tm1637Init(&gDisplay2, TM1637_DISPLAY_2);
  tm1637SetBrightness(&gDisplay1, 8);
  tm1637SetBrightness(&gDisplay2, 8);
  tm1637Clear(&gDisplay1);
  tm1637Clear(&gDisplay2);
  ProgramButtons_Init();
  ProgramLeds_Set(PROGRAM_NONE);

  Max31865_Init(&gMax31865, &hspi3, CS_GPIO_Port, CS_Pin, 430.0f, 100.0f);
  gSensorReady = Max31865_Begin(&gMax31865, MAX31865_2WIRE, 1U);
  if (gSensorReady == 0U) {
      DisplayErrorOnDisplay2(1U);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    ProgramButtons_Process(now);

    ProgramDisplay_Update(now);

    TemperatureDisplay_Process(now);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LD_C3_Pin|LD_C4_Pin|LD_C5_Pin|LD_C6_Pin
                          |LD_C7_Pin|LD_Alarm_Pin|LD_LW_Pin|LD_HW_Pin
                          |SSR_Heater_Pin|SSR_HResistor_Pin|Relay_Valve1_Pin|Relay_Valve2_Pin
                          |Relay_Valve3_Pin|LD_C1_Pin|LD_C2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|CLK1_Pin|DIO1_Pin|CLK2_Pin
                          |DIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Relay_Pump_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|LD_P3_Pin|LD_P2_Pin|LD_P1_Pin
                          |LD_P4_Pin|LD_P5_Pin|LD_P6_Pin|LD_Start_Pin
                          |LD_User_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD_C3_Pin LD_C4_Pin LD_C5_Pin LD_C6_Pin
                           LD_C7_Pin LD_Alarm_Pin LD_LW_Pin LD_HW_Pin
                           SSR_Heater_Pin SSR_HResistor_Pin Relay_Valve1_Pin Relay_Valve2_Pin
                           Relay_Valve3_Pin LD_C1_Pin LD_C2_Pin */
  GPIO_InitStruct.Pin = LD_C3_Pin|LD_C4_Pin|LD_C5_Pin|LD_C6_Pin
                          |LD_C7_Pin|LD_Alarm_Pin|LD_LW_Pin|LD_HW_Pin
                          |SSR_Heater_Pin|SSR_HResistor_Pin|Relay_Valve1_Pin|Relay_Valve2_Pin
                          |Relay_Valve3_Pin|LD_C1_Pin|LD_C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_P1_Pin B_P2_Pin B_P3_Pin B_P4_Pin
                           B_P5_Pin B_P6_Pin B_Start_Pin B_Set_Pin
                           B_Up_Pin B_Down_Pin Water_S_Pin B_User_Pin */
  GPIO_InitStruct.Pin = B_P1_Pin|B_P2_Pin|B_P3_Pin|B_P4_Pin
                          |B_P5_Pin|B_P6_Pin|B_Start_Pin|B_Set_Pin
                          |B_Up_Pin|B_Down_Pin|Water_S_Pin|B_User_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin L_Switch_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin|L_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin CLK1_Pin DIO1_Pin CLK2_Pin
                           DIO2_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|CLK1_Pin|DIO1_Pin|CLK2_Pin
                          |DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_Pump_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD6_Pin LD_P3_Pin LD_P2_Pin LD_P1_Pin
                           LD_P4_Pin LD_P5_Pin LD_P6_Pin LD_Start_Pin
                           LD_User_Pin */
  GPIO_InitStruct.Pin = Relay_Pump_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|LD_P3_Pin|LD_P2_Pin|LD_P1_Pin
                          |LD_P4_Pin|LD_P5_Pin|LD_P6_Pin|LD_Start_Pin
                          |LD_User_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
