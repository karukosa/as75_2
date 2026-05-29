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

typedef enum {
  USER_EDIT_TEMPERATURE = 0,
  USER_EDIT_STERILIZE = 1,
  USER_EDIT_DRY = 2
} UserEditField;

typedef enum {
  MAIN_PHASE_STANDBY = 0,
  MAIN_PHASE_VACUUM,
  MAIN_PHASE_HEATING,
  MAIN_PHASE_HOLDING,
  MAIN_PHASE_EXHAUST,
  MAIN_PHASE_DRYING,
  MAIN_PHASE_DONE
} MainCyclePhase;

typedef struct {
  uint8_t pulseCount;
  uint8_t active;
  uint8_t outputOn;
  uint32_t onMs;
  uint32_t offMs;
  uint32_t lastToggleTick;
} BuzzerSequence;

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
#define USER_BLINK_MS 500U
#define USER_TEMPERATURE_MIN_TENTHS 1100U
#define USER_TEMPERATURE_MAX_TENTHS 1340U
#define USER_TEMPERATURE_STEP_TENTHS 10
#define USER_TEMPERATURE_FAST_STEP_TENTHS 50
#define USER_TIME_MIN_MINUTES 0U
#define USER_TIME_MAX_MINUTES 99U
#define USER_TIME_STEP_MINUTES 1
#define USER_TIME_FAST_STEP_MINUTES 10
#define BUZZER_BUTTON_MS 300U
#define BUZZER_START_MS 500U
#define BUZZER_STOP_MS 1000U
#define BUZZER_COMPLETE_MS 1000U
#define BUZZER_BETWEEN_PULSES_MS 150U
#define BUZZER_START_GAP_MS 500U
#define MAIN_VACUUM_MS 60000U
#define MAIN_EXHAUST_MS 30000U
#define MINUTE_MS 60000U
#define MAIN_CYCLE_LED_BLINK_MS 500U
#define ACTIVE_CHANNEL_USER 0U

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
ButtonInput gUserButton;
ButtonInput gSetButton;
ButtonInput gUpButton;
ButtonInput gDownButton;
ButtonInput gStartButton;
int16_t gTemperatureTenthsC = 0;
uint8_t gSensorReady = 0U;
uint8_t gSelectedProgram = PROGRAM_NONE;
uint8_t gProgramTimeDisplayPhase = 0xffU;
uint32_t gProgramSelectedTick = 0U;
uint32_t gLastTemperatureReadTick = 0U;
uint8_t gUserModeActive = 0U;
UserEditField gUserEditField = USER_EDIT_TEMPERATURE;
ProgramConfig gUserProgram = {1210U, 15U, 0U};
uint8_t gUserBlinkPhase = 0xffU;
uint8_t gUserDisplayRefresh = 0U;
uint32_t gUserEditTick = 0U;
uint8_t gMainCycleActive = 0U;
MainCyclePhase gMainCyclePhase = MAIN_PHASE_STANDBY;
ProgramConfig gActiveProgram = {1210U, 15U, 0U};
uint8_t gActiveProgramChannel = ACTIVE_CHANNEL_USER;
uint32_t gMainPhaseStartTick = 0U;
uint8_t gCycleLedBlinkPhase = 0xffU;
uint32_t gCycleLedBlinkTick = 0U;
MainCyclePhase gMainDisplayPhase = MAIN_PHASE_STANDBY;
uint16_t gMainDisplayValue = 0xffffU;
BuzzerSequence gBuzzer = {0U, 0U, 0U, 0U, 0U, 0U};

static const ProgramConfig programPresets[PROGRAM_COUNT] = {
  {1210U, 15U, 0U}, {1210U, 20U, 15U}, {1320U, 7U, 10U},
  {1340U, 7U, 10U}, {1340U, 10U, 20U}, {1340U, 5U, 5U}
};

static const GpioOutput programLeds[PROGRAM_COUNT] = {
  {LD_P1_GPIO_Port, LD_P1_Pin}, {LD_P2_GPIO_Port, LD_P2_Pin},
  {LD_P3_GPIO_Port, LD_P3_Pin}, {LD_P4_GPIO_Port, LD_P4_Pin},
  {LD_P5_GPIO_Port, LD_P5_Pin}, {LD_P6_GPIO_Port, LD_P6_Pin}
};

static const GpioOutput cycleLeds[7] = {
  {LD_C1_GPIO_Port, LD_C1_Pin}, {LD_C2_GPIO_Port, LD_C2_Pin},
  {LD_C3_GPIO_Port, LD_C3_Pin}, {LD_C4_GPIO_Port, LD_C4_Pin},
  {LD_C5_GPIO_Port, LD_C5_Pin}, {LD_C6_GPIO_Port, LD_C6_Pin},
  {LD_C7_GPIO_Port, LD_C7_Pin}
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
static void UserButtons_Init(void);
static void UserButtons_Process(uint32_t now);
static void UserMode_Enter(uint32_t now);
static void UserMode_Exit(void);
static void UserLed_Update(void);
static void UserMode_NextField(uint32_t now);
static void UserMode_Adjust(int16_t delta, uint32_t now);
static uint16_t ClampUint16(int32_t value, uint16_t minValue, uint16_t maxValue);
static void UserDisplay_Update(uint32_t now);
static void UserDisplay_MarkDirty(uint32_t now);
static void StartButton_Init(void);
static void StartButton_Process(uint32_t now);
static void MainCycle_Start(uint32_t now);
static void MainCycle_Stop(void);
static void MainCycle_Process(uint32_t now);
static void MainCycle_SetPhase(MainCyclePhase phase, uint32_t now);
static void MainCycle_UpdateTemperatureDisplay(uint32_t now);
static void MainCycleDisplay_Update(uint32_t now);
static void DisplayCycleChannel(void);
static void DisplayEndMessage(void);
static uint16_t MainCycle_RemainingMinutes(uint32_t elapsed, uint16_t totalMinutes);
static void MainCycleLeds_Clear(void);
static void MainCycleLeds_Update(uint32_t now);
static void MainCycleLed_Set(uint8_t ledIndex, uint8_t on);
static void MainCycleHoldingLeds_Update(uint32_t elapsed, uint8_t blinkOn);
static const ProgramConfig *GetSelectedProgramConfig(void);
static void Buzzer_Start(uint8_t pulseCount, uint32_t onMs);
static void Buzzer_StartWithGap(uint8_t pulseCount, uint32_t onMs, uint32_t offMs);
static void Buzzer_Process(uint32_t now);
static void Buzzer_Set(uint8_t on);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DisplayErrorOnDisplay2(uint8_t code)
{
  /* E r r <code> */
  uint8_t segments[4] = {0x79U, 0x50U, 0x50U, 0x00U};
  static const uint8_t digitMap[10] = {0x3fU, 0x06U, 0x5bU, 0x4fU, 0x66U, 0x6dU, 0x7dU, 0x07U, 0x7fU, 0x6fU};
  segments[3] = digitMap[code % 10U];
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
    case 'C': return 0x39U;
    case 'D': return 0x5eU;
    case 'E': return 0x79U;
    case 'H': return 0x76U;
    case 'S': return 0x6dU;
    case 'U': return 0x3eU;
    case 'd': return 0x5eU;
    case 'n': return 0x54U;
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
    if (gMainCycleActive != 0U) {
      (void)ButtonInput_ConsumePressed(&gProgramButtons[i]);
    }
    else if (ButtonInput_ConsumePressed(&gProgramButtons[i]) != 0U) {
      Program_Select(i, now);
      Buzzer_Start(1U, BUZZER_BUTTON_MS);
    }
  }
}

static void Program_Select(uint8_t programIndex, uint32_t now)
{
  if (programIndex >= PROGRAM_COUNT || gMainCycleActive != 0U) {
    return;
  }

  UserMode_Exit();
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

  if (gMainCycleActive != 0U || gUserModeActive != 0U || gSelectedProgram >= PROGRAM_COUNT) {
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
  if (gMainCycleActive != 0U) {
    MainCycle_UpdateTemperatureDisplay(now);
    return;
  }

  if (gUserModeActive != 0U) {
    return;
  }

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

static void UserButtons_Init(void)
{
  ButtonInput_Init(&gUserButton, B_User_GPIO_Port, B_User_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&gSetButton, B_Set_GPIO_Port, B_Set_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&gUpButton, B_Up_GPIO_Port, B_Up_Pin, GPIO_PIN_SET);
  ButtonInput_Init(&gDownButton, B_Down_GPIO_Port, B_Down_Pin, GPIO_PIN_SET);
}

static void StartButton_Init(void)
{
  ButtonInput_Init(&gStartButton, B_Start_GPIO_Port, B_Start_Pin, GPIO_PIN_SET);
}

static void UserButtons_Process(uint32_t now)
{
  ButtonInput_Update(&gUserButton, now, PROGRAM_DEBOUNCE_MS, PROGRAM_LONG_PRESS_MS, PROGRAM_REPEAT_MS);
  ButtonInput_Update(&gSetButton, now, PROGRAM_DEBOUNCE_MS, PROGRAM_LONG_PRESS_MS, PROGRAM_REPEAT_MS);
  ButtonInput_Update(&gUpButton, now, PROGRAM_DEBOUNCE_MS, PROGRAM_LONG_PRESS_MS, PROGRAM_REPEAT_MS);
  ButtonInput_Update(&gDownButton, now, PROGRAM_DEBOUNCE_MS, PROGRAM_LONG_PRESS_MS, PROGRAM_REPEAT_MS);

  if (gMainCycleActive != 0U) {
    (void)ButtonInput_ConsumePressed(&gUserButton);
    (void)ButtonInput_ConsumePressed(&gSetButton);
    (void)ButtonInput_ConsumePressed(&gUpButton);
    (void)ButtonInput_ConsumeRepeat(&gUpButton);
    (void)ButtonInput_ConsumePressed(&gDownButton);
    (void)ButtonInput_ConsumeRepeat(&gDownButton);
    return;
  }

  if (ButtonInput_ConsumePressed(&gUserButton) != 0U) {
    if (gMainCycleActive == 0U) {
      UserMode_Enter(now);
    }
    Buzzer_Start(1U, BUZZER_BUTTON_MS);
  }

  if (gUserModeActive == 0U) {
    (void)ButtonInput_ConsumePressed(&gSetButton);
    (void)ButtonInput_ConsumePressed(&gUpButton);
    (void)ButtonInput_ConsumeRepeat(&gUpButton);
    (void)ButtonInput_ConsumePressed(&gDownButton);
    (void)ButtonInput_ConsumeRepeat(&gDownButton);
    return;
  }

  if (ButtonInput_ConsumePressed(&gSetButton) != 0U) {
    UserMode_NextField(now);
    Buzzer_Start(1U, BUZZER_BUTTON_MS);
  }

  if (ButtonInput_ConsumePressed(&gUpButton) != 0U) {
    UserMode_Adjust(1, now);
  }
  if (ButtonInput_ConsumeRepeat(&gUpButton) != 0U) {
    UserMode_Adjust(10, now);
  }

  if (ButtonInput_ConsumePressed(&gDownButton) != 0U) {
    UserMode_Adjust(-1, now);
  }
  if (ButtonInput_ConsumeRepeat(&gDownButton) != 0U) {
    UserMode_Adjust(-10, now);
  }
}

static void UserMode_Enter(uint32_t now)
{
  if (gMainCycleActive != 0U) {
    return;
  }

  gUserModeActive = 1U;
  gSelectedProgram = PROGRAM_NONE;
  gUserEditField = USER_EDIT_TEMPERATURE;
  ProgramLeds_Set(PROGRAM_NONE);
  UserLed_Update();
  UserDisplay_MarkDirty(now);
  UserLed_Update();
}

static void UserMode_Exit(void)
{
  gUserModeActive = 0U;
  UserLed_Update();
}

static void UserLed_Update(void)
{
  HAL_GPIO_WritePin(LD_User_GPIO_Port, LD_User_Pin,
                    (gUserModeActive != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void UserMode_NextField(uint32_t now)
{
  if (gUserEditField == USER_EDIT_TEMPERATURE) {
    gUserEditField = USER_EDIT_STERILIZE;
  }
  else if (gUserEditField == USER_EDIT_STERILIZE) {
    gUserEditField = USER_EDIT_DRY;
  }
  else {
    gUserEditField = USER_EDIT_TEMPERATURE;
  }

  UserDisplay_MarkDirty(now);
}

static void UserMode_Adjust(int16_t delta, uint32_t now)
{
  if (gUserEditField == USER_EDIT_TEMPERATURE) {
    int16_t step = (delta > 0) ? USER_TEMPERATURE_STEP_TENTHS : (int16_t)-USER_TEMPERATURE_STEP_TENTHS;
    if (delta >= 10) {
      step = USER_TEMPERATURE_FAST_STEP_TENTHS;
    }
    else if (delta <= -10) {
      step = (int16_t)-USER_TEMPERATURE_FAST_STEP_TENTHS;
    }
    gUserProgram.temperatureTenthsC = ClampUint16((int32_t)gUserProgram.temperatureTenthsC + step,
                                                  USER_TEMPERATURE_MIN_TENTHS,
                                                  USER_TEMPERATURE_MAX_TENTHS);
  }
  else if (gUserEditField == USER_EDIT_STERILIZE) {
    int16_t step = (delta > 0) ? USER_TIME_STEP_MINUTES : (int16_t)-USER_TIME_STEP_MINUTES;
    if (delta >= 10) {
      step = USER_TIME_FAST_STEP_MINUTES;
    }
    else if (delta <= -10) {
      step = (int16_t)-USER_TIME_FAST_STEP_MINUTES;
    }
    gUserProgram.sterilizeMinutes = ClampUint16((int32_t)gUserProgram.sterilizeMinutes + step,
                                                USER_TIME_MIN_MINUTES,
                                                USER_TIME_MAX_MINUTES);
  }
  else {
    int16_t step = (delta > 0) ? USER_TIME_STEP_MINUTES : (int16_t)-USER_TIME_STEP_MINUTES;
    if (delta >= 10) {
      step = USER_TIME_FAST_STEP_MINUTES;
    }
    else if (delta <= -10) {
      step = (int16_t)-USER_TIME_FAST_STEP_MINUTES;
    }
    gUserProgram.dryMinutes = ClampUint16((int32_t)gUserProgram.dryMinutes + step,
                                          USER_TIME_MIN_MINUTES,
                                          USER_TIME_MAX_MINUTES);
  }

  UserDisplay_MarkDirty(now);
}

static uint16_t ClampUint16(int32_t value, uint16_t minValue, uint16_t maxValue)
{
  if (value < (int32_t)minValue) {
    return minValue;
  }
  if (value > (int32_t)maxValue) {
    return maxValue;
  }
  return (uint16_t)value;
}

static void UserDisplay_Update(uint32_t now)
{
  uint8_t blinkPhase;
  uint8_t blinkOn;

  if (gMainCycleActive != 0U || gUserModeActive == 0U) {
    return;
  }

  blinkPhase = (uint8_t)(((now - gUserEditTick) / USER_BLINK_MS) & 0x01U);
  if (blinkPhase == gUserBlinkPhase && gUserDisplayRefresh == 0U) {
    return;
  }

  gUserBlinkPhase = blinkPhase;
  gUserDisplayRefresh = 0U;
  blinkOn = (blinkPhase == 0U) ? 1U : 0U;

  if (gUserEditField == USER_EDIT_TEMPERATURE) {
    DisplayProgramTime(&gDisplay1, "St", gUserProgram.sterilizeMinutes);
    if (blinkOn != 0U) {
      tm1637DisplayDecimalTenths(&gDisplay2, gUserProgram.temperatureTenthsC);
    }
    else {
      tm1637Clear(&gDisplay2);
    }
  }
  else if (gUserEditField == USER_EDIT_STERILIZE) {
    tm1637DisplayDecimalTenths(&gDisplay2, gUserProgram.temperatureTenthsC);
    if (blinkOn != 0U) {
      DisplayProgramTime(&gDisplay1, "St", gUserProgram.sterilizeMinutes);
    }
    else {
      tm1637Clear(&gDisplay1);
    }
  }
  else {
    tm1637DisplayDecimalTenths(&gDisplay2, gUserProgram.temperatureTenthsC);
    if (blinkOn != 0U) {
      DisplayProgramTime(&gDisplay1, "Dr", gUserProgram.dryMinutes);
    }
    else {
      tm1637Clear(&gDisplay1);
    }
  }
}

static void UserDisplay_MarkDirty(uint32_t now)
{
  gUserBlinkPhase = 0xffU;
  gUserDisplayRefresh = 1U;
  gUserEditTick = now;
  UserDisplay_Update(now);
}

static void StartButton_Process(uint32_t now)
{
  ButtonInput_Update(&gStartButton, now, PROGRAM_DEBOUNCE_MS, PROGRAM_LONG_PRESS_MS, PROGRAM_REPEAT_MS);
  if (ButtonInput_ConsumePressed(&gStartButton) == 0U) {
    return;
  }

  if (gMainCycleActive != 0U) {
    MainCycle_Stop();
    Buzzer_Start(1U, BUZZER_STOP_MS);
  }
  else {
    MainCycle_Start(now);
    Buzzer_StartWithGap(3U, BUZZER_START_MS, BUZZER_START_GAP_MS);
  }
}

static void MainCycle_Start(uint32_t now)
{
  const ProgramConfig *selectedProgram = GetSelectedProgramConfig();

  if (selectedProgram != NULL) {
    gActiveProgram = *selectedProgram;
    gActiveProgramChannel = (gSelectedProgram < PROGRAM_COUNT) ? (uint8_t)(gSelectedProgram + 1U) : ACTIVE_CHANNEL_USER;
  }
  else {
    gActiveProgram = gUserProgram;
    gActiveProgramChannel = ACTIVE_CHANNEL_USER;
  }

  gMainCycleActive = 1U;
  UserMode_Exit();
  HAL_GPIO_WritePin(LD_Start_GPIO_Port, LD_Start_Pin, GPIO_PIN_SET);
  tm1637Clear(&gDisplay1);
  tm1637Clear(&gDisplay2);
  gCycleLedBlinkPhase = 0xffU;
  gMainDisplayPhase = MAIN_PHASE_STANDBY;
  gMainDisplayValue = 0xffffU;
  gLastTemperatureReadTick = now - TEMPERATURE_REFRESH_MS;
  MainCycle_SetPhase(MAIN_PHASE_VACUUM, now);
  MainCycle_UpdateTemperatureDisplay(now);
  MainCycleDisplay_Update(now);
  MainCycleLeds_Update(now);
}

static void MainCycle_Stop(void)
{
  gMainCycleActive = 0U;
  gMainCyclePhase = MAIN_PHASE_STANDBY;
  gSelectedProgram = PROGRAM_NONE;
  gActiveProgramChannel = ACTIVE_CHANNEL_USER;
  gProgramTimeDisplayPhase = 0xffU;
  gMainDisplayPhase = MAIN_PHASE_STANDBY;
  gMainDisplayValue = 0xffffU;
  UserMode_Exit();
  ProgramLeds_Set(PROGRAM_NONE);
  MainCycleLeds_Clear();
  gCycleLedBlinkPhase = 0xffU;
  HAL_GPIO_WritePin(LD_Start_GPIO_Port, LD_Start_Pin, GPIO_PIN_RESET);
  tm1637Clear(&gDisplay1);
  tm1637Clear(&gDisplay2);
}

static void MainCycle_Process(uint32_t now)
{
  uint32_t elapsed;

  if (gMainCycleActive == 0U) {
    return;
  }

  elapsed = now - gMainPhaseStartTick;
  switch (gMainCyclePhase) {
    case MAIN_PHASE_VACUUM:
      if (elapsed >= MAIN_VACUUM_MS) {
        MainCycle_SetPhase(MAIN_PHASE_HEATING, now);
      }
      break;

    case MAIN_PHASE_HEATING:
      if (gTemperatureTenthsC >= 0 && (uint16_t)gTemperatureTenthsC >= gActiveProgram.temperatureTenthsC) {
        MainCycle_SetPhase(MAIN_PHASE_HOLDING, now);
      }
      break;

    case MAIN_PHASE_HOLDING:
      if (elapsed >= ((uint32_t)gActiveProgram.sterilizeMinutes * MINUTE_MS)) {
        MainCycle_SetPhase(MAIN_PHASE_EXHAUST, now);
      }
      break;

    case MAIN_PHASE_EXHAUST:
      if (elapsed >= MAIN_EXHAUST_MS) {
        MainCycle_SetPhase(MAIN_PHASE_DRYING, now);
      }
      break;

    case MAIN_PHASE_DRYING:
      if (elapsed >= ((uint32_t)gActiveProgram.dryMinutes * MINUTE_MS)) {
        MainCycle_SetPhase(MAIN_PHASE_DONE, now);
        Buzzer_StartWithGap(5U, BUZZER_COMPLETE_MS, 500U);
      }
      break;

    case MAIN_PHASE_DONE:
      break;

    default:
      MainCycle_Stop();
      break;
  }
}

static void MainCycle_SetPhase(MainCyclePhase phase, uint32_t now)
{
  gMainCyclePhase = phase;
  gMainPhaseStartTick = now;
  gCycleLedBlinkPhase = 0xffU;
  gCycleLedBlinkTick = now;
  gMainDisplayPhase = MAIN_PHASE_STANDBY;
  gMainDisplayValue = 0xffffU;
}

static void MainCycle_UpdateTemperatureDisplay(uint32_t now)
{
  if ((now - gLastTemperatureReadTick) < TEMPERATURE_REFRESH_MS) {
    return;
  }

  gLastTemperatureReadTick = now;
  if (gSensorReady != 0U) {
    if (Max31865_ReadTemperatureTenthsC(&gMax31865, &gTemperatureTenthsC) != 0U) {
      tm1637DisplayDecimalTenths(&gDisplay2, gTemperatureTenthsC);
    }
    else {
      DisplayErrorOnDisplay2(Max31865_ReadFault(&gMax31865, MAX31865_FAULT_NONE));
    }
  }
}

static void MainCycleDisplay_Update(uint32_t now)
{
  uint16_t displayValue = 0xffffU;
  uint32_t elapsed;

  if (gMainCycleActive == 0U) {
    return;
  }

  elapsed = now - gMainPhaseStartTick;
  switch (gMainCyclePhase) {
    case MAIN_PHASE_VACUUM:
    case MAIN_PHASE_HEATING:
    case MAIN_PHASE_EXHAUST:
      if (gMainDisplayPhase != gMainCyclePhase) {
        DisplayCycleChannel();
      }
      gMainDisplayPhase = gMainCyclePhase;
      gMainDisplayValue = 0xffffU;
      break;

    case MAIN_PHASE_HOLDING:
      displayValue = MainCycle_RemainingMinutes(elapsed, gActiveProgram.sterilizeMinutes);
      if (gMainDisplayPhase != gMainCyclePhase || displayValue != gMainDisplayValue) {
        DisplayProgramTime(&gDisplay1, "St", displayValue);
      }
      gMainDisplayPhase = gMainCyclePhase;
      gMainDisplayValue = displayValue;
      break;

    case MAIN_PHASE_DRYING:
      displayValue = MainCycle_RemainingMinutes(elapsed, gActiveProgram.dryMinutes);
      if (gMainDisplayPhase != gMainCyclePhase || displayValue != gMainDisplayValue) {
        DisplayProgramTime(&gDisplay1, "Dr", displayValue);
      }
      gMainDisplayPhase = gMainCyclePhase;
      gMainDisplayValue = displayValue;
      break;

    case MAIN_PHASE_DONE:
      displayValue = (gCycleLedBlinkPhase == 0U) ? 0U : 1U;
      if (gMainDisplayPhase != gMainCyclePhase || displayValue != gMainDisplayValue) {
        if (displayValue != 0U) {
          DisplayEndMessage();
        }
        else {
          tm1637Clear(&gDisplay1);
        }
      }
      gMainDisplayPhase = gMainCyclePhase;
      gMainDisplayValue = displayValue;
      break;

    default:
      if (gMainDisplayPhase != gMainCyclePhase) {
        tm1637Clear(&gDisplay1);
      }
      gMainDisplayPhase = gMainCyclePhase;
      gMainDisplayValue = 0xffffU;
      break;
  }
}

static void DisplayCycleChannel(void)
{
  uint8_t segments[4];

  segments[0] = SegmentForCharacter('C');
  segments[1] = SegmentForCharacter('H') | (1U << 7);
  if (gActiveProgramChannel == ACTIVE_CHANNEL_USER) {
    segments[2] = SegmentForCharacter('U');
    segments[3] = SegmentForCharacter('S');
  }
  else {
    segments[2] = SegmentForCharacter('0');
    segments[3] = SegmentForCharacter((char)('0' + (gActiveProgramChannel % 10U)));
  }

  tm1637DisplaySegments(&gDisplay1, segments);
}

static void DisplayEndMessage(void)
{
  uint8_t segments[4];

  segments[0] = 0x00U;
  segments[1] = SegmentForCharacter('E');
  segments[2] = SegmentForCharacter('n');
  segments[3] = SegmentForCharacter('d');
  tm1637DisplaySegments(&gDisplay1, segments);
}

static uint16_t MainCycle_RemainingMinutes(uint32_t elapsed, uint16_t totalMinutes)
{
  uint32_t totalMs = (uint32_t)totalMinutes * MINUTE_MS;
  uint32_t remainingMs;

  if (elapsed >= totalMs) {
    return 0U;
  }

  remainingMs = totalMs - elapsed;
  return (uint16_t)((remainingMs + MINUTE_MS - 1U) / MINUTE_MS);
}

static void MainCycleLeds_Clear(void)
{
  for (uint8_t i = 0U; i < 7U; ++i) {
    MainCycleLed_Set(i, 0U);
  }
}

static void MainCycleLeds_Update(uint32_t now)
{
  uint8_t blinkOn;
  uint32_t elapsed;

  if (gMainCycleActive == 0U) {
    return;
  }

  if (gCycleLedBlinkPhase == 0xffU || (now - gCycleLedBlinkTick) >= MAIN_CYCLE_LED_BLINK_MS) {
    if (gCycleLedBlinkPhase == 0xffU) {
      gCycleLedBlinkPhase = 1U;
    }
    else {
      gCycleLedBlinkPhase ^= 1U;
    }
    gCycleLedBlinkTick = now;
  }

  blinkOn = gCycleLedBlinkPhase;
  elapsed = now - gMainPhaseStartTick;
  MainCycleLeds_Clear();

  switch (gMainCyclePhase) {
    case MAIN_PHASE_VACUUM:
      MainCycleLed_Set(0U, blinkOn);
      break;

    case MAIN_PHASE_HEATING:
      MainCycleLed_Set(0U, 1U);
      MainCycleLed_Set(1U, blinkOn);
      break;

    case MAIN_PHASE_HOLDING:
      MainCycleLed_Set(0U, 1U);
      MainCycleLed_Set(1U, 1U);
      MainCycleHoldingLeds_Update(elapsed, blinkOn);
      break;

    case MAIN_PHASE_EXHAUST:
      for (uint8_t i = 0U; i < 5U; ++i) {
        MainCycleLed_Set(i, 1U);
      }
      MainCycleLed_Set(5U, blinkOn);
      break;

    case MAIN_PHASE_DRYING:
      for (uint8_t i = 0U; i < 6U; ++i) {
        MainCycleLed_Set(i, 1U);
      }
      MainCycleLed_Set(6U, blinkOn);
      break;

    case MAIN_PHASE_DONE:
      for (uint8_t i = 0U; i < 7U; ++i) {
        MainCycleLed_Set(i, blinkOn);
      }
      break;

    default:
      break;
  }
}

static void MainCycleLed_Set(uint8_t ledIndex, uint8_t on)
{
  if (ledIndex >= 7U) {
    return;
  }
  HAL_GPIO_WritePin(cycleLeds[ledIndex].port, cycleLeds[ledIndex].pin,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void MainCycleHoldingLeds_Update(uint32_t elapsed, uint8_t blinkOn)
{
  uint32_t holdMs = (uint32_t)gActiveProgram.sterilizeMinutes * MINUTE_MS;
  uint32_t firstThird;
  uint32_t secondThird;

  if (holdMs == 0U) {
    for (uint8_t i = 2U; i <= 4U; ++i) {
      MainCycleLed_Set(i, 1U);
    }
    return;
  }

  firstThird = holdMs / 3U;
  secondThird = (holdMs * 2U) / 3U;
  if (firstThird == 0U) {
    firstThird = 1U;
  }
  if (secondThird <= firstThird) {
    secondThird = firstThird + 1U;
  }

  if (elapsed < firstThird) {
    MainCycleLed_Set(2U, blinkOn);
  }
  else if (elapsed < secondThird) {
    MainCycleLed_Set(2U, 1U);
    MainCycleLed_Set(3U, blinkOn);
  }
  else {
    MainCycleLed_Set(2U, 1U);
    MainCycleLed_Set(3U, 1U);
    MainCycleLed_Set(4U, blinkOn);
  }
}

static const ProgramConfig *GetSelectedProgramConfig(void)
{
  if (gUserModeActive != 0U) {
    return &gUserProgram;
  }
  if (gSelectedProgram < PROGRAM_COUNT) {
    return &programPresets[gSelectedProgram];
  }
  return NULL;
}

static void Buzzer_Start(uint8_t pulseCount, uint32_t onMs)
{
  Buzzer_StartWithGap(pulseCount, onMs, BUZZER_BETWEEN_PULSES_MS);
}

static void Buzzer_StartWithGap(uint8_t pulseCount, uint32_t onMs, uint32_t offMs)
{
  if (pulseCount == 0U || onMs == 0U) {
    return;
  }

  gBuzzer.pulseCount = pulseCount;
  gBuzzer.active = 1U;
  gBuzzer.outputOn = 1U;
  gBuzzer.onMs = onMs;
  gBuzzer.offMs = offMs;
  gBuzzer.lastToggleTick = HAL_GetTick();
  Buzzer_Set(1U);
}

static void Buzzer_Process(uint32_t now)
{
  if (gBuzzer.active == 0U) {
    return;
  }

  if (gBuzzer.outputOn != 0U) {
    if ((now - gBuzzer.lastToggleTick) >= gBuzzer.onMs) {
      Buzzer_Set(0U);
      gBuzzer.outputOn = 0U;
      gBuzzer.lastToggleTick = now;
      if (gBuzzer.pulseCount > 0U) {
        --gBuzzer.pulseCount;
      }
    }
  }
  else if ((now - gBuzzer.lastToggleTick) >= gBuzzer.offMs) {
    if (gBuzzer.pulseCount == 0U) {
      gBuzzer.active = 0U;
    }
    else {
      Buzzer_Set(1U);
      gBuzzer.outputOn = 1U;
      gBuzzer.lastToggleTick = now;
    }
  }
}

static void Buzzer_Set(uint8_t on)
{
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  UserButtons_Init();
  StartButton_Init();
  ProgramLeds_Set(PROGRAM_NONE);
  UserLed_Update();
  HAL_GPIO_WritePin(LD_Start_GPIO_Port, LD_Start_Pin, GPIO_PIN_RESET);
  Buzzer_Set(0U);

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
    StartButton_Process(now);
    ProgramButtons_Process(now);
    UserButtons_Process(now);
    UserLed_Update();
    ProgramDisplay_Update(now);
    UserDisplay_Update(now);
    TemperatureDisplay_Process(now);
    MainCycle_Process(now);
    MainCycleDisplay_Update(now);
    MainCycleLeds_Update(now);
    Buzzer_Process(now);
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
