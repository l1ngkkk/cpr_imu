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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "imu_process.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 调试开关：正式使用时都设为 0，串口只保留每次按压的 # 行。
 * 需要看波形或状态时再改回 1。 */
#define CPR_DEBUG_STATUS   0   /* 1 = 每秒打印 av / depth / rate / [diag] */
#define CPR_DEBUG_WAVE     0   /* 1 = 30 Hz 打印 W 行波形 */
#define CPR_DEBUG_OLD      0   /* 1 = 额外打印旧方法(dft1)的每一次结果，做 A/B 对比 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  I2C 总线扫描：打印 0x08~0x77 之间所有有应答的 7 位地址
 * @note   用来判断“芯片到底在不在总线上、地址是 0x6A 还是 0x6B”
 */
static void imu_i2c_scan(void)
{
    uint8_t addr;
    uint8_t found = 0;

    printf("I2C scan (7-bit):");

    for (addr = 0x08; addr <= 0x77; addr++)
    {
        if (IMU_ProbeAddr(addr) == IMU_OK)
        {
            printf(" 0x%02X", addr);
            found = 1;
        }
    }

    if (found == 0)
    {
        printf(" <no device>");
    }

    printf("\r\n");
}

/**
 * @brief  重新初始化 I2C 外设
 * @note   STM32F1 的 HAL I2C 偶尔会卡在 BUSY 状态，DeInit + Init 可以复位状态机
 */
static void imu_i2c_recover(void)
{
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  IMU_Config_t imu_cfg;
  IMU_Data_t   imu_data;
  IMU_Status_t imu_st;
  CPR_Result_t cpr = {0};
  uint32_t     last_tick;
  uint8_t      ev;
#if CPR_DEBUG_STATUS
  uint32_t     last_status;
#endif

  printf("\r\nLSM6DSV demo start\r\n");

  IMU_DefaultConfig(&imu_cfg);          /* 120 Hz / ±4 g / ±500 dps */
  imu_cfg.acc_odr = IMU_ODR_120;
  imu_cfg.gyr_odr = IMU_ODR_120;

  imu_st = IMU_Init(&imu_cfg);
  if (imu_st != IMU_OK)
  {
      uint8_t id = 0;

      printf("IMU init failed, code = %d\r\n", (int)imu_st);
      imu_i2c_recover();
      imu_i2c_scan();

      if (IMU_WhoAmI(&id) == IMU_OK)
      {
          printf("WHO_AM_I = 0x%02X\r\n", id);
      }
      else
      {
          printf("WHO_AM_I read failed\r\n");
      }

      /* 总线复位后再初始化一次（上电第一次失败的典型恢复流程） */
      printf("retry IMU init...\r\n");
      imu_st = IMU_Init(&imu_cfg);
  }

  if (imu_st == IMU_OK)
  {
      printf("IMU init ok\r\n");
      IMU_CalibrateGyro(200);           /* 保持静止约 1 秒 */
      printf("gyro bias calibrated\r\n");

      /* CPR 算法：采样率要和上面的 ODR 一致，120 Hz 直接传 0 用默认值 */
      CPR_Init(0);
      printf("CPR monitor ready, start compressions\r\n");
  }
  else
  {
      printf("IMU init still failed, check wiring\r\n");
  }

  last_tick = HAL_GetTick();
#if CPR_DEBUG_STATUS
  last_status = last_tick;
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 等一帧新数据，而不是固定延时，这样 ODR 变了也不用改这里 */
    if (IMU_WaitDataReady(20U) == IMU_OK)
    {
        uint32_t now = HAL_GetTick();
        float dt = (float)(now - last_tick) / 1000.0f;

        last_tick = now;

        /* dt 异常保护：首帧、调试暂停、丢帧 */
        if ((dt <= 0.0f) || (dt > 0.05f))
        {
            dt = 1.0f / 120.0f;
        }

        if (IMU_Read(&imu_data) == IMU_OK)
        {
            /* CPR_Update 每帧都会更新结果。两个"算完一次"的标志：
             *   返回值 1     —— 旧方法（dft1）在滞回过零处，比压到底早约 0.24 s
             *   cpr.ev2 == 1 —— 新方法（漏积分）在"压到底"那一个采样，延迟约 8.5 ms
             * 这里只在 ev2 时打印一行，保证 depth / dt / rate 都是同一次按压的。
             * 注意：out->t_ms / out->interval_ms 是旧方法专用的，在 ev2 时刻还是 0，
             *       所以 dt 和 rate 直接由新方法自己的事件时刻算。 */
            ev = CPR_Update(&imu_data, dt, &cpr);
            (void)ev;                       /* 只在 CPR_DEBUG_OLD 时用到 */

#if CPR_DEBUG_OLD
            if (ev != 0U)
            {
                printf("[old] #%lu t=%lu dt=%lu depth=%5.1f avg=%5.1f rate=%3.0f\r\n",
                       (unsigned long)cpr.count,
                       (unsigned long)cpr.t_ms,
                       (unsigned long)cpr.interval_ms,
                       cpr.depth_mm, cpr.depth_avg_mm, cpr.rate_cpm);
            }
#endif

            if (cpr.ev2 != 0U)
            {
                static uint32_t last_ev2 = 0U;
                uint32_t t_ev  = HAL_GetTick();
                uint32_t dt_ev = (last_ev2 == 0U) ? 0U : (t_ev - last_ev2);
                float    rate  = (dt_ev > 0U) ? (60000.0f / (float)dt_ev) : 0.0f;

                last_ev2 = t_ev;

                /* 结果行：序号、时刻(ms)、间隔(ms)、本次深度、2次平均、频率 */
                printf("#%lu  t=%lu ms  dt=%lu ms  depth=%5.1f mm  avg=%5.1f mm  rate=%3.0f cpm\r\n",
                       (unsigned long)cpr.count2,
                       (unsigned long)t_ev,
                       (unsigned long)dt_ev,
                       cpr.depth2_mm, cpr.depth2_avg_mm, rate);
            }

#if CPR_DEBUG_WAVE
            /* 30 Hz 输出波形：t(ms), 竖直加速度(mg), 深度(mm), 状态 */
            {
                static uint8_t wdiv = 0U;

                if (++wdiv >= 4U)
                {
                    wdiv = 0U;
                    printf("W,%lu,%.1f,%.1f,%u\r\n",
                           (unsigned long)HAL_GetTick(),
                           cpr.acc_v_mg, cpr.depth_mm, cpr.stat);
                }
            }
#endif
        }
    }

#if CPR_DEBUG_STATUS
    /* 每秒打印一次当前位移：按压时 d 应该在 ±20~30 mm 摆动。
       如果一直是 0，说明加速度没进来、或者运动方向不对 */
    {
        uint32_t now2 = HAL_GetTick();

        if ((now2 - last_status) >= 1000U)
        {
            last_status = now2;
            printf("[stat] av=%7.1f mg  depth=%5.1f (avg %5.1f)  rate=%3.0f  cnt=%lu  |a|=%4.0fmg  w=%3.0fdps\r\n",
                   cpr.acc_v_mg, cpr.depth_mm, cpr.depth_avg_mm, cpr.rate_cpm,
                   (unsigned long)cpr.count,
                   cpr.acc_mag_mg, cpr.gyr_mag_dps);

            /* 计数诊断：过零总数 / 周期超范围 / 深度超范围 / 转动过大 */
            printf("[diag] cross=%u  rejP=%u  rejD=%u  rejG=%u\r\n",
                   (unsigned)cpr.cross_n, (unsigned)cpr.rej_period,
                   (unsigned)cpr.rej_depth, (unsigned)cpr.rej_gyro);
        }
    }
#endif
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
