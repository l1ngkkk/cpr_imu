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
/* 璋冭瘯寮€鍏筹細姝ｅ紡浣跨敤鏃堕兘璁句负 0锛屼覆鍙ｅ彧淇濈暀姣忔鎸夊帇鐨?# 琛屻€? * 闇€瑕佺湅娉㈠舰鎴栫姸鎬佹椂鍐嶆敼鍥?1銆?*/
#define CPR_DEBUG_STATUS   0   /* 1 = 姣忕鎵撳嵃 av / depth / rate / [diag] */
#define CPR_DEBUG_WAVE     0   /* 1 = 30 Hz 鎵撳嵃 W 琛屾尝褰?*/
#define CPR_DEBUG_AV       0   /* 1 = 每个采样打印 AV 行（会占满串口，只在必要时开） */
#define CPR_DEBUG_DISP     0   /* 1 = 30 Hz 打印 DV 行（新方案位移/速度，用来画波形） */
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
/* ================= 涓插彛鍥炴斁锛氭帴鏀朵笂浣嶆満鍙戞潵鐨?IMU 鍘熷鏁版嵁 ================= */
#define UART_RX_RING   2048
#define UART_LINE_MAX  80

static volatile uint8_t  s_rx_ring[UART_RX_RING];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static volatile uint8_t  s_rx_byte;

static char    s_line[UART_LINE_MAX];
static uint8_t s_line_len = 0U;

static CPR_Result_t s_cpr;
static uint32_t     s_last_t_ms = 0U;
static uint8_t      s_have_last = 0U;
static volatile uint32_t s_line_count = 0U;

/**
 * @brief  澶勭悊涓€琛屽畬鏁存暟鎹? * @note   鏍煎紡锛歵_ms,ax,ay,az,gx,gy,gz,temp锛堝師濮?LSB锛? *         涓庨噰闆嗘枃浠跺畬鍏ㄤ竴鑷达紝鍙洿鎺ュ洖鏀? */
static void uart_process_line(void)
{
    long  t_ms;
    int   ax, ay, az, gx, gy, gz, temp;
    float dt;
    IMU_Data_t d;

    if (s_line_len == 0U)
    {
        return;
    }
    s_line[s_line_len] = '\0';

    if (s_line[0] == '#')
    {
        return;                             /* 娉ㄩ噴琛岋紝璺宠繃 */
    }

    { static uint32_t echo_n = 0U; if ((++echo_n) <= 5U) { printf("RX,%s\r\n", s_line); } }
    if (sscanf(s_line, "%ld,%d,%d,%d,%d,%d,%d,%d",
               &t_ms, &ax, &ay, &az, &gx, &gy, &gz, &temp) != 8)
    {
        return;                             /* 鏍煎紡涓嶅锛屼涪寮?*/
    }

    if (s_have_last == 0U)
    {
        dt = 1.0f / 120.0f;
        s_have_last = 1U;
    }
    else
    {
        dt = (float)((uint32_t)t_ms - s_last_t_ms) / 1000.0f;
        if ((dt <= 0.0f) || (dt > 0.05f))
        {
            dt = 1.0f / 120.0f;             /* 寮傚父 dt 淇濇姢锛屽拰瀹炴満涓€鑷?*/
        }
    }
    s_last_t_ms = (uint32_t)t_ms;
    s_line_count++;

    d.acc_mg.x = (float)ax * 0.122f;
    d.acc_mg.y = (float)ay * 0.122f;
    d.acc_mg.z = (float)az * 0.122f;
    d.gyr_dps.x = (float)gx * 0.0175f;
    d.gyr_dps.y = (float)gy * 0.0175f;
    d.gyr_dps.z = (float)gz * 0.0175f;

    uint8_t ev = CPR_Update(&d, dt, &s_cpr);

#if CPR_DEBUG_AV
    printf("AV,%ld,%d\r\n", t_ms, (int)s_cpr.acc_v_mg);
#endif

#if CPR_DEBUG_DISP
    {
        static uint8_t dvdiv = 0U;
        if (++dvdiv >= 4U)      /* 117 Hz / 4 约 30 Hz */
        {
            dvdiv = 0U;
            printf("DV,%ld,%.2f,%.1f\r\n", t_ms,
                   (double)s_cpr.disp_mm, (double)s_cpr.vel_mm_s);
        }
    }
#endif

    if (ev != 0U)
    {
        /* 旧方案：单频正交 dft1，在滞回上升沿触发。
         * 末尾新增"数据时间 t_ms"字段，方便和平台 csv 对齐；
         * 前 6 个字段含义不变，老脚本照常能解析。 */
        printf("EV,%lu,%lu,%lu,%.1f,%.0f,%ld\r\n",
               (unsigned long)s_cpr.count,
               (unsigned long)s_cpr.t_ms,
               (unsigned long)s_cpr.interval_ms,
               (double)s_cpr.depth_mm,
               (double)s_cpr.rate_cpm,
               t_ms);
    }

    if (s_cpr.ev2 != 0U)
    {
        /* 新方案：漏积分峰谷差，在"压到底"那一个采样就触发（延迟约 8.5 ms）
         * EVL,<序号>,<数据时间ms>,<本次深度mm>,<2次平均mm> */
        printf("EVL,%lu,%ld,%.1f,%.1f\r\n",
               (unsigned long)s_cpr.count2,
               t_ms,
               (double)s_cpr.depth2_mm,
               (double)s_cpr.depth2_avg_mm);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\nREPLAY_READY\r\n");
  CPR_Init(0);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&s_rx_byte, 1U);
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    {
        static uint32_t last_hb = 0U;
        if ((HAL_GetTick() - last_hb) >= 1000U)
        {
            last_hb = HAL_GetTick();
            printf("HB,%lu,%u,%u,%u,%u\r\n", (unsigned long)s_line_count, (unsigned)s_cpr.rej_gyro, (unsigned)s_cpr.rej_period, (unsigned)s_cpr.cross_n, (unsigned)(CPR_GetPeriodAvg()*100.0f));
        }
    }
    while (s_rx_tail != s_rx_head)
    {
        char c = (char)s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % UART_RX_RING);

        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            uart_process_line();
            s_line_len = 0U;
        }
        else if (s_line_len < (UART_LINE_MAX - 1U))
        {
            s_line[s_line_len++] = c;
        }
    }
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
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&s_rx_byte, 1U);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint16_t next = (uint16_t)((s_rx_head + 1U) % UART_RX_RING);

        if (next != s_rx_tail)
        {
            s_rx_ring[s_rx_head] = s_rx_byte;
            s_rx_head = next;
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&s_rx_byte, 1U);
    }
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
