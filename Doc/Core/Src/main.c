/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "dht11.h"
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 适配byte类型，屏蔽Arduino的PROGMEM宏（STM32环境无需）
typedef uint8_t byte;
#define PROGMEM
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 新图案尺寸：64×64（数组总长度512字节）
#define _1_HEIGHT 64
#define _1_WIDTH 64

// 64×64图案点阵数据（512字节）
static const byte _1[] PROGMEM  = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x07, 0x00, 
  0x00, 0x0f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xf7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0x7f, 0xcf, 0xc1, 0xe0, 0xe0, 0x70, 
  0xf8, 0xfe, 0xff, 0x81, 0x01, 0x07, 0x0f, 0x1f, 
  0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xef, 0xf7, 
  0xfb, 0xfd, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xfa, 0xfd, 0xff, 0xfb, 0xf8, 0xf8, 0xf8, 
  0xf8, 0xf8, 0xf9, 0xf3, 0xf7, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xfb, 0xfb, 0xff, 0xef, 0xef, 0xff, 0xff, 0xdf, 
  0xff, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xff, 0xef, 
  0xff, 0xf9, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xbf, 0x97, 0x07, 0x3f, 
  0x8f, 0xcf, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x6f, 
  0x2f, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0x3f, 0x8f, 0xff, 0xff, 0x1f, 0x1f, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0xbf, 0x3f, 
  0x3f, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0x9f, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 
  0x6f, 0x0f, 0x0f, 0x0f, 0x3f, 0x67, 0xff, 0xff, 
  0xff, 0xfe, 0xfc, 0xfd, 0xfd, 0xfc, 0x00, 0xfe, 
  0xff, 0xff, 0xff, 0xff, 0xe3, 0xf8, 0xdf, 0xd1, 
  0xc0, 0xe1, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xfe, 
  0xff, 0x83, 0xff, 0xe7, 0xc6, 0xcf, 0xfb, 0xff, 
  0xff, 0xff, 0xff, 0xf1, 0xf0, 0xfe, 0xcf, 0xe8, 
  0xe8, 0xfd, 0xf7, 0xf7, 0xff, 0xff, 0xfd, 0xed, 
  0xf0, 0xfe, 0xf6, 0xe6, 0xff, 0xff, 0xff, 0xcb, 
  0xec, 0xe6, 0x87, 0xd1, 0xf7, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define true 1
#define false 0
	int BPM;
	int Signal;
	int IBI = 600;
	unsigned char Pulse=false;
	unsigned char QS=false;
	int rate[10];
	unsigned long sampleCounter=0;
	unsigned long lastBeatTime=0;
	int P=512;
	int T=512;
	int thresh=512;
	int amp=100;
	int Num;
	unsigned char firstBeat=true;
	unsigned char secondBeat=true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void OLED_Draw64x64Pattern_1(void); // 适配_1数组的绘制函数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 绘制64×64的_1图案到OLED左侧（0~63列），完整无裁剪
void OLED_Draw64x64Pattern_1(void) {
  uint8_t page, col;
  uint32_t idx = 0; // _1数组从0开始索引，共512字节
  
  // 遍历8个页（64行 = 8页 × 8行/页）
  for(page = 0; page < 8; page++) {
    // 设置OLED页地址（0xB0~0xB7对应0~7页）
    OLED_WR_Byte(0xB0 + page, OLED_CMD);
    // 设置列起始地址：0列（低4位0x00 + 高4位0x10）
    OLED_WR_Byte(0x00, OLED_CMD);  // 列地址低4位
    OLED_WR_Byte(0x10, OLED_CMD);  // 列地址高4位
    
    // 绘制64列（0~63列），刚好匹配_1数组的64宽度
    for(col = 0; col < 64; col++) {
      if(idx < 512) { // 仅遍历512字节（64×64）
        OLED_WR_Byte(_1[idx], OLED_DATA);
        idx++;
      } else {
        OLED_WR_Byte(0x00, OLED_DATA); // 超出部分填0，避免乱码
      }
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // OLED初始化（保留你原有的函数名，避免编译错误）
  OLED_Init();
  OLED_ColorTurn(0);
  OLED_DisplayTurn(0);
  OLED_DisPlay_On();
  
  uint8_t temperature = 0;
  uint8_t humidity = 0;
  uint8_t OLED_Buf[20] = {0};
  DHT11_Strat(); // 保留原有DHT11初始化逻辑
  
  // 绘制_1数组的64×64图案（仅绘制一次，永久显示在左侧）
  OLED_Draw64x64Pattern_1();
  HAL_Delay(500); // 确保图案完全显示
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

uint8_t DHT11_flag=0;
HAL_TIM_Base_Stop_IT(&htim1); //读数据之前一定要关闭 一定要
DHT11_flag=DHT11_Read_Data(&temperature, &humidity);
HAL_TIM_Base_Start_IT(&htim1);
if(DHT11_flag==1)  //读取温湿度
{
    OLED_Clear();
    sprintf((char*)OLED_Buf,"No Response" );  //OLED显示部分
    OLED_ShowString(0,0,OLED_Buf,12,1);
}
else
{
    sprintf((char*)OLED_Buf,"temperature:%02d ",temperature );  //OLED显示部分
    OLED_ShowString(0,0,OLED_Buf,12,1);
    sprintf((char*)OLED_Buf,"humidity:%02d%% ",humidity );
    OLED_ShowString(0,14,OLED_Buf,12,1);
}

if(QS==true)
{
    QS=false;
    sprintf((char*)OLED_Buf,"BPM:%02d ", BPM);
    OLED_ShowString(0,32,OLED_Buf,12,1);
}
    sprintf((char*)OLED_Buf,"BPM:%02d ", BPM);
    OLED_ShowString(0,32,OLED_Buf,12,1);
    OLED_Refresh();
    HAL_Delay(1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t ADC_Read(uint32_t Channel)
{
	ADC_ChannelConfTypeDef sConfig={0};
	sConfig.Channel=Channel;
	sConfig.Rank=ADC_REGULAR_RANK_1;
	if(HAL_ADC_ConfigChannel(&hadc1,&sConfig)!=HAL_OK)
	{
		Error_Handler();
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
	return (uint16_t)HAL_ADC_GetValue(&hadc1);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    unsigned int runningTotal;  // 用于累加计算最近10次心跳间隔的总和

    // 判断是否是TIM1的定时中断（定时周期为2ms，用于脉搏信号采样）
    if(htim->Instance==htim1.Instance)
    {
        Signal=ADC_Read(ADC_CHANNEL_0);		// 读取脉搏传感器信号（ADC采样）
        sampleCounter += 2;                  // 使用此变量以毫秒为单位记录时间（每次中断2ms，累加计时）
        Num = sampleCounter - lastBeatTime;  // 计算距离上一次心跳的时间，以避免噪声干扰
        HAL_ADC_Start(&hadc1);				// 重新启动 ADC 转换，为下一次采样做准备

        // 寻找脉搏波的峰值和谷值
        // 等待上一个心跳间隔的 3/5 时间，避免重搏波干扰（重搏波是脉搏波的次峰，非有效心跳）
        if(Signal < thresh && Num > (IBI/5)*3){       
            if (Signal < T){                         // T 为当前检测到的最低点（谷值）
                T = Signal;                         // 记录脉搏波的最低点（更新谷值）
            }
        }

        // 超过阈值且高于当前峰值时更新峰值（有效心跳的上升沿判断）
        if(Signal > thresh && Signal > P){            
            P = Signal;                                 // 记录脉搏波的最高点（更新峰值）
        }                                             // 保持峰值记录

        // 检测心跳
        // 每次信号上升时都会出现一个脉搏峰
        // 过滤250ms内的高频噪声（避免误判为心跳）
        if (Num > 250){
            // 信号超过阈值 + 当前无心跳标记 + 超过重搏波过滤时间 → 判定为有效心跳
            if ( (Signal > thresh) && (Pulse == false) && (Num > (IBI/5)*3) ){
                Pulse = true;                          // 标记当前检测到心跳上升沿
                IBI = sampleCounter - lastBeatTime;    // 计算本次心跳间隔（IBI：Inter-Beat Interval，单位ms）
                lastBeatTime = sampleCounter;          // 更新上一次心跳的时间戳

                // 处理第二次有效心跳（初始化心率数组）
                if(secondBeat){
                    secondBeat = false;                // 清除第二次心跳标记
                    // 将前10次心跳间隔初始化为当前IBI（避免数组初始值干扰）
                    for(int i=0; i<=9; i++)
                    {   
                        rate[i] = IBI;
                    }
                }

                // 处理第一次有效心跳（仅标记，不计算心率）
                if(firstBeat){
                    firstBeat = false;                 // 清除第一次心跳标记
                    secondBeat = true;                 // 标记即将到来的第二次心跳
                    return;                            // 第一次心跳不参与心率计算，直接返回
                }

                // 计算最近 10 次心跳的平均值（滑动窗口平均，提高心率稳定性）
                runningTotal = 0;                      // 初始化累加和
                // 心率数组左移（丢弃最旧的一次IBI，保留最新9次）
                for(int i=0; i<=8; i++){
                    rate[i] = rate[i+1];               // 数组元素左移一位
                    runningTotal += rate[i];           // 累加前9次IBI
                }
                rate[9] = IBI;                         // 最新的IBI存入数组最后一位
                runningTotal += rate[9];               // 累加第10次IBI
                runningTotal /= 10;                    // 计算10次IBI的平均值（单位ms）
                BPM = 60000/runningTotal;              // 计算心率：60秒=60000ms ÷ 平均心跳间隔（BPM：次/分钟）
                QS = true;                             // 标记心率计算完成，可输出BPM
            }
        }

        // 当信号下降时，表示一次心跳结束（脉搏波下降沿）
        if (Signal < thresh && Pulse == true){        
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);  // 心跳指示灯熄灭（可选：可视化心跳结束）
            Pulse = false;                             // 清除心跳标记，准备下一次检测
            amp = P - T;                               // 计算本次脉搏波的振幅（峰值-谷值）
            thresh = amp/2 + T;                        // 更新阈值为振幅的1/2 + 谷值（自适应阈值，适配不同信号强度）
            P = thresh;                                 // 重置峰值为新阈值，避免残留峰值干扰下一次检测
            T = thresh;                                 // 重置谷值为新阈值，避免残留谷值干扰下一次检测
        }

        // 若 2.5 秒内未检测到心跳（判定为无有效脉搏）
        if (Num > 2500){                              
            thresh = 512;                               // 重置阈值为默认值（ADC 12位采样中点，0-4095）
            P = 512;                                    // 重置峰值为默认值
            T = 512;                                    // 重置谷值为默认值
            lastBeatTime = sampleCounter;               // 更新最后心跳时间，避免Num持续增大
            firstBeat = true;                           // 重置首次心跳标记
            secondBeat = false;                         // 重置第二次心跳标记
        }
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
