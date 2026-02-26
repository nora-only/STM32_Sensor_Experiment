
//接口    ：PA1-DATA                   !!!!!!!!!!!!!!注意
 
#include "dht11.h"
#include "main.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include	"stdio.h"
#include  "string.h"
 
/* USER CODE END Includes */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DHT11_HIGH     HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin,	GPIO_PIN_SET) //输出高电平
#define DHT11_LOW      HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_RESET)//输出低电平
 
#define DHT11_IO_IN      HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin)//读取IO口电平
 
/* USER CODE END PD */

/**
  * @brief  DATA引脚（PA1）设置为输出模式
  * @param  无
  * @retval 无
  */
void DHT11_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/**
  * @brief  DATA引脚（PA1）设置为输入模式
  * @param  无
  * @retval	无 
  */
void DHT11_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct = {0};
 
	GPIO_InitStruct.Pin  = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  us级延时函数
  * @param  delay 控制延时的时长
  * @retval 无
  */
void Delay_us(uint32_t udelay)  //通过滴答定时器完成
{
    uint32_t startval, tickn, delays, wait;
    uint32_t ticks_per_us = (SystemCoreClock) / 1000000U; // 动态计算
    
    startval = SysTick->VAL;
    tickn = HAL_GetTick();
    delays = udelay * ticks_per_us;
    uint32_t reload = SysTick->LOAD + 1U; // 一周期tick数
    
    if (delays > startval)
    {
        while (HAL_GetTick() == tickn) { }
        wait = reload + startval - delays;
        while (wait < SysTick->VAL) { }
    }
    else
    {
        wait = startval - delays;
        while (wait < SysTick->VAL && HAL_GetTick() == tickn) { }
    }
}


/**
  * @brief  DHT11检测起始信号
  * @param  无
  * @retval 无
  */
void DHT11_Strat(void)
{
	DHT11_OUT();   //设置为输出模式
	DHT11_LOW;     //主机拉低总线
	HAL_Delay(20); //延迟必须大于18ms ； 
	DHT11_HIGH;    //主机拉高总线等待DHT11响应
	Delay_us(30);   
}
// 在oled.c中添加函数实现


/**
  * @brief  DHT11发送响应信号
  * @param  无
  * @retval 返回值0/1  0：响应成功 1：响应失败
  */
uint8_t DHT11_Check(void)
{
	uint8_t retry = 0 ;
	DHT11_IN();
	//采用while循环的方式检测响应信号
	while(DHT11_IO_IN && retry <100) // DHT11会拉低 40us ~80us
	{
		retry++;
		Delay_us(1);//1us
	}
	if(retry>=100) //判断当DHT11延迟超过80us时return 1 ， 说明响应失败
	{return  1;}
	else retry =  0 ;
		
	while(!DHT11_IO_IN && retry<100)// // DHT11拉低之后会拉高 40us ~80us
	{
		retry++;
		Delay_us(1);//1us
	}
		
	if(retry>=100)
	{return 1;}
	return 0 ;
}

/**
  * @brief  DHT11读取一位数据
  * @param  无
  * @retval 返回值0/1  1：读取成功 0：读取失败
  */
uint8_t DHT11_Read_Bit(void)
{
	uint8_t retry = 0 ;
	while(DHT11_IO_IN && retry <100)//同上采用while循环的方式去采集数据
	{
		retry++;
		Delay_us(1);
	}
	retry = 0 ;
	while(!DHT11_IO_IN && retry<100)
	{
		retry++;
		Delay_us(1);
	}
 
	Delay_us(40);              //结束信号，延时40us 
	if(DHT11_IO_IN) return 1;  //结束信号后，总线会被拉高 则返回1表示读取成功
	else 
	return 0 ;
}


/**
  * @brief  DHT11读取一个字节数据
  * @param  无
  * @retval 返回值：dat 将采集到的一个字节的数据返回
  */
uint8_t DHT11_ReadByte(void)
{
	uint8_t i , dat ;
	dat = 0 ;
	for(i=0; i<8; i++)
	{
		dat <<= 1; //通过左移存储数据
		dat |= DHT11_Read_Bit();
	}
	return dat ; 
}


/**
  * @brief  DHT11读取数据
  * @param  temp：温度值 humi ：湿度值
  * @retval 返回值0/1 0：读取数据成功 1：读取数据失败
  */
uint8_t DHT11_Read_Data(uint8_t* temp , uint8_t* humi)
{
	uint8_t buf[5];        //储存五位数据
    uint8_t i;    
	DHT11_Strat();         //起始信号
	if(DHT11_Check() == 0) //响应信号
    {
		for(i=0; i<5; i++)
		{
			buf[i] = DHT11_ReadByte();
		}
		if(buf[0]+buf[1]+buf[2]+buf[3] == buf[4]) //校验数据
		{
		    *humi = buf[0]; // 湿度
			*temp = buf[2]; // 温度
		}
	}else return 1;
	
   return 0 ;
}
