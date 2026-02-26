
#define DHT11_H

#include "main.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include	"stdio.h"
#include  "string.h"
 
/* USER CODE END Includes */
//PA11
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//
 
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
 
void Delay_us(uint32_t delay);               //us����ʱ����
void OLED_WriteCmd(uint8_t cmd);
// 在oled.h中添加函数声明
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *bmp);
 
//DHT11_DATA ,IO������Ϊ���ģʽ�������IO��ָ����STM32CubeMX�����õ�IO�ڣ�
void DHT11_OUT(void);	
												
void DHT11_IN(void);                          //DHT11_Data IO����Ϊ����ģʽ
void DHT11_Strat(void);				          //����������ʼ�ź�
uint8_t DHT11_Check(void);                    //DHT11������Ӧ�ź�
uint8_t DHT11_Read_Bit(void);                 //��ȡDHT11һ��BIT������
uint8_t DHT11_Read_Byte(void);                //��ȡDHT11һ��Byte������
uint8_t DHT11_Read_Data(uint8_t* temp , uint8_t* humi);  //��ȡDHT11ʪ�Ⱥ��¶ȵ�����
 
/* USER CODE BEGIN PFP */


