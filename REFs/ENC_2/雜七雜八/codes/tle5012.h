/*
 * tle5012.h
 *
 *  Created on: May 26, 2021
 *      Author: Administrator
 */
#include "main.h"
#ifndef APPLICATION_USER_TLE5012_H_
#define APPLICATION_USER_TLE5012_H_

#define SPI_CS_ENABLE  HAL_GPIO_WritePin( GPIOB, GPIO_PIN_6, GPIO_PIN_RESET )
#define SPI_CS_DISABLE HAL_GPIO_WritePin( GPIOB, GPIO_PIN_6, GPIO_PIN_SET )

#define TLE5012_MODE1	  //输出模式1 MOSI MISO经过4.7K-10K电阻相连
//#define TLE5012_MODE2   //输出模式2 MOSI MISO直连， 每次读取 需要将MOSI改为输入模式

extern SPI_HandleTypeDef hspi1;


extern UART_HandleTypeDef huart2;

#ifdef TLE5012_MODE2
#define SPI_TX_OFF  {GPIOB->MODER&=~(3<<(15*2));GPIOB->MODER|=0<<15*2;}	//PB5 配置成输入模式  00(0):输入 01(1):通用输出 10(2):复用功能 11(3):模拟
#define SPI_TX_ON {GPIOB->MODER&=~(3<<(15*2));GPIOB->MODER|=2<<15*2;} //PB5配置为复用输出模式
#endif

#define TLE5012_IIF_MODE_4096 4096
#define TLE5012_IIF_MODE_512  512

#define TLE5012_HALL_MODE_POLEPAIR_1 1
#define TLE5012_HALL_MODE_POLEPAIR_2 2
#define TLE5012_HALL_MODE_POLEPAIR_3 3
#define TLE5012_HALL_MODE_POLEPAIR_4 4
#define TLE5012_HALL_MODE_POLEPAIR_5 5
#define TLE5012_HALL_MODE_POLEPAIR_6 6
#define TLE5012_HALL_MODE_POLEPAIR_7 7
#define TLE5012_HALL_MODE_POLEPAIR_8 8
#define TLE5012_HALL_MODE_POLEPAIR_9 9
#define TLE5012_HALL_MODE_POLEPAIR_10 10
#define TLE5012_HALL_MODE_POLEPAIR_11 11
#define TLE5012_HALL_MODE_POLEPAIR_12 12
#define TLE5012_HALL_MODE_POLEPAIR_13 13
#define TLE5012_HALL_MODE_POLEPAIR_14 14
#define TLE5012_HALL_MODE_POLEPAIR_15 15
#define TLE5012_HALL_MODE_POLEPAIR_16 16



/* SPI command for TLE5012 */
#define READ_STATUS				0x8001			//8000
#define READ_ANGLE_VALUE		0x8021			//8020
#define READ_SPEED_VALUE		0x8031			//8030


#define WRITE_MOD1_VALUE	0x5061							//0_1010_0_000110_0001
#define MOD1_VALUE	0x0001
#define MOD1_E1000_DEFAULT	0x4001
#define MOD1_E3005_DEFAULT	0x4000


#define WRITE_MOD2_VALUE	0x5081							//0_1010_0_001000_0001
#define MOD2_E1000_DIR_0	0x1001   // TLE5012BE1000(IFF)磁体逆时针转动
#define MOD2_E1000_DIR_1	0x1009	 // TLE5012BE1000(IFF)磁体顺逆时针转动

#define MOD2_E3005_DIR_0	0x1005   // TLE5012BE3005(HSM)磁体逆时针转动
#define MOD2_E3005_DIR_1	0x100D	 // TLE5012BE3005(HSM)磁体顺逆时针转动

#define WRITE_MOD3_VALUE	0x5091							//0_1010_0_001001_0001
#define MOD3_VALUE	0x0000


#define WRITE_MOD4_VALUE	0x50E1							//0_1010_0_001110_0001
#define MOD4_VALUE			0x0098				//9bit 512
#define MOD4_HSM_1_PAIR		0x0002
#define MOD4_HSM_2_PAIR		0x0022
#define MOD4_HSM_3_PAIR		0x0042
#define MOD4_HSM_4_PAIR		0x0062
#define MOD4_HSM_5_PAIR		0x0082
#define MOD4_HSM_6_PAIR		0x00A2
#define MOD4_HSM_7_PAIR		0x00C2
#define MOD4_HSM_8_PAIR		0x00E2
#define MOD4_HSM_9_PAIR		0x0102
#define MOD4_HSM_10_PAIR	0x0122
#define MOD4_HSM_11_PAIR	0x0142
#define MOD4_HSM_12_PAIR	0x0162
#define MOD4_HSM_13_PAIR	0x0182
#define MOD4_HSM_14_PAIR	0x01A2
#define MOD4_HSM_15_PAIR	0x01C2
#define MOD4_HSM_16_PAIR	0x01E2

#define MOD4_IIF_512 		0x0018				//IIF模式       512 步进
#define MOD4_IIF_4096 		0x0000				//IIF模式       4096 步进


#define WRITE_IFAB_VALUE		0x50B1
#define IFAB_VALUE 0x000D
/* Functionality mode */
#define REFERESH_ANGLE		0

uint16_t Write_Command(uint16_t dev_reg,uint16_t Command);
uint16_t ReadValue(uint16_t u16RegValue);
int ReadSpeed(void);
double ReadAngle(void);
void TLE5012_E1000_DIR (uint8_t dir);
void TLE5012_E3005_DIR (uint8_t dir);
void TLE5012_IIF (uint16_t step);
void TLE5012_HSM (uint16_t pole_pair);
#endif /* APPLICATION_USER_TLE5012_H_ */
