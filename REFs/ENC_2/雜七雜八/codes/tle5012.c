/*
 * tle5012.c
 *
 *  Created on: May 26, 2021
 *      Author: Administrator
 */



#include "tle5012.h"

/*
 * 	写命令
 * 	参数1 寄存器地址
 * 	参数2 写入命令
 */

uint16_t Write_Command(uint16_t dev_reg,uint16_t Command)
{
	uint16_t Safe_Word = 0;
	SPI_CS_ENABLE;
	HAL_SPI_Transmit( &hspi1, (uint8_t *)(&dev_reg), sizeof(dev_reg)/sizeof(uint16_t), 0xff );
	HAL_SPI_Transmit( &hspi1, (uint8_t *)(&Command), sizeof(Command)/sizeof(uint16_t), 0xff );

	HAL_SPI_Receive( &hspi1,(uint8_t *)(&Safe_Word), sizeof(Safe_Word)/sizeof(uint16_t), 0xff );
	return Safe_Word;
}

/*
 * 	读数据
 */

uint16_t ReadValue(uint16_t u16RegValue)
{
	uint16_t u16Data;

	SPI_CS_ENABLE;
	HAL_SPI_Transmit( &hspi1, (uint8_t *)(&u16RegValue), sizeof(u16RegValue)/sizeof(uint16_t), 0xff );

#ifdef TLE5012_MODE2
	SPI_TX_OFF;
#endif

	HAL_SPI_Receive( &hspi1,(uint8_t *)(&u16Data), sizeof(u16Data)/sizeof(uint16_t), 0xff );
	SPI_CS_DISABLE;

#ifdef TLE5012_MODE2
	SPI_TX_ON;
#endif

	return((u16Data & 0x7FFF )<<1);
}

/*
 * 	读取转速
 */
int ReadSpeed(void)
{
	int speed;
	uint16_t DELETE_7BITS        =        0x01FF;
	uint16_t CHANGE_UNIT_TO_INT_9    =    512;
	uint16_t CHECK_BIT_9            =     0x0100;


	speed = ReadValue(READ_SPEED_VALUE);
	speed = (speed & (DELETE_7BITS));
    //check if the value received is positive or negative
    if (speed & CHECK_BIT_9)
    {
    	speed = speed - CHANGE_UNIT_TO_INT_9;
    }

	return speed;
}

/*
 * 	读取角度
 */
double ReadAngle(void)
{
	return ( ReadValue(READ_ANGLE_VALUE) * 360.0 / 0x10000 );  //0x10000为2的15次方，也就是芯片的分辨率 公式 角度= （360度 / 2pow15(分辨率) * angle_value ）
}



/*
 * 	设置TLE5012B为霍尔模式(HSM)
 *	 参数 1: 级对子数
 */
void TLE5012_HSM (uint16_t pole_pair)
{
	uint16_t safety_word = 0;
	printf("HSM MODE\r\n");
	switch(pole_pair)
	{
	case TLE5012_HALL_MODE_POLEPAIR_1:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_1_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_2:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_2_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_3:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_3_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_4:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_4_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_5:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_5_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_6:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_6_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_7:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_7_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_8:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_8_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_9:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_9_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_10:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_10_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_11:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_11_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_12:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_12_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_13:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_13_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_14:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_14_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_15:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_15_PAIR);
		 break;
	case TLE5012_HALL_MODE_POLEPAIR_16:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_HSM_16_PAIR);
		 break;
	default:
		break;
	}
	  printf("HSM mod4 safty word :0x%x\r\n",safety_word);
	  ReadAngle();
	  ReadAngle();

	  safety_word = Write_Command(WRITE_MOD1_VALUE,MOD1_E3005_DEFAULT);
	  printf("HSM mod1 safty word :0x%x\r\n",safety_word);
	  ReadAngle();
	  ReadAngle();
}

/*	设置TLE5012B为增量编码器模式(IIF)
 *	 参数1 : 步进数
 */
void TLE5012_IIF (uint16_t step)
{
	uint16_t safety_word = 0;
	printf("IIF MODE\r\n");
	switch(step)
	{
	case TLE5012_IIF_MODE_512:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_IIF_512);
		 break;
	case TLE5012_IIF_MODE_4096:
		 safety_word = Write_Command(WRITE_MOD4_VALUE,MOD4_IIF_4096);
		 break;
	default:
		break;
	}
	  printf("IIF mod4 safty word :0x%x\r\n",safety_word);
	  ReadAngle();
	  ReadAngle();

	  safety_word = Write_Command(WRITE_MOD1_VALUE,MOD1_E1000_DEFAULT);
	  printf("IIF mod1 safty word :0x%x\r\n",safety_word);
	  ReadAngle();
	  ReadAngle();
}

/*	更改TLE5012BE3005的磁体方向
 *	 参数1 : 0逆向 1正向
 */
void TLE5012_E3005_DIR (uint8_t dir)
{
	uint16_t safety_word = 0;
	if(dir ==0)
		{
		 safety_word = Write_Command(WRITE_MOD2_VALUE,MOD2_E3005_DIR_0);
		}
	else
		{
		 safety_word = Write_Command(WRITE_MOD2_VALUE,MOD2_E3005_DIR_1);
		}
	  printf("Change DIR safty word :0x%x\r\n",safety_word);
	  ReadAngle();
	  ReadAngle();
}


/*	更改TLE5012BE3005的磁体方向
 *	 参数1 : 0逆向 1正向
 */
void TLE5012_E1000_DIR (uint8_t dir)
{
	uint16_t safety_word = 0;
	printf("HMS MODE\r\n");
	if(dir ==0)
		{
		 safety_word = Write_Command(WRITE_MOD2_VALUE,MOD2_E1000_DIR_0);
		}
	else
		{
		 safety_word = Write_Command(WRITE_MOD2_VALUE,MOD2_E1000_DIR_1);
		}
	  printf("Change DIR safty word :0x%x\r\n",safety_word);
	  ReadAngle();
	  ReadAngle();

}

