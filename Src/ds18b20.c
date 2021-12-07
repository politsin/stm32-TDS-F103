/*
 * ds18b20.c
 *
 *  Created on: 5 мая 2019 г.
 *      Author: istarik.ru
 */


#include "ds18b20.h"
#include "delay_micros.h"

#include "string.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim3;

uint32_t delay_wait_convert = DELAY_T_CONVERT;

char trans_ds18[64] = {0,};
extern volatile uint8_t flag_ds18b20;


uint8_t getDevider(DS18B20_Resolution resolution)
{
	uint8_t devider;

	switch(resolution)
	{
		case DS18B20_Resolution_9_bit:
			devider = 8;
			break;

		case DS18B20_Resolution_10_bit:
			devider = 4;
			break;

		case DS18B20_Resolution_11_bit:
			devider = 2;
			break;

		case DS18B20_Resolution_12_bit:
		default:
			devider = 1;
			break;
	}

	return devider;
}

void writeBit(uint8_t bit)
{
	DB18B20_PORT->BSRR = (uint32_t)DB18B20_PIN << 16u;
	delay_us(bit ? DELAY_WRITE_1 : DELAY_WRITE_0);
	DB18B20_PORT->BSRR = DB18B20_PIN;
	delay_us(bit ? DELAY_WRITE_1_PAUSE : DELAY_WRITE_0_PAUSE);
}

void writeByte(uint8_t data)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		writeBit(data >> i & 1);
		delay_us(DELAT_PROTECTION);
	}
}

uint8_t readBit()
{
	uint8_t bit = 0;
	DB18B20_PORT->BSRR = (uint32_t)DB18B20_PIN << 16u;
	delay_us(DELAY_READ_SLOT);
	DB18B20_PORT->BSRR = DB18B20_PIN;
	delay_us(DELAY_BUS_RELAX);
	bit = (DB18B20_PORT->IDR & DB18B20_PIN) ? 1 : 0;
	delay_us(DELAY_READ_PAUSE);
	return bit;
}


void setResolution(DS18B20_Resolution resolution)
{
	DB18B20_PORT->BSRR = (uint32_t)DB18B20_PIN << 16u;
	delay_us(DELAY_RESET);
	DB18B20_PORT->BSRR = DB18B20_PIN;
	delay_us(DELAY_RESET);

	writeByte(SKIP_ROM);
	writeByte(WRITE_SCRATCHPAD);
	writeByte(TH_REGISTER);
	writeByte(TL_REGISTER);
	writeByte(resolution);
	delay_wait_convert = DELAY_T_CONVERT / getDevider(resolution);
}





