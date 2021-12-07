/*
 * tm1637.c
 *
 *  Created on: 29 сент. 2019 г.
 *      Author: dima
 */

#include "delay_micros.h"
#include "tm1637.h"

uint8_t Cmd_DispCtrl = 0;
uint8_t point_flag = 0;

static int8_t TubeTab[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f, 0x77,0x7c,0x39,0x5e,0x79,0x71,0x3E,0x38,0x40,0x46,0x70};
//0~9, A(10), b(11), C(12), d(13), E(14), F(15), U(16), L(17), -(18), -|(19), |-(20)


void writeByte_tm1637(int8_t wr_data)
{
  uint8_t count = 0;

  for(uint8_t i = 0; i < 8; i++)
  {
    CLK_LOW;

    if(wr_data & 0x01) DIO_HIGH;
    else DIO_LOW;

    delay_us(6);
    wr_data >>= 1;
    delay_us(6);
    CLK_HIGH;
    delay_us(8);
  }

  CLK_LOW;
  delay_us(6);
  DIO_HIGH;
  delay_us(6);
  CLK_HIGH;
  delay_us(8);

  while(DIO_READ)
  {
    count += 1;

    if(count == 200)
    {
    	DIO_LOW;
    	count = 0;
    }
  }
}

void start(void)
{
	CLK_HIGH;
	delay_us(6);
	DIO_HIGH;
	delay_us(6);
	DIO_LOW;
	delay_us(6);
	CLK_LOW;
}

void stop(void)
{
	CLK_LOW;
	delay_us(6);
	DIO_LOW;
	delay_us(6);
	CLK_HIGH;
	delay_us(6);
	DIO_HIGH;
}

void display_mass(int8_t DispData[])
{
	int8_t SegData[4];

	for(uint8_t i = 0; i < 4; i++)
	{
		SegData[i] = DispData[i];
	}

	coding_mass(SegData);
	start();
	writeByte_tm1637(ADDR_AUTO);
	stop();
	start();
	writeByte_tm1637(0xc0);

	for(uint8_t i = 0; i < 4; i++)
	{
		writeByte_tm1637(SegData[i]);
	}

	stop();
	start();
	writeByte_tm1637(Cmd_DispCtrl);
	stop();
}

void display(uint8_t BitAddr, int8_t DispData)
{
	int8_t SegData;

	SegData = coding(DispData);
	start();
	writeByte_tm1637(ADDR_FIXED);

	stop();
	start();
	writeByte_tm1637(BitAddr | 0xc0);

	writeByte_tm1637(SegData);
	stop();
	start();

	writeByte_tm1637(Cmd_DispCtrl);
	stop();
}

void clearDisplay(void)
{
	display(0x00, 0x7f);
	display(0x01, 0x7f);
	display(0x02, 0x7f);
	display(0x03, 0x7f);
}

void set_brightness(uint8_t brightness)
{
	Cmd_DispCtrl = 0x88 + brightness;
}

void point(uint8_t cmd)
{
	if(cmd == 0) point_flag = (~point_flag) & 0x01;
	else point_flag = 1;
}

void coding_mass(int8_t DispData[])
{
	uint8_t PointData;

	if(point_flag == 1) PointData = 0x80;
	else PointData = 0;

	for(uint8_t i = 0; i < 4; i++)
	{
		if(DispData[i] == 0x7f) DispData[i] = 0x00;
		else DispData[i] = TubeTab[DispData[i]] + PointData;
	}
}

int8_t coding(int8_t DispData)
{
	uint8_t PointData;

	if(point_flag == 1) PointData = 0x80;
	else PointData = 0;

	if(DispData == 0x7f) DispData = 0x00 + PointData;
	else DispData = TubeTab[DispData] + PointData;

	return DispData;
}
