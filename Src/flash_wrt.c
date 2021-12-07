/*
 * flash_wrt.c
 *
 *  Created on: 13 авг. 2020 г.
 *      Author: dima
 */
#include "main.h"
#include "flash_wrt.h"

#define COUNT     13

uint8_t my_erse_flash(uint32_t adres, uint8_t numb)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = adres;
	EraseInitStruct.NbPages = numb;
	uint32_t page_error = 0;

	uint8_t res = 0;

	HAL_FLASH_Unlock();

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK)
	{
		//trans_to_usart1("NOT ERASE");
		res = 0;
	}
	else
	{
		//trans_to_usart1("ERASE OK");
		res = 1;
	}

	HAL_FLASH_Lock();

	return res;
}


uint8_t Write_flash_data(uint32_t PAGE, uint32_t *data)
{
	uint32_t address = PAGE;
	uint8_t res = 0;

	////////////////// ОЧИСТКА //////////////////
	res = my_erse_flash(address, 1);
	if(res == 0) return 0;

	//////////////////////// ЗАПИСЬ ////////////////////////////
	HAL_FLASH_Unlock();

	for(uint8_t i = 0; i < COUNT; i++)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return 0;
		}

		address = (address + 4);
	}

	HAL_FLASH_Lock();
	return 1;
}

void Read_flash_data(uint32_t PAGE, uint32_t *data)
{
	uint32_t address = PAGE;

	for(uint8_t i = 0; i < COUNT; i++)
	{
		data[i] = *(uint32_t*)address;
		address = (address + 4);
	}
}







