/*
 * tm1637.h
 *
 *  Created on: 29 сент. 2019 г.
 *      Author: dima
 */

#ifndef TM1637_H_
#define TM1637_H_

#include "main.h"

#define CLK_HIGH (HAL_GPIO_WritePin(X_SCL_GPIO_Port, X_SCL_Pin, GPIO_PIN_SET))
#define CLK_LOW (HAL_GPIO_WritePin(X_SCL_GPIO_Port, X_SCL_Pin, GPIO_PIN_RESET))

#define DIO_HIGH (HAL_GPIO_WritePin(X_SDA_GPIO_Port, X_SDA_Pin, GPIO_PIN_SET))
#define DIO_LOW (HAL_GPIO_WritePin(X_SDA_GPIO_Port, X_SDA_Pin, GPIO_PIN_RESET))

#define DIO_READ (HAL_GPIO_ReadPin(X_SDA_GPIO_Port, X_SDA_Pin))

#define ADDR_AUTO 0x40
#define ADDR_FIXED 0x44

#define STARTADDR 0xc0

void tm1637_writeByte(int8_t wr_data); // write 8bit data to tm1637
void start(void);               // send start bits
void stop(void);                // send stop bits
void display_mass(int8_t DispData[]);
void display(uint8_t BitAddr, int8_t DispData);
void clearDisplay(void);
// To take effect the next time it displays.
void set_brightness(uint8_t brightness);
// Whether to light the clock point ":". Eeffect the next time.
void point(uint8_t cmd); 
void coding_mass(int8_t DispData[]);
int8_t coding(int8_t DispData);

void displayShow(int16_t number);
#endif /* TM1637_H_ */
