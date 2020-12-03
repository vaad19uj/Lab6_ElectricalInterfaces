/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lcd.h
  * @brief          : Header for lcd.c file.
  *                   This file contains the LCD driver (LCD1602).
  ******************************************************************************
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"

typedef struct {
	GPIO_TypeDef *dataPort;
	GPIO_TypeDef *controlPort;
	uint16_t rsPin;
	uint16_t rwPin;
	uint16_t enPin;

	/*uint16_t D0Pin;
	uint16_t D1Pin;
	uint16_t D2Pin;
	uint16_t D3Pin;
	uint16_t D4Pin;
	uint16_t D5Pin;
	uint16_t D6Pin;
	uint16_t D7Pin;	*/

	// state init?

} TextLCDType;

void TextLCD_Init(TextLCDType *lcd, GPIO_TypeDef *controlPort, uint16_t rsPin, uint16_t rwPin, uint16_t enPin, GPIO_TypeDef *dataPort);

void TextLCD_Home(TextLCDType *lcd);

void TextLCD_Clear(TextLCDType *lcd);

void TextLCD_Position(TextLCDType *lcd, int x, int y);

void TextLCD_Putchar(TextLCDType *lcd, uint8_t data);

void TextLCD_Puts(TextLCDType *lcd, char *string);

void TextLCD_Printf(TextLCDType *lcd, char *message);



#ifdef __cplusplus
}
#endif

#endif /* __LCD_H */

