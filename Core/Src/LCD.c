/*
 * lcd.c
 *
 *  Created on: 13 apr. 2020
 *      Author: Adina
 */

#include "lcd.h"

//void delay_Microsecs(uint16_t);

static void TextLCD_Strobe(TextLCDType *lcd){
	// enable set reset with delay
	HAL_GPIO_WritePin(lcd->controlPort, lcd->enPin, GPIO_PIN_SET);
	HAL_Delay(5);
//	delay_Microsecs(50);
	HAL_GPIO_WritePin(lcd->controlPort, lcd->enPin, GPIO_PIN_RESET);
//	delay_Microsecs(110);
	HAL_Delay(5);

}

static void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd){
	// reset rs to select instructions register
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rsPin, GPIO_PIN_RESET);
	lcd->dataPort->ODR = (lcd->dataPort->ODR & 0xFFFFFF00) | cmd;
}

static void TextLCD_Data(TextLCDType *lcd, uint8_t data){
	// set rs to select data register
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rsPin, GPIO_PIN_SET);
	lcd->dataPort->ODR = (lcd->dataPort->ODR & 0xFFFFFF00) | data;
}


void TextLCD_Init(TextLCDType *lcd, GPIO_TypeDef *controlPort, uint16_t rsPin, uint16_t rwPin, uint16_t enPin, GPIO_TypeDef *dataPort) {
	// initialize struct variables
	lcd->rsPin = rsPin;
	lcd->enPin = enPin;
	lcd->rwPin = rwPin;
	lcd->controlPort = controlPort; 	// port controlpins, rw, rs, en
	lcd->dataPort = dataPort;		// port D pins

	// reset rw and rs
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rwPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->controlPort, lcd->rsPin, GPIO_PIN_RESET);

	// 8 bit init
	uint8_t cmd = 0x38;
	TextLCD_Cmd(lcd, cmd);
	TextLCD_Strobe(lcd);
	HAL_Delay(4);

	TextLCD_Cmd(lcd, cmd);
	TextLCD_Strobe(lcd);

	TextLCD_Cmd(lcd, cmd);
	TextLCD_Strobe(lcd);

	// clear Display
	cmd = 0x01;
	TextLCD_Cmd(lcd, cmd);
	TextLCD_Strobe(lcd);

	// Entry mode
	cmd = 0x06;
	TextLCD_Cmd(lcd, cmd);
	TextLCD_Strobe(lcd);

	// Display on
	cmd = 0x0c;
	TextLCD_Cmd(lcd, cmd);
	TextLCD_Strobe(lcd);
}


void TextLCD_Home(TextLCDType *lcd){
	uint8_t cmd = 0x02;
	TextLCD_Cmd(lcd, cmd);		// cursor home
	TextLCD_Strobe(lcd);
//	delay_Microsecs(2000);
	HAL_Delay(2);
}

void TextLCD_Clear(TextLCDType *lcd){
	uint8_t cmd = 0x01;
	TextLCD_Cmd(lcd, cmd);		// clear display
	TextLCD_Strobe(lcd);
//	delay_Microsecs(2000);
	HAL_Delay(2);
}

void TextLCD_Position(TextLCDType *lcd, int x, int y){
	//move cursor
	if(x >= 0 && x <= 15 && y >= 0 && y <= 1){		// to stay inside the screen
		uint8_t cmd = 0x80+x+0x40+y;
		TextLCD_Cmd(lcd, cmd);
		TextLCD_Strobe(lcd);
	}
}

void TextLCD_Putchar(TextLCDType *lcd, uint8_t data){
	TextLCD_Data(lcd, data);
	TextLCD_Strobe(lcd);
}

void TextLCD_Puts(TextLCDType *lcd, char *string){
	int strLength = strlen(string);

	// loop string
	for(int i = 0; i < strLength; i+=1){
		TextLCD_Putchar(lcd, string[i]);
	}
}

void TextLCD_Printf(TextLCDType *lcd, char *message){

	// loop string
	// can handle %d
}
