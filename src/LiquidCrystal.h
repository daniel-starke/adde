/**
 * @file LiquidCrystal.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-15
* @version 2019-05-01
 */
#ifndef __ADDE_LIQUIDCRYSTAL_H__
#define __ADDE_LIQUIDCRYSTAL_H__

#include <stddef.h>
#include <stdint.h>
#include "utility/adde.hpp"
#include "Print.h"


/* commands */
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

/* flags for display entry mode */
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00
#define LCD_ENTRYSHIFTINCREMENT 0x01

/* flags for display on/off control */
#define LCD_DISPLAYOFF 0x00
#define LCD_DISPLAYON 0x04
#define LCD_CURSOROFF 0x00
#define LCD_CURSORON 0x02
#define LCD_BLINKOFF 0x00
#define LCD_BLINKON 0x01

/* flags for display/cursor shift */
#define LCD_CURSORMOVE 0x00
#define LCD_DISPLAYMOVE 0x08
#define LCD_MOVELEFT 0x00
#define LCD_MOVERIGHT 0x04

/* flags for function set */
#define LCD_5x8DOTS 0x00
#define LCD_5x10DOTS 0x04
#define LCD_1LINE 0x00
#define LCD_2LINE 0x08
#define LCD_4BITMODE 0x00
#define LCD_8BITMODE 0x10


class LiquidCrystal : public Print {
private:
	uint8_t inst;
public:
	LiquidCrystal(uint8_t rs, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
	LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
	LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
	LiquidCrystal(uint8_t rs, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
	~LiquidCrystal();

	void begin(uint8_t cols, uint8_t rows, uint8_t charSize = LCD_5x8DOTS);

	void clear();
	void home();

	void noDisplay();
	void display();
	void noBlink();
	void blink();
	void noCursor();
	void cursor();
	void scrollDisplayLeft();
	void scrollDisplayRight();
	void leftToRight();
	void rightToLeft();
	void noAutoscroll();
	void autoscroll();

	void setRowOffsets(int row1, int row2, int row3, int row4);
	void createChar(uint8_t location, uint8_t * charMap);
	void setCursor(uint8_t col, uint8_t row);
	void command(uint8_t val);
	
	virtual size_t write(uint8_t data);

	using Print::write;
};


#endif /* __ADDE_LIQUIDCRYSTAL_H__ */
