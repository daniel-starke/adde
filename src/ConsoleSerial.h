/**
 * @file ConsoleSerial.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-16
* @version 2019-04-22
 */
#ifndef __ADDE_CONSOLESERIAL_H__
#define __ADDE_CONSOLESERIAL_H__

#include <stddef.h>
#include <stdint.h>
#include "Stream.h"


class ConsoleSerial : public Stream {
public:
	enum {
		ONE_STOP_BIT = 0,
		ONE_AND_HALF_STOP_BIT = 1,
		TWO_STOP_BITS = 2
	};
	enum {
		NO_PARITY = 0,
		ODD_PARITY = 1,
		EVEN_PARITY = 2,
		MARK_PARITY = 3,
		SPACE_PARITY = 4
	};
public:
	ConsoleSerial();
	~ConsoleSerial();
	
	void begin(unsigned long speed);
	void begin(unsigned long speed, uint8_t mode);
	void end();

	virtual int available();
	virtual int availableForWrite();
	virtual int peek();
	virtual int read();
	virtual void flush();
	virtual size_t write(uint8_t val);
	
	operator bool();

	using Print::write;
	
	int32_t readBreak() { return -1; }
	uint32_t baud() { return 115200; }
	uint8_t stopbits() { return ONE_STOP_BIT; }
	uint8_t paritytype() { return NO_PARITY; }
	uint8_t numbits() { return 8; }
	bool dtr() { return true; }
	bool rts() { return true; }
};


typedef ConsoleSerial Serial_;
extern ConsoleSerial Serial;


#endif /* __ADDE_CONSOLESERIAL_H__ */
