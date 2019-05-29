/**
 * @file HardwareSerial.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-12
* @version 2019-05-01
 */
#ifndef __ADDE_HARDWARESERIAL_H__
#define __ADDE_HARDWARESERIAL_H__

#include <stddef.h>
#include <stdint.h>
#include "utility/adde.hpp"
#include "Stream.h"


#define SERIAL_5N1 _SERIAL_5N1
#define SERIAL_6N1 _SERIAL_6N1
#define SERIAL_7N1 _SERIAL_7N1
#define SERIAL_8N1 _SERIAL_8N1
#define SERIAL_5N2 _SERIAL_5N2
#define SERIAL_6N2 _SERIAL_6N2
#define SERIAL_7N2 _SERIAL_7N2
#define SERIAL_8N2 _SERIAL_8N2
#define SERIAL_5E1 _SERIAL_5E1
#define SERIAL_6E1 _SERIAL_6E1
#define SERIAL_7E1 _SERIAL_7E1
#define SERIAL_8E1 _SERIAL_8E1
#define SERIAL_5E2 _SERIAL_5E2
#define SERIAL_6E2 _SERIAL_6E2
#define SERIAL_7E2 _SERIAL_7E2
#define SERIAL_8E2 _SERIAL_8E2
#define SERIAL_5O1 _SERIAL_5O1
#define SERIAL_6O1 _SERIAL_6O1
#define SERIAL_7O1 _SERIAL_7O1
#define SERIAL_8O1 _SERIAL_8O1
#define SERIAL_5O2 _SERIAL_5O2
#define SERIAL_6O2 _SERIAL_6O2
#define SERIAL_7O2 _SERIAL_7O2
#define SERIAL_8O2 _SERIAL_8O2


extern uint8_t _SERIAL_5N1;
extern uint8_t _SERIAL_6N1;
extern uint8_t _SERIAL_7N1;
extern uint8_t _SERIAL_8N1;
extern uint8_t _SERIAL_5N2;
extern uint8_t _SERIAL_6N2;
extern uint8_t _SERIAL_7N2;
extern uint8_t _SERIAL_8N2;
extern uint8_t _SERIAL_5E1;
extern uint8_t _SERIAL_6E1;
extern uint8_t _SERIAL_7E1;
extern uint8_t _SERIAL_8E1;
extern uint8_t _SERIAL_5E2;
extern uint8_t _SERIAL_6E2;
extern uint8_t _SERIAL_7E2;
extern uint8_t _SERIAL_8E2;
extern uint8_t _SERIAL_5O1;
extern uint8_t _SERIAL_6O1;
extern uint8_t _SERIAL_7O1;
extern uint8_t _SERIAL_8O1;
extern uint8_t _SERIAL_5O2;
extern uint8_t _SERIAL_6O2;
extern uint8_t _SERIAL_7O2;
extern uint8_t _SERIAL_8O2;


class HardwareSerial : public Stream {
private:
	const uint8_t inst;
public:
	HardwareSerial(const uint8_t instance);
	~HardwareSerial();
	
	void begin(unsigned long speed);
	void begin(unsigned long speed, uint8_t mode);
	void end();

	virtual int available();
	virtual int availableForWrite();
	virtual int peek();
	virtual int read();
	virtual void flush();
	virtual size_t write(uint8_t val);
	
	operator bool() { return true; }

	using Print::write;
};


extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;
extern HardwareSerial Serial5;
extern HardwareSerial Serial6;
extern HardwareSerial Serial7;
extern HardwareSerial Serial8;


#endif /* __ADDE_HARDWARESERIAL_H__ */
