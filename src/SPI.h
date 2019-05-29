/**
 * @file SPI.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-13
* @version 2019-05-01
 */
#ifndef __ADDE_SPI_H__
#define __ADDE_SPI_H__

#include <stddef.h>
#include <stdint.h>
#include "utility/adde.hpp"


#define SPI_HAS_TRANSACTION 1
#define SPI_HAS_NOTUSINGINTERRUPT 1


#define SPI_CLOCK_DIV2 _SPI_CLOCK_DIV2
#define SPI_CLOCK_DIV4 _SPI_CLOCK_DIV4
#define SPI_CLOCK_DIV8 _SPI_CLOCK_DIV8
#define SPI_CLOCK_DIV16 _SPI_CLOCK_DIV16
#define SPI_CLOCK_DIV32 _SPI_CLOCK_DIV32
#define SPI_CLOCK_DIV64 _SPI_CLOCK_DIV64
#define SPI_CLOCK_DIV128 _SPI_CLOCK_DIV128
#define SPI_CLOCK_DIV256 _SPI_CLOCK_DIV256
#define SPI_CLOCK_DIV512 _SPI_CLOCK_DIV512
#define SPI_CLOCK_DIV1024 _SPI_CLOCK_DIV1024
#define SPI_MODE0 _SPI_MODE0
#define SPI_MODE1 _SPI_MODE1
#define SPI_MODE2 _SPI_MODE2
#define SPI_MODE3 _SPI_MODE3


extern uint8_t _SPI_CLOCK_DIV2;
extern uint8_t _SPI_CLOCK_DIV4;
extern uint8_t _SPI_CLOCK_DIV8;
extern uint8_t _SPI_CLOCK_DIV16;
extern uint8_t _SPI_CLOCK_DIV32;
extern uint8_t _SPI_CLOCK_DIV64;
extern uint8_t _SPI_CLOCK_DIV128;
extern uint8_t _SPI_CLOCK_DIV256;
extern uint8_t _SPI_CLOCK_DIV512;
extern uint8_t _SPI_CLOCK_DIV1024;
extern uint8_t _SPI_MODE0;
extern uint8_t _SPI_MODE1;
extern uint8_t _SPI_MODE2;
extern uint8_t _SPI_MODE3;


class SPISettings {
private:
	friend class SPIClass;
	uint32_t clock;
	uint8_t bitOrder;
	uint8_t dataMode;
public:
  SPISettings():
	clock(UINT32_C(4000000)),
	bitOrder(MSBFIRST),
	dataMode(SPI_MODE0)
  {}
  
  SPISettings(uint32_t aClock, uint8_t aBitOrder, uint8_t aDataMode):
	clock(aClock),
	bitOrder(aBitOrder),
	dataMode(aDataMode)
  {}
};


class SPIClass {
private:
	uint8_t inst;
public:
	SPIClass(const uint8_t instance);
	~SPIClass();
	
	void begin();
	void usingInterrupt(uint8_t interrupt);
	void notUsingInterrupt(uint8_t interrupt);
	void beginTransaction(SPISettings settings);
	ADDE_F(_uint8_t) transfer(uint8_t data);
	ADDE_F(_uint16_t) transfer16(uint16_t data);
	void transfer(void * buf, size_t size);
	void endTransaction();
	void end();
	
	/* deprecated functions */
	void setClockDivider(uint8_t val);
	void setBitOrder(uint8_t val);
	void setDataMode(uint8_t val);
};


extern SPIClass SPI;


#endif /* __ADDE_SPI_H__ */
