/**
 * @file SoftwareSerial.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-12
* @version 2019-05-01
 */
#ifndef __ADDE_SOFTWARESERIAL_H__
#define __ADDE_SOFTWARESERIAL_H__

#include <stddef.h>
#include <stdint.h>
#include "utility/adde.hpp"
#include "Stream.h"


class SoftwareSerial : public Stream {
private:
	uint8_t inst;
public:
	SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverseLogic = false);
	~SoftwareSerial();
	
	void begin(long speed);
	ADDE_F(_bool) listen();
	void end();
	ADDE_F(_bool) isListening();
	ADDE_F(_bool) stopListening();
	ADDE_F(_bool) overflow();

	virtual int available();
	virtual int peek();
	virtual int read();
	virtual void flush() {}
	virtual size_t write(uint8_t val);
	
	operator bool() { return true; }

	using Print::write;
};


#endif /* __ADDE_SOFTWARESERIAL_H__ */
