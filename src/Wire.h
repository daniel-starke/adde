/**
 * @file Wire.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-15
 * @version 2019-05-01
 */
#ifndef __ADDE_WIRE_H__
#define __ADDE_WIRE_H__

#include <stddef.h>
#include <stdint.h>
#include "arduino/Meta.hpp"
#include "utility/adde.hpp"
#include "Stream.h"


#define WIRE_HAS_END 1


class TwoWire : public Stream {
private:
	const uint8_t inst;
public:
	TwoWire(const uint8_t instance);
	~TwoWire();
	
	void begin();
	void begin(uint8_t address);
	inline void begin(int address) { return this->begin(static_cast<uint8_t>(address)); }
	void end();

	void setClock(uint32_t clock);
	void beginTransmission(uint8_t address);
	inline void beginTransmission(int address) { return this->beginTransmission(static_cast<uint8_t>(address)); }
	inline ADDE_F(_uint8_t) endTransmission() { return this->endTransmission(uint8_t(true)); }
	ADDE_F(_uint8_t) endTransmission(uint8_t sendStop);
	ADDE_F(_uint8_t) requestFrom(uint8_t address, uint8_t size);
	ADDE_F(_uint8_t) requestFrom(uint8_t address, uint8_t size, uint8_t sendStop);
	// @todo ADDE_F(_uint8_t) requestFrom(uint8_t address, uint8_t size, uint32_t iaddress, uint8_t isize, uint8_t sendStop);
	inline ADDE_F(_uint8_t) requestFrom(int address, int size) { return this->requestFrom(static_cast<uint8_t>(address), static_cast<uint8_t>(size), static_cast<uint8_t>(true)); }
	inline ADDE_F(_uint8_t) requestFrom(int address, int size, int sendStop) { return this->requestFrom(static_cast<uint8_t>(address), static_cast<uint8_t>(size), static_cast<uint8_t>(sendStop)); }
	virtual int available();
	virtual int peek();
	virtual int read();
	virtual void flush();
	virtual size_t write(uint8_t data);
	virtual size_t write(const uint8_t * data, size_t size);

	template <typename T>
	inline size_t write(T val) { static_assert(!is_same<uint8_t, T>::value, "Cast to uint8_t is missing for write()."); return this->write(static_cast<uint8_t>(val)); }
	
	// @todo void onReceive(void (*)(int));
	// @todo void onRequest(void (*)(void));

	using Print::write;
};


extern TwoWire Wire;
extern TwoWire Wire1;


#endif /* __ADDE_WIRE_H__ */
