/**
 * @file EEPROM.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-12
* @version 2019-05-01
 */
#ifndef __ADDE_EEPROM_H__
#define __ADDE_EEPROM_H__

#include <stddef.h>
#include <stdint.h>
#include "utility/adde.hpp"


class EERef {
private:
	const int index;
	const size_t inst;
public:
	explicit EERef(const int idx, const size_t instance = 0):
		index(idx),
		inst(instance)
	{}
	
	EERef & operator= (const EERef & ref) {
		return *this = *ref;
	}
	
	ADDE_F(_uint8_t) operator* () const;
	EERef & operator= (uint8_t val);
	EERef & update(uint8_t val);
	
	EERef & operator+= (uint8_t val) { return (*this = static_cast<uint8_t>(**this + val)); }
	EERef & operator-= (uint8_t val) { return (*this = static_cast<uint8_t>(**this - val)); }
	EERef & operator*= (uint8_t val) { return (*this = static_cast<uint8_t>(**this * val)); }
	EERef & operator/= (uint8_t val) { return (*this = static_cast<uint8_t>(**this / val)); }
	EERef & operator^= (uint8_t val) { return (*this = static_cast<uint8_t>(**this ^ val)); }
	EERef & operator%= (uint8_t val) { return (*this = static_cast<uint8_t>(**this % val)); }
	EERef & operator&= (uint8_t val) { return (*this = static_cast<uint8_t>(**this & val)); }
	EERef & operator|= (uint8_t val) { return (*this = static_cast<uint8_t>(**this | val)); }
	EERef & operator<<= (uint8_t val) { return (*this = static_cast<uint8_t>(**this << val)); }
	EERef & operator>>= (uint8_t val) { return (*this = static_cast<uint8_t>(**this >> val)); }
	
	EERef & operator++ () {
		(*this)++;
		return *this;
	}
	
	EERef & operator-- () {
		(*this)--;
		return *this;
	}
	
	uint8_t operator++ (int) {
		const uint8_t res = **this;
		++(*this);
		return res;
	}

	uint8_t operator-- (int) {
		const uint8_t res = **this;
		--(*this);
		return res;
	}
	
	operator uint8_t() const {
		return **this;
	}
};


class EEPtr {
private:
	int index;
	const size_t inst;
public:
	explicit EEPtr(const int idx, const size_t instance = 0):
		index(idx),
		inst(instance)
	{}
		
	EERef operator* () const {
		return EERef(this->index, this->inst);
	}
	
	EEPtr & operator= (int val) {
		this->index = val;
		return *this;
	}
	
	bool operator!= (const EEPtr & o) {
		return this->index != o.index;
	}
	
	EEPtr & operator++() {
		++(this->index);
		return *this;
	}
	
	EEPtr & operator--() {
		--(this->index);
		return *this;
	}
	
	EEPtr operator++ (int) {
		return EEPtr(this->index++);
	}
	
	EEPtr operator-- (int) {
		return EEPtr(this->index--);
	}

	operator int() const {
		return this->index;
	}
};


class EEPROMClass {
private:
	const uint8_t inst;
public:
	explicit EEPROMClass(const uint8_t instance);

	ADDE_F(_uint16_t) length();
	
	uint8_t read(int idx) {
		return EERef(idx, this->inst);
	}
	
	void write(int idx, uint8_t val) {
		EERef(idx, this->inst) = val;
	}
	
	void update(int idx, uint8_t val) {
		EERef(idx, this->inst).update(val);
	}
	
	EERef operator[] (const int idx) {
		return EERef(idx, this->inst);
	}
	
	EEPtr begin() {
		return EEPtr(0, this->inst);
	}
	
	EEPtr end() {
		return EEPtr(this->length(), this->inst);
	}
	
	template <typename T>
	T & get(int idx, T & out) {
		EEPtr src(idx, this->inst);
		uint8_t * dst = reinterpret_cast<uint8_t *>(&out);
		for (size_t i = 0; i < sizeof(T); ++i, ++src, ++dst) *dst = *src;
		return out;
	}
	
	template <typename T>
	const T & put(int idx, const T & val){
		EEPtr dst(idx, this->inst);
		const uint8_t * src = reinterpret_cast<const uint8_t *>(&val);
		for (size_t i = 0; i < sizeof(T); ++i, ++src, ++dst) (*dst).update(*src);
		return val;
	}
};


extern EEPROMClass EEPROM;


#endif /* __ADDE_EEPROM_H__ */
