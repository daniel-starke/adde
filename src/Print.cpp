/**
 * @file Print.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-09
 * @version 2019-05-01
 */
#include <math.h>
#include <string.h>
#include "Arduino.h"
#include "Print.h"


Print::Print():
	writeError(0)
{}


int Print::getWriteError(void) {
	return this->writeError;
}


void Print::clearWriteError(void) {
	this->setWriteError(0);
}


size_t Print::write(const char * str) {
	if (str == NULL) return 0;
	const size_t l = strlen(str);
	return this->write(reinterpret_cast<const uint8_t *>(str), l);
}


size_t Print::write(const uint8_t * buffer, size_t size) {
	for (size_t i = 0; i < size; i++, buffer++) {
		if (this->write(*buffer) == 0) return i;
	}
	return size;
}


size_t Print::write(const char * buffer, size_t size) {
	return this->write(reinterpret_cast<const uint8_t *>(buffer), size);
}


int Print::availableForWrite(void) {
	return 0;
}


size_t Print::print(const __FlashStringHelper * str) {
	return this->print(reinterpret_cast<const char *>(str));
}


size_t Print::print(const String & str) {
	return this->write(str.c_str(), str.length());
}


size_t Print::print(const char str[]) {
	return this->write(str);
}


size_t Print::print(char c) {
	return this->write(c);
}


size_t Print::print(unsigned char val, int base) {
	return this->print(static_cast<unsigned long>(val), base);
}


size_t Print::print(int val, int base) {
	return this->print(static_cast<long>(val), base);
}


size_t Print::print(unsigned int val, int base) {
	return this->print(static_cast<unsigned long>(val), base);
}


size_t Print::print(long val, int base) {
	if (base == 0) return this->write(static_cast<uint8_t>(val));
	if (base != 10) return this->printNumber(val, static_cast<uint8_t>(base));
	if (val < 0) {
		if (this->write('-') == 0) return 0;
		return this->printNumber(-val, 10) + 1;
	}
	return this->printNumber(val, 10);
}


size_t Print::print(unsigned long val, int base) {
	if (base == 0) return this->write(static_cast<uint8_t>(val));
	return this->printNumber(val, static_cast<uint8_t>(base));
}


size_t Print::print(double val, int digits) {
	return this->printFloat(val, static_cast<uint8_t>(digits));
}


size_t Print::print(const Printable & obj) {
	return obj.printTo(*this);
}


size_t Print::println(const __FlashStringHelper * str) {
	return this->println(reinterpret_cast<const char *>(str));
}


size_t Print::println(const String & str) {
	return this->print(str) + this->println();
}


size_t Print::println(const char str[]) {
	return this->print(str) + this->println();
}


size_t Print::println(char c) {
	return this->print(c) + this->println();
}


size_t Print::println(unsigned char val, int base) {
	return this->println(static_cast<unsigned long>(val), base);
}


size_t Print::println(int val, int base) {
	return this->println(static_cast<long>(val), base);
}


size_t Print::println(unsigned int val, int base) {
	return this->println(static_cast<unsigned long>(val), base);
}


size_t Print::println(long val, int base) {
	return this->print(val, base) + this->println();
}


size_t Print::println(unsigned long val, int base) {
	return this->print(val, base) + this->println();
}


size_t Print::println(double val, int digits) {
	return this->print(val, static_cast<uint8_t>(digits)) + this->println();
}


size_t Print::println(const Printable & obj) {
	return this->print(obj) + this->println();
}


size_t Print::println(void) {
	return this->write("\r\n");
}


void Print::flush(void) {
}


void Print::setWriteError(int err) {
	this->writeError = err;
}


size_t Print::printNumber(unsigned long val, uint8_t base) {
	char buf[8 * sizeof(long) + 1];
	char * str = buf + sizeof(buf) - 1;

	*str = 0;
	if (base < 2) base = 10;

	do {
		char c = static_cast<char>(val % base);
		val /= base;
		*(--str) = static_cast<char>((c < 10) ? c + '0' : c - 10 + 'A');
	} while (val);

	return this->write(str);
}


size_t Print::printFloat(double val, uint8_t digits) {
	if (isnan(val) != 0) return this->print("nan");
	if (isinf(val) != 0) return this->print("inf");
	/* values taken from Arduino core for compatibility (limit of a float mantissa + 1 bit) */
	if (val > 4294967040.0) return this->print("ovf");
	if (val < -4294967040.0) return this->print("ovf");
	
	size_t written = 0;
	
	if (val < 0) {
		written += this->write('-');
		val = -val;
	}
	
	double rounding = 0.5;
	for (uint8_t i = 0; i < digits; i++) {
		rounding /= 10.0;
	}
	val += rounding;
	
	unsigned long integer = static_cast<unsigned long>(val);
	val -= static_cast<double>(integer);
	
	written += this->print(integer);
	if (digits > 0) written += this->write('.');
	
	for (; digits > 0; digits--) {
		val *= 10.0;
		const uint8_t digit = static_cast<uint8_t>(val);
		written += this->write(static_cast<uint8_t>(digit + '0'));
		val -= static_cast<double>(digit);
	}
	
	return written;
}
