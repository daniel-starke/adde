/**
 * @file Arduino.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-08
* @version 2019-05-23
 */
#ifndef __ADDE_ARDUINO_H__
#define __ADDE_ARDUINO_H__

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "utility/adde.hpp"
#include "binary.h"
#include "pins_arduino.h"


#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352


#undef abs
#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define abs(x) (((x) > 0) ? (x) : -(x))
#define constrain(x, low, high) (((x) < (low)) ? (low) : (((x) > (high)) ? (high) : (x)))
#define round(x) (((x) >= 0) ? ((long)((x) + 0.5)) : ((long)((x) - 0.5)))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))
#undef RANDOM_MAX
#define	RANDOM_MAX 0x7FFFFFFF


#define HIGH _HIGH
#define LOW _LOW
#define INPUT _INPUT
#define INPUT_PULLUP _INPUT_PULLUP
#define OUTPUT _OUTPUT
#define SERIAL _SERIAL
#define DISPLAY _DISPLAY
#define LSBFIRST _LSBFIRST
#define MSBFIRST _MSBFIRST
#define CHANGE _CHANGE
#define FALLING _FALLING
#define RISING _RISING
#define DEFAULT _DEFAULT
#define EXTERNAL _EXTERNAL
#define INTERNAL1V1 _INTERNAL1V1
#define INTERNAL _INTERNAL
#define INTERNAL2V56 _INTERNAL2V56
#define INTERNAL2V56_EXTCAP _INTERNAL2V56_EXTCAP
#define F_CPU _F_CPU


#define interrupts() _INTERRUPTS = true
#define noInterrupts() _INTERRUPTS = false


#define PSTR(...) __VA_ARGS__


#define clockCyclesPerMicrosecond() (F_CPU / 1000000L)
#define clockCyclesToMicroseconds(x) ((x) / clockCyclesPerMicrosecond())
#define microsecondsToClockCycles(x) ((x) * clockCyclesPerMicrosecond())


#define lowByte(x) ((uint8_t)((x) & 0xFF))
#define highByte(x) ((uint8_t)((x) >> 8))


#define bit(x) (1UL << (x))
#define bitRead(x, bit) (((x) >> (bit)) & 0x01)
#define bitSet(x, bit) ((x) = (__typeof__(x))((x) | (1UL << (bit))))
#define bitClear(x, bit) ((x) = (__typeof__(x))((x) & (~(1UL << (bit)))))
#define bitWrite(x, bit, val) ((val) ? bitSet((x), (bit)) : bitClear((x), (bit)))


#ifndef _NOP
#define _NOP() do { __asm__ volatile ("nop"); } while (0)
#endif


typedef uint8_t byte;
typedef uint16_t word;


#ifdef __cplusplus
extern "C" {
#endif


extern volatile bool _INTERRUPTS;
extern uint8_t _HIGH;
extern uint8_t _LOW;
extern uint8_t _INPUT;
extern uint8_t _INPUT_PULLUP;
extern uint8_t _OUTPUT;
extern uint8_t _SERIAL;
extern uint8_t _DISPLAY;
extern uint8_t _LSBFIRST;
extern uint8_t _MSBFIRST;
extern uint8_t _CHANGE;
extern uint8_t _FALLING;
extern uint8_t _RISING;
extern uint8_t _DEFAULT;
extern uint8_t _EXTERNAL;
extern uint8_t _INTERNAL1V1;
extern uint8_t _INTERNAL;
extern uint8_t _INTERNAL2V56;
extern uint8_t _INTERNAL2V56_EXTCAP;
extern uint32_t _F_CPU;


#ifndef NOT_AN_INTERRUPT
#define NOT_AN_INTERRUPT (-1)
#endif /* NOT_AN_INTERRUPT */


#ifndef INVALID_PIN
/** Protocol specific and may not be changed. */
#define INVALID_PIN (0xFF)
#endif /* INVALID_PIN */


#ifndef INVALID_PORT
/** Protocol specific and may not be changed. */
#define INVALID_PORT (0xFF)
#endif /* INVALID_PORT */


#ifndef INVALID_REFERENCE
/** Protocol specific and may not be changed. */
#define INVALID_REFERENCE (0xFF)
#endif /* INVALID_REFERENCE */


#ifndef INVALID_SERIAL_MODE
/** Protocol specific and may not be changed. */
#define INVALID_SERIAL_MODE (0xFF)
#endif /* INVALID_SERIAL_MODE */


#ifndef INVALID_SPI_CLOCK
/** Protocol specific and may not be changed. */
#define INVALID_SPI_CLOCK (0xFF)
#endif /* INVALID_SPI_CLOCK */


#ifndef INVALID_SPI_MODE
/** Protocol specific and may not be changed. */
#define INVALID_SPI_MODE (0xFF)
#endif /* INVALID_SPI_MODE */


void yield(void);

void init(void);
void initVariant(void);

int _atexit(void (* func)()) __attribute__((weak));
#define atexit _atexit

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReference(uint8_t mode);
void analogWrite(uint8_t pin, int val);
void analogReadResolution(int val);
void analogWriteResolution(int val);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long val);
void delayMicroseconds(unsigned int val);
unsigned long pulseIn(uint8_t pin, uint8_t val, unsigned long timeout);
unsigned long pulseInLong(uint8_t pin, uint8_t val, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t interrupt, void (* isr)(void), int mode);
void detachInterrupt(uint8_t interrupt);

void setup(void);
void loop(void);


/* non-Arduino helper function for debugging (works like printf) */
#if defined(__GNUC__)
int addeLog(const char * fmt, ...) __attribute__((format(gnu_printf, 1, 2)));
#else /* !GCC */
int addeLog(const char * fmt, ...);
#endif /* GCC */


#ifdef __cplusplus
} /* extern "C" */
#endif


#ifdef __cplusplus


#include <new>
#include "WCharacter.h"
#include "WString.h"
#include "ConsoleSerial.h"
#include "HardwareSerial.h"


typedef bool boolean;


ADDE_F(_int) _digitalRead(uint8_t pin);
#define digitalRead _digitalRead
ADDE_F(_int) _analogRead(uint8_t pin);
#define analogRead _analogRead

unsigned long pulseIn(uint8_t pin, uint8_t val, unsigned long timeout = 1000000UL);
ADDE_F(_unsigned_long) _pulseIn(uint8_t pin, uint8_t val, unsigned long timeout = 1000000UL);
#define pulseIn _pulseIn
unsigned long pulseInLong(uint8_t pin, uint8_t val, unsigned long timeout = 1000000UL);
ADDE_F(_unsigned_long) _pulseInLong(uint8_t pin, uint8_t val, unsigned long timeout = 1000000UL);
#define pulseInLong _pulseInLong

ADDE_F(_uint8_t) _shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);
#define shiftIn _shiftIn

void tone(uint8_t pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t pin);

long random(long _max);
long random(long _min, long _max);
void randomSeed(unsigned long seed);
long map(long val, long fromLow, long fromHigh, long toLow, long toHigh);

uint16_t makeWord(uint16_t w);
uint16_t makeWord(uint8_t h, uint8_t l);
#define word(...) makeWord(__VA_ARGS__)


#endif /* __cplusplus */


#endif /* __ADDE_ARDUINO_H__ */
