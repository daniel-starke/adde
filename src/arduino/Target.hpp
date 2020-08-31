/**
 * @file Target.hpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-06
 * @version 2019-07-08
 * 
 * Target platform configuration.
 */
#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include <Arduino.h>
#include "Pre.hpp"


/** Defines a list of available hardware serial interface instances. Can be empty. */
#define HAS_HARDWARESERIAL Serial1
/** Defines a list of SoftwareSerial interface instances. Any name is possible. Can be empty. */
#define HAS_SOFTWARESERIAL softSerial
/** Defines a list of LCD interface instances. Any name is possible. Can be empty. */
#define HAS_LCD lcd
/** Defines a list of available EEPROM instances. Can be empty. */
#define HAS_EEPROM EEPROM
/** Defines a list of available SPI interface instances. Can be empty. */
#define HAS_SPI SPI
/** Defines a list of available I2C interface instances. Can be empty. */
#define HAS_WIRE Wire


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


#ifndef NUM_ANALOG_INPUTS
#ifdef BOARD_NR_ADC_PINS
#define NUM_ANALOG_INPUTS BOARD_NR_ADC_PINS
#else /* BOARD_NR_ADC_PINS */
#error NUM_ANALOG_INPUTS is not defined. Please define it here.
#endif /* BOARD_NR_ADC_PINS */
#endif /* NUM_ANALOG_INPUTS */


#ifndef NUM_DIGITAL_PINS
#ifdef BOARD_NR_GPIO_PINS
#define NUM_DIGITAL_PINS BOARD_NR_GPIO_PINS
#else /* BOARD_NR_GPIO_PINS */
#error NUM_DIGITAL_PINS is not defined. Please define it here.
#endif /* BOARD_NR_GPIO_PINS */
#endif /* NUM_DIGITAL_PINS */


/* fine control over supported functions (define empty to support the function, undefined for not supported) */
#define HAS_handlePinMode
#define HAS_handleDigitalRead
#define HAS_handleDigitalWrite
#define HAS_handleAnalogRead
#define HAS_handleAnalogReference
#define HAS_handleAnalogWrite
#if defined(ARDUINO_SAM_DUE) || defined(ARDUINO_ARCH_ESP32)
#define HAS_handleAnalogReadResolution
#define HAS_handleAnalogWriteResolution
#endif /* ARDUINO_SAM_DUE || ARDUINO_ARCH_ESP32 */
#define HAS_handlePulseIn
#define HAS_handlePulseInLong
#define HAS_handleShiftIn
#define HAS_handleShiftOut
#define HAS_handleTone
#define HAS_handleNoTone
#define HAS_handleAttachInterrupt
#define HAS_handleDetachInterrupt
#define HAS_handleHardwareSerialGetConstants
#define HAS_handleHardwareSerialAvailable
#define HAS_handleHardwareSerialAvailableForWrite
#define HAS_handleHardwareSerialBegin1
#define HAS_handleHardwareSerialBegin2
#define HAS_handleHardwareSerialEnd
#define HAS_handleHardwareSerialPeek
#define HAS_handleHardwareSerialRead
#define HAS_handleHardwareSerialFlush
#define HAS_handleHardwareSerialWrite
#define HAS_handleSoftwareSerial
#define HAS_handleSoftwareSerialAvailable
#define HAS_handleSoftwareSerialBegin
#define HAS_handleSoftwareSerialEnd
#define HAS_handleSoftwareSerialIsListening
#define HAS_handleSoftwareSerialStopListening
#define HAS_handleSoftwareSerialOverflow
#define HAS_handleSoftwareSerialPeek
#define HAS_handleSoftwareSerialRead
#define HAS_handleSoftwareSerialListen
#define HAS_handleSoftwareSerialWrite
#define HAS_handleEepromLength
#define HAS_handleEepromRead
#define HAS_handleEepromWrite
#define HAS_handleEepromUpdate
#define HAS_handleLiquidCrystal1
#define HAS_handleLiquidCrystal2
#define HAS_handleLiquidCrystal3
#define HAS_handleLiquidCrystal4
#define HAS_handleLiquidCrystalBegin
#define HAS_handleLiquidCrystalClear
#define HAS_handleLiquidCrystalHome
#define HAS_handleLiquidCrystalNoDisplay
#define HAS_handleLiquidCrystalDisplay
#define HAS_handleLiquidCrystalNoBlink
#define HAS_handleLiquidCrystalBlink
#define HAS_handleLiquidCrystalNoCursor
#define HAS_handleLiquidCrystalCursor
#define HAS_handleLiquidCrystalScrollDisplayLeft
#define HAS_handleLiquidCrystalScrollDisplayRight
#define HAS_handleLiquidCrystalLeftToRight
#define HAS_handleLiquidCrystalRightToLeft
#define HAS_handleLiquidCrystalNoAutoscroll
#define HAS_handleLiquidCrystalAutoscroll
#define HAS_handleLiquidCrystalSetRowOffsets
#define HAS_handleLiquidCrystalCreateChar
#define HAS_handleLiquidCrystalSetCursor
#define HAS_handleLiquidCrystalCommand
#define HAS_handleLiquidCrystalWrite
#define HAS_handleSpiGetConstants
#define HAS_handleSpiBegin
#define HAS_handleSpiEnd
#define HAS_handleSpiBeginTransaction
#define HAS_handleSpiEndTransaction
#define HAS_handleSpiSetClockDivider
#define HAS_handleSpiSetBitOrder
#define HAS_handleSpiSetDataMode
#define HAS_handleSpiTransfer
#define HAS_handleSpiTransfer16
#define HAS_handleSpiUsingInterrupt
#define HAS_handleSpiNotUsingInterrupt
#define HAS_handleWireBegin1
#define HAS_handleWireBegin2
#define HAS_handleWireEnd
#define HAS_handleWireSetClock
#define HAS_handleWireRequestFrom
#define HAS_handleWireBeginTransmission
#define HAS_handleWireEndTransmission
#define HAS_handleWireAvailable
#define HAS_handleWirePeak
#define HAS_handleWireRead
#define HAS_handleWireFlush
#define HAS_handleWireWrite
#define HAS_handleWireOnReceive
#define HAS_handleWireOnRequest


#ifndef LED_BUILTIN
#define LED_BUILTIN INVALID_PIN
#endif /* LED_BUILTIN */


#ifndef LED_BUILTIN_RX
#ifdef PIN_LED_RXL
#define LED_BUILTIN_RX PIN_LED_RXL
#else
#define LED_BUILTIN_RX INVALID_PIN
#endif
#endif /* LED_BUILTIN_RX */


#ifndef LED_BUILTIN_TX
#ifdef PIN_LED_TXL
#define LED_BUILTIN_TX PIN_LED_TXL
#else
#define LED_BUILTIN_TX INVALID_PIN
#endif
#endif /* LED_BUILTIN_TX */


#ifndef PIN_SPI_SS0
#ifdef PIN_SPI_SS
#define PIN_SPI_SS0 PIN_SPI_SS
#else
#define PIN_SPI_SS0 INVALID_PIN
#endif
#endif /* PIN_SPI_SS0 */


#ifndef PIN_SPI_SS1
#define PIN_SPI_SS1 INVALID_PIN
#endif /* PIN_SPI_SS1 */


#ifndef PIN_SPI_SS2
#define PIN_SPI_SS2 INVALID_PIN
#endif /* PIN_SPI_SS2 */


#ifndef PIN_SPI_SS3
#define PIN_SPI_SS3 INVALID_PIN
#endif /* PIN_SPI_SS3 */


#ifndef PIN_SPI_MOSI
#define PIN_SPI_MOSI INVALID_PIN
#endif /* PIN_SPI_MOSI */


#ifndef PIN_SPI_MISO
#define PIN_SPI_MISO INVALID_PIN
#endif /* PIN_SPI_MISO */


#ifndef PIN_SPI_SCK
#define PIN_SPI_SCK INVALID_PIN
#endif /* PIN_SPI_SCK */


#ifndef PIN_WIRE_SDA
#define PIN_WIRE_SDA INVALID_PIN
#endif /* PIN_WIRE_SDA */


#ifndef PIN_WIRE_SCL
#define PIN_WIRE_SCL INVALID_PIN
#endif /* PIN_WIRE_SCL */


#ifndef PIN_WIRE1_SDA
#define PIN_WIRE1_SDA INVALID_PIN
#endif /* PIN_WIRE1_SDA */


#ifndef PIN_WIRE1_SCL
#define PIN_WIRE1_SCL INVALID_PIN
#endif /* PIN_WIRE1_SCL */


#ifndef DEFAULT
#define DEFAULT INVALID_REFERENCE
#endif /* DEFAULT */


#ifndef EXTERNAL
#define EXTERNAL INVALID_REFERENCE
#endif /* EXTERNAL */


#ifndef INTERNAL1V1
#define INTERNAL1V1 INVALID_REFERENCE
#endif /* INTERNAL1V1 */


#ifndef INTERNAL
#define INTERNAL INVALID_REFERENCE
#endif /* INTERNAL */


#ifndef INTERNAL2V56
#define INTERNAL2V56 INVALID_REFERENCE
#endif /* INTERNAL2V56 */


#ifndef INTERNAL2V56_EXTCAP
#define INTERNAL2V56_EXTCAP INVALID_REFERENCE
#endif /* INTERNAL2V56_EXTCAP */


#if PP_IS_VA(HAS_SOFTWARESERIAL)
#include <SoftwareSerial.h>
#endif /* HAS_SOFTWARESERIAL */


#if PP_IS_VA(HAS_LCD)
#include <LiquidCrystal.h>
#endif /* HAS_LCD */


#if PP_IS_VA(HAS_EEPROM)
#include <EEPROM.h>
#endif /* HAS_EEPROM */


#if PP_IS_VA(HAS_SERIAL)
#ifndef SERIAL_5N1
#define SERIAL_5N1 INVALID_SERIAL_MODE
#endif /* SERIAL_5N1 */

#ifndef SERIAL_6N1
#define SERIAL_6N1 INVALID_SERIAL_MODE
#endif /* SERIAL_6N1 */

#ifndef SERIAL_7N1
#define SERIAL_7N1 INVALID_SERIAL_MODE
#endif /* SERIAL_7N1 */

#ifndef SERIAL_8N1
#define SERIAL_8N1 INVALID_SERIAL_MODE
#endif /* SERIAL_8N1 */

#ifndef SERIAL_5N2
#define SERIAL_5N2 INVALID_SERIAL_MODE
#endif /* SERIAL_5N2 */

#ifndef SERIAL_6N2
#define SERIAL_6N2 INVALID_SERIAL_MODE
#endif /* SERIAL_6N2 */

#ifndef SERIAL_7N2
#define SERIAL_7N2 INVALID_SERIAL_MODE
#endif /* SERIAL_7N2 */

#ifndef SERIAL_8N2
#define SERIAL_8N2 INVALID_SERIAL_MODE
#endif /* SERIAL_8N2 */

#ifndef SERIAL_5E1
#define SERIAL_5E1 INVALID_SERIAL_MODE
#endif /* SERIAL_5E1 */

#ifndef SERIAL_6E1
#define SERIAL_6E1 INVALID_SERIAL_MODE
#endif /* SERIAL_6E1 */

#ifndef SERIAL_7E1
#define SERIAL_7E1 INVALID_SERIAL_MODE
#endif /* SERIAL_7E1 */

#ifndef SERIAL_8E1
#define SERIAL_8E1 INVALID_SERIAL_MODE
#endif /* SERIAL_8E1 */

#ifndef SERIAL_5E2
#define SERIAL_5E2 INVALID_SERIAL_MODE
#endif /* SERIAL_5E2 */

#ifndef SERIAL_6E2
#define SERIAL_6E2 INVALID_SERIAL_MODE
#endif /* SERIAL_6E2 */

#ifndef SERIAL_7E2
#define SERIAL_7E2 INVALID_SERIAL_MODE
#endif /* SERIAL_7E2 */

#ifndef SERIAL_8E2
#define SERIAL_8E2 INVALID_SERIAL_MODE
#endif /* SERIAL_8E2 */

#ifndef SERIAL_5O1
#define SERIAL_5O1 INVALID_SERIAL_MODE
#endif /* SERIAL_5O1 */

#ifndef SERIAL_6O1
#define SERIAL_6O1 INVALID_SERIAL_MODE
#endif /* SERIAL_6O1 */

#ifndef SERIAL_7O1
#define SERIAL_7O1 INVALID_SERIAL_MODE
#endif /* SERIAL_7O1 */

#ifndef SERIAL_8O1
#define SERIAL_8O1 INVALID_SERIAL_MODE
#endif /* SERIAL_8O1 */

#ifndef SERIAL_5O2
#define SERIAL_5O2 INVALID_SERIAL_MODE
#endif /* SERIAL_5O2 */

#ifndef SERIAL_6O2
#define SERIAL_6O2 INVALID_SERIAL_MODE
#endif /* SERIAL_6O2 */

#ifndef SERIAL_7O2
#define SERIAL_7O2 INVALID_SERIAL_MODE
#endif /* SERIAL_7O2 */

#ifndef SERIAL_8O2
#define SERIAL_8O2 INVALID_SERIAL_MODE
#endif /* SERIAL_8O2 */
#endif /* HAS_SERIAL */


#if PP_IS_VA(HAS_SPI)
#include <SPI.h>

#ifndef SPI_CLOCK_DIV2
#define SPI_CLOCK_DIV2 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV2 */

#ifndef SPI_CLOCK_DIV4
#define SPI_CLOCK_DIV4 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV4 */

#ifndef SPI_CLOCK_DIV8
#define SPI_CLOCK_DIV8 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV8 */

#ifndef SPI_CLOCK_DIV16
#define SPI_CLOCK_DIV16 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV16 */

#ifndef SPI_CLOCK_DIV32
#define SPI_CLOCK_DIV32 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV32 */

#ifndef SPI_CLOCK_DIV64
#define SPI_CLOCK_DIV64 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV64 */

#ifndef SPI_CLOCK_DIV128
#define SPI_CLOCK_DIV128 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV128 */

#ifndef SPI_CLOCK_DIV256
#define SPI_CLOCK_DIV256 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV256 */

#ifndef SPI_CLOCK_DIV512
#define SPI_CLOCK_DIV512 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV512 */

#ifndef SPI_CLOCK_DIV1024
#define SPI_CLOCK_DIV1024 INVALID_SPI_CLOCK
#endif /* SPI_CLOCK_DIV1024 */

#ifndef SPI_MODE0
#define SPI_MODE0 INVALID_SPI_MODE
#endif /* SPI_MODE0 */

#ifndef SPI_MODE1
#define SPI_MODE1 INVALID_SPI_MODE
#endif /* SPI_MODE1 */

#ifndef SPI_MODE2
#define SPI_MODE2 INVALID_SPI_MODE
#endif /* SPI_MODE2 */

#ifndef SPI_MODE3
#define SPI_MODE3 INVALID_SPI_MODE
#endif /* SPI_MODE3 */
#endif /* HAS_SPI */


#if PP_IS_VA(HAS_WIRE)
#include <Wire.h>
#endif /* HAS_WIRE */


/** Maximum frame size excluding sequence number and CRC16 before quotation. */
#define MAX_FRAME_SIZE 256


#ifdef __AVR__
#include <avr/pgmspace.h>
#define PCF_ROM PROGMEM
#define PCF_ROM_READ_U8(x, o) pgm_read_byte((x) + (o))
#define PCF_ROM_READ_U16(x, o) pgm_read_word((x) + (o))
#define PCF_ROM_READ_U32(x, o) pgm_read_dword((x) + (o))
#define PCF_ROM_READ_PTR(x, o) pgm_read_ptr((x) + (o))
#else
#define PCF_ROM
#define PCF_ROM_READ_U8(x, o) (x)[o]
#define PCF_ROM_READ_U16(x, o) (x)[o]
#define PCF_ROM_READ_U32(x, o) (x)[o]
#define PCF_ROM_READ_PTR(x, o) (x)[o]
#endif /* __AVR__ */


#endif /* __TARGET_HPP__ */
