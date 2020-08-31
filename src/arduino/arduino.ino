/**
 * @file arduino.ino
 * @author Daniel Starke
 * @copyright Copyright 2019-2020 Daniel Starke
 * @date 2019-02-20
 * @version 2020-08-30
 * 
 * Tested with Arduino IDE 1.8.5, GCC 7.4 and ATmega32U4 (a.k.a. Arduino Pro Micro).
 * 
 * @todo add analogPinToChannel()
 */
#include "arduino.hpp"


#if PP_IS_VA(HAS_SOFTWARESERIAL)
Optional<SoftwareSerial> HAS_SOFTWARESERIAL;
#endif /* HAS_SOFTWARESERIAL */

#if PP_IS_VA(HAS_LCD)
Optional<LiquidCrystal> HAS_LCD;
#endif /* HAS_LCD */

Framing<MAX_FRAME_SIZE> framing(handleWrite);


/**
 * Maps the OP codes to the handling functions. These decode the function arguments in the frame and
 * call the actual function.
 * 
 * @warning This needs to match the order and values in OpCode::Type.
 */
const tHandler handler[OpCode::COUNT] PCF_ROM = {
	handleReset,
	handleGetProtVersion,
	handleGetDevName,
	handleGetConstants,
	handleGetPinMapD,
	handleGetPinMapA,
	handleGetPinMapS,
	handleGetPinMapP,
	handleGetPinMapI,
	handlePinMode,
	handleDigitalRead,
	handleDigitalWrite,
	handleAnalogRead,
	handleAnalogReference,
	handleAnalogWrite,
	handleAnalogReadResolution,
	handleAnalogWriteResolution,
	handlePulseIn,
	handlePulseInLong,
	handleShiftIn,
	handleShiftOut,
	handleTone,
	handleNoTone,
	handleAttachInterrupt,
	handleDetachInterrupt,
	handleHardwareSerialGetConstants,
	handleHardwareSerialAvailable,
	handleHardwareSerialAvailableForWrite,
	handleHardwareSerialBegin1,
	handleHardwareSerialBegin2,
	handleHardwareSerialEnd,
	handleHardwareSerialPeek,
	handleHardwareSerialRead,
	handleHardwareSerialFlush,
	handleHardwareSerialWrite,
	handleSoftwareSerial,
	handleSoftwareSerialAvailable,
	handleSoftwareSerialBegin,
	handleSoftwareSerialEnd,
	handleSoftwareSerialIsListening,
	handleSoftwareSerialStopListening,
	handleSoftwareSerialOverflow,
	handleSoftwareSerialPeek,
	handleSoftwareSerialRead,
	handleSoftwareSerialListen,
	handleSoftwareSerialWrite,
	handleEepromLength,
	handleEepromRead,
	handleEepromWrite,
	handleEepromUpdate,
	handleLiquidCrystal1,
	handleLiquidCrystal2,
	handleLiquidCrystal3,
	handleLiquidCrystal4,
	handleLiquidCrystalBegin,
	handleLiquidCrystalClear,
	handleLiquidCrystalHome,
	handleLiquidCrystalNoDisplay,
	handleLiquidCrystalDisplay,
	handleLiquidCrystalNoBlink,
	handleLiquidCrystalBlink,
	handleLiquidCrystalNoCursor,
	handleLiquidCrystalCursor,
	handleLiquidCrystalScrollDisplayLeft,
	handleLiquidCrystalScrollDisplayRight,
	handleLiquidCrystalLeftToRight,
	handleLiquidCrystalRightToLeft,
	handleLiquidCrystalNoAutoscroll,
	handleLiquidCrystalAutoscroll,
	handleLiquidCrystalSetRowOffsets,
	handleLiquidCrystalCreateChar,
	handleLiquidCrystalSetCursor,
	handleLiquidCrystalCommand,
	handleLiquidCrystalWrite,
	handleSpiGetConstants,
	handleSpiBegin,
	handleSpiEnd,
	handleSpiBeginTransaction,
	handleSpiEndTransaction,
	handleSpiSetClockDivider,
	handleSpiSetBitOrder,
	handleSpiSetDataMode,
	handleSpiTransfer,
	handleSpiTransfer16,
	handleSpiUsingInterrupt,
	handleSpiNotUsingInterrupt,
	handleWireBegin1,
	handleWireBegin2,
	handleWireEnd,
	handleWireSetClock,
	handleWireRequestFrom,
	handleWireBeginTransmission,
	handleWireEndTransmission,
	handleWireAvailable,
	handleWirePeek,
	handleWireRead,
	handleWireFlush,
	handleWireWrite,
	handleWireOnReceive,
	handleWireOnRequest
};


/**
 * List of interrupt states. True if triggered, else false.
 */
volatile bool intValue[MAX_NUM_INTERRUPTS];


/**
 * Interrupt number currently used by the interrupt handler at the given index.
 */
uint8_t intNum[MAX_NUM_INTERRUPTS] = {static_cast<uint8_t>(NOT_AN_INTERRUPT)};



/**
 * Buffer with a bit field of interrupts triggered.
 */
uint8_t intBuffer[(MAX_NUM_INTERRUPTS + 7) / 8];


/* Interrupt handlers. */
#define DEF_HANDLE_INT(i, ...) \
	void handleInt##i() { \
		intValue[i] = true; \
	}
PP_REPEAT_TPL(MAX_NUM_INTERRUPTS, DEF_HANDLE_INT)


/**
 * Return the result of a function call to the host.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[in] val - function return value
 * @return true on success, false on function argument extraction error
 * @tparam T - encapsulated result type
 */
template <typename T>
ErrorCode::Type handleResult(FrameHandlerArgs & hal, const CallResult<T> & val) {
	if ( val.success ) {
		framing.beginTransmission(hal.seq);
		framing.write(uint8_t(ResultCode::RESULT));
		framing.write(uint8_t(hal.op));
		framing.write(val.result);
		framing.endTransmission();
	}
	return val.success ? ErrorCode::SUCCESS : ErrorCode::SIGNATURE_MISMATCH;
}


/**
 * Return the result of a function call to the host.
 * 
 * @param[in,out] hal - frame handler argument list
 * @param[in] val - function return value
 * @return true on success, false on function argument extraction error
 * @remarks Specialization for functions with no return value.
 */
template <>
ErrorCode::Type handleResult(FrameHandlerArgs & hal, const CallResult<void> & val) {
	if ( val.success ) {
		framing.beginTransmission(hal.seq);
		framing.write(uint8_t(ResultCode::RESULT));
		framing.write(uint8_t(hal.op));
		framing.endTransmission();
	}
	return val.success ? ErrorCode::SUCCESS : ErrorCode::SIGNATURE_MISMATCH;
}


/**
 * Performs the requested operation.
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleGetProtVersion(FrameHandlerArgs & hal) {
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	framing.write(uint16_t(ADDE_PROT_VERSION));
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}


/**
 * Performs the requested operation.
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleGetDevName(FrameHandlerArgs & hal) {
	static const char devName[] PROGMEM = "7C4B6669-C616-4F1E-AD6A-42546CE45D93";
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	for (uint8_t i = 0; (i + 1) < static_cast<uint8_t>(sizeof(devName)); i++) {
		framing.write(uint8_t(PCF_ROM_READ_U8(devName, i)));
	}
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}


/**
 * Returns the digital I/O pin map (e.g. D0 -> 3).
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleGetPinMapD(FrameHandlerArgs & hal) {
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	framing.write(uint8_t(NUM_DIGITAL_PINS));
	for (uint8_t i = 0; i < static_cast<uint8_t>(NUM_DIGITAL_PINS); i++) {
		framing.write(uint8_t(digitalPinToPort(i)));
	}
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}


/**
 * Returns the digital pins with PWM support.
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleGetPinMapP(FrameHandlerArgs & hal) {
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	uint8_t count = 0;
	for (uint8_t i = 0; i < static_cast<uint8_t>(NUM_DIGITAL_PINS); i++) {
		if ( digitalPinHasPWM(i) ) count++;
	}
	framing.write(count);
	for (uint8_t i = 0; i < static_cast<uint8_t>(NUM_DIGITAL_PINS); i++) {
		if ( digitalPinHasPWM(i) ) framing.write(uint8_t(i));
	}
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}


/**
 * Returns the pin to interrupt number map (e.g. 4 -> 0).
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleGetPinMapI(FrameHandlerArgs & hal) {
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	framing.write(uint8_t(MAX_NUM_INTERRUPTS));
	for (uint8_t i = 0; i < static_cast<uint8_t>(NUM_DIGITAL_PINS); i++) {
		const int val = digitalPinToInterrupt(i);
		framing.write(uint8_t((val == NOT_AN_INTERRUPT) ? INVALID_PIN : val)); 
	}
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}


/**
 * Helper function to map interrupts to special callback functions.
 * 
 * @param[in] interrupt - interrupt number
 * @param[in] isr - callback routine ID (less than MAX_NUM_INTERRUPTS)
 * @param[in] mode - LOW, CHANGE, RISING or FALLING
 */
void attachInterruptHelper(uint8_t interrupt, uint8_t isr, int mode) {
	switch (isr) {
#define CASE_HANDLE_INT(i, ...) case i: intValue[i] = false; attachInterrupt(interrupt, &handleInt##i, mode); intNum[i] = interrupt; break;
	PP_REPEAT_TPL(MAX_NUM_INTERRUPTS, CASE_HANDLE_INT)
	default: break;
	}
}


/**
 * Helper function to unmap interrupts to special callback functions.
 * 
 * @param[in] interrupt - interrupt number
 * @param[in] isr - callback routine ID (less than MAX_NUM_INTERRUPTS)
 */
void detachInterruptHelper(uint8_t interrupt, uint8_t isr) {
	detachInterrupt(interrupt);
	switch (isr) {
#define CASE_HANDLE_DEINT(i, ...) case i: intNum[i] = NOT_AN_INTERRUPT; break;
	PP_REPEAT_TPL(MAX_NUM_INTERRUPTS, CASE_HANDLE_DEINT)
	default: break;
	}
}


/* general functions */
DEF_FUNCTION_HANDLER(handlePinMode, pinMode)
DEF_FUNCTION_HANDLER(handleDigitalRead, digitalRead)
DEF_FUNCTION_HANDLER(handleDigitalWrite, digitalWrite)
DEF_FUNCTION_HANDLER(handleAnalogRead, analogRead)
DEF_FUNCTION_HANDLER(handleAnalogReference, analogReference)
DEF_FUNCTION_HANDLER(handleAnalogWrite, analogWrite)
DEF_FUNCTION_HANDLER(handleAnalogReadResolution, analogReadResolution)
DEF_FUNCTION_HANDLER(handleAnalogWriteResolution, analogWriteResolution)
DEF_FUNCTION_HANDLER(handleNoTone, noTone)
DEF_FUNCTION_HANDLER(handlePulseIn, pulseIn)
DEF_FUNCTION_HANDLER(handlePulseInLong, pulseInLong)
DEF_FUNCTION_HANDLER(handleShiftIn, shiftIn)
DEF_FUNCTION_HANDLER(handleShiftOut, shiftOut)
DEF_FUNCTION_HANDLER(handleTone, tone)
DEF_FUNCTION_HANDLER(handleAttachInterrupt, attachInterruptHelper)
DEF_FUNCTION_HANDLER(handleDetachInterrupt, detachInterruptHelper)


/* hardware serial */
#if PP_IS_VA(HAS_HARDWARESERIAL) && defined(HAS_handleHardwareSerialGetConstants)
/**
 * Returns the implementation specific predefined constants for the hardware serial interface.
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleHardwareSerialGetConstants(FrameHandlerArgs & hal) {
	static const uint8_t consts[] PCF_ROM = {
		uint8_t(SERIAL_5N1), uint8_t(SERIAL_6N1), uint8_t(SERIAL_7N1), uint8_t(SERIAL_8N1),
		uint8_t(SERIAL_5N2), uint8_t(SERIAL_6N2), uint8_t(SERIAL_7N2), uint8_t(SERIAL_8N2),
		uint8_t(SERIAL_5E1), uint8_t(SERIAL_6E1), uint8_t(SERIAL_7E1), uint8_t(SERIAL_8E1),
		uint8_t(SERIAL_5E2), uint8_t(SERIAL_6E2), uint8_t(SERIAL_7E2), uint8_t(SERIAL_8E2),
		uint8_t(SERIAL_5O1), uint8_t(SERIAL_6O1), uint8_t(SERIAL_7O1), uint8_t(SERIAL_8O1),
		uint8_t(SERIAL_5O2), uint8_t(SERIAL_6O2), uint8_t(SERIAL_7O2), uint8_t(SERIAL_8O2)
	};
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	for (uint8_t i = 0; i < static_cast<uint8_t>(sizeof(consts)); i++) {
		framing.write(PCF_ROM_READ_U8(consts, i));
	}
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}
#else /* HAS_HARDWARESERIAL */
DEF_NO_FUNCTION_HANDLER(handleHardwareSerialGetConstants)
#endif /* HAS_HARDWARESERIAL */

#define X_HARDWARESERIAL decltype(PP_VA_GET_FIRST(HAS_HARDWARESERIAL)) PP_VA_COMMA_ARGS(HAS_HARDWARESERIAL)
DEF_MEMBER_FUNCTION_HANDLER(handleHardwareSerialAvailable, X_HARDWARESERIAL, available)
DEF_MEMBER_FUNCTION_HANDLER(handleHardwareSerialAvailableForWrite, X_HARDWARESERIAL, availableForWrite)
DEF_MEMBER_FUNCTION_HANDLER_AS(handleHardwareSerialBegin1, X_HARDWARESERIAL, begin, void, (unsigned long))
DEF_MEMBER_FUNCTION_HANDLER_AS(handleHardwareSerialBegin2, X_HARDWARESERIAL, begin, void, (unsigned long, uint8_t))
DEF_MEMBER_FUNCTION_HANDLER(handleHardwareSerialEnd, X_HARDWARESERIAL, end)
DEF_MEMBER_FUNCTION_HANDLER(handleHardwareSerialPeek, X_HARDWARESERIAL, peek)
DEF_MEMBER_FUNCTION_HANDLER(handleHardwareSerialRead, X_HARDWARESERIAL, read)
DEF_MEMBER_FUNCTION_HANDLER(handleHardwareSerialFlush, X_HARDWARESERIAL, flush)
DEF_MEMBER_FUNCTION_HANDLER_AS(handleHardwareSerialWrite, X_HARDWARESERIAL, write, size_t, (uint8_t))


/* software serial */
#define X_SOFTWARESERIAL Optional<SoftwareSerial> PP_VA_COMMA_ARGS(HAS_SOFTWARESERIAL)
#define Y_SOFTWARESERIAL_TPL(i, data, arg) , *arg
#define Y_SOFTWARESERIAL SoftwareSerial PP_EVAL(PP_VA_FOREACH(Y_SOFTWARESERIAL_TPL, ~, HAS_SOFTWARESERIAL))
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerial, X_SOFTWARESERIAL, (set<uint8_t, uint8_t, bool>))
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialAvailable, Y_SOFTWARESERIAL, available)
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialBegin, Y_SOFTWARESERIAL, begin)
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialEnd, Y_SOFTWARESERIAL, end)
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialIsListening, Y_SOFTWARESERIAL, isListening)
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialStopListening, Y_SOFTWARESERIAL, stopListening)
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialOverflow, Y_SOFTWARESERIAL, overflow)
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialPeek, Y_SOFTWARESERIAL, peek)
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialRead, Y_SOFTWARESERIAL, read)
DEF_MEMBER_FUNCTION_HANDLER(handleSoftwareSerialListen, Y_SOFTWARESERIAL, listen)
DEF_MEMBER_FUNCTION_HANDLER_AS(handleSoftwareSerialWrite, Y_SOFTWARESERIAL, write, size_t, (uint8_t))


/* EEPROM */
#define X_EEPROM decltype(PP_VA_GET_FIRST(HAS_EEPROM)) PP_VA_COMMA_ARGS(HAS_EEPROM)
DEF_MEMBER_FUNCTION_HANDLER(handleEepromLength, X_EEPROM, length)
DEF_MEMBER_FUNCTION_HANDLER(handleEepromRead, X_EEPROM, read)
DEF_MEMBER_FUNCTION_HANDLER(handleEepromWrite, X_EEPROM, write)
DEF_MEMBER_FUNCTION_HANDLER(handleEepromUpdate, X_EEPROM, update)


/* LCD */
#define X_LCD Optional<LiquidCrystal> PP_VA_COMMA_ARGS(HAS_LCD)
#define Y_LCD_TPL(i, data, arg) , *arg
#define Y_LCD LiquidCrystal PP_EVAL(PP_VA_FOREACH(Y_LCD_TPL, ~, HAS_LCD))
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystal1, X_LCD, (set<uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t>))
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystal2, X_LCD, (set<uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t>))
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystal3, X_LCD, (set<uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t>))
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystal4, X_LCD, (set<uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t>))
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalBegin, Y_LCD, begin)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalClear, Y_LCD, clear)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalHome, Y_LCD, home)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalNoDisplay, Y_LCD, noDisplay)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalDisplay, Y_LCD, display)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalNoBlink, Y_LCD, noBlink)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalBlink, Y_LCD, blink)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalNoCursor, Y_LCD, noCursor)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalCursor, Y_LCD, cursor)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalScrollDisplayLeft, Y_LCD, scrollDisplayLeft)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalScrollDisplayRight, Y_LCD, scrollDisplayRight)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalLeftToRight, Y_LCD, leftToRight)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalRightToLeft, Y_LCD, rightToLeft)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalNoAutoscroll, Y_LCD, noAutoscroll)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalAutoscroll, Y_LCD, autoscroll)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalSetRowOffsets, Y_LCD, setRowOffsets)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalCreateChar, Y_LCD, createChar)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalSetCursor, Y_LCD, setCursor)
DEF_MEMBER_FUNCTION_HANDLER(handleLiquidCrystalCommand, Y_LCD, command)
DEF_MEMBER_FUNCTION_HANDLER_AS(handleLiquidCrystalWrite, Y_LCD, write, size_t, (uint8_t))


/* SPI */
#if PP_IS_VA(HAS_SPI)
static bool readFrameValue(FrameHandlerArgs & hal, SPISettings & out) {
	uint32_t clock;
	if ( ! readFrameValue(hal, clock) ) return false;
	uint8_t bitOrder;
	if ( ! readFrameValue(hal, bitOrder) ) return false;
	uint8_t dataMode;
	if ( ! readFrameValue(hal, dataMode) ) return false;
	out = SPISettings(clock, bitOrder, dataMode);
	return true;
}
#endif /* HAS_SPI */


#if PP_IS_VA(HAS_SPI) && defined(HAS_handleSpiGetConstants)
/**
 * Returns the implementation specific predefined constants for the SPI interface.
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleSpiGetConstants(FrameHandlerArgs & hal) {
	static const uint8_t consts[] PCF_ROM = {
		uint8_t(SPI_CLOCK_DIV2), uint8_t(SPI_CLOCK_DIV4),
		uint8_t(SPI_CLOCK_DIV8), uint8_t(SPI_CLOCK_DIV16),
		uint8_t(SPI_CLOCK_DIV32), uint8_t(SPI_CLOCK_DIV64),
		uint8_t(SPI_CLOCK_DIV128), uint8_t(SPI_CLOCK_DIV256),
		uint8_t(SPI_CLOCK_DIV512), uint8_t(SPI_CLOCK_DIV1024),
		uint8_t(SPI_MODE0), uint8_t(SPI_MODE1),
		uint8_t(SPI_MODE2), uint8_t(SPI_MODE3)
	};
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	for (uint8_t i = 0; i < static_cast<uint8_t>(sizeof(consts)); i++) {
		framing.write(PCF_ROM_READ_U8(consts, i));
	}
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}
#else /* !HAS_SPI */
DEF_NO_FUNCTION_HANDLER(handleSpiGetConstants)
#endif /* HAS_SPI */

#define X_SPI decltype(PP_VA_GET_FIRST(HAS_SPI)) PP_VA_COMMA_ARGS(HAS_SPI)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiBegin, X_SPI, begin)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiEnd, X_SPI, end)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiBeginTransaction, X_SPI, beginTransaction)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiEndTransaction, X_SPI, endTransaction)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiSetClockDivider, X_SPI, setClockDivider)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiSetBitOrder, X_SPI, setBitOrder)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiSetDataMode, X_SPI, setDataMode)
DEF_MEMBER_FUNCTION_HANDLER_AS(handleSpiTransfer, X_SPI, transfer, uint8_t, (uint8_t))
DEF_MEMBER_FUNCTION_HANDLER(handleSpiTransfer16, X_SPI, transfer16)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiUsingInterrupt, X_SPI, usingInterrupt)
DEF_MEMBER_FUNCTION_HANDLER(handleSpiNotUsingInterrupt, X_SPI, notUsingInterrupt)


/* I2C */
#define X_WIRE decltype(PP_VA_GET_FIRST(HAS_WIRE)) PP_VA_COMMA_ARGS(HAS_WIRE)
DEF_MEMBER_FUNCTION_HANDLER_AS(handleWireBegin1, X_WIRE, begin, void, ())
DEF_MEMBER_FUNCTION_HANDLER_AS(handleWireBegin2, X_WIRE, begin, void, (uint8_t))
DEF_MEMBER_FUNCTION_HANDLER(handleWireEnd, X_WIRE, end)
DEF_MEMBER_FUNCTION_HANDLER(handleWireSetClock, X_WIRE, setClock)
DEF_MEMBER_FUNCTION_HANDLER_AS(handleWireRequestFrom, X_WIRE, requestFrom, uint8_t, (uint8_t, uint8_t, uint8_t))
DEF_MEMBER_FUNCTION_HANDLER_AS(handleWireBeginTransmission, X_WIRE, beginTransmission, void, (uint8_t))
DEF_MEMBER_FUNCTION_HANDLER_AS(handleWireEndTransmission, X_WIRE, endTransmission, uint8_t, (uint8_t))
DEF_MEMBER_FUNCTION_HANDLER(handleWireAvailable, X_WIRE, available)
DEF_MEMBER_FUNCTION_HANDLER(handleWirePeek, X_WIRE, peek)
DEF_MEMBER_FUNCTION_HANDLER(handleWireRead, X_WIRE, read)
DEF_MEMBER_FUNCTION_HANDLER(handleWireFlush, X_WIRE, flush)
DEF_MEMBER_FUNCTION_HANDLER_AS(handleWireWrite, X_WIRE, write, size_t, (uint8_t))
DEF_NO_FUNCTION_HANDLER(handleWireOnReceive)
DEF_NO_FUNCTION_HANDLER(handleWireOnRequest)


/**
 * Processes a received frame.
 * 
 * @param[in] seq - frame sequence number
 * @param[in,out] buf - frame data
 * @param[in] len - frame data length
 * @param[in] err - true on error, else false
 */
void handleFrame(const uint8_t seq, uint8_t * buf, const size_t len, const bool err) {
	if (len < 1) {
		handleError(seq, ErrorCode::BROKEN_FRAME, seq);
		return;
	}
	if (err || *buf >= OpCode::COUNT) {
		handleError(seq, ErrorCode::BROKEN_FRAME, seq);
		return;
	}
	const tHandler fn = (tHandler)(PCF_ROM_READ_PTR(handler, *buf));
	if (fn == NULL) {
		handleError(seq, ErrorCode::UNSUPPORTED_OPCODE, seq, *buf);
		return;
	}
	FrameHandlerArgs hal(seq, static_cast<OpCode::Type>(*buf), buf + 1, static_cast<size_t>(len - 1));
	ErrorCode::Type res = fn(hal);
	if (res != ErrorCode::SUCCESS) {
		handleError(seq, res, seq, *buf);
	}
}


/**
 * Writes out a single byte to the host.
 * 
 * @param[in] val - byte to send
 * @param[in] eof - end-of-frame mark
 * @return true on success, else false
 */
inline bool handleWrite(const uint8_t val, const bool eof) {
	if ( ! isSerialConnected(Serial) ) return false;
	const bool res = Serial.write(val) == 1;
#ifdef USBCON
	if ( eof ) Serial.flush();
#endif
	return res;
}


/**
 * Initialize the environment.
 */
void setup(void) {
	Serial.begin(115200);
	while ( ! Serial );
	delay(100);
}


/**
 * Main processing loop.
 */
void loop(void) {
	/* perform soft reset if host connection was closed */
	if ( ! isSerialConnected(Serial) ) {
		softReset();
		while ( ! isSerialConnected(Serial) );
	}
	/* process up to n bytes at a time */
	for (uint8_t i = 0; i < 16 && Serial.available() > 0; i++) {
		const uint8_t val = static_cast<uint8_t>(Serial.read());
		if ( ! framing.read(val, handleFrame) ) {
			handleError(0, ErrorCode::BROKEN_FRAME);
		}
	}
	/* process interrupts */
	bool hasInt = false;
	for (uint8_t i = 0, m = 0, b = 0; i < MAX_NUM_INTERRUPTS; i++, m <<= 1) {
		if (m == 0) {
			if (i != 0) b++;
			intBuffer[b] = 0;
			m = 1;
		}
		if ( intValue[i] ) {
			intValue[i] = false;
			intBuffer[b] |= m;
			hasInt = true;
		}
	}
	if ( hasInt ) {
		framing.beginTransmission();
		framing.write(uint8_t(ResultCode::INTERRUPT));
		framing.write(intBuffer, sizeof(intBuffer));
		framing.endTransmission();
	}
}
