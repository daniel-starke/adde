/**
 * @file arduino.hpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-02-20
 * @version 2019-04-15
 */
#ifndef __ARDUINO_HPP__
#define __ARDUINO_HPP__

#include <stdint.h>
#include "Optional.hpp"
#include "Protocol.hpp"
#include "Framing.hpp"
#include "Pre.hpp"
#include "Utility.hpp"
#include "Target.hpp"


/** Gets a single element sequence value or returns the argument list for non-sequence arguments. */
#define PP_SEQ1_DECAY(...) PP_IF(PP_IS_SEQ(__VA_ARGS__))(PP_SEQ_GET_FIRST(__VA_ARGS__), __VA_ARGS__)


/** Defines a new function handler of the given name for an unhandled function. */
#define DEF_NO_FUNCTION_HANDLER(name) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & /* hal */) { \
		return ErrorCode::UNSUPPORTED_OPCODE; \
	}


/** Defines a new function handler with the given name for the passed function. */
#define DEF_FUNCTION_HANDLER(name, fn) PP_IF(PP_NOT_EMPTY(HAS_##name))( \
	DEF_NO_FUNCTION_HANDLER(name), \
	DEF_FUNCTION_HANDLER_HELPER(name, fn) \
)
#define DEF_FUNCTION_HANDLER_HELPER(name, fn) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & hal) { \
		return handleResult(hal, callWith(hal, fn)); \
	}


/** Defines a new function handler with the given name for the passed function. */
#define DEF_FUNCTION_HANDLER_AS(name, fn, ret, args) PP_IF(PP_NOT_EMPTY(HAS_##name))( \
	DEF_NO_FUNCTION_HANDLER(name), \
	DEF_FUNCTION_HANDLER_AS_HELPER(name, fn, ret, args) \
)
#define DEF_FUNCTION_HANDLER_AS_HELPER(name, fn, ret, args) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & hal) { \
		return handleResult(hal, callWith(hal, static_cast<ret(*)args>(fn))); \
	}


/** Defines a new member function handler with the given name for the passed object and function. */
#define DEF_MEMBER_FUNCTION_HANDLER(name, ...) PP_IF(PP_NOT_EMPTY(HAS_##name))( \
	DEF_NO_FUNCTION_HANDLER(name), \
	PP_EXPAND(PP_CAT(DEF_MEMBER_FUNCTION_HANDLER_, PP_VA_SIZE(__VA_ARGS__))(name, __VA_ARGS__)) \
)


/** Defines a new member function handler with the given name for the passed object and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_AS(name, ...) PP_IF(PP_NOT_EMPTY(HAS_##name))( \
	DEF_NO_FUNCTION_HANDLER(name), \
	PP_EXPAND(PP_CAT(DEF_MEMBER_FUNCTION_HANDLER_AS_, PP_VA_SIZE(__VA_ARGS__))(name, __VA_ARGS__)) \
)


/** Defines a new member function handler with the given name for the passed object and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_2(name, type, fn) \
	DEF_NO_FUNCTION_HANDLER(name)


/** Defines a new member function handler with the given name for the passed object and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_AS_4(name, type, fn, ret, args) \
	DEF_NO_FUNCTION_HANDLER(name)


/** Defines a new member function handler with the given name for the passed object and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_3(name, type, obj0, fn) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & hal) { \
		uint8_t inst; \
		if ( ! readFrameValue(hal, inst) ) return ErrorCode::BROKEN_FRAME; \
		switch (inst) { \
		case 0: \
			return handleResult(hal, callWith(hal, &type::PP_SEQ1_DECAY(fn), obj0)); \
		default: \
			return ErrorCode::INVALID_ARGUMENT; \
		} \
	}


/** Defines a new member function handler with the given name for the passed object and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_AS_5(name, type, obj0, fn, ret, args) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & hal) { \
		uint8_t inst; \
		if ( ! readFrameValue(hal, inst) ) return ErrorCode::BROKEN_FRAME; \
		switch (inst) { \
		case 0: \
			return handleResult(hal, callWith<ret, type PP_VA_COMMA_ARGS(PP_SEQ_GET_FIRST(args))>(hal, &type::PP_SEQ1_DECAY(fn), obj0)); \
		default: \
			return ErrorCode::INVALID_ARGUMENT; \
		} \
	}


/** Defines a new member function handler with the given name for the passed objects and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_4(name, type, obj0, obj1, fn) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & hal) { \
		uint8_t inst; \
		if ( ! readFrameValue(hal, inst) ) return ErrorCode::BROKEN_FRAME; \
		type * const objs[] = {&obj0, &obj1}; \
		switch (inst) { \
		case 0: \
		case 1: \
			return handleResult(hal, callWith(hal, &type::PP_SEQ1_DECAY(fn), *(objs[inst]))); \
		default: \
			return ErrorCode::INVALID_ARGUMENT; \
		} \
	}


/** Defines a new member function handler with the given name for the passed objects and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_AS_6(name, type, obj0, obj1, fn, ret, args) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & hal) { \
		uint8_t inst; \
		if ( ! readFrameValue(hal, inst) ) return ErrorCode::BROKEN_FRAME; \
		type * const objs[] = {&obj0, &obj1}; \
		switch (inst) { \
		case 0: \
		case 1: \
			return handleResult(hal, callWith<ret, type PP_VA_COMMA_ARGS(PP_SEQ_GET_FIRST(args))>(hal, &type::PP_SEQ1_DECAY(fn), *(objs[inst]))); \
		default: \
			return ErrorCode::INVALID_ARGUMENT; \
		} \
	}


/** Defines a new member function handler with the given name for the passed objects and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_5(name, type, obj0, obj1, obj2, fn) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & hal) { \
		uint8_t inst; \
		if ( ! readFrameValue(hal, inst) ) return ErrorCode::BROKEN_FRAME; \
		type * const objs[] = {&obj0, &obj1, &obj2}; \
		switch (inst) { \
		case 0: \
		case 1: \
		case 2: \
			return handleResult(hal, callWith(hal, &type::PP_SEQ1_DECAY(fn), *(objs[inst]))); \
		default: \
			return ErrorCode::INVALID_ARGUMENT; \
		} \
	}


/** Defines a new member function handler with the given name for the passed objects and function. */
#define DEF_MEMBER_FUNCTION_HANDLER_AS_7(name, type, obj0, obj1, obj2, fn, ret, args) \
	/** @copydoc handleGetProtVersion */ \
	ErrorCode::Type name(FrameHandlerArgs & hal) { \
		uint8_t inst; \
		if ( ! readFrameValue(hal, inst) ) return ErrorCode::BROKEN_FRAME; \
		type * const objs[] = {&obj0, &obj1, &obj2}; \
		switch (inst) { \
		case 0: \
		case 1: \
		case 2: \
			return handleResult(hal, callWith<ret, type PP_VA_COMMA_ARGS(PP_SEQ_GET_FIRST(args))>(hal, &type::PP_SEQ1_DECAY(fn), *(objs[inst]))); \
		default: \
			return ErrorCode::INVALID_ARGUMENT; \
		} \
	}


/** Defines the function type for the remote function handlers. */
typedef ErrorCode::Type (* tHandler)(FrameHandlerArgs & hal);


#if PP_IS_VA(HAS_SOFTWARESERIAL)
extern Optional<SoftwareSerial> HAS_SOFTWARESERIAL;
#endif /* HAS_SOFTWARESERIAL */

#if PP_IS_VA(HAS_LCD)
extern Optional<LiquidCrystal> HAS_LCD;
#endif /* HAS_LCD */

extern Framing<MAX_FRAME_SIZE> framing;


/** Performs a reset of the board. */


extern const tHandler handler[OpCode::COUNT] PCF_ROM;
extern uint8_t intNum[MAX_NUM_INTERRUPTS];
extern volatile bool intValue[MAX_NUM_INTERRUPTS];
extern uint8_t intBuffer[(MAX_NUM_INTERRUPTS + 7) / 8];


namespace {


/**
 * Returns an error to the host.
 * 
 * @param[in] seq - sequence number
 * @param[in] type - result type
 * @tparam ...Args - argument types
 */
template <typename ...Args>
void handleError(const uint8_t seq, const ErrorCode::Type type, Args... args) {
	framing.beginTransmission(seq);
	framing.write(uint8_t(ResultCode::ERROR));
	framing.write(uint8_t(type));
	framing.write(args...);
	framing.endTransmission();
}


/**
 * Return the result of an function call to the host.
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
 * Return the result of an function call to the host.
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


DEF_HAS_MEMBER(dtr)
DEF_HAS_MEMBER(getDTR)
template <typename T> typename enable_if<!has_member_dtr<T>::value, bool>::type serialDtr(T & ser) { return bool(ser); }
template <typename T> typename enable_if<has_member_dtr<T>::value, bool>::type serialDtr(T & ser) { return ser.dtr(); }
template <typename T> typename enable_if<!has_member_getDTR<T>::value, bool>::type serialGetDtr(T & ser) { return serialDtr(ser); }
template <typename T> typename enable_if<has_member_getDTR<T>::value, bool>::type serialGetDtr(T & ser) { return ser.getDTR(); }


/**
 * Checks if the given serial interface is connected to the host.
 * 
 * @param[in] ser - serial interface instance
 * @return true if connected, else false
 * @remarks The functions serialGetDtr() and serialDtr() are used to detect the target specific API for this check.
 * @tparam T - type of the serial interface instance
 */
template <typename T>
bool isSerialConnected(T & ser) {
	return serialGetDtr(ser);
}


} /* anonymous namespace */


#define DECL_HANDLE_INT(i, ...) void handleInt##i();
PP_REPEAT_TPL(MAX_NUM_INTERRUPTS, DECL_HANDLE_INT)


ErrorCode::Type handleReset(FrameHandlerArgs & args);
ErrorCode::Type handleGetProtVersion(FrameHandlerArgs & args);
ErrorCode::Type handleGetDevName(FrameHandlerArgs & args);
ErrorCode::Type handleGetConstants(FrameHandlerArgs & args);
ErrorCode::Type handleGetPinMapD(FrameHandlerArgs & args);
ErrorCode::Type handleGetPinMapA(FrameHandlerArgs & args);
ErrorCode::Type handleGetPinMapS(FrameHandlerArgs & args);
ErrorCode::Type handleGetPinMapP(FrameHandlerArgs & args);
ErrorCode::Type handleGetPinMapI(FrameHandlerArgs & args);
ErrorCode::Type handlePinMode(FrameHandlerArgs & args);
ErrorCode::Type handleDigitalRead(FrameHandlerArgs & args);
ErrorCode::Type handleDigitalWrite(FrameHandlerArgs & args);
ErrorCode::Type handleAnalogRead(FrameHandlerArgs & args);
ErrorCode::Type handleAnalogReference(FrameHandlerArgs & args);
ErrorCode::Type handleAnalogWrite(FrameHandlerArgs & args);
ErrorCode::Type handleAnalogReadResolution(FrameHandlerArgs & args);
ErrorCode::Type handleAnalogWriteResolution(FrameHandlerArgs & args);
ErrorCode::Type handleNoTone(FrameHandlerArgs & args);
ErrorCode::Type handlePulseIn(FrameHandlerArgs & args);
ErrorCode::Type handlePulseInLong(FrameHandlerArgs & args);
ErrorCode::Type handleShiftIn(FrameHandlerArgs & args);
ErrorCode::Type handleShiftOut(FrameHandlerArgs & args);
ErrorCode::Type handleTone(FrameHandlerArgs & args);
ErrorCode::Type handleAttachInterrupt(FrameHandlerArgs & args);
ErrorCode::Type handleDetachInterrupt(FrameHandlerArgs & args);

ErrorCode::Type handleHardwareSerialGetConstants(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialAvailable(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialAvailableForWrite(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialBegin1(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialBegin2(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialEnd(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialPeek(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialRead(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialFlush(FrameHandlerArgs & args);
ErrorCode::Type handleHardwareSerialWrite(FrameHandlerArgs & args);

ErrorCode::Type handleSoftwareSerial(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialAvailable(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialBegin(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialEnd(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialIsListening(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialStopListening(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialOverflow(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialPeek(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialRead(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialListen(FrameHandlerArgs & args);
ErrorCode::Type handleSoftwareSerialWrite(FrameHandlerArgs & args);

ErrorCode::Type handleEepromLength(FrameHandlerArgs & args);
ErrorCode::Type handleEepromRead(FrameHandlerArgs & args);
ErrorCode::Type handleEepromWrite(FrameHandlerArgs & args);
ErrorCode::Type handleEepromUpdate(FrameHandlerArgs & args);

ErrorCode::Type handleLiquidCrystal1(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystal2(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystal3(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystal4(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalBegin(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalClear(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalHome(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalNoDisplay(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalDisplay(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalNoBlink(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalBlink(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalNoCursor(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalCursor(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalScrollDisplayLeft(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalScrollDisplayRight(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalLeftToRight(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalRightToLeft(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalNoAutoscroll(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalAutoscroll(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalSetRowOffsets(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalCreateChar(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalSetCursor(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalCommand(FrameHandlerArgs & args);
ErrorCode::Type handleLiquidCrystalWrite(FrameHandlerArgs & args);

ErrorCode::Type handleSpiGetConstants(FrameHandlerArgs & args);
ErrorCode::Type handleSpiBegin(FrameHandlerArgs & args);
ErrorCode::Type handleSpiEnd(FrameHandlerArgs & args);
ErrorCode::Type handleSpiBeginTransaction(FrameHandlerArgs & args);
ErrorCode::Type handleSpiEndTransaction(FrameHandlerArgs & args);
ErrorCode::Type handleSpiSetClockDivider(FrameHandlerArgs & args);
ErrorCode::Type handleSpiSetBitOrder(FrameHandlerArgs & args);
ErrorCode::Type handleSpiSetDataMode(FrameHandlerArgs & args);
ErrorCode::Type handleSpiTransfer(FrameHandlerArgs & args);
ErrorCode::Type handleSpiTransfer16(FrameHandlerArgs & args);
ErrorCode::Type handleSpiUsingInterrupt(FrameHandlerArgs & args);
ErrorCode::Type handleSpiNotUsingInterrupt(FrameHandlerArgs & args);

ErrorCode::Type handleWireBegin1(FrameHandlerArgs & args);
ErrorCode::Type handleWireBegin2(FrameHandlerArgs & args);
ErrorCode::Type handleWireEnd(FrameHandlerArgs & args);
ErrorCode::Type handleWireSetClock(FrameHandlerArgs & args);
ErrorCode::Type handleWireRequestFrom(FrameHandlerArgs & args);
ErrorCode::Type handleWireBeginTransmission(FrameHandlerArgs & args);
ErrorCode::Type handleWireEndTransmission(FrameHandlerArgs & args);
ErrorCode::Type handleWireAvailable(FrameHandlerArgs & args);
ErrorCode::Type handleWirePeek(FrameHandlerArgs & args);
ErrorCode::Type handleWireRead(FrameHandlerArgs & args);
ErrorCode::Type handleWireFlush(FrameHandlerArgs & args);
ErrorCode::Type handleWireWrite(FrameHandlerArgs & args);
ErrorCode::Type handleWireOnReceive(FrameHandlerArgs & args);
ErrorCode::Type handleWireOnRequest(FrameHandlerArgs & args);

void softReset(void);
void handleFrame(const uint8_t seq, uint8_t * buf, const size_t len, const bool err);
inline bool handleWrite(const uint8_t val, const bool eof);
void setup(void);
void loop(void);


#endif /* __ARDUINO_HPP__ */
