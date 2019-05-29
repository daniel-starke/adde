/**
 * @file GetPinMapA.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-02-27
 * @version 2019-04-10
 */
#include "arduino.hpp"


/**
 * Returns the analog I/O pin map (e.g. A0 -> 3).
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleGetPinMapA(FrameHandlerArgs & hal) {
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	framing.write(uint8_t(NUM_ANALOG_INPUTS));
#define WRITE_PIN(i, ...) framing.write(uint8_t(A##i));
	PP_REPEAT_TPL(NUM_ANALOG_INPUTS, WRITE_PIN)
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}
