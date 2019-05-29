/**
 * @file Reset.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-28
 * @version 2019-04-09
 */
#include "arduino.hpp"


/**
 * Resets the internal states.
 */
void softReset(void) {
	for (uint8_t i = 0; i < uint8_t(NUM_DIGITAL_PINS); i++) {
		const int val = digitalPinToInterrupt(i);
		if (val != NOT_AN_INTERRUPT) detachInterrupt(val);
		noTone(i);
		pinMode(i, INPUT);
		digitalWrite(i, LOW);
	}
#define X_RESET_TPL(i, data, arg) arg.reset();
	PP_EVAL(PP_VA_FOREACH(X_RESET_TPL, ~, HAS_SOFTWARESERIAL))
	PP_EVAL(PP_VA_FOREACH(X_RESET_TPL, ~, HAS_LCD))
}


/**
 * Resets the internal states.
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleReset(FrameHandlerArgs & /* hal */) {
	softReset();
	return ErrorCode::SUCCESS;
}
