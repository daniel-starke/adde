/**
 * @file GetConstants.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-04-06
 * @version 2019-04-13
 */
#include "arduino.hpp"


/**
 * Returns the sizes for primitive data types and implementation specific predefined constants.
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleGetConstants(FrameHandlerArgs & hal) {
	static const uint8_t consts[] PCF_ROM = {
		uint8_t(sizeof(bool)),
		uint8_t(sizeof(signed char)),
		uint8_t(sizeof(unsigned char)),
		uint8_t(sizeof(signed short)),
		uint8_t(sizeof(unsigned short)),
		uint8_t(sizeof(signed int)),
		uint8_t(sizeof(unsigned int)),
		uint8_t(sizeof(signed long)),
		uint8_t(sizeof(unsigned long)),
		uint8_t(sizeof(signed long long)),
		uint8_t(sizeof(unsigned long long)),
		uint8_t(sizeof(float)),
		uint8_t(sizeof(double)),
		uint8_t(sizeof(long double)),
		uint8_t(sizeof(void *)),
		uint8_t(sizeof(ptrdiff_t)),
		uint8_t(sizeof(wchar_t)),
		uint8_t(sizeof(size_t)),
		uint8_t(HIGH),
		uint8_t(LOW),
		uint8_t(INPUT),
		uint8_t(INPUT_PULLUP),
		uint8_t(OUTPUT),
		uint8_t(SERIAL),
		uint8_t(DISPLAY),
		uint8_t(LSBFIRST),
		uint8_t(MSBFIRST),
		uint8_t(CHANGE),
		uint8_t(FALLING),
		uint8_t(RISING),
		uint8_t(DEFAULT),
		uint8_t(EXTERNAL),
		uint8_t(INTERNAL1V1),
		uint8_t(INTERNAL),
		uint8_t(INTERNAL2V56),
		uint8_t(INTERNAL2V56_EXTCAP),
		uint8_t(PP_VA_SIZE(HAS_HARDWARESERIAL)),
		uint8_t(PP_VA_SIZE(HAS_SOFTWARESERIAL)),
		uint8_t(PP_VA_SIZE(HAS_LCD)),
		uint8_t(PP_VA_SIZE(HAS_EEPROM)),
		uint8_t(PP_VA_SIZE(HAS_SPI)),
		uint8_t(PP_VA_SIZE(HAS_WIRE))
	};
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	for (uint8_t i = 0; i < sizeof(consts); i++) {
		framing.write(PCF_ROM_READ_U8(consts, i));
	}
	framing.write(uint32_t(F_CPU));
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}
