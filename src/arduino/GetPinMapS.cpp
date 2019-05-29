/**
 * @file GetPinMapS.cpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-02-28
 * @version 2019-04-08
 */
#include "arduino.hpp"


/**
 * Returns the special I/O pin map (e.g. SDA -> 3).
 * 
 * @param[in,out] hal - frame handler argument list
 * @return error code
 */
ErrorCode::Type handleGetPinMapS(FrameHandlerArgs & hal) {
	framing.beginTransmission(hal.seq);
	framing.write(uint8_t(ResultCode::RESULT));
	framing.write(uint8_t(hal.op));
	framing.write(uint8_t(LED_BUILTIN));
	framing.write(uint8_t(LED_BUILTIN_RX));
	framing.write(uint8_t(LED_BUILTIN_TX));
	framing.write(uint8_t(PIN_SPI_SS0));
	framing.write(uint8_t(PIN_SPI_SS1));
	framing.write(uint8_t(PIN_SPI_SS2));
	framing.write(uint8_t(PIN_SPI_SS3));
	framing.write(uint8_t(PIN_SPI_MOSI));
	framing.write(uint8_t(PIN_SPI_MISO));
	framing.write(uint8_t(PIN_SPI_SCK));
	framing.write(uint8_t(PIN_WIRE_SDA));
	framing.write(uint8_t(PIN_WIRE_SCL));
	framing.write(uint8_t(PIN_WIRE1_SDA));
	framing.write(uint8_t(PIN_WIRE1_SCL));
	framing.endTransmission();
	return ErrorCode::SUCCESS;
}
