/**
 * @file Protocol.hpp
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-02-20
 * @version 2019-04-15
 */
#ifndef __PROTOCOL_HPP__
#define __PROTOCOL_HPP__


/**
 * Changed if this file or any protocol related definition was changed. User specific changed which
 * do not change but just add functions can be expressed by the lower byte of this value.
 */
#define ADDE_PROT_VERSION 0x0100


#ifndef INVALID_PIN
#define INVALID_PIN (0xFF)
#endif /* INVALID_PIN */


/** Maximum number of different interrupt routines that can be defined via attachInterrupt(). */
#define MAX_NUM_INTERRUPTS 16


struct ErrorCode {
	/** Possible error codes in remote function call results. */
	enum Type {
		SUCCESS, /**< only used internally and not within the protocol */
		UNSUPPORTED_OPCODE,
		BROKEN_FRAME,
		SIGNATURE_MISMATCH,
		INVALID_ARGUMENT,
		COUNT
	};
};
	
struct ResultCode {
	/** Possible remote function call result types. */
	enum Type {
		RESULT,
		INTERRUPT,
		ERROR,
		COUNT
	};
};


struct OpCode {
	/**
	 * Possible remote function call operations codes.
	 * 
	 * @remarks RESET and GET_PROT_VERSION shall always be on the top in the given order to ensure
	 * protocol version downward compatibility. The first 7 values shall remain in the given order
	 * for upward compatibility.
	 */
	enum Type {
		RESET, /**< restart MCU */
		GET_PROT_VERSION, /**< return the protocol version; shall match ADDE_PROT_VERSION */
		GET_DEV_NAME, /**< return the device name or GUID v4 (see https://www.uuidgenerator.net/guid) as UTF-8 string */
		GET_CONSTANTS, /**< sizes for primitive data types and predefined implementation specific constants */
		GET_PIN_MAP_D, /**< digital, like D0 with mapping of ports */
		GET_PIN_MAP_A, /**< analog, like A0 */
		GET_PIN_MAP_S, /**< special, like SDA */
		GET_PIN_MAP_P, /**< PWM supported pins */
		GET_PIN_MAP_I, /**< pin to interrupt number */
		PIN_MODE,
		DIGITAL_READ,
		DIGITAL_WRITE,
		ANALOG_READ,
		ANALOG_REFERENCE,
		ANALOG_WRITE,
		ANALOG_READ_RESOLUTION,
		ANALOG_WRITE_RESOLUTION,
		PULSE_IN,
		PULSE_IN_LONG,
		SHIFT_IN,
		SHIFT_OUT,
		TONE,
		NO_TONE,
		ATTACH_INTERRUPT,
		DETACH_INTERRUPT,
		HARDWARE_SERIAL_GET_CONSTANTS,
		HARDWARE_SERIAL_AVAILABLE,
		HARDWARE_SERIAL_AVAILABLE_FOR_WRITE,
		HARDWARE_SERIAL_BEGIN1,
		HARDWARE_SERIAL_BEGIN2,
		HARDWARE_SERIAL_END,
		HARDWARE_SERIAL_PEEK,
		HARDWARE_SERIAL_READ,
		HARDWARE_SERIAL_FLUSH,
		HARDWARE_SERIAL_WRITE,
		SOFTWARE_SERIAL,
		SOFTWARE_SERIAL_AVAILABLE,
		SOFTWARE_SERIAL_BEGIN,
		SOFTWARE_SERIAL_END,
		SOFTWARE_SERIAL_IS_LISTENING,
		SOFTWARE_SERIAL_STOP_LISTENING,
		SOFTWARE_SERIAL_OVERFLOW,
		SOFTWARE_SERIAL_PEEK,
		SOFTWARE_SERIAL_READ,
		SOFTWARE_SERIAL_LISTEN,
		SOFTWARE_SERIAL_WRITE,
		EEPROM_LENGTH,
		EEPROM_READ,
		EEPROM_WRITE,
		EEPROM_UPDATE,
		LIQUID_CRYSTAL1,
		LIQUID_CRYSTAL2,
		LIQUID_CRYSTAL3,
		LIQUID_CRYSTAL4,
		LIQUID_CRYSTAL_BEGIN,
		LIQUID_CRYSTAL_CLEAR,
		LIQUID_CRYSTAL_HOME,
		LIQUID_CRYSTAL_NO_DISPLAY,
		LIQUID_CRYSTAL_DISPLAY,
		LIQUID_CRYSTAL_NO_BLINK,
		LIQUID_CRYSTAL_BLINK,
		LIQUID_CRYSTAL_NO_CURSOR,
		LIQUID_CRYSTAL_CURSOR,
		LIQUID_CRYSTAL_SCROLL_DISPLAY_LEFT,
		LIQUID_CRYSTAL_SCROLL_DISPLAY_RIGHT,
		LIQUID_CRYSTAL_LEFT_TO_RIGHT,
		LIQUID_CRYSTAL_RIGHT_TO_LEFT,
		LIQUID_CRYSTAL_NO_AUTOSCROLL,
		LIQUID_CRYSTAL_AUTOSCROLL,
		LIQUID_CRYSTAL_SET_ROW_OFFSETS,
		LIQUID_CRYSTAL_CREATE_CHAR,
		LIQUID_CRYSTAL_SET_CURSOR,
		LIQUID_CRYSTAL_COMMAND,
		LIQUID_CRYSTAL_WRITE,
		SPI_GET_CONSTANTS,
		SPI_BEGIN,
		SPI_END,
		SPI_BEGIN_TRANSACTION,
		SPI_END_TRANSACTION,
		SPI_SET_CLOCK_DIVIDER,
		SPI_SET_BIT_ORDER,
		SPI_SET_DATA_MODE,
		SPI_TRANSFER,
		SPI_TRANSFER16,
		SPI_USING_INTERRUPT,
		SPI_NOT_USING_INTERRUPT,
		WIRE_BEGIN1,
		WIRE_BEGIN2,
		WIRE_END,
		WIRE_SET_CLOCK,
		WIRE_REQUEST_FROM,
		WIRE_BEGIN_TRANSMISSION,
		WIRE_END_TRANSMISSION,
		WIRE_AVAILABLE,
		WIRE_PEEK,
		WIRE_READ,
		WIRE_FLUSH,
		WIRE_WRITE,
		WIRE_ON_RECEIVE,
		WIRE_ON_REQUEST,
		COUNT
	};
};


#endif /* __PROTOCOL_HPP__ */
