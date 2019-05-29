/**
 * @file pins_arduino.h
 * @author Daniel Starke
 * @copyright Copyright 2019 Daniel Starke
 * @date 2019-03-08
* @version 2019-04-22
 */
#ifndef __ADDE_PINS_ARDUINO_H__
#define __ADDE_PINS_ARDUINO_H__

#include "Arduino.h"


#ifdef __cplusplus
extern "C" {
#endif


extern uint8_t _NUM_DIGITAL_PINS;
extern uint8_t * _DIGITAL_PIN_TO_PORT;
extern uint8_t * _DIGITAL_PIN_TO_INTERRUPT;
extern bool * _DIGITAL_PIN_PWM_SUPPORT;
extern uint8_t _LED_BUILTIN;
extern uint8_t _LED_BUILTIN_RX;
extern uint8_t _LED_BUILTIN_TX;
extern uint8_t _PIN_SPI_SS0;
extern uint8_t _PIN_SPI_SS1;
extern uint8_t _PIN_SPI_SS2;
extern uint8_t _PIN_SPI_SS3;
extern uint8_t _PIN_SPI_MOSI;
extern uint8_t _PIN_SPI_MISO;
extern uint8_t _PIN_SPI_SCL;
extern uint8_t _PIN_WIRE_SDA;
extern uint8_t _PIN_WIRE_SCL;
extern uint8_t _PIN_WIRE1_SDA;
extern uint8_t _PIN_WIRE1_SCL;
extern uint8_t _NUM_ANALOG_INPUTS;
extern uint8_t _A0;
extern uint8_t _A1;
extern uint8_t _A2;
extern uint8_t _A3;
extern uint8_t _A4;
extern uint8_t _A5;
extern uint8_t _A6;
extern uint8_t _A7;
extern uint8_t _A8;
extern uint8_t _A9;
extern uint8_t _A10;
extern uint8_t _A11;
extern uint8_t _A12;
extern uint8_t _A13;
extern uint8_t _A14;
extern uint8_t _A15;
extern uint8_t _A16;
extern uint8_t _A17;
extern uint8_t _A18;
extern uint8_t _A19;
extern uint8_t _A20;
extern uint8_t _A21;
extern uint8_t _A22;
extern uint8_t _A23;
extern uint8_t _A24;
extern uint8_t _A25;
extern uint8_t _A26;
extern uint8_t _A27;
extern uint8_t _A28;
extern uint8_t _A29;
extern uint8_t _A30;
extern uint8_t _A31;
extern uint8_t _A32;
extern uint8_t _A33;
extern uint8_t _A34;
extern uint8_t _A35;
extern uint8_t _A36;
extern uint8_t _A37;
extern uint8_t _A38;
extern uint8_t _A39;
extern uint8_t _A40;
extern uint8_t _A41;
extern uint8_t _A42;
extern uint8_t _A43;
extern uint8_t _A44;
extern uint8_t _A45;
extern uint8_t _A46;
extern uint8_t _A47;
extern uint8_t _A48;
extern uint8_t _A49;
extern uint8_t _A50;
extern uint8_t _A51;
extern uint8_t _A52;
extern uint8_t _A53;
extern uint8_t _A54;
extern uint8_t _A55;
extern uint8_t _A56;
extern uint8_t _A57;
extern uint8_t _A58;
extern uint8_t _A59;
extern uint8_t _A60;
extern uint8_t _A61;
extern uint8_t _A62;
extern uint8_t _A63;


#define NUM_DIGITAL_PINS _NUM_DIGITAL_PINS
#define digitalPinToPort(x) ((_DIGITAL_PIN_TO_PORT == NULL) ? INVALID_PORT : _DIGITAL_PIN_TO_PORT[(x)])
#define digitalPinToInterrupt(x) ((_DIGITAL_PIN_TO_INTERRUPT == NULL) ? NOT_AN_INTERRUPT : _DIGITAL_PIN_TO_INTERRUPT[(x)])
#define digitalPinHasPWM(x) ((_DIGITAL_PIN_PWM_SUPPORT == NULL) ? false : _DIGITAL_PIN_PWM_SUPPORT[(x)])
#define LED_BUILTIN _LED_BUILTIN
#define LED_BUILTIN_RX _LED_BUILTIN_RX
#define LED_BUILTIN_TX _LED_BUILTIN_TX
#define PIN_SPI_SS0 _PIN_SPI_SS0
#define PIN_SPI_SS1 _PIN_SPI_SS1
#define PIN_SPI_SS2 _PIN_SPI_SS2
#define PIN_SPI_SS3 _PIN_SPI_SS3
#define PIN_SPI_MOSI _PIN_SPI_MOSI
#define PIN_SPI_MISO _PIN_SPI_MISO
#define PIN_SPI_SCL _PIN_SPI_SCL
#define PIN_WIRE_SDA _PIN_WIRE_SDA
#define PIN_WIRE_SCL _PIN_WIRE_SCL
#define PIN_WIRE1_SDA _PIN_WIRE1_SDA
#define PIN_WIRE1_SCL _PIN_WIRE1_SCL
#define NUM_ANALOG_INPUTS _NUM_ANALOG_INPUTS
#define A0  _A0 
#define A1  _A1 
#define A2  _A2 
#define A3  _A3 
#define A4  _A4 
#define A5  _A5 
#define A6  _A6 
#define A7  _A7 
#define A8  _A8 
#define A9  _A9 
#define A10 _A10
#define A11 _A11
#define A12 _A12
#define A13 _A13
#define A14 _A14
#define A15 _A15
#define A16 _A16
#define A17 _A17
#define A18 _A18
#define A19 _A19
#define A20 _A20
#define A21 _A21
#define A22 _A22
#define A23 _A23
#define A24 _A24
#define A25 _A25
#define A26 _A26
#define A27 _A27
#define A28 _A28
#define A29 _A29
#define A30 _A30
#define A31 _A31
#define A32 _A32
#define A33 _A33
#define A34 _A34
#define A35 _A35
#define A36 _A36
#define A37 _A37
#define A38 _A38
#define A39 _A39
#define A40 _A40
#define A41 _A41
#define A42 _A42
#define A43 _A43
#define A44 _A44
#define A45 _A45
#define A46 _A46
#define A47 _A47
#define A48 _A48
#define A49 _A49
#define A50 _A50
#define A51 _A51
#define A52 _A52
#define A53 _A53
#define A54 _A54
#define A55 _A55
#define A56 _A56
#define A57 _A57
#define A58 _A58
#define A59 _A59
#define A60 _A60
#define A61 _A61
#define A62 _A62
#define A63 _A63


#define SERIAL_PORT_MONITOR Serial
#define SERIAL_PORT_HARDWARE Serial


#ifdef __cplusplus
} /* extern "C" */
#endif


#endif /* __ADDE_PINS_ARDUINO_H__ */
