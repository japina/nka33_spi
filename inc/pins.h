#ifndef __PINS_H
#define __PINS_H

#include "stm32f0xx.h"
#include "peripherals/stm32f0xx_rcc.h"
#include "peripherals/stm32f0xx_gpio.h"

#define HIGH 1
#define LOW 0
#define IN 1
#define OUT 0

/* list of pins */
enum pins_list{ A0=0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15,
		B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15, C13};

/* pinMode(int pinName, int dir)
   set mode of the pin (input, output)
   input: int pinName (A0 ... A15, B0 ... B15, C13), int dir (IN, OUT)
   output: returns 0 if success and -1 if error
*/
int8_t pinMode(uint8_t pinName, uint8_t dir);
/* digitalWrite(int pinName, int value)
   put pin to high or low value
   input: int pinName (A0 ... A15, B0 ... B15, C13), int value (HIGH, LOW)
   output: returns 0 if success and -1 if error
*/
int8_t digitalWrite(uint8_t pinName, uint8_t value);

/* digitalRead(int pinName)
   reads pin value
   input: int pinName (A0 ... A15, B0 ... B15, C13)
   output: returns pinValue (LOW, HIGH) if success and -1 if error
*/
int8_t digitalRead(uint8_t pinName);

#endif
