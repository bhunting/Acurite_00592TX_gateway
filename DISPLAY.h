
#pragma once
#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <avr/pgmspace.h>
#include <SPI.h>			// SPI interface to radio and display
#include <Arduino.h>		// standard Arduino library
#include "printf.h"

#include <U8x8lib.h>		// character formatting for display
void setupU8X8(void);
void loopU8X8(void);

#endif /* __DISPLAY_H__ */

