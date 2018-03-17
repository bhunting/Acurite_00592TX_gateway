
#pragma once
#ifndef __NRF24INF_H__
#define __NRF24INF_H__

#include <avr/pgmspace.h>
#include <SPI.h>			// SPI interface to radio and display
#include <Arduino.h>		// standard Arduino library
#include "printf.h"

#include <RF24Network.h>	// nRF24L01 radio support
#include <RF24.h>			// nRF24L01 radio support

void setupNRF();
void loopNRF();

#endif /* __NRF24INF_H__ */

