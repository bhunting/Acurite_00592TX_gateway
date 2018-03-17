/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
 
/**
 * @file printf.h
 *
 * Setup necessary to direct stdout to the Arduino Serial library, which
 * enables 'printf'
 */

#pragma once
#ifndef __PRINTF_H__
#define __PRINTF_H__

#ifdef ARDUINO
#include <stdio.h>
//#include <Arduino.h>

int serial_putc( char c, FILE * ); 
void printf_begin(void);

#else
#error This example is only for use on Arduino.
#endif // ARDUINO

#endif // __PRINTF_H__
