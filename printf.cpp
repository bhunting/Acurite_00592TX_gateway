/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "printf.h"
#include <HardwareSerial.h>

/**
 * @file printf.c
 *
 * Setup necessary to direct stdout to the Arduino Serial library, which
 * enables 'printf'
 */

int serial_putc( char c, FILE * ) 
{
  Serial.write( c );
  return c;
} 

void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}




