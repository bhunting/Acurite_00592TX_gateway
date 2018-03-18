

#pragma once
#ifndef __A00592TX_H__
#define __A00592TX_H__

#include <Arduino.h>
#include "printf.h"

//--------------------------------------------------------------------------
//-------- Data & Defines for Acurite Temperature Probe sniffing -----------
//--------------------------------------------------------------------------
// ring buffer size has to be large enough to fit data and sync signal,
//  at least 120 bytes, round up to 128 for now
#define RING_BUFFER_SIZE  128

// The pulse durations are the measured time in micro seconds between pulse edges.
#define SYNC_HIGH       600				// sync pulse high time
#define SYNC_LOW        600				// sync pulse low time
#define PULSE_LONG      400				// data bit long pulse time
#define PULSE_SHORT     220				// data bit short pulse time
#define BIT1_HIGH       PULSE_LONG		// 1 bit high time
#define BIT1_LOW        PULSE_SHORT		// 1 bit low time
#define BIT0_HIGH       PULSE_SHORT		// 0 bit high time
#define BIT0_LOW        PULSE_LONG		// 0 bit low time

// On the arduino connect the data pin, the pin that will be 
// toggling with the incomming data from the RF module, to
// digital pin 3. Pin D3 is interrupt 1 and can be configured
// for interrupt on change, change to high or low.
// The squelch pin in an input to the radio that squelches, or
// blanks the incoming data stream. Use the squelch pin to 
// stop the data stream and prevent interrupts between the 
// data packets if desired.
//
#define DATAPIN         (3)             // D3 is interrupt 1 (board layout specific)
#define SQUELCHPIN      (4)				// D4 is used to squelch data from 433 MHz radio
#define SYNCPULSECNT    (4)             // 4 pulses (8 edges)
#define SYNCPULSEEDGES  (SYNCPULSECNT*2)
#define DATABYTESCNT    (7)             // 7 bytes in the total data returned
#define DATABITSCNT     (DATABYTESCNT*8)// 7 bytes * 8 bits
#define DATABITSEDGES   (DATABITSCNT*2)

// The pulse durations are the measured time in micro seconds between pulse edges.
extern unsigned long pulseDurations[RING_BUFFER_SIZE];
extern unsigned int syncIndex;    	// index of the last bit time of the sync signal
extern unsigned int dataIndex;    	// index of the first bit time of the data bits (syncIndex+1)
extern bool         syncFound; 		// true if sync pulses found
extern bool         received; 		// true if sync plus enough bits found
extern unsigned int changeCount;	// count edges of data
extern const byte   interruptPin;

//--------------------------------------------------------------
//--- data structures for returning temperature data -----------
//--------------------------------------------------------------

// raw byte stream from sensors
typedef struct acurite_00592TX
{
    uint8_t     id_high;
    uint8_t     id_low;
    uint8_t     status;
    uint8_t     rsvd;
    uint8_t     temperature_high;
    uint8_t     temperature_low;
    uint8_t     crc;
} acurite_00592TX;
 
 // formatted data pulled from raw sensor data
typedef struct sensorTemperatureData
{
    uint8_t     id;             // sensor id (1 - N_sensors)
    uint8_t     status;         // 0x80 = BATTERY LOW bit, 0x40 = Data Fresh bit, 
    uint16_t    temperature;    // temperature value in C, no offset
    uint32_t    timestamp;      // number of seconds since startup
} sensorTemperatureData;

// number of sensor / data structures that _I_ have in my house
const uint8_t _numSensors = 6; // I happen to have 6 sensor probes

// The binary addresses assigned by the manufacturer for the sensors
// _I_ have in my house (change to your sensor addressses as needed)
const uint16_t SENSORID01 = 0x0C34;
const uint16_t SENSORID02 = 0x1E09;
const uint16_t SENSORID03 = 0x26ED;
const uint16_t SENSORID04 = 0x36E7;
const uint16_t SENSORID05 = 0x0604;
const uint16_t SENSORID06 = 0x386C;

// reduced sensor status
const uint8_t SENSOR_BATTERY_LOW_MASK = 0xC0;
const uint8_t SENSOR_BATTERY_LOW_VAL  = 0x80;
const uint8_t SENSOR_BATTERY_OK_VAL   = 0x40;
const uint8_t SENSOR_BATTERY_LOW      = 0x80;

// reduced sensor status
const uint8_t SENSOR_DATA_FRESH_MASK  = 0x40;
const uint8_t SENSOR_DATA_FRESH_VAL   = 0x40;
const uint8_t SENSOR_DATA_STALE_VAL   = 0x00;

const uint32_t SENSOR_STALE_DATA_TIMEOUT = 50; // seconds

extern struct sensorTemperatureData sensorData[ _numSensors ];

void PrintHex8(uint8_t *data, uint8_t length);
void setup592();
void loop592();

#endif /* __A00592TX_H__ */
