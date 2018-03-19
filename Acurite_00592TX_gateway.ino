
 
 // Include various headers for the nRF24L01 radio, SPI to interface with the display and nRF24L01 radio, and the OLED character formatting
#define SERIAL_DEBUG
//#include <avr/pgmspace.h>
//#include <SPI.h>			// SPI interface to radio and display
//#include "printf.h"
//#include <Arduino.h>		// standard Arduino library
#include "A00592TX.h"
#include "DISPLAY.h"
#include "NRF24INF.h"


/***********************************************************************/

/***********************************************************************/
void setup()
{
	Serial.begin(115200);
    printf_begin();
    printf_P(PSTR("\n\r A00592TX_GATEWAY \n\r"));

	SPI.begin();                                           // Bring up the RF network
	setupU8X8();
	setupNRF();
	setup592();
}

/***********************************************************************/
/***********************************************************************/
void loop()
{
	loop592();
	loopNRF();
	loopU8X8();
}

