


#include "DISPLAY.h"
#include "A00592TX.h"

static const unsigned long display_update_interval = 1000; // ms       // Delay.
static unsigned long last_time_display_update;

/***********************************************************************/
static U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(/* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 6);

/***********************************************************************/
void setupU8X8(void)
{
	/* U8g2 Project: SSD1306 Test Board */
	u8x8.begin();
	u8x8.setPowerSave(0);
}

/***********************************************************************/
void display_sensordata_U8X8(void)
{
  char buffer[128/8+1];
  int i = 0;

  for( i = 0; i < _numSensors; i++ )
  {
    sprintf(buffer, "%1d %1X %3d %7d", 
                    sensorData[i].id,
                    sensorData[i].status,
                    sensorData[i].temperature,
                    sensorData[i].timestamp);
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0,i+2,buffer);
  }
}

/***********************************************************************/
void loopU8X8(void)
{
  static int clearSent = 0;
  
  unsigned long now = millis();                         // Send a ping to the next node every 'interval' ms
  if ( (now - last_time_display_update) >= display_update_interval )
  {
    last_time_display_update = now;
    switch( clearSent )
    {
      case 0:
      u8x8.clearDisplay();
      clearSent = 1;
      break;

      case 1:
      //u8x8.setFont(u8x8_font_chroma48medium8_r);
      //u8x8.drawString(0,0,"Hello World!");
      display_sensordata_U8X8();
      clearSent = 0;
      break;

      default:
      clearSent = 0;
      break;
    }
  }
}
