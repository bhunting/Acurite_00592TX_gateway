
#include "DISPLAY.h"
#include "A00592TX.h"

static const unsigned long display_update_interval = 1000; // ms       // Delay.
static unsigned long last_time_display_update;

/***********************************************************************/
static U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(/* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 6);

/***********************************************************************/
void setupU8X8(void)
{
    pinMode(7, OUTPUT);     // OLED CS
    digitalWrite(7, HIGH);  // de-select OLED CS
    pinMode(8, OUTPUT);     // OLED DC
    digitalWrite(8, HIGH);  // set OLED DC
    
    /* U8g2 Project: SSD1306 Test Board */
    u8x8.begin();
    u8x8.setPowerSave(0);
}

/***********************************************************************/
void display_sensorstatus_U8X8(void)
{
    char buffer[128/8+1];  // 128 pixels across / 8 pixels per character plus a trailing null
    // OLED 1306 display is 128 wide by 64 pixels tall. Using 8x8 characters allows
    // 16 character spaces across and 8 characters tall.
    // The top 16 pixels are yellow masked, the lower 48 are blue masked.
    // Using the yellow mask area for status and writing to the 48 blue
    // pixels allows for 6 rows of 8 pixel tall characters.

    u8x8.setFont(u8x8_font_chroma48medium8_r);
    int i = 0;
    for( i = 0; i < _numSensors; i++ )
    {
        if( (sensorData[i].status & SENSOR_DATA_FRESH_MASK) == SENSOR_DATA_FRESH_VAL )
        {
            u8x8.setInverseFont(true);
            buffer[0] = '1'+i;
        }
        else
        {
            u8x8.setInverseFont(false);
            buffer[0] = ' ';
        }
        buffer[1] = 0;      // trailing null 
        u8x8.drawString(i,0,buffer);
    }
    u8x8.setInverseFont(false);
}

/***********************************************************************/
void display_sensordata_U8X8(void)
{
    char buffer[128/8+1];  // 128 pixels across / 8 pixels per character plus a trailing null
    // OLED 1306 display is 128 wide by 64 pixels tall. Using 8x8 characters allows
    // 16 character spaces across and 8 characters tall.
    // The top 16 pixels are yellow masked, the lower 48 are blue masked.
    // Using the yellow mask area for status and writing to the 48 blue
    // pixels allows for 6 rows of 8 pixel tall characters.
    int i = 0;

    int write_rows = min( 48/8, _numSensors);
    for( i = 0; i < write_rows; i++ )
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
      //u8x8.clearDisplay();
      clearSent = 1;
      break;

      case 1:
      //u8x8.setFont(u8x8_font_chroma48medium8_r);
      //u8x8.drawString(0,0,"Hello World!");
      display_sensordata_U8X8();
      display_sensorstatus_U8X8();
      clearSent = 0;
      break;

      default:
      clearSent = 0;
      break;
    }
  }
}
