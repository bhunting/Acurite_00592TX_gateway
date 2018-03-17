/**********************************************************************
 * Arduino code to sniff the Acurite 00592TX wireless temperature
 * probe output data stream.
 *
 * Ideas on decoding protocol and prototype code from
 * Ray Wang (Rayshobby LLC) http://rayshobby.net/?p=8998
 *
 * Sniff the AcuRite model 00771W Indoor / Outdoor Thermometer
 * wireless data stream and display the results.
 * http://www.acurite.com/media/manuals/00754-instructions.pdf
 *
 * Code based on Ray Wang's humidity_display.ino source.
 * Heavily modified by Brad Hunting.
 *
 * The 00592TX wireless temperature probe contains a 433 MHz
 * wireless transmitter. The temperature from the probe is
 * sent approximately every 16 seconds.
 *
 * The 00592TX typically only sends one SYNC pulse + DATA stream
 * per temperature reading. Infrequently two sync/data streams
 * are sent during the same transmit window but that seems to 
 * be the exception.
 *
 * Ray Wang's code is for a different model of probe, one that 
 * transmits both temperature and humidity. Ray' code relies on 
 * two sync streams with a preceeding delay. 
 * 
 * The 00592TX usually starts the data sync bits right after
 * the RF sync pulses which are random length and polarity.
 *
 * Do not rely on a dead/mark time at the beginning of the 
 * data sync stream.
 *
 * The 00592TX first emits a seemingly random length string of 
 * random width hi/lo pulses, most like to provide radio
 * radio synchronization.
 *
 * The probe then emits 4 data sync pulses of approximately 50% 
 * duty cycle and 1.2 ms period. The sync pulses start with a 
 * high level and continue for 4 high / low pulses.
 *
 * The data bits immediately follow the fourth low of the data
 * sync pulses. Data bits are sent every ~0.61 msec as:
 *
 * 1 bit ~0.4 msec high followed by ~0.2 msec low
 * 0 bit ~0.2 msec high followed by ~0.4 msec low
 *
 * The 00592TX sends the 4 sync pulses followed by
 * 7 bytes of data equalling 56 bits.
 *
 * The code below works by receiving a level change interrupt 
 * on each changing edge of the data stream from the RF module
 * and recording the time in uSec between each edge.
 *
 * 8 measured hi and lo pulses in a row, 4 high and 4 low, of 
 * approximately 600 uSec each constitue a sync stream.
 *
 * The remaining 56 bits of data, or 112 edges, are measured
 * and converted to 1s and 0s by checking the high to low
 * pulse times.
 *
 * The first 4 pulses, or 8 edges, are the sync pulses followed
 * by the 56 bits, or 112 edges, of the data pulses.
 *
 * We measure 8 sync edges followed by 112 data edges so the 
 * time capture buffer needs to be at least 120 long.
 *
 * This code presently does not calculate the checksum of
 * the data stream. It simply displays the results of what was 
 * captured from the RF module.
 *
 * The data stream is 7 bytes long.
 * The first and second bytes are unique address bytes per probe.
 *   The upper two bits of the first byte are the probe channel indicator:
 *   11 = channel A
 *   10 = channel B
 *   00 = channel C
 *   The remaining 6 bits of the first byte and the 8 bits of the second
 *   byte are a unique identifier per probe.
 * The upper nybble of the third byte carries the remote probe low battery indication.
 *   When the remote probe batteries are fresh, voltage above 2.5V, the third byte is 0x44.
 *   When the remote probe batteries get low, below 2.4V, the third byte changes to 0x84.
 * The fourth byte continues to stay at 0x90 for all conditions.
 * The next two bytes are the temperature. The temperature is encoded as the
 *   lower 7 bits of both bytes with the most significant bit being an
 *   even parity bit.  The MSB will be set if required to insure an even
 *   number of bits are set to 1 in the byte. If the least significant
 *   seven bits have an even number of 1 bits set the MSB will be 0,
 *   otherwise the MSB will be set to 1 to insure an even number of bits.
 * The last byte is a simple running sum, modulo 256, of the previous 6 data bytes.
 */

/**
 * Example: Network topology, and pinging across a tree/mesh network
 *
 * Using this sketch, each node will send a ping to every other node in the network every few seconds. 
 * The RF24Network library will route the message across the mesh to the correct node.
 *
 * This sketch is greatly complicated by the fact that at startup time, each
 * node (including the base) has no clue what nodes are alive.  So,
 * each node builds an array of nodes it has heard about.  The base
 * periodically sends out its whole known list of nodes to everyone.
 *
 * To see the underlying frames being relayed, compile RF24Network with
 * #define SERIAL_DEBUG.
 *
 * Update: The logical node address of each node is set below, and are grouped in twos for demonstration.
 * Number 0 is the master node. Numbers 1-2 represent the 2nd layer in the tree (02,05).
 * Number 3 (012) is the first child of number 1 (02). Number 4 (015) is the first child of number 2.
 * Below that are children 5 (022) and 6 (025), and so on as shown below 
 * The tree below represents the possible network topology with the addresses defined lower down
 *
 *     Addresses/Topology                            Node Numbers  (To simplify address assignment in this demonstration)
 *             00                  - Master Node         ( 0 )
 *           02  05                - 1st Level children ( 1,2 )
 *    32 22 12    15 25 35 45    - 2nd Level children (7,5,3-4,6,8)
 *
 * eg:
 * For node 4 (Address 015) to contact node 1 (address 02), it will send through node 2 (address 05) which relays the payload
 * through the master (00), which sends it through to node 1 (02). This seems complicated, however, node 4 (015) can be a very
 * long way away from node 1 (02), with node 2 (05) bridging the gap between it and the master node.
 *
 * To use the sketch, upload it to two or more units and set the NODE_ADDRESS below. If configuring only a few
 * units, set the addresses to 0,1,3,5... to configure all nodes as children to each other. If using many nodes,
 * it is easiest just to increment the NODE_ADDRESS by 1 as the sketch is uploaded to each device.
 */
 
 // Include various headers for the nRF24L01 radio, SPI to interface with the display and nRF24L01 radio, and the OLED character formatting
#define SERIAL_DEBUG
#include <avr/pgmspace.h>
#include <RF24Network.h>	// nRF24L01 radio support
#include <RF24.h>			// nRF24L01 radio support
#include <SPI.h>			// SPI interface to radio and display
#include "printf.h"
#include <U8x8lib.h>		// character formatting for display
#include <Arduino.h>		// standard Arduino library

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
unsigned long pulseDurations[RING_BUFFER_SIZE];
unsigned int syncIndex  	= 0;    	// index of the last bit time of the sync signal
unsigned int dataIndex  	= 0;    	// index of the first bit time of the data bits (syncIndex+1)
bool         syncFound 		= false; 	// true if sync pulses found
bool         received  		= false; 	// true if sync plus enough bits found
unsigned int changeCount 	= 0;		// count edges of data
const byte   interruptPin 	= 3;

/*-------------------------------------------------------------------
 * ------------- PrintHex8() ----------------------------------------
 * helper code to print formatted hex 
 * prints 8-bit data in hex
 */
void PrintHex8(uint8_t *data, uint8_t length)
{
	char tmp[length*2+1];
	byte first;
	int j = 0;
	for (uint8_t i = 0; i < length; i++) 
	{
		first = (data[i] >> 4) | 48;
		if (first > 57) tmp[j] = first + (byte)39;
		else tmp[j] = first ;
		j++;

		first = (data[i] & 0x0F) | 48;
		if (first > 57) tmp[j] = first + (byte)39; 
		else tmp[j] = first;
		j++;
	}
	tmp[length*2] = 0;
	Serial.print(tmp);
}

/*-------------------------------------------------------------------
 * ------------------ isSync() --------------------------------------
 * Look for the sync pulse train, 4 high-low pulses of
 * 600 uS high and 600 uS low.
 * idx is index of last captured bit duration.
 * Search backwards 8 times looking for 4 pulses
 * approximately 600 uS long.
 */
bool isSync(unsigned int idx) 
{
   // check if we've received 4 pulses of matching timing
   for( int i = 0; i < SYNCPULSEEDGES; i += 2 )
   {
      unsigned long t1 = pulseDurations[(idx+RING_BUFFER_SIZE-i) % RING_BUFFER_SIZE];
      unsigned long t0 = pulseDurations[(idx+RING_BUFFER_SIZE-i-1) % RING_BUFFER_SIZE];    
      
      // any of the preceeding 8 pulses are out of bounds, short or long,
      // return false, no sync found
      if( t0<(SYNC_HIGH-100) || t0>(SYNC_HIGH+100) ||
          t1<(SYNC_LOW-100)  || t1>(SYNC_LOW+100) )
      {
         return false;
      }
   }
   return true;
}

//----------------------------- handler() ------------------------
/* Interrupt 1 handler 
 * Tied to pin 3 INT1 of arduino.
 * Set to interrupt on edge (level change) high or low transition.
 * Change the state of the Arduino LED (pin 13) on each interrupt. 
 * This allows scoping pin 13 to see the interrupt / data pulse train.
 */
void handler() 
{
   static unsigned long duration = 0;
   static unsigned long lastTime = 0;
   static unsigned int ringIndex = 0;
   static unsigned int syncCount = 0;
   static unsigned int bitState  = 0;

   bitState = digitalRead(DATAPIN);
   digitalWrite(13, bitState);

   // ignore if we haven't finished processing the previous 
   // received signal in the main loop.
   if( received == true )
   {
      return;
   }

   // calculating timing since last change
   long time = micros();
   duration = time - lastTime;
   lastTime = time;

   // Known error in bit stream is runt/short pulses.
   // If we ever get a really short, or really long, 
   // pulse we know there is an error in the bit stream
   // and should start over.
   if( (duration > (PULSE_LONG+100)) || (duration < (PULSE_SHORT-100)) )
   {
      received = false;
      syncFound = false;
      changeCount = 0;  // restart looking for data bits
   }

   // store data in ring buffer
   ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
   pulseDurations[ringIndex] = duration;
   changeCount++; // found another edge

   // detect sync signal
   if( isSync(ringIndex) )
   {
      syncFound = true;
      changeCount = 0;  // restart looking for data bits
      syncIndex = ringIndex;
      dataIndex = (syncIndex + 1)%RING_BUFFER_SIZE;
   }

   // If a sync has been found the start looking for the
   // DATABITSEDGES data bit edges.
   if( syncFound )
   {
      // if not enough bits yet, no message received yet
      if( changeCount < DATABITSEDGES )
      {
         received = false;
      }
      else if( changeCount > DATABITSEDGES )
      {
        // if too many bits received then reset and start over
         received = false;
         syncFound = false;
      }
      else
      {
         received = true;
      }
   }
}

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
typedef struct sensortemperatureData
{
    uint8_t     id;             // sensor id (1 - N_sensors)
    uint8_t     status;         // 0x80 = BATTERY LOW bit
    uint16_t    temperature;    // temperature value in C, no offset
    uint32_t    timestamp;      // number of seconds since startup
} sensortemperatureData;

// number of sensor / data structures that _I_ have in my house
static const uint8_t _numSensors = 6; // I happen to have 6 sensor probes

// The binary addresses assigned by the manufacturer for the sensors
// _I_ have in my house (change to your sensor addressses as needed)
static uint16_t probeIdArray[_numSensors] = 
{ 0x0C34, 0x1E09, 0x26ED, 0x36E7, 0x0604, 0x386C };

static uint8_t CRC = 0;

// reduced sensor status
static const uint8_t BATTERY_LOW_MASK = 0xC0;
static const uint8_t BATTERY_LOW_VAL  = 0x80;
static const uint8_t BATTERY_OK_VAL   = 0x40;
static const uint8_t BATTERY_LOW      = 0x80;

//--------------------------------------------------------------
//--------------------------------------------------------------
sensortemperatureData sensorData[ _numSensors ];


/***********************************************************************
************* Set the Node Address for NRF Network *********************
/***********************************************************************/

// These are the Octal addresses that will be assigned
const uint16_t node_address_set[10] = { 00, 02, 05, 012, 015, 022, 025, 032, 035, 045 };
 
// 0 = Master
// 1-2 (02,05)   = Children of Master(00)
// 3,5 (012,022) = Children of (02)
// 4,6 (015,025) = Children of (05)
// 7   (032)     = Child of (02)
// 8,9 (035,045) = Children of (05)

uint8_t NODE_ADDRESS = 1;  // Use numbers 0 through to select an address from the array

/***********************************************************************/
/***********************************************************************/
RF24 radio(9,10);                              // CE & CS pins to use (Using 7,8 on Uno,Nano)
RF24Network network(radio); 

uint16_t this_node;                           // Our node address

const unsigned long interval = 1000; // ms       // Delay manager to send pings regularly.
unsigned long last_time_sent;


const short max_active_nodes = 10;            // Array of nodes we are aware of
uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;


bool send_T(uint16_t to);                      // Prototypes for functions to send & handle messages
bool send_N(uint16_t to);
void handle_T(RF24NetworkHeader& header);
void handle_N(RF24NetworkHeader& header);
void add_node(uint16_t node);


/***********************************************************************/
U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(/* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 6);

const unsigned long display_update_interval = 1000; // ms       // Delay.
unsigned long last_time_display_update;

/***********************************************************************/
void setupNRF(){
  
  printf_begin();
  printf_P(PSTR("\n\rRF24Network/examples/meshping/\n\r"));

  this_node = node_address_set[NODE_ADDRESS];            // Which node are we?
  
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  network.begin(/*channel*/ 100, /*node address*/ this_node );

}

/***********************************************************************/
void setupU8X8(void)
{
  /* U8g2 Project: SSD1306 Test Board */
  u8x8.begin();
  u8x8.setPowerSave(0);
}


/***********************************************************************/
//---------------- setup() -------------------------------------------
void setup592()
{
   pinMode(DATAPIN, INPUT);             // data interrupt input
   attachInterrupt(digitalPinToInterrupt(interruptPin), handler, CHANGE);
   pinMode(SQUELCHPIN, OUTPUT);         // data squelch pin on radio module
   digitalWrite(SQUELCHPIN, HIGH);      // UN-squelch data

   memset( sensorData, 0, sizeof(sensorData));
  // pre-fill various values into the data structure.
  for(uint8_t i = 0; i < _numSensors; i++)
  {
    sensorData[i].id = i+1;
  }
}

/***********************************************************************/
void setup()
{
  Serial.begin(115200);

   pinMode(7, OUTPUT);     // OLED CS
   digitalWrite(7, HIGH);  // de-select OLED CS

   pinMode(8, OUTPUT);     // OLED DC
   digitalWrite(8, HIGH);  // set OLED DC

  
  SPI.begin();                                           // Bring up the RF network
  setupNRF();
  setupU8X8();
  setup592();
}

/***********************************************************************/
//---------------- convertTimingToBit() -------------------------------
/*
 * Convert pulse durations to bits.
 * 
 * 1 bit ~0.4 msec high followed by ~0.2 msec low
 * 0 bit ~0.2 msec high followed by ~0.4 msec low
 */
int convertTimingToBit(unsigned int t0, unsigned int t1) 
{
   if( t0 > (BIT1_HIGH-100) && t0 < (BIT1_HIGH+100) && t1 > (BIT1_LOW-100) && t1 < (BIT1_LOW+100) )
   {
      return 1;
   }
   else if( t0 > (BIT0_HIGH-100) && t0 < (BIT0_HIGH+100) && t1 > (BIT0_LOW-100) && t1 < (BIT0_LOW+100) )
   {
      return 0;
   }
   return -1;  // undefined
}

//#define PRINT_DATA_ARRAY
#define PRINT_NEW_DATA

/***********************************************************************/
//-------------------- loop592() ----------------------------------------------
/*
 * Main Loop
 * Wait for received to be true, meaning a sync stream plus
 * all of the data bit edges have been found.
 * Convert all of the pulse timings to bits and calculate
 * the results.
 */
void loop592()
{

   if( received == true )
   {
      // disable interrupt to avoid new data corrupting the buffer
      detachInterrupt(digitalPinToInterrupt(interruptPin));

      // convert bits to bytes
      unsigned int startIndex, stopIndex, ringIndex;
      uint8_t dataBytes[DATABYTESCNT];
      // clear the data bytes array
      for( int i = 0; i < DATABYTESCNT; i++ )
      {
        dataBytes[i] = 0;
      }
        
      ringIndex = (syncIndex+1)%RING_BUFFER_SIZE;

      for( int i = 0; i < DATABITSCNT; i++ )
      {
         int bit = convertTimingToBit( pulseDurations[ringIndex%RING_BUFFER_SIZE], 
                                       pulseDurations[(ringIndex+1)%RING_BUFFER_SIZE] );
                                       
         if( bit < 0 )
         {  
            Serial.println("Bit Timing : Decoding error.");
            // reset flags to allow next capture
            received = false;
            syncFound = false;
            // re-enable interrupt
            attachInterrupt(digitalPinToInterrupt(interruptPin), handler, CHANGE);
            return;      // exit due to error
         }
         else
         {
            dataBytes[i/8] |= bit << (7-(i%8));
         }
         
         ringIndex += 2;
      }

      // calculate CRC as sum of first 6 bytes
      CRC = 0; 
      for( int i = 0; i < DATABYTESCNT-1; i++ )
      {
        CRC += dataBytes[i]; 
      }            

      // overlay typed stucture over raw bytes 
      acurite_00592TX * acurite_data = (acurite_00592TX *)&dataBytes[0];

      // CRC ERROR in received data, ignore
      if( CRC != acurite_data->crc )
      {
            Serial.println("Sensor Data CRC : CRC error.");
            // reset flags to allow next capture
            received = false;
            syncFound = false;
            // re-enable interrupt
            attachInterrupt(digitalPinToInterrupt(interruptPin), handler, CHANGE);
            return;      // exit due to error
      }
      
      // fill in sensor data array
      uint16_t hexID = acurite_data->id_high * 256 + acurite_data->id_low;
      uint8_t  id = _numSensors+1; // preset to illegal

      // find which sensor id, search sendor id array
      for( int i = 0; i < _numSensors; i++ )
      {
          if( hexID == probeIdArray[i] )
          {
                id = i;
          }
      }

      if( id > _numSensors )
      {
            Serial.println("Sensor ID : out of bounds error.");
            // reset flags to allow next capture
            received = false;
            syncFound = false;
            // re-enable interrupt
            attachInterrupt(digitalPinToInterrupt(interruptPin), handler, CHANGE);
            return;      // exit due to error
      }

      sensorData[id].id = id+1;
      
      // check for a low battery indication
      if( (acurite_data->status & BATTERY_LOW_MASK) == BATTERY_LOW_VAL )
      {
         sensorData[id].status |= BATTERY_LOW;
      }
      else
      {
        sensorData[id].status &= ~BATTERY_LOW;
      }

      // extract temperature value
      uint16_t temperature = 0;
      sensorData[id].temperature = 0;
      
      // data bytes have already been decoded
      // 7 bits are significant, 8th bit is even parity bit
      // only shift by 7 bits because low byte is 7 bits of data
      temperature = ((acurite_data->temperature_high) & 0x7F) << 7;
      temperature += (acurite_data->temperature_low) & 0x7F;
      // temperature is offset by 1024 (= 0x400 = b0100 0000 0000)
      sensorData[id].temperature = (uint16_t)((temperature-1024)+0.5);
      sensorData[id].timestamp = millis() / 1000;  // convert milli-seconds into seconds
      
      
#ifdef PRINT_NEW_DATA
        Serial.print("id = ");
        Serial.print(sensorData[id].id);
        Serial.print(", status = ");
        Serial.print(sensorData[id].status, HEX);
        Serial.print(", temperature = ");
        Serial.print(sensorData[id].temperature);
        Serial.print(", time = ");
        Serial.println(sensorData[id].timestamp);
#endif // PRINT_NEW_DATA
      
#ifdef PRINT_DATA_ARRAY
      for( int i = 0; i < _numSensors; i++ )
      {
        Serial.print("id = ");
        Serial.print(sensorData[i].id);
        Serial.print(", status = ");
        Serial.print(sensorData[i].status, HEX);
        Serial.print(", temperature = ");
        Serial.print(sensorData[i].temperature);
        Serial.print(", time = ");
        Serial.println(sensorData[i].timestamp);
      }
#endif // PRINT_DATA_ARRAY

      // clear sensor data received flag to read next data      
      received = false;
      syncFound = false;

      // re-enable interrupt
      attachInterrupt(digitalPinToInterrupt(interruptPin), handler, CHANGE);
   } // new data received
} // loop592



/***********************************************************************/
void loopNRF(){
    
  network.update();                                      // Pump the network regularly

   while ( network.available() )  {                      // Is there anything ready for us?
     
    RF24NetworkHeader header;                            // If so, take a look at it
    network.peek(header);

    
      switch (header.type){                              // Dispatch the message to the correct handler.
        case 'T': handle_T(header); break;
        case 'N': handle_N(header); break;
        case 'A': handle_A(header); break;
        default:  printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"),header.type);
                  network.read(header,0,0);
                  break;
      };
    }

  
  unsigned long now = millis();                         // Send a ping to the next node every 'interval' ms
  if ( now - last_time_sent >= interval ){
    last_time_sent = now;


    uint16_t to = 00;                                   // Who should we send to? By default, send to base
    
    
    if ( num_active_nodes ){                            // Or if we have active nodes,
        to = active_nodes[next_ping_node_index++];      // Send to the next active node
        if ( next_ping_node_index > num_active_nodes ){ // Have we rolled over?
      next_ping_node_index = 0;                   // Next time start at the beginning
      to = 00;                                    // This time, send to node 00.
        }
    }

    bool ok;

    
    if ( this_node > 00 || to == 00 ){                    // Normal nodes send a 'T' ping
        ok = send_T(to);   
    }else{                                                // Base node sends the current active nodes out
        ok = send_N(to);
    }
    
    if (ok){                                              // Notify us of the result
        printf_P(PSTR("%lu: APP Send ok\n\r"),millis());
    }else{
        printf_P(PSTR("%lu: APP Send failed\n\r"),millis());
        last_time_sent -= 100;                            // Try sending at a different time next time
    }
  }


//  delay(50);                          // Delay to allow completion of any serial printing
//  if(!network.available()){
//      network.sleepNode(2,0);         // Sleep this node for 2 seconds or a payload is received (interrupt 0 triggered), whichever comes first
//  }
}

/***********************************************************************/
/**
 * Send a 'T' message, the current time
 */
bool send_T(uint16_t to)
{
  RF24NetworkHeader header(/*to node*/ to, /*type*/ 'T' /*Time*/);
  
  // The 'T' message that we send is just a ulong, containing the time
  unsigned long message = millis();
  printf_P(PSTR("---------------------------------\n\r"));
  printf_P(PSTR("%lu: APP Sending %lu to 0%o...\n\r"),millis(),message,to);
  return network.write(header,&message,sizeof(unsigned long));
}

/***********************************************************************/
/**
 * Send an 'N' message, the active node list
 */
bool send_N(uint16_t to)
{
  RF24NetworkHeader header(/*to node*/ to, /*type*/ 'N' /*Time*/);
  
  printf_P(PSTR("---------------------------------\n\r"));
  printf_P(PSTR("%lu: APP Sending active nodes to 0%o...\n\r"),millis(),to);
  return network.write(header,active_nodes,sizeof(active_nodes));
}

/***********************************************************************/
/**
 * Handle a 'A' message
 * 
 */
void handle_A(RF24NetworkHeader& header){

  unsigned long message;                                                                      // The 'T' message is just a ulong, containing the time
  network.read(header,&message,sizeof(unsigned long));
  printf_P(PSTR("%lu: APP Received %lu from 0%o\n\r"),millis(),message,header.from_node);

  RF24NetworkHeader rspHeader(/*to node*/ header.from_node, /*type*/ 'A' /*Time*/);
  
  printf_P(PSTR("---------------------------------\n\r"));
  printf_P(PSTR("ID     0x%X\n\r"), sensorData[0].id);
  printf_P(PSTR("STATUS 0x%X\n\r"), sensorData[0].status);
  printf_P(PSTR("TEMP   0x%x\n\r"), sensorData[0].temperature);
  printf_P(PSTR("TIME   0x%x\n\r"), sensorData[0].timestamp);

  printf_P(PSTR("Sending TEMP to 0%o...\n\r"),header.from_node);
  network.write(rspHeader,&sensorData[0],sizeof(sensortemperatureData));
}

/***********************************************************************/
/**
 * Handle a 'T' message
 * Add the node to the list of active nodes
 */
void handle_T(RF24NetworkHeader& header){

  unsigned long message;                                                                      // The 'T' message is just a ulong, containing the time
  network.read(header,&message,sizeof(unsigned long));
  printf_P(PSTR("%lu: APP Received %lu from 0%o\n\r"),millis(),message,header.from_node);


  if ( header.from_node != this_node || header.from_node > 00 )                                // If this message is from ourselves or the base, don't bother adding it to the active nodes.
    add_node(header.from_node);
}

/***********************************************************************/
/**
 * Handle an 'N' message, the active node list
 */
void handle_N(RF24NetworkHeader& header)
{
  static uint16_t incoming_nodes[max_active_nodes];

  network.read(header,&incoming_nodes,sizeof(incoming_nodes));
  printf_P(PSTR("%lu: APP Received nodes from 0%o\n\r"),millis(),header.from_node);

  int i = 0;
  while ( i < max_active_nodes && incoming_nodes[i] > 00 )
    add_node(incoming_nodes[i++]);
}

/***********************************************************************/
/**
 * Add a particular node to the current list of active nodes
 */
void add_node(uint16_t node){
  
  short i = num_active_nodes;                                    // Do we already know about this node?
  while (i--)  {
    if ( active_nodes[i] == node )
        break;
  }
  
  if ( i == -1 && num_active_nodes < max_active_nodes ){         // If not, add it to the table
      active_nodes[num_active_nodes++] = node; 
      printf_P(PSTR("%lu: APP Added 0%o to list of active nodes.\n\r"),millis(),node);
  }
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

/***********************************************************************/
/***********************************************************************/
void loop()
{
  loopNRF();
  loopU8X8();
  loop592();
}

