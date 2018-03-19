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
 
 #include "NRF24INF.h"
 #include "A00592TX.h"
 
/***********************************************************************
************* Set the Node Address for NRF Network *********************
/***********************************************************************/

// These are the Octal addresses that will be assigned
const uint16_t node_address_set[10] = { 00, 01, 02, 03, 04, 05, 011, 021, 031, 041 };
 
// 0 = Master
// 1-5 (01, 02, 03, 04, 05)   = Children of Master(00)

uint8_t NODE_ADDRESS = 1;  // Use numbers 0 through N to select an address from the array

/***********************************************************************/
/***********************************************************************/
static RF24 radio(9,10);                            // CE & CS pins to use (Using 7,8 on Uno,Nano)
static RF24Network network(radio); 

uint16_t this_node;                                 // Our node address

static const unsigned long interval = 1000;         // ms       
static unsigned long last_time_sent;                // Delay manager to send pings regularly.

static const short max_active_nodes = 10;           // Array of nodes we are aware of
static uint16_t active_nodes[max_active_nodes];
static short num_active_nodes = 0;
static short next_ping_node_index = 0;

// Prototypes for functions to send & handle messages
static bool send_Status (RF24NetworkHeader& cmdHeader); 
static bool send_Data   (RF24NetworkHeader& cmdHeader);
static void handle_A    (RF24NetworkHeader& cmdHeader);
static void add_node    (uint16_t node);

/***********************************************************************/
void setupNRF()
{
    //printf_begin();
    //printf_P(PSTR("\n\rRF24Network/examples/meshping/\n\r"));
    this_node = node_address_set[NODE_ADDRESS];            // Which node are we?
    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);
    network.begin(/*channel*/ 100, /*node address*/ this_node );
}

/***********************************************************************/
void loopNRF()
{    
    network.update();                           // Pump the network regularly
    while ( network.available() )               // Is there anything ready for us?
    {                    
        RF24NetworkHeader header;               // If so, take a look at it
        network.peek(header);
        switch (header.type)
        {                             // Dispatch the message to the correct handler.
            case 'S': send_Status(header); break;
            case 'D': send_Data(header); break;
            case 'A': handle_A(header); break;
            default:  printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"),header.type);
                      network.read(header,0,0);
                      break;
        };
    }  
}

/***********************************************************************/
/**
 * Send 'S' message, the current status
 */
bool send_Status(RF24NetworkHeader& cmdHeader)
{
    printf_P(PSTR("CMD: S : Received from 0%o\n\r"), cmdHeader.from_node);
    RF24NetworkHeader rspHeader(/*to node*/ cmdHeader.from_node, /*type*/ 'S' /*Status*/);
    
    // The 'S' message is status of all sensor nodes
    uint8_t statusArray[ _numSensors ];
    network.read(cmdHeader, 0, 0);

    for( int i = 0; i < _numSensors; i++ )
    {
        statusArray[i] = sensorData[i].status;
    }
    
    return network.write(rspHeader, statusArray, sizeof(statusArray));
}

/***********************************************************************/
/**
 * Send 'D' message, send data
 */
bool send_Data(RF24NetworkHeader& cmdHeader)
{
    printf_P(PSTR("CMD: D : Received from 0%o\n\r"), cmdHeader.from_node);
    RF24NetworkHeader rspHeader(/*to node*/ cmdHeader.from_node, /*type*/ 'D' /*Sensor Data*/);
    
    // The 'D' message is data from a sensor node
    uint8_t node_id;
    network.read(cmdHeader,&node_id,sizeof(node_id));
    
    if( (node_id > 0) && (node_id < (_numSensors+1)) )
    {
        return network.write(rspHeader, &sensorData[node_id-1], sizeof(sensorTemperatureData));
    }
}

/***********************************************************************/
/**
 * Handle a 'A' message
 */
void handle_A(RF24NetworkHeader& cmdHeader)
{
    printf_P(PSTR("CMD: A : Received from 0%o\n\r"), cmdHeader.from_node);
}

/***********************************************************************/
/**
 * Add a particular node to the current list of active nodes
 */
void add_node(uint16_t node)
{
  short i = num_active_nodes;            // Do we already know about this node?
  while (i--)  {
    if ( active_nodes[i] == node )
        break;
  }
  
  if ( i == -1 && num_active_nodes < max_active_nodes )
  {                                     // If not, add it to the table
      active_nodes[num_active_nodes++] = node; 
      printf_P(PSTR("%lu: APP Added 0%o to list of active nodes.\n\r"),millis(),node);
  }
}



 
 
