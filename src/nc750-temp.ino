#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#include "CustomSoftwareSerial.h"

//#define  DEBUG_SIMULATION 1



/*#undef int
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>*/


#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))

// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3);


#define DEBUG_WITH_SOFTSERIAL 1




char buffer[64];
unsigned char bufferIndex = 0;

#define MAX_DATA_RECV_SIZE 64
byte bufferData[MAX_DATA_RECV_SIZE];


#define K_IN    10
#define K_OUT   13

#define SOFT_PIN_RX K_IN
#define SOFT_PIN_TX K_OUT

CustomSoftwareSerial mySerial(K_IN, K_OUT);

#define ISO_14230_fast 1


//P4: inter byte tester 5 - 20
#define ISORequestByteDelay 10

// P3: from ecu response to new request 55 - 5000
#define ISORequestDelay 60

#define READ_ATTEMPTS 125

/*
Keep alive:
From SDS: 80 12 F1 01 3E C2
From ECU: 80 F1 12 01 7E 02
*/

static byte KEEP_ALIVE[] = {0x80,0x12, 0xF1, 0x01, 0x3E, 0xC2};
#define KEEP_ALIVE_SIZE 6


#define REQUEST_STATUS_SIZE 7
static byte REQUEST_STATUS[REQUEST_STATUS_SIZE];

byte totalDataSize = 0;


byte ISO_InitStep = 0;  // Init is multistage, this is the counter
bool ECUconnection;  // Have we connected to the ECU or not


//FSM
uint8_t currentStatus;
enum STATUS { S_NOT_CONNECTED , S_REQUEST_DATA, S_IDLE };

uint8_t  currentGear = 0;
uint16_t currentRPM  = 0;
float    temperature = 0;
float    tempAir     = 0;

uint8_t  showScreenType = 0;

#define S1_BUTTON 9

//------------------

byte iso_checksum(byte *data, byte len)
{
  byte crc=0;
  for(byte i=0; i<len; i++)
  {
    crc=crc+data[i];
  }
  return crc;
}



void send_keepalive()
{
 
  mySerial.disableRX();
  for ( byte i = 0 ; i < KEEP_ALIVE_SIZE ; ++i)
  {
    
    mySerial.write( KEEP_ALIVE[i] );
    delay(ISORequestByteDelay);    
   
  }
  mySerial.enableRX();
  delay(ISORequestDelay);
  
  //Ignore response
  mySerial.flush();
  
}


bool request_data()
{
  
  
  totalDataSize          = 0;
  uint8_t i              = 0;

  byte d                 = 0;
  #ifdef DEBUG_WITH_SOFTSERIAL
     Serial.println("Req...");
  #endif
  
  /*display.clearDisplay();
  display.println("Request...");
  display.display();*/
  
  mySerial.disableRX();
  
  mySerial.write(REQUEST_STATUS[0]);
  delay(ISORequestByteDelay); 
  mySerial.write(REQUEST_STATUS[1]);
  delay(ISORequestByteDelay); 
  mySerial.write(REQUEST_STATUS[2]);
  delay(ISORequestByteDelay); 
  mySerial.write(REQUEST_STATUS[3]);
  delay(ISORequestByteDelay); 
  mySerial.write(REQUEST_STATUS[4]);
  delay(ISORequestByteDelay); 
  mySerial.write(REQUEST_STATUS[5]);
  delay(ISORequestByteDelay); 
  mySerial.write(REQUEST_STATUS[6]);
  delay(5); 
  mySerial.flush();
  mySerial.enableRX();
  
  /*display.clearDisplay();
  display.println("Waiting...");
  display.display();*/
  
  #ifdef DEBUG_WITH_SOFTSERIAL
     Serial.println("Wait...");
  #endif
  
  
  #ifdef DEBUG_SIMULATION
    for( i = 0 ; i < MAX_DATA_RECV_SIZE; ++i)
    {
      bufferData[i] = MAX_DATA_RECV_SIZE;
    }
    bufferData[26] = 4;
    bufferData[17]=0x1F; 
    bufferData[18]=0xFF;
    totalDataSize = i;
    return true;
  #endif
  
  while(mySerial.available() == 0)
  {

    //Wait for data...
  }
  
  i = 0;
  while(mySerial.available() > 0)
  {
     d = (byte) mySerial.read();
     bufferData[i] = d;
     i++;
     
     if( i > MAX_DATA_RECV_SIZE )
     {
       break;
     }
     
     #ifdef DEBUG_WITH_SOFTSERIAL
	Serial.print(" > ");
	Serial.println(d, HEX);
     #endif
     
    
  }
  totalDataSize = i;
  /*display.clearDisplay();
  display.println("Rev size: ");
  display.println(i);
  display.display();*/
  
  
  
  
  
  
  return true;
  
	
	
	
      

}


void my_iso_init(long baudrate)
{
  #ifdef DEBUG_WITH_SOFTSERIAL
       Serial.println(__FUNCTION__);
  #endif
  mySerial.end();
  delay(200);
  digitalWrite(K_OUT, HIGH);
  display.clearDisplay();
  display.println("Connecting...");
  display.println(baudrate);
  display.display();
  delay(6000);
  
  //Pulse!
  digitalWrite(K_OUT, LOW);
  delay(25);
  digitalWrite(K_OUT, HIGH);
  delay(25);

  //startCommunication Request 
  mySerial.begin(baudrate);
  mySerial.disableRX();
  byte dataStream[5];
  dataStream[0] = 0x81;
  dataStream[1] = 0x12;
  dataStream[2] = 0xf1; 
  dataStream[3] = 0x81;
  dataStream[4] = 0x05;
  mySerial.write(dataStream[0]);
  delay(ISORequestByteDelay); 
  mySerial.write(dataStream[1]);
  delay(ISORequestByteDelay); 
  mySerial.write(dataStream[2]);
  delay(ISORequestByteDelay); 
  mySerial.write(dataStream[3]);
  delay(ISORequestByteDelay); 
  mySerial.write(dataStream[4]);
  delay(ISORequestByteDelay); 
  mySerial.flush();
  mySerial.enableRX();
  
  //Wait 50ms for response
  delay(50);
  
  byte response[32];
  byte i = 0;
 
  #ifdef DEBUG_WITH_SOFTSERIAL
     Serial.print("Data available:");
     Serial.println(mySerial.available());
  #endif

  while(( mySerial.available() > 0 ) && (i < 31))
  {
    ECUconnection = true;
    response[i]    = (byte)mySerial.read();
    /*display.print(response[i]);
    display.print(" ");
    display.display();*/
    #ifdef DEBUG_WITH_SOFTSERIAL
      Serial.print(" ");
      Serial.print(response[i]);
    #endif
    
    ++i; 
    
    //P1: iter byte time in ECU response 0 - 20
    delay(10);
  }
  
  #ifdef DEBUG_WITH_SOFTSERIAL
    Serial.println("");
  #endif
  
  i = 0;
  if ( false == ECUconnection )
  {
    //display.clearDisplay();
    //display.println("Extra waiting...");
    #ifdef DEBUG_WITH_SOFTSERIAL
      Serial.println("Extra waiting");
    #endif
    //display.display();
    delay(5000);
    
    while(( mySerial.available() > 0 ) && (i < 31))
    {
      response[i] = (byte)mySerial.read();
      /*display.print(response[i]);
      display.print(" ");
      display.display();*/
      #ifdef DEBUG_WITH_SOFTSERIAL
	Serial.print(" ");
	Serial.print(response[i]);
      #endif
      ++i;
       delay(10);
       ECUconnection = true;
    }
  }
  
  
  Serial.println("");
  if ( true ==  ECUconnection )
  {
  
    ECUconnection = true;
    display.clearDisplay();
    display.println("Success!");
    display.display();
    #ifdef DEBUG_WITH_SOFTSERIAL
	Serial.println("Success");
    #endif
    currentStatus = S_REQUEST_DATA;
  }else
  {
    ECUconnection = false;
    display.clearDisplay();
    display.println("Error");
    display.display();
    #ifdef DEBUG_WITH_SOFTSERIAL
	Serial.println("Error connecting");
    #endif
  }
}

void start_connection()
{
  #ifdef DEBUG_WITH_SOFTSERIAL
     Serial.println("Trying to connect...");
  #endif
  
  display.clearDisplay();
  display.println("Trying to connect");
  display.display();
  delay(1000);
  
  
  long baudrate = 10400;
  //while((false == ECUconnection) && ( baudrate < 10403 ))
  while(false == ECUconnection) 
  {
    
    display.clearDisplay();
    display.print("Trying at ");
    display.println(baudrate);
    #ifdef DEBUG_WITH_SOFTSERIAL
	Serial.print("Trying at ");
	Serial.println(baudrate);
    #endif
    display.display();
    delay(1000);
    mySerial.flush();
    my_iso_init(baudrate);
   
    #ifdef DEBUG_SIMULATION
      ///////////NOOOOOOOOOOOOO
      ECUconnection = true;
    #endif
    
    if(false == ECUconnection)
    {
      display.clearDisplay();
      display.print("Error connecting at ");
      display.println(baudrate);
      #ifdef DEBUG_WITH_SOFTSERIAL
	  Serial.print("Error connecting at ");
	  Serial.println(baudrate);
      #endif
      display.display();
      delay(5000);
    }else
    {
      display.clearDisplay();
      display.println("Connected!");
      
      #ifdef DEBUG_WITH_SOFTSERIAL
	  Serial.println("Connected");
      #endif
      display.display();
      
      //FSM new status
      currentStatus = S_REQUEST_DATA;
      delay(200);
    }
    
    //++baudrate;
    
  }
}



void setup() 
{
  currentStatus =  S_NOT_CONNECTED;
  pinMode(S1_BUTTON, INPUT_PULLUP); 
  
  for(byte k = 0 ; k < MAX_DATA_RECV_SIZE ; ++k)
  {
    bufferData[k] = 0;
  }
  
  //Request status SDS

  
  REQUEST_STATUS[0] = 0x80; 
  REQUEST_STATUS[1] = 0x12; 
  REQUEST_STATUS[2] = 0xF1;
  REQUEST_STATUS[3] = 0x02;
  REQUEST_STATUS[4] = 0x21;
  REQUEST_STATUS[5] = 0x08;
  REQUEST_STATUS[6] = 0xAE;

  
  
  bool success;
  Serial.begin(9600);

  display.begin();
  // init done

  display.setContrast(50);
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1.1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  
  #ifdef DEBUG_SIMULATION
  display.clearDisplay();
  display.println("SIMULATION");
    display.println("*********");
  display.display();
  delay(5000);
  #endif
  start_connection();
  
  
  
}




void loop()
{
 

  uint8_t i              = 0;
  if ( HIGH == digitalRead(S1_BUTTON) )
  {
    showScreenType         = ~showScreenType;
  }
  

  
  
  if ( false == ECUconnection)
  {
    display.clearDisplay();
    display.println("Not connected");
    display.display();
    delay(5000);
    start_connection();
  }else
  {
    
    switch ( currentStatus )
    {
      case S_NOT_CONNECTED:
        display.clearDisplay();
        display.println("Not connected");
        display.display();
        
        delay(5000);
        my_iso_init(10400);

        /*if(false == ECUconnection)
        {
          
         display.clearDisplay();
         display.println("Trying 10401");
         display.display();
         delay(5000);
         my_iso_init(10401);    
          
        }
        if(false == ECUconnection)
        {
          
         display.clearDisplay();
         display.println("Trying 10402");
         display.display();
         delay(5000);
         my_iso_init(10402);    
          
        }*/
        
        
      break;
      
      case S_REQUEST_DATA:
        /*display.clearDisplay();
        display.println("Request data...");
        display.display();*/
        mySerial.flush();
        
        if ( true == request_data() )
        {
          currentStatus = S_IDLE;
        }
	
        
      break;
      
      case S_IDLE:
      
      
        if (totalDataSize > 30)
        {
          currentGear = bufferData[26];
          currentRPM  = (bufferData[17]*((unsigned int)256) + (unsigned int)bufferData[18]) / 4;
          temperature = (bufferData[21] - 48) / 1.6;
          tempAir     = (bufferData[22] - 48) / 1.6;
        }
        else
        {
          
        }
        
        if ( 0 == showScreenType )
        {
          //Display some data
          display.clearDisplay();
          display.setTextSize(3);
          //display.setTextColor(BLACK);
          display.setCursor(0,0);
          display.print("  ");
          display.println(currentGear);
          display.setTextSize(3);        
          display.println(currentRPM);
            
          display.display();
        }else
        {
          //temperatures
          //Display some data
          display.clearDisplay();
          display.setTextSize(2);
          //display.setTextColor(BLACK);
          display.setCursor(0,0);
          //display.print("R:");       
          display.println(temperature);
          display.setTextSize(2);        
          //display.print("A:");
          display.println(tempAir);
          display.display();
        }

        
        delay(200);
        //send_keepalive();
        //delay(50);
        currentStatus = S_REQUEST_DATA;
      break;
      
    }
    
    /*
       if (mySerial.available())
    Serial.write(mySerial.read());
  if (Serial.available())
    mySerial.write(Serial.read());
     
     */
    
    
  }
  
  

  
}


/*
 * 
0 80 Header byte
1 F1 Tester ID
2 12 ECU ID
3 34 Message length
4 61 Message type (Sensor data)
5 08
6 02
7 05
8 05
9 A0
10 17
11 69
12 A2
13 FF
14 FF
15 FF
16 00 Speed = byte 16 * 2 in km/h
17 00 RPM byte 1 
18 00 RPM byte 2: RPM = 10 * byte 17 +byte 18 / 10
19 37 Throttle position: 0x37 = 0%, 0xDD = 100%
20 B8
21 6B Engine Temperature = (byte 21 - 48) / 1.6 in Celsius degrees
22 61 Intake Air Temperature, same formula as above
23 B9
24 00
25 FF
26 00 Gear indicator 0-neutral; 1- 1st gear... a.s.o.
27 FF
28 5E
29 1F
30 FF
31 00
32 00
33 00
34 00
35 00
36 00
37 00
38 00
39 FF
40 FF
41 40
42 40
43 40
44 40
45 FF
46 1A
47 00
48 CB
49 1A
50 30
51 00
52 04 Clutch sensor (bit 8?): 0x04 = clutch released, 0x14 clutch "pressed"
53 00 Neutral sensor (bit 1?): 0x00 = neutral, 0x02 = in gear
54 FF
55 FF
56 07 Checksum

*/

