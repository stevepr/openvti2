/*

OpenVTI

This is VERSION 5 - reworked by SBP from Version 4 by MichaelF

ChangeLog

Version 1 
- initial using SoftwareSerial for GPS comms

Version 2 
- switch to HardwareSerial
- implemented field detection using VSYNC/HSYNC outputs from MAX7456 using external 555 or ATTINY13

Version 3
- use "arduino only" solution for field detection
- final version has working limovie time insertion

Version 4
- start with working limovie compatible time insertion
- move LED handler into VSYNC pin change interrupt handler
- make display more sparse to provide more area for stars!

Version 5
- interrupt driven PPS
- update OSD during vertical blanking
- write OSD update directly using auto-increment mode
- added code for parsing GPS serial data
- enabled  XXXX ublox proprietary sentence

VTI for following setup:

TinySine Video Overlay using Max-7456
Gowoops GPS Module U-blox NEO-6M
  (Amazon: https://www.amazon.com/Gowoops-Module-Antenna-Arduino-Microcomputer/dp/B01MRNN3YZ)

Use 3.3V power for GPS.

Configure ublox GPS to the following:

- output GPGGA and PUBX,04 NMEA messages
- Timepulse set to UTC time (defaults to GPS)


LIBRARY NOTES:

I decided to start including library files in sketch directory - this way I can keep it self contained

MAX7456

I included zip files as well for original library
      
*/

// Included Libraries //////////////////////////////////////////////////////////

#include "pinconfig.h"
#include <SPI.h>
#include "MAX7456.h"

#define VERSION  5

typedef enum CountSource
{
  TCNT,
  ICR
};

// Global Constants & Variables ////////////////////////////////////////////////////////////
const unsigned long gpsBaud = 9600;       

// Startup Mode
//
bool StartupExec;       // true => run , false => programming (don't do anything this time)
  
// GPS
// we are going to use the hardware serial port for gps
bool gpsStatus = false;

// OSD object
//
MAX7456 OSD( osdChipSelect );

// location of UI elements
#define TOP_ROW 1
//#define BOTTOM_ROW  12
#define BOTTOM_ROW  11      // ***for testing

// solid character that blinks each PPS we receive
#define PPSCHAR_COL 1
#define PPSCHAR_ROW (BOTTOM_ROW)

#define VSYNCCHAR_COL 0
#define VSYNCCHAR_ROW (BOTTOM_ROW)

#define FIELD1TS_COL 12
#define FIELD1TS_ROW BOTTOM_ROW

#define FIELD2TS_COL 17
#define FIELD2TS_ROW BOTTOM_ROW

#define FIELDTOT_COL 22
#define FIELDTOT_ROW BOTTOM_ROW

#define FIX_ROW TOP_ROW
#define FIX_COL 1

#define CYCLE_ROW TOP_ROW
#define CYCLE_COL 4

#define TIME_ROW BOTTOM_ROW
#define TIME_COL 3

// pixel offset of display    
#define OSD_X_OFFSET  0
#define OSD_Y_OFFSET  0

//************
// Video info
//
volatile bool EnableOverlay = false;
volatile unsigned long fieldCount;

volatile int fieldParity = 0;            // 1=> field 1, 2=> field 2  (0 => not yet set)
bool blnPE1 = false;
bool blnPE2 = false;

volatile unsigned long tk_VSYNC;      // vsync "time" = 2mhz ticks
volatile unsigned long tk_HSYNC;      // hsync "time" = 2mhz ticks

//***********************
//  Time
//
volatile unsigned short timer1_ov;    // timer 1 overflow count = high word of "time" (32ms per overflow)
#define Timer_Second 2000000          // timer1 is running at about 2mhz
#define Timer_Milli 2000              // approx ticks per millisecond

volatile unsigned long tk_PPS;        // tick "time" of most recent PPS int
volatile bool pps_now = false;

//************
//  OSD rows
//
char TopRow[30];
uint16_t TopRow_Addr = 30;            // Top Row is row 1

char BottomRow[30];
uint16_t BottomRow_Addr = 9 * 30;    // Top Row is row 11 (good for NTSC)

//========================================
//  SETUP ROUTINE
//========================================
void setup() {

  //**************
  //  check startup mode
  //
#if 1
  STARTUP_CFG_INPUT();
  if (STARTUP_READ())
  {
    StartupExec = true;
  }
  else
  {
    // just programming - leave now
    StartupExec = false;
    return;
  }
#endif

  //************
  //  General init
  //
  EnableOverlay = false;      // turn off overlay update for now...
  fieldCount = 0;
  for (int i = 0; i< 30; i++) 
  {
    TopRow[i] = 0x00;
    BottomRow[i] = 0x00;
  }

  //************
  // Init GPS
  //
#if 1
  gpsStatus = gpsInit();
#endif

  //************
  //  Init OSD
  //
  
  // Initialize the SPI connection:
  //
  SPI.begin();
  SPI.setClockDivider( SPI_CLOCK_DIV2 );      // Must be less than 10MHz.
  
  // Initialize the MAX7456 OSD:
  //
  uint8_t rows = OSD.safeVideoRows[MAX7456_NTSC][MAX7456_FULLSCREEN];
  uint8_t cols = OSD.safeVideoCols[MAX7456_NTSC][MAX7456_FULLSCREEN];    
  OSD.begin();                // Use NTSC with full area.
  OSD.setDefaultSystem(MAX7456_NTSC) ;
  OSD.setTextArea(rows, cols, MAX7456_FULLSCREEN);
  //OSD.setSyncSource(MAX7456_EXTSYNC);
  OSD.setSyncSource(MAX7456_AUTOSYNC);
  OSD.setWhiteLevel(0);  // should be 0% black 120% white

  // align with IOTA-VTI
  OSD.setTextOffset(OSD_X_OFFSET, OSD_Y_OFFSET);
  
  OSD.display();                              // Activate the text display.

  //
  // wait for VSYNC to start
  while (OSD.notInVSync());                   // Wait for VSync to start

  //*************
  // setup interrupt sources
  //  * PPS
  //  * VSYNC
  //  * HSYNC
  //  

  // PPS
  // Connected to INT1 interrupt as rising edge trigger
  //
  PPS_CFG_INPUT();
  PPS_CFG_EICRA();
  PPS_CFG_PCMSK();
  PPS_CFG_EIMSK();
  
  
  // HSYNC
  //  connected to Timer1 ICP falling edge (start of horizontal blanking)
  //  enable Hsync first to generate these times before vsync starts up
  //
  tk_HSYNC = 0;
  HSYNC_CFG_INPUT();
  TIMSK1 = 0;                               // disable all timer 1 ints
  TCCR1A = 0;                               // normal mode
  TCCR1B = (1 << CS11);                     // falling edge ICP & 2mhz
  TCCR1C = 0;                               // normal mode
  
  TCNT1 = 0;                                // reset count
  TIFR1 = 0;                                // reset any pending ints
  PRR &= ~(1 << PRTIM1);                    // turn on timer 1
  timer1_ov = 0;                            // reset overflow count
  TIMSK1 = (1 << ICIE1) | (1 << TOIE1);     // enable ICP & overflow

  // wait 50ms before tracking VSYNC
  //
  delay(50);
  
  // VSYNC
  //
  tk_VSYNC = 0;
  VSYNC_CFG_INPUT();
  VSYNC_CFG_EICRA();
  VSYNC_CFG_PCMSK();
  VSYNC_CFG_EIMSK();

  // reset error flags
  //
  delay(50);
  blnPE1 = false;
  blnPE2 = false;
  
  EnableOverlay = true;   // start the overlay...

 
} // end of setup

//========================================
// LOOP ROUTINE
//========================================
void loop() {

  while (true)
  {
    // do nothing for now
    //
  }

} // end of loop()

//*****************************************************************************************
// ISR routines
//*****************************************************************************************

//===============================
//  VSYNC ISR
//===============================
VSYNC_ISR()
{
  volatile int prevParity;
  bool ParityError;
  unsigned long timeDiff;
  unsigned long timePrev;
  
  // get time
  //
  timePrev = tk_VSYNC;
  tk_VSYNC = GetTicks(TCNT);
  

  //**********
  //  write overlay?
  //
  if (!EnableOverlay)
  {
    return;   // nope, leave now...
  }

  // increment field count
  //
  fieldCount++;

  // Field parity detection
  //    Field 1:
  //      prev HSYNC (start of blanking) should be "close" to this VSYNC time
  //        - must check for case where HSYNC happens just AFTER VSYNC (assume 64us period)
  //
  if (tk_VSYNC >= tk_HSYNC)
  {
    timeDiff = tk_VSYNC - tk_HSYNC;
  }
  else
  {
    timeDiff = 0 - (tk_HSYNC - tk_VSYNC);
  }

  prevParity = fieldParity;
  if ((timeDiff < 40) || (timeDiff > 100))    // < 20ms or > 50ms
  {
    fieldParity = 1;
  }
  else
  {
    fieldParity = 2;
  }

  if (fieldParity == prevParity)
  {
    ParityError = true;
    if (fieldParity == 1)
    {
      blnPE1 = true;
    }
    else
    {
      blnPE2 = true;
    }
  }
  else
  {
    ParityError = false;
  }

  //**************
  //  determine field time delay from latest PPS
  //
  if (tk_VSYNC >= tk_PPS)
  {
    timeDiff = tk_VSYNC - tk_PPS;
  }
  else
  {
    timeDiff = 0 - (tk_PPS - tk_VSYNC);
  }

  // convert ticks to 0.1 ms
  //
  timeDiff /= (Timer_Milli / 10);

  //**********************
  //  update display
  //

  if (!gpsStatus)
  {
    BottomRow[1] = gpsStatus;    // flax GPS fail with 'X'
  }

  // pps test...
  //
  if (pps_now)
  {
    BottomRow[0] = 0xFA;
    pps_now = false;
  }
  else
  {
    BottomRow[0] = 0x00;
  }

  // field parity
  //
  if (fieldParity == 1)
  {
    TopRow[3] = 0x01;
    TopRow[4] = 0x00;
  }
  else
  {
    TopRow[3] = 0x00;
    TopRow[4] = 0x02;
  }

  TopRow[5] = (blnPE1? 0x22 : 0x00);
  TopRow[6] = (blnPE2? 0x23 : 0x00);
  
  // field count
  //
  ultodec(BottomRow + FIELDTOT_COL,fieldCount,-6);

  // field time
  //
  if (fieldParity == 1)
  {
    ultodec(BottomRow + FIELD1TS_COL, timeDiff, -4);  // write field 1 stamp
    for (int i = 0; i < 4; i++)  // clear field 2 stamp
    {
      BottomRow[FIELD2TS_COL + i] = 0x00;
    }
  }
  else
  {
    ultodec(BottomRow + FIELD2TS_COL, timeDiff, -4);  // write field 2 stamp
    for (int i = 0; i < 4; i++)  // clear field 1 stamp
    {
      BottomRow[FIELD1TS_COL + i] = 0x00;
    }
  }
  
  //***
  //  ENABLE interrupts again
  //
  interrupts();

  // update display
  //
  OSD.sendArray(TOP_ROW*30,TopRow,30);
  OSD.sendArray(BOTTOM_ROW*30,BottomRow,30);

 
} // end of VSYNC_ISR

//===============================
//  HSYNC ISR
//===============================
HSYNC_ISR()
{
  unsigned long timeCurrent;
  unsigned long timePrev;
  unsigned long timeDiff;

  // get the HSYNC time from the input capture register
  //   falling edge => start of Horizontal blanking
  //
  timeCurrent = GetTicks(ICR);

  // 
  //
  timePrev = tk_HSYNC;
  tk_HSYNC = timeCurrent;


} // end of HSYNC_ISR

//===============================
//  PPS ISR
//===============================
PPS_ISR()
{
  unsigned long timeCurrent;
  unsigned long timePrev;
  unsigned long timeDiff;

  // get the HSYNC time from the input capture register
  //   falling edge => start of Horizontal blanking
  //
  timeCurrent = GetTicks(TCNT);

  // 
  //
  timePrev = tk_PPS;
  tk_PPS = timeCurrent;

  //
  pps_now = true;
  
} // end of PPS_ISR

//========================================
// ISR for Timer 1 overflow
//=========================================
ISR( TIMER1_OVF_vect)
{
  
  timer1_ov++;   // just increment overflow count
  
} // end of Timer1 overflow vector

//========================================
// GetTicks() - get timer tick count from either TCNT or ICP
//    *** call with interrupts OFF
//=========================================
unsigned long GetTicks(CountSource src)
{
  unsigned long tickCount;

  if (src == TCNT)
  {
    tickCount = ((unsigned long)timer1_ov << 16) + (unsigned long)TCNT1;
  }
  else
  {
    tickCount = ((unsigned long)timer1_ov << 16) + (unsigned long)ICR1;
  }
  
  // watch for overflow pending
  //
  if ( TIFR1 & (1 << TOV1) )
  {
    // overflow pending
    //  ignore it if the counter number is close to overflow
    //

    if ((tickCount & 0xFFFF) < 0xF000)
    {
      tickCount += 0x00010000;    // add in pending overflow    
    }
  }

  return( tickCount );

} // end of GetTicks()

//*****************************************************************************************
// Utility routines
//*****************************************************************************************

//===========================================================================
// ultohex - convert unsigned long to 8 hex MAX7456 characters in a character array
//
//===========================================================================
void ultohex(char *dest, unsigned long ul)
{

  char hex[16] = {0x0A,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0B,0x0C,0x0D,0x0E,0x0F,0x10};

  char *pn;
  unsigned long nibble;

  pn= dest + 7;
  
  for(int i = 0; i < 8; i++)
  {

    // get nibble 
    //
    nibble = (ul & 0xF);
    *pn = hex[nibble];

    // move to next nibble
    //
    ul = ul >> 4;
    pn--;
    
  } // end of for loop through the nibbles
  
} // end of ultohex

//===========================================================================
// ultodec - convert unsigned long to decimal MAX7456 characters in a character array
//    dest = ptr to destination of most significant char of decimal number
//    ul = unsigned long value to convert
//    len = # of chars to write in destination (pos => leading zeros, negative => leading spaces) 
//===========================================================================
void ultodec(char *dest, unsigned long ul, int len)
{

  char dec[10] = {0x0A,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};

  char *pn;

  int pad;
  unsigned long divisor;
  unsigned short remainder;
  
  unsigned long nibble;

  pad = abs(len);   // # of chars to write
  
  pn= dest + (pad - 1);

  for(int i = 0; i < pad; i++)
  {

    // get least significan number/char
    //
    divisor = ul/10;
    remainder = ul - (divisor * 10);
    ul = divisor;
    
    // set char & move
    //
    if ((remainder == 0) && (ul == 0) && (len > 0))
    {
      // we have run out of digits, time to pad with spaces now
      //
      *pn = 0x00;
    }
    else
    {
      *pn = dec[remainder];
    }
    pn--;
    
  } // end of for loop through the nibbles
  
} // end of ultodec

//**********************************************************************************************************
// NEO 6 GPS routines
//**********************************************************************************************************
#define gpsSerial Serial
#define gps_OK        0
#define gps_E_RMC     1
#define gps_E_GGA     2
#define gps_E_DTM     3
#define gps_E_PUBX04  4
#define gps_E_CFGTP   5

//=================================================
//  gpsInit - initialize GPS device
//
//  This version is for the UBX NEO-6 gps module
//    * uses millis() system call for timeout detection
//    * assumes NEO-6 boots to 9600 baud
//    * initializes NEO-6 to output the following SENTENCES on the serial port
//        $GPRMC
//        $GPGGA
//        $GPDTM
//        $PUBX,04
//    
//=================================================
int gpsInit()
{
  uint8_t enableRMC[] = {0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01};       // Set GPRMC rate on current target
  uint8_t enableGGA[] = {0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01};       // Set GPGGA rate on current target
  uint8_t enableDTM[] = {0x06, 0x01, 0x03, 0x00, 0xF0, 0x0A, 0x01};       // Set GPDTM rate on current target
  uint8_t enablePUBX04[] = {0x06, 0x01, 0x03, 0x00, 0xF1, 0x04, 0x01};    // Set PUBX,04 rate on current target

  uint8_t configTimepulse[] = {0x06, 0x07, 0x14, 0x00,        //   configure timepulse
                          0x40, 0x42, 0x0F, 0x00,             // time interval = 1,000,000 us
                          0xA0, 0x86, 0x01, 0x00,             // pulse length = 100,000 us = 100ms
                          0x01,                               // positive pulse
                          0x00,                               // align to UTC
                          0x00,                               // time pulse only when sync'd to valid GPS
                          0x00,                               // reserved
                          0x00, 0x00,                         // Antenna cable delay
                          0x00, 0x00,                         // Receiver RF group delay
                          0x00, 0x00, 0x00, 0x00              // user time function delay
                          };
                          
  // 9600 NMEA is the default rate for the GPS
  //
  gpsSerial.begin(9600);
  
  //********************************
  //  TURN OFF everything to keep the serial port quiet
  //
  gpsSerial.println("$PUBX,40,RMC,0,0,0,0,0,0*46");   // RMC OFF
  gpsSerial.println("$PUBX,40,GGA,0,0,0,0,0,0*5A");   // GGA OFF
  gpsSerial.println("$PUBX,40,DTM,0,0,0,0,0,0*46");   // DTM OFF
  gpsSerial.println("$PUBX,40,VTG,0,0,0,0,0,0*5E");   // VTG OFF
  gpsSerial.println("$PUBX,40,GSA,0,0,0,0,0,0*4E");   // GSA OFF
  gpsSerial.println("$PUBX,40,GSV,0,0,0,0,0,0*59");   // GSV OFF
  gpsSerial.println("$PUBX,40,GLL,0,0,0,0,0,0*5C");   // GLL OFF

  //*********************************
  //  TURN ON sentences that we want
  //    GPRMC
  //    GPGGA
  //    GPDTM
  //    PUBX,04
  //
  ubxSend(enableRMC,sizeof(enableRMC)/sizeof(uint8_t));
  if (!ubxGetAck(enableRMC))
  {
    return gps_E_RMC;
  }

  
  ubxSend(enableGGA,sizeof(enableGGA)/sizeof(uint8_t));
  if (!ubxGetAck(enableGGA))
  {
    return gps_E_GGA;
  }

  ubxSend(enableDTM,sizeof(enableDTM)/sizeof(uint8_t));
  if (!ubxGetAck(enableDTM))
  {
    return gps_E_DTM;
  }

  ubxSend(enablePUBX04,sizeof(enablePUBX04)/sizeof(uint8_t));
  if (!ubxGetAck(enablePUBX04))
  {
    return gps_E_PUBX04;
  }
#if 0
  ubxSend(configTimepulse,sizeof(configTimepulse)/sizeof(uint8_t));
  if (!ubxGetAck(configTimepulse))
  {
    return gps_E_CFGTP;
  }
#endif

  return gps_OK;
  
} // end of gpsInit

//=================================================
//  ubxSend
//
//  Send a UBX command to the GPS
//    MSG = byte array containing the UBX command data
//    len = # of bytes in MSG
//=================================================
void ubxSend(uint8_t *MSG, uint32_t len) 
{

  uint32_t CK_A = 0, CK_B = 0;
  uint8_t sum1=0x00, sum2=0x00;
  uint8_t ubxPacket[len+4];


  // UBX prefixes
  //
  ubxPacket[0]=0xB5;
  ubxPacket[1]= 0x62;
  
  for(int i=0; i<len; i++) 
  {
    ubxPacket[i+2]=MSG[i];
  }

  // Calculate checksum
  //
  for(int i=0; i<len; i++)
  {
    CK_A = CK_A + MSG[i];
    CK_B = CK_B + CK_A;
  }
  sum1 = CK_A &0xff;//Mask the checksums to be one byte
  sum2= CK_B &0xff;

  // add the checksum to the end of the UBX packet
  //
  ubxPacket[len+2]=sum1;
  ubxPacket[len+3]=sum2;

  // flush the output buffer - just in case
  //
  gpsSerial.flush();
  
  // send the UBX command
  //
  gpsSerial.write(ubxPacket,len+4);
  
}   // end of sendUBX

//=================================================
//    ubxGetAck - wait for ack after command
//
//=================================================
bool ubxGetAck(uint8_t *MSG)
{
  
  uint8_t b;
  uint8_t ackByteID;
  uint8_t ackPacket[10];
  uint8_t ackReceived[10];
  unsigned long startTime;
  unsigned long currentTime;
  unsigned long waitTime;
  uint32_t CK_A=0, CK_B=0;

  //*********************************
  // Construct the expected ACK packet    
  //
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[0];    // MGS class
  ackPacket[7] = MSG[1];    // MSG id
  ackPacket[8] = 0;     // CK_A
  ackPacket[9] = 0;     // CK_B

  // checksums
  //
  for (uint8_t i=2; i<8; i++) 
  {
    CK_A = CK_A + ackPacket[i];
    CK_B= CK_B + CK_A;
  }

  ackPacket[8]= CK_A &0xff;//Mask the checksums to be one byte
  ackPacket[9]= CK_B &0xff;

  //***********************************************
  // get the 10 byte ack packet (with timeout)
  //    
  //
  startTime = millis();
  for( int i = 0; i < 10;  )
  {

    // Timeout if no ACK in 500ms
    //
    //
    currentTime = millis();
    waitTime = currentTime - startTime;
    if (waitTime > 500)
    {
      return false;   // timeout
    }

    // got a byte?, save it
    //
    if (gpsSerial.available()) 
    {
      b = gpsSerial.read();
      ackReceived[i] = b;
      i++;                  // move to next byte in sequence
    }
    
  } // end of for loop to get the bytes

  //****************************
  //  check the packet
  //
  for( int i = 0; i < 10; i++ )
  {
    if (ackReceived[i] != ackPacket[i])
    {
      return(false);
    }
  }  // end of checking the received packet

  //*********************
  //  all OK
  //
  return( true );
    
} // end of ubxAck


