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

enum CountSource
{
  TCNT,
  ICR
};

// Global Constants & Variables ////////////////////////////////////////////////////////////
const unsigned long gpsBaud = 9600;       

// Startup Mode
//
bool StartupExec;       // true => run , false => programming (don't do anything this time)


//***************
// OSD info
//

// OSD object
//
MAX7456 OSD( osdChipSelect );

//  OSD rows
//
uint8_t TopRow[30];
uint16_t TopRow_Addr = 30;            // Top Row is row 1

uint8_t BottomRow[30];
uint16_t BottomRow_Addr = 9 * 30;    // Top Row is row 11 (good for NTSC)

#if 1
uint8_t TestRow[30];
#endif

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

//
// Video info
//
volatile bool EnableOverlay = false;
volatile unsigned long fieldCount;

volatile int fieldParity = 0;           // 1=> field 1, 2=> field 2  (0 => not yet set)
volatile bool fieldSync;                // true => determine field 1 by timing, false => alternate fields on vsync

bool blnPE1 = false;
bool blnPE2 = false;

volatile unsigned long tk_VSYNC;      // vsync "time" = 2mhz ticks
volatile unsigned long tk_HSYNC;      // hsync "time" = 2mhz ticks

#if 1
volatile bool InVsync = false;
#endif

//***********************
//  Time
//
volatile unsigned short timer1_ov;    // timer 1 overflow count = high word of "time" (32ms per overflow)
#define Timer_Second 2000000          // timer1 is running at about 2mhz
#define Timer_Milli 2000              // approx ticks per millisecond

//******************
// GPS info
// 
int gpsInitStatus;                        // GPS initialization status (0 => good)
volatile bool gpsValid = false;           // true => GPS & PPS data currently valid

// PPS
//
volatile unsigned long tk_PPS;        // tick "time" of most recent PPS int
volatile bool pps_now = false;        // true => PPS just happened
                                      
volatile bool pps_valid = false;      // true => PPS is valid
volatile bool time_UTC = false;       // true => time is current UTC , false => time is currently GPS

volatile int pps_HH;                  //  current pps time (HH:MM:SS)
volatile int pps_MM;
volatile int pps_SS;

volatile int pps_countdown;           // # of pps interrupts until pps valid - used for synch with NMEA data

// GPS Serial data
//

#define NMEA_MAX  201    // max length of a nmea sentence

uint8_t nmeaSentence[NMEA_MAX];       // current NMEA sentence
int nmeaPos = -1;                     // position of next char in NMEA sentence, -1 => no current sentence

typedef struct {
  bool valid;
  uint8_t hh[2];
  uint8_t mm[2];
  uint8_t ss[2];
} nmea_rmc;
nmea_rmc gpsRMC;

//========================================
//  SETUP ROUTINE
//========================================
void setup() {

  //**************
  //  check startup mode
  //
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
  delay(50);        // wait a bit
  gpsInitStatus = gpsInit();


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
//  TIMSK1 = (1 << ICIE1) | (1 << TOIE1);     // enable ICP & overflow
#if 1
  TIMSK1 = (1 << TOIE1);     // overflow
  fieldParity = 1;
  fieldSync = false;
#endif

  // wait 50ms before tracking VSYNC
  //
  delay(50);
  
  // VSYNC
  //
#if 0
//  fieldSync = true;       // look for field 1/ field 2
#endif

  tk_VSYNC = 0;
  VSYNC_CFG_INPUT();
  VSYNC_CFG_EICRA();
  VSYNC_CFG_PCMSK();
  VSYNC_CFG_EIMSK();

  // now turn off hsync ICP interrupt
  //
  delay(50);
#if 0  
//  fieldSync = false;
//  TIMSK1 &= !(1 << ICIE1);        // turn off ICP ints
#endif

  blnPE1 = false;
  blnPE2 = false;

  OSD.home();
  OSD.setCharEncoding(MAX7456_ASCII);
  EnableOverlay = true;   // start the overlay...

 
} // end of setup

//========================================
// LOOP ROUTINE
//========================================
void loop() {

  while (true)
  {
    // get & parse serial data from GPS
    //
#if 1
    ReadGPS();
#endif

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
  uint8_t utmp;


#if 1
  if (InVsync)
  {
    TestRow[2] = 0x20;
  }
#endif  

  // get time
  //
  timePrev = tk_VSYNC;
  tk_VSYNC = GetTicks(TCNT);
  
  // increment field count
  //
  fieldCount++;

  // determine field parity : field 1 or field 2 of a frame
  //
  if (!fieldSync)
  {
    // just alternate fields
    //
    if (fieldParity == 1)
    {
      fieldParity = 2;
    }
    else
    {
      fieldParity = 1;
    }
  }
  else
  {
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
  } // end of fieldSync
  
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
  if (!EnableOverlay)
  {
    return;   // nope, leave now...
  }


  // gps init status
  //
  BottomRow[1] = gpsInitStatus;    // 0 /space => OK
  
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

  // field times
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

  //************
  // NMEA data
  //
  if (gpsRMC.valid)
  {
    OSD.atomax(BottomRow + 3,gpsRMC.hh,2);
    BottomRow[5] = 0x44;
    OSD.atomax(BottomRow + 6,gpsRMC.mm,2);
    BottomRow[8] = 0x44;
    OSD.atomax(BottomRow + 9,gpsRMC.ss,2);
  }
  
  //***
  //  ENABLE interrupts again
  //
  interrupts();

  // update display
  //

#if 0
  OSD.setCursor(0,TOP_ROW);
  for(int i = 0; i < 30; i++)
  {
    OSD.write(TopRow[i]);
  }

  OSD.setCursor(0,BOTTOM_ROW);
  for(int i = 0; i < 30; i++)
  {
    OSD.write(BottomRow[i]);
  }

  OSD.setCursor(0,5);
  for(int i = 0; i < 30; i++)
  {
    OSD.write(TestRow[i]);
  }
#endif

#if 0
  // writeArray version
  OSD.setCursor(0,TOP_ROW);
  OSD.writeArray(TopRow,30);

  OSD.setCursor(0,BOTTOM_ROW);
  OSD.writeArray(BottomRow,30);

  OSD.setCursor(0,5);
  OSD.writeArray(TestRow,30);
#endif

#if 1
  OSD.sendArray(TOP_ROW*30,TopRow,30);
  OSD.sendArray(BOTTOM_ROW*30,BottomRow,30);

//  OSD.atomax(TestRow,nmeaSentence,15);
  OSD.sendArray(5*30,TestRow,30); // testing...
#endif

#if 1
  InVsync = false;
#endif

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
// return 0 on success
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
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,GGA,0,0,0,0,0,0*5A");   // GGA OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,DTM,0,0,0,0,0,0*46");   // DTM OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,VTG,0,0,0,0,0,0*5E");   // VTG OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,GSA,0,0,0,0,0,0*4E");   // GSA OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,GSV,0,0,0,0,0,0*59");   // GSV OFF
  gpsSerial.flush(); // wait for it...

  gpsSerial.println("$PUBX,40,GLL,0,0,0,0,0,0*5C");   // GLL OFF
  gpsSerial.flush(); // wait for it...

  // wait 100ms and empty any pending incoming data
  //
  delay(100);
  while (gpsSerial.available())
    gpsSerial.read();

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
 
  // send the UBX command
  //
  gpsSerial.write(ubxPacket,len+4);
  gpsSerial.flush();  // wait for it ...
  
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
  //  wait for first header byte of ACK (0xB5) - should arrive within timeout period..
  //    
  //
  startTime = millis();
 
  
  for( int i = 0; i < 10;  )
  {

    // Timeout if no ACK in 500ms
    //
    //
    currentTime = millis();
    if (currentTime >= startTime)
    {
      waitTime = currentTime - startTime;
    }
    else
    {
      // rollover
      waitTime = 0 - (startTime - currentTime);
    }
    if (waitTime > 500)
    {
      return false;   // timeout
    }

    // got a byte?
    //   if starting header, save it and continue
    //
    if (gpsSerial.available()) 
    {
      b = gpsSerial.read();
      ackReceived[i] = b;
      if (i != 0)
      {
        i++;    // next byte
      }
      else
      {
        // i == 0
        // we are looking for the first byte of the header
        //  move forward IFF the data bye matches the first header byte value
        if (b == 0xB5)
        {
          // ok, we have the starting header byte and can move forward
          //
          i++;
        }
      } // end of i== 0 check

    } // end of check for data available
    
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

//**********************************************************************************************************
// NEO 6 GPS serial data parsing
//**********************************************************************************************************

//=============================================================
//  ReadGPS - gather and parse any pending serial data from GPS
//    returns false if error parsing data
//=============================================================
bool ReadGPS()
{

  char c;

  //************
  // Read/process all currently available characters
  //  nmeaPos < 0 => not in a sentence now
  //

  while (gpsSerial.available() > 0)
  {
    // get the char
    //
    c = gpsSerial.read();

    // Watch for beginning/ending of a sentence
    //

    if (c == '$')
    {
      // start of a sentence
      //

      // are we in the right place?
      //
      if (nmeaPos >= 0)
      {
        // oops! - currently in a sentence, should not be here
        //
        return false;
      }

      //  ok, save the start char and get ready for next one
      //
      nmeaSentence[0] = (uint8_t)c;
      nmeaPos = 1;
      
    }
    else
    {
      // we are somewhere IN a sentence
      //

      // are we in the right place?
      //
      if (nmeaPos < 0)
      {
        // oops! - not in a sentence now
        //
        return false;
      }

      // ok, save the end of the sentence and call the parser
      //
      nmeaSentence[nmeaPos] = (uint8_t)c;
      nmeaPos++;                      // nmeaPos = # of chars in sentence
      nmeaSentence[nmeaPos] = 0;      // null terminate the sentence

      // if we just ended a sentence, call the parser
      //
      if (c == '\n')
      {

        // call the parser
        //
        if (!ParseNMEA((char *)nmeaSentence, nmeaPos))
        {
          nmeaPos = -1;   // restart
          return false;   // exit on parsing error
        }
        
        // looking for new sentence now...
        //
        nmeaPos = -1;
       
      }
      
    }  // end of check for start/non-start char
    
    
  } // end of loop through available characters
  
  
} // end of ReadGPS

//=============================================================
//  ParseNMEA - parse the current NMEA sentence
//    currently supports the following sentences
//        RMC, GGA, DTM, and PUBX,04
//  INPUTS:
//    nmeaChars[] - array of chars containing sentence
//    iChars = # of chars in sentence (including terminating CRLF)
//=============================================================
#define MAX_FIELDS 17       // GGA has 17 fields (including the terminating CRLF)
bool ParseNMEA(char *nmeaChars, int iChars)
{
  
  int fieldStart[MAX_FIELDS];   // start position of each field
                                // end of field = (start of next field - 2)
  int fieldEnd;     // end position of a field
  int iField;       // current field #
  int fieldCount;   // # of fields in sentence (including CRLF)
  int iPos;         // current position in nmea string

  int ID_start;
  int ID_len;
  
  //********
  // find the start,end of each field
  //
  iField = 0;
  fieldStart[iField] = 0;
  iPos = 1;                 // next char to be tested

  while (iPos < iChars)
  {
    // start of a new field?
    //
    if (nmeaChars[iPos] == ',')
    {
      //  end of this field inside the sentence
      //
      iField++;           // new field start
      iPos++;             // start with position past ','
      if (iPos >= iChars)
      {
        return false;     // no start of next field => unexpected end of sentence
      }
      fieldStart[iField] = iPos;

    }
    else if (nmeaChars[iPos] == '\r')
    {
      // CR => end of sentence and end of this field
      //
      iPos++;             // -> one past the CR = LF
      iField++;           // last field = CRLF
      fieldStart[iField] = iPos;
      break;
    }
    else
    {
      // in a field, move to next char
      //
      iPos++;
    }
  } // loop through all chars in sentence

  //  done scanning for fields
  //   iPos should point to the terminating LF now
  //   iField = field # of last field in sentence (CRLF)
  //
  if (nmeaChars[iPos] != '\n')
  {
    return false;       // terminated loop without finding CRLF
  }
  fieldCount = iField + 1;
  if (fieldCount < 3)
  {
    return false;     // all sentences have at least three fields
  }
  
  //************
  // found the fields, now parse sentence data
  //  
  ID_start = fieldStart[0];                 // start of message ID field
  ID_len = fieldStart[1] - ID_start - 1;    // length of message ID field

  if (strncmp(nmeaChars + ID_start,"$GP",3) == 0)
  {
    // this is a standard NMEA sentence
    //
    char cmd[4];
    cmd[3] = 0;     // null terminate it
    strncpy(cmd,&nmeaChars[ID_start+3],3);    // get the cmd chars
    if (strcmp(cmd,"RMC") == 0)
    {      
      return ParseRMC((uint8_t *)nmeaChars,fieldStart,fieldCount);
    }
    else if ( strcmp(cmd,"GGA") == 0 )
    {
      return true;
    }
    else if ( strcmp(cmd,"DTM") == 0 )
    {
      return true;
    }
    else
    {
        return true;
    } // end of if/else block parsing standard NMEA sentences
  }
  else if (strncmp(&nmeaChars[ID_start],"$PUBX",5) == 0)
  {
    // this is a UBX proprietary sentence
    //
    int f1_start;
    int f1_len;

    f1_start = fieldStart[1];
    f1_len = fieldStart[2] - f1_start - 1;
    if (strncmp(&nmeaChars[f1_start],"04",2) == 0)
    {
      // PUBX,04 sentence
      //
      
    } 
    else
    {
      // unkown PUBX sentence => do nothing
      //
      return true;
      
    }// end of check for PUBX sentence type
  }
  else
  {
    // unknown sentence type
    //
    //  DO NOTHING and no error
    return true;
  }
  
} // end of ParseNMEA

//=============================================================
//  ParseRMC - parse & save the RMC data of interest
//  INPUTS:
//    nmeaChars[] - array of chars containing sentence
//    fieldStart[] - array of starting indicies for fields
//    fieldCount = # of fields (including CRLF)
//=============================================================

bool ParseRMC(uint8_t *nmeaChars, int *fieldStart, int fieldCount)
{
  int iStart;
  int iLen;

  gpsRMC.valid = false;
  
  // field 1 = time
  //
  iStart = fieldStart[1];
  iLen = fieldStart[2] - iStart - 1;

  if (iLen < 6)
  {
    return false; 
  }
  gpsRMC.hh[0] = nmeaChars[iStart++];
  gpsRMC.hh[1] = nmeaChars[iStart++];
  gpsRMC.mm[0] = nmeaChars[iStart++];
  gpsRMC.mm[1] = nmeaChars[iStart++];  
  gpsRMC.ss[0] = nmeaChars[iStart++];  
  gpsRMC.ss[1] = nmeaChars[iStart++];  

  // all done
  //
  gpsRMC.valid = true;
  
  return true;
  
} // end of parseRMC

//*****************************************************************************************
// Utility routines
//*****************************************************************************************

//===========================================================================
// ultohex - convert unsigned long to 8 hex MAX7456 characters in a character array
//
//===========================================================================
void ultohex(uint8_t *dest, unsigned long ul)
{

  uint8_t hex[16] = {0x0A,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0B,0x0C,0x0D,0x0E,0x0F,0x10};

  uint8_t *pn;
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
// bytetohex - convert byte to 2 hex MAX7456 characters in a character array
//
//===========================================================================
void bytetohex(uint8_t *dest, uint8_t byt)
{

  uint8_t hex[16] = {0x0A,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0B,0x0C,0x0D,0x0E,0x0F,0x10};

  uint8_t nibble;

  nibble = (byt & 0xF0) >> 4;
  *dest = hex[nibble];

  dest++;
  nibble = (byt & 0x0F);
  *dest = hex[nibble];
    
} // end of bytetohex

//===========================================================================
// ultodec - convert unsigned long to decimal MAX7456 characters in a character array
//    dest = ptr to destination of most significant char of decimal number
//    ul = unsigned long value to convert
//    len = # of chars to write in destination (pos => leading zeros, negative => leading spaces) 
//===========================================================================
void ultodec(uint8_t *dest, unsigned long ul, int len)
{

  uint8_t dec[10] = {0x0A,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};

  uint8_t *pn;

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

