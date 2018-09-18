/*

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

OpenVTI2

This is VERSION 6

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
- SBP (from original OpenVTI code by MichaelF)
- interrupt driven PPS
- update OSD during vertical blanking
- write OSD update directly using auto-increment mode
- added code for parsing GPS serial data
- enabled  XXXX ublox proprietary sentence

Version 6
- moved to Mega2560 as required platform

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
#include <util/crc16.h>

//  Global Settings ///////////////////////////////////////////////////////////////
#define VERSION  6

#define GRAYBACKGROUND 1    // == 1 => set gray background behind OSD lines
                            // == 0 => transparent background behind OSD lines


#define TESTROW 0           // == 1 => enable third row at top of display for testing/debug
                            // == 0 =>  no third row...

// Global Constants & Variables ////////////////////////////////////////////////////////////
//

// CRC data
//
uint32_t ProgramLength = 0x3000;
uint16_t pgmCRC = 0;

//*********************************
//  General state info
//
enum CountSource
{
  CNT4,
  CNT5,
  CR4,
  CR5
};

// Operating Modes
//
enum OperatingMode
{
  InitMode,               // => Power up initialization
  WaitingForGPS,          // => Waiting to receive valid GPS data
  Syncing,                // => Synchronizing internal time base with GPS data
  TimeValid,              // => Time is valid  (nominal operating mode for time insertion)
  ErrorMode,              // => Error was found
  FailMode                // => Unrecoverable failure found
};
volatile OperatingMode CurrentMode;    // Current operating mode


#define SYNC_SECONDS 10                // # of seconds for syncing to GPS time
volatile short int TimeSync;          // ( > 0 ) => we are syncing to GPS sentence times = # of seconds remaining for sync (small value so no problem with ints)

volatile int ErrorCountdown;
#define ERROR_DISPLAY_SECONDS 2

// Error Messsages
//
char msgGPSfail[] ="GPS init failed";
#define len_msgGPSfail  15
char msgWaiting[] = "Waiting for GPS";
#define len_msgWaiting  15
char msgSync[] = "Sync ";         // space for remainder
#define len_msgSync  4
char msgNoPPS[] = "*PPS ERROR*";
#define len_msgNoPPS 11
char msgSyncFailed[] = "*Sync FAILED*";
#define len_msgSyncFailed 13
char msgUnknownError[] = "*FATAL ERROR*";
#define len_msgUnknownError 13

char msgErrorCodes[] = "   ";
#define len_msgErrorCodes 3

// Command string from the PC
//
#define CommandSize 30            // max size of a command string 
char strCommand[ CommandSize + 1 ];
int Cmd_Next;                     // speed optimization ( -1 => full command pending )

//***************
// OSD info
//
volatile bool EnableOverlay = false;      // update Overlay?

// OSD object
//
MAX7456 OSD( osdChipSelect );

//  Local OSD rows
//
uint8_t TopRow[30];
uint16_t TopRow_Addr = 30;            // Top Row is row 1

uint8_t BottomRow[30];
uint16_t BottomRow_Addr = 9 * 30;    // Top Row is row 11 (good for NTSC)

#if (TESTROW == 1)
uint8_t TestRow[30];
#endif

// location of UI elements
//
#define TOP_ROW_NTSC 11
#define BOTTOM_ROW_NTSC  12
#define TOP_ROW_PAL 14
#define BOTTOM_ROW_PAL 15

#define FIELD1TS_COL 12
#define FIELD1TS_ROW BOTTOM_ROW

#define FIELD2TS_COL 17
#define FIELD2TS_ROW BOTTOM_ROW

#define FIELDTOT_COL 22
#define FIELDTOT_ROW BOTTOM_ROW
#define FIELDTOT_MAX 9999999        // max field count (row is only 29 char wide reliably)

int osdTop_RowOffset = TOP_ROW_NTSC*30;   // display memory offset to start of ROW for cycling through data
int osdTop_Col = 1;                       // starting colum in this ROW

int osdBottom_RowOffset = BOTTOM_ROW_NTSC*30;   // display memory offset to start of ROW for time data
int osdBottom_Col = 1;

// pixel offset of display to Match IOTA-VTI
//
#define OSD_X_OFFSET  0
#define OSD_Y_OFFSET  0

//
// Video input info
//
uint8_t videoStd;     // NTSC or PAL setting

volatile unsigned long fieldTotal;      // total number of fields (VSYNC pulses encountered)

volatile int fieldParity = 0;           // 1=> field 1, 2=> field 2  (0 => not yet set)

volatile unsigned long tk_VSYNC;      // vsync "time" = 2mhz ticks
volatile unsigned long tk_HSYNC;      // hsync "time" = 2mhz ticks

volatile unsigned short osdRotation = 0;             // rotation counter for top row (uses only one byte of short so we don't have to worry about interrupts)

//***********************
//  Time
//
volatile unsigned short timer4_ov;                      // timer 4 overflow count = high word of "time" (32ms per overflow)
volatile unsigned long Timer_Second = 2000000;          // # of ticks for 1 second
volatile unsigned long Timer_100ms = 200000;            // ticks / 100ms
volatile unsigned long PPS_TOLERANCE = 2000;            //  500us tolerance for PPS interval
#define Timer_Milli 2000              // approx ticks per millisecond

volatile unsigned long tk_pps_interval_total=0;         // sum of pps intervals
volatile unsigned long tk_pps_interval_count=0;         // # of pps interval
volatile unsigned long tk_pps_interval_ave=0;           // average pps interval

// YY-MM-DD HH:MM:SS time of current second
//   note: all values are less than 255 => don't need to protect individual changes from interrupts during read/write
//         HOWEVER ... the VSYNC ISR uses these values, so we should only update the groups of these variables with interrupts OFF
//
volatile bool time_UTC = false;         // true => time is currently UTC , false => time is currently GPS
volatile int sec_Year;                  // 2 digit
volatile int sec_Mon;
volatile int sec_Day;
volatile int sec_hh;
volatile int sec_mm;
volatile int sec_ss;
volatile int offsetUTC_Default = -1;      // receiver default value for GPS-UTC offset
volatile int offsetUTC_Current = -1;      // current/valid GPS-UTC offset

//******************
// GPS info
// 
int gpsInitStatus;                        // GPS initialization status (0 => good)
const unsigned long gpsBaud = 9600;       

// PPS
//
volatile unsigned long tk_PPS;        // tick "time" of most recent PPS int
volatile unsigned long tk_PPS_valid;  // time of last VALID PPS int
volatile bool pps_now = false;        // true => PPS just happened
                                      
// GPS serial data
//

#define NMEA_MAX  201    // max length of a nmea sentence

uint8_t nmeaSentence[NMEA_MAX];       // current NMEA sentence
int nmeaCount = -1;                   // position of next char in NMEA sentence = # of chars in current sentence, 0 => no current sentence
  
#define MAX_FIELDS 17       // GGA has 17 fields (including the terminating CRLF)
int fieldStart[MAX_FIELDS];   // start position of each field
                              // end of field = (start of next field - 2)                         

volatile unsigned long tk_NMEAStart;
volatile int n_hh;
volatile int n_mm;
volatile int n_ss;
volatile bool n_blnUTC;
volatile unsigned long tk_GPSRMC;   // time (ticks) for start of current RMC data (if valid)
volatile unsigned long tk_PUBX04;   // time of start of current PUBX04 data (if valid)

volatile struct {
  bool valid;
  char mode;          // A or D => valid fix
  uint8_t hh;
  uint8_t mm;
  uint8_t ss;
  uint8_t yr;
  uint8_t mon;
  uint8_t day;
} gpsRMC;

#define MAX_LATLONG 12
#define MAX_ALT 10
struct {
  bool valid;
  uint8_t lat[MAX_LATLONG];     // latitude
  uint8_t NS;                   // North / South indicator for latitude
  uint8_t lng[MAX_LATLONG];     // longitude
  uint8_t EW;                   // East / West indicator for longitude
  uint8_t alt_msl[MAX_ALT];     // MSL altitude
  uint8_t alt_units;            // units for altitude (should be m for meters)
  uint8_t geoid_sep[MAX_ALT];   // geoid separation (N)
  uint8_t sep_units;            // units for geoid separation 
} gpsGGA;

struct {
  bool valid;
  bool dtmWGS84;      // true => datum is currently WGS84
} gpsDTM;

struct {
  bool valid;
  uint8_t hh;
  uint8_t mm;
  uint8_t ss;
  uint8_t usLeapSec;      // current leap seconds
  bool  blnLeapValid;     // true => leap seconds is up to date, false => using firmware default value
  uint8_t cLeap[3];       // leap seconds field from sentence
} gpsPUBX04;

volatile bool blnEchoPPS = false;
char msgEchoPPS[] = "<P>TTTTTTTT</P>";
#define len_msgEchoPPS 15

volatile bool blnEchoVSYNC = false;
char msgEchoVSYNC[] = "<V>TTTTTTTT</V>";
#define len_msgEchoVSYNC 15


//========================================
//  SETUP ROUTINE
//========================================
void setup() {
  uint8_t rows;
  uint8_t cols;    
  
  //************
  //  General init
  //
  EnableOverlay = false;      // turn off overlay update for now...
  fieldTotal = 0;
  for (int i = 0; i< 30; i++) 
  {
    TopRow[i] = 0x00;
    BottomRow[i] = 0x00;
  }

  for (int i = 0; i < CommandSize; i++)
  {
    strCommand[i] = ' ';
  }

  // caculate CRC of some program memory
  //
  pgmCRC = getFlashCRC(ProgramLength);
  
  // set D10 - D13 to input since the TinySine shield connects them to the SPI lines of the Mega
  //      this is the default startup state but it doesn't hurt to make sure
  //
  pinMode(10,INPUT);
  pinMode(11,INPUT);
  pinMode(12,INPUT);
  pinMode(13,INPUT);

  // connect to USB port at 115200 so we send data quickly
  //
  //
  Serial.begin(115200);

  //*****************
  //  Delay to allow startup time for external devices
  //   max7456 typically needs 50ms
  //
  delay(100);     // 100 ms to make sure
  
  //************
  // Init GPS
  //
  gpsInitStatus = gpsInit();
  if (gpsInitStatus != 0)
  {
    // GPS init failed => non-recoveable
    CurrentMode = FailMode;

    // clear the current display lines
    //
    for( int i = 0; i < 30; i++ )
    {
      TopRow[i] = 0x00;
      BottomRow[i] = 0x00;
    }
    
    // GPS failure message
    //
    OSD.atomax(&BottomRow[1],(uint8_t*)msgGPSfail,len_msgGPSfail);
    bytetodec2(&BottomRow[1 + len_msgGPSfail + 1],gpsInitStatus);
  }

  

  //************
  //  Init max OSD chip
  //
  
  // Initialize the SPI connection:
  //
  SPI.begin();
  SPI.setClockDivider( SPI_CLOCK_DIV2 );      // Must be less than 10MHz.
  
  // Initialize the MAX7456 OSD:
  //
  OSD.begin();                // Use NTSC with full area.

  // NTSC or PAL?
  //
  videoStd = 0;
  while (videoStd == 0)
  {
    videoStd = OSD.videoSystem();
    delay(100);
  }

  if ( videoStd == MAX7456_NTSC )
  {
    rows = OSD.safeVideoRows[MAX7456_NTSC][MAX7456_FULLSCREEN];
    cols = OSD.safeVideoCols[MAX7456_NTSC][MAX7456_FULLSCREEN];    
    OSD.setDefaultSystem(MAX7456_NTSC);
    
    osdTop_RowOffset = TOP_ROW_NTSC*30;    
    osdBottom_RowOffset = BOTTOM_ROW_NTSC*30;
  }
  else if (videoStd == MAX7456_PAL)
  {
    rows = OSD.safeVideoRows[MAX7456_PAL][MAX7456_FULLSCREEN];
    cols = OSD.safeVideoCols[MAX7456_PAL][MAX7456_FULLSCREEN];    
    OSD.setDefaultSystem(MAX7456_PAL);
    
    osdTop_RowOffset = TOP_ROW_PAL*30;
    osdBottom_RowOffset = BOTTOM_ROW_PAL*30;
  }
  else
  {
    EnableOverlay = false;
    return;      // unknown video standard... do nothing
  }
  
  OSD.setTextArea(rows, cols, MAX7456_FULLSCREEN);
  OSD.setSyncSource(MAX7456_AUTOSYNC);
  OSD.setWhiteLevel(0);  // should be 0% black 120% white
  OSD.setCharEncoding(MAX7456_ASCII);       // use this char set

  // setup background for OSD area
  //
#if (GRAYBACKGROUND != 1)
  OSD.videoBackground();
#else
  OSD.setGrayLevel(4);
  OSD.home();                               //
  OSD.videoBackground();
  for ( int r = 0; r < rows; r++ )
  {
    for ( int c = 0; c < cols; c++ )
    {
      OSD.write((uint8_t)0);
    }
  }
  OSD.grayBackground();     // remaing writes will be gray background
#endif

  // align with IOTA-VTI
  OSD.setTextOffset(OSD_X_OFFSET, OSD_Y_OFFSET);
  
  OSD.display();                              // Activate the text display.

  //*****************  
  // HSYNC input
  //  no interrupt - just polled as an input
  //
  HSYNC_CFG_INPUT();

  //**********************
  //  Init timers for input capture and timing
  //    Timer 4 - 1pps
  //    Timer 5 - VSYNC
  //  * get them setup with Mask OFF to prevent interrupts until we are ready
  //


  //  PPS : Timer 4 - used to capture times of 1pps interrrupts
  //    ICP4 pin (digital 49) set to input
  //    Normal mode
  //    Prescaler = f/8
  //    Input Capture = edge detect
  //    MASK OFF ints for now
  //
  pinMode(49,INPUT);                    // ICP4 = pin 49 as input
  
  TIMSK4 = 0;                             // mask off all interrupts for now
  TCCR4A = 0;                             // all OC ports off and normal mode
  TCCR4B = (1 << ICES4) | (1 << CS41);    // positive edge trigger IC & f/8 prescaler
  PRR1 &= ~(1 << PRTIM4);                 // turn OFF power reduction for timer 4 => turn it ON!
  
  //  VSYNC : Timer 5 - VSYNC times
  //    ICP5 pin (digital 48) set to input
  //    Normal mode
  //    Prescaler = f/8
  //    Input Capture = edge detect AND noise canceller ON
  //     note: since timer5 is sync'd to timer4, we will use the same overflow counter for both => no overflow int for timer5
  //    MASK OFF ints for now
  //
  pinMode(48,INPUT);                    // ICP5 = pin 48 as input
  
  TIMSK5 = 0;                             // mask off all interrupts for now
  TCCR5A = 0;                             // all OC ports off and normal mode
  TCCR5B = (1 << ICNC5) | (1 << ICES5) | (1 << CS51);    // noise cancel on, positive edge trigger IC & f/8 prescaler
  PRR1 &= ~(1 << PRTIM5);                 // turn OFF power reduction for timer 5 => turn it ON!

  
  //  SYNC all synchronous timers via Prescaler reset
  //    MASK ON => start input captures
  //
  GTCCR = (1 << TSM) | (1 << PSRSYNC);    // STOP prescaler and all syncronous timers

  TCNT4 = 0;        // timer4: reset count
  TIFR4 = 0;        // timer4: reset all pending interrupts
  TIMSK4 = (1 << ICIE4) | (1 << TOIE4);   // timer 4: turn on IC capture and overflow interrupts

  // VSYNC ...
  //
  tk_VSYNC = 0;     // ok ... ready for vsync interrupts now
  
  TCNT5 = 0;        // timer5: reset count
  TIFR5 = 0;        // timer5: reset all pending interrupts
  TIMSK5 = (1 << ICIE5);   // timer 5: turn on IC capture and NO overfloww interrupts

  timer4_ov = 0;    // timer4: reser overflow count (used for timer5 overflow as well)
  GTCCR = 0;    // RESTART prescaler and all synchronous timers => Timer4 and Timer5 are now sync'd

  //*********************
  //  VSYNC in now happening
  //   Tell the VSYNC ISR to try to detect the field order on this first VSYNC
  //
  fieldParity = 0;
  
  //*********************
  //  OK, startup the regular timing operation
  //    * WaitingForGPS mode = start by looking for GPS NMEA and PPS to get in sync
  //
  time_UTC = false;     // assume NOT UTC for now...
  
  // "Waiting for GPS" mode
  //
  for( int i = 0; i < FIELDTOT_COL; i++ )   // clear all but the field count
  {
    BottomRow[i] = 0x00;
  }   
  OSD.atomax(&BottomRow[1], (uint8_t*)msgWaiting, len_msgWaiting);
  
  CurrentMode = WaitingForGPS;        // set mode

  // reset command stream from PC
  //     clear incomming data buffer from PC
  //
  while( Serial.available() )
  {
    Serial.read();
  }
  strCommand[0] = 0;  // no command
  Cmd_Next = 0;

  // start the overlay
  //
  EnableOverlay = true;
 
} // end of setup

//========================================
// LOOP ROUTINE
//
//    - ReadGPS data (and echo)
//    - Read command data from USB port
//    - if command found, do it
//
//========================================
void loop() 
{

  while (true)
  {

    // get & parse serial data from GPS
    //
    ReadGPS();

    // compute PPS stats
    //
    if ( tk_pps_interval_count == 10 )
    {
      noInterrupts();
      tk_pps_interval_ave = tk_pps_interval_total / tk_pps_interval_count;      // compute the average delay between PPS intervals
      Timer_Second = tk_pps_interval_ave;                                       // use this average as the new definition of a second
      Timer_100ms = Timer_Second / 10;
      
      tk_pps_interval_total = 0;                                                // reset
      tk_pps_interval_count = 0;
      interrupts();
    }

    // Read any pending command data from USB port
    //
    ReadCMD();

    // Execute pending command
    //
    if (Cmd_Next < 0)
    {
      
      ExecCMD();
    } // end of executing commands
    
  } // end of loop

} // end of loop()

//=====================================
//  ReadCMD - get pending command data from USB port
//    set CMD_Next = -1 when full command line available
//
//======================================
void ReadCMD()
{

  byte bIn;
  
  // read command from PC
  //  - one byte at a time
  //  - command terminates with a Newline char (\n)
  //
  while ( Serial.available() > 0 )
  {
    // was a command pending?
    //
    if (Cmd_Next < 0)
    {
      Cmd_Next = 0;     // not any more...
    }
    else if (Cmd_Next >= CommandSize)
    {
      // ran out of room => error
      //
      Cmd_Next = 0;   // reset to new command
    }
      
    // ok -> get the character & save it
    //   but don't save CR or LF
    //
    bIn = Serial.read();
      
    // if \n, terminate command line and don't save the \n
    //
    if (bIn == '\n')
    {
      strCommand[Cmd_Next] = 0;
      Cmd_Next = -1;
      break;
    }
    else if (bIn != '\r')
    {
      // here if NOT \r or \n
      //
      strCommand[ Cmd_Next ] = bIn;
      Cmd_Next++;
    }
    // if \r, do nothing with char
    
  } // end of loop reading incoming data
  
} // end of ReadCMD

//=====================================
//  ExecCMD - execute command from USB port
//
//======================================
void ExecCMD()
{
#if (TESTROW == 1)
      OSD.atomax(&TestRow[1], (uint8_t*)strCommand, 10);
#endif
  // parse the command and do it
  //  PPS ON => turn on PPS echo
  //  PPS OFF => turn off PPS echo
  //  VSYNC ON => turn on VSYNC echo
  //  VSYNC OFF => turn off VSYNC echo
  //
  if (strncmp(strCommand,"PPS ",4) == 0)
  {
    if (strncmp(strCommand + 4,"ON",2) == 0)
    {
      blnEchoPPS = true;
    }
    if (strncmp(strCommand + 4,"OFF",3) == 0)
    {
      blnEchoPPS = false;
    }
    // else ... do nothing
  }
  else if (strncmp(strCommand,"VSYNC ",6) == 0)
  {
    if (strncmp(strCommand + 6,"ON",2) == 0)
    {
      blnEchoVSYNC = true;
    }
    if (strncmp(strCommand + 6,"OFF",3) == 0)
    {
      blnEchoVSYNC = false;
    }
    // else ... do nothing
  }
  // else ... didn't recognize the command ... do nothing

  // reset / clear command
  //
  Cmd_Next = 0;

  // all done
  //
  
} // end of ExecCMD

//*****************************************************************************************
// ISR routines
//*****************************************************************************************

//===============================
//  VSYNC ISR = ICP5
//===============================
VSYNC_ISR()
{
  unsigned long timeDiff;
  uint8_t utmp;
  unsigned long ulTmp;

  int vs_hh;      // local copies of this data to match the values at start of this ISR and before
  int vs_mm;      // a subsequent PPS can change them
  int vs_ss;
  bool vs_blnUTC;

  //****************************************
  //  determine field parity
  //  after startup
  //    just swap back and forth
  //  during startup
  //    wait for falling edge of HSYNC, then get ICR count and compare
  //    **** This code will hang if no HSYNC! but that is probably ok.  No Hsync => no video signal
  //
  if (fieldParity != 0)
  {
    // after startup
    //
    
    // get the ICR time for the start of VSYNC
    //
    tk_VSYNC = GetTicks(CR5);

    // just set the parity based on last field
    //
    fieldParity = (fieldParity == 1)? 2 : 1;
    
  }
  else
  {
    // startup - detect the parity based on VSYNC/HSYNC timing
    //
    // wait until HSYNC high
    //
    while( !HSYNC_READ() );
  
    // then wait for it to fall
    //
    while( HSYNC_READ() );
  
    // now get the current time as the falling edge of HSYNC
    //
    tk_HSYNC = GetTicks(CNT5);
  
    // and get the ICR time for the start of VSYNC
    //
    tk_VSYNC = GetTicks(CR5);
  
    // determine time to nearest HSYNC
    //   During field 1, HSYNC should occur near VSYNC
    //
    if (tk_HSYNC >= tk_VSYNC)
    {
      timeDiff = tk_HSYNC - tk_VSYNC;
    }
    else
    {
      timeDiff = 0 - (tk_VSYNC - tk_HSYNC);
    }
  
    if (timeDiff > 128)     // > 64us
    {
      timeDiff -= 128;      // modulo 64us
    }
    else if (timeDiff > 64)      // 32 - 64 us
    {
      timeDiff = 128-timeDiff;
    }
  
    // HSYNC within 15us of VSYNC => field 1
    //
    if (timeDiff < 30)
    { 
      fieldParity = 1;
    }
    else
    {
      fieldParity = 2;
    }
    
  } // end of fieldParity detection

  //**********************
  //  echo VSYNC time to USB port
  //
  if (blnEchoVSYNC)
  {
    ultohexA((uint8_t *)msgEchoVSYNC + 3, tk_VSYNC);
    for (int i = 0; i < len_msgEchoVSYNC; i++)
    {
      Serial.write(msgEchoVSYNC[i]);
    }
  }

  //************************
  // increment field count
  //  - interrupts are OFF - ok to update the value
  //  - rollover after maximum value that can be displayed
  //
  fieldTotal++;
  if (fieldTotal > FIELDTOT_MAX)
    fieldTotal = 0;

  //*********************
  //  BEFORE enabling interrupts - save current time seconds in local variables to match time of VSYNC pulse
  //
  vs_hh = sec_hh;
  vs_mm = sec_mm;
  vs_ss = sec_ss;
  vs_blnUTC = time_UTC;
  
  //****************************
  //  ENABLE interrupts again
  //
  interrupts();

  //*************************************************
  //  Update local display lines based on CurrentMode
  //
  //
  //    * InitMode
  //      - No display during this mode so just return
  //
  //    * WaitingForGPS
  //    * ErrorMode
  //    * FailMode
  //    * Syncing
  //    * TimeValid
  //      - error checking
  //      - update overlay
  //
  if (CurrentMode == InitMode)
  {
    return;     // leave now => don't update the display
  }

  //  Current mode must be WaitingForGPS, Syncing, TimeValid, ErrorMode, or FailMode
  //  Display will be active in all of these modes
  //
  
  //*************************************************
  //  VSYNC time
  //    - determine field time delay from latest PPS
  //
  noInterrupts();           // protect tk_pps from interrupts
  if (tk_VSYNC >= tk_PPS)
  {
    timeDiff = tk_VSYNC - tk_PPS;
  }
  else
  {
    timeDiff = 0 - (tk_PPS - tk_VSYNC);
  }
  interrupts();
  
  //
  // was the last PPS more than one second ago?
  //   if we are in FailMode or WaitingforGPS mode, don't go to ErrorMode
  //
  if ( (timeDiff > (Timer_Second + PPS_TOLERANCE)) && (CurrentMode != FailMode) && (CurrentMode != WaitingForGPS) )
  {
    
    // opps - we should have seen a PPS by now... this is an error => move to error mode
    //
    noInterrupts();                 // clear the ave interval computation
    tk_pps_interval_total = 0;
    tk_pps_interval_count = 0;
    interrupts();
    
    CurrentMode = ErrorMode;
    ErrorCountdown = ERROR_DISPLAY_SECONDS;
    
    // bottom line - message & PPS
    //
    for( int i = 0; i < FIELDTOT_COL; i++ )
    {
      BottomRow[i] = 0x00;
    }   
    OSD.atomax(&BottomRow[1], (uint8_t*)msgNoPPS, len_msgNoPPS);
    bytetodec2(BottomRow + len_msgNoPPS + 1,0);
    
  }

  //***************************************************
  //  UPDATE local display lines based on current mode
  //

  //  Branch by CurrentMode
  //
  //    *FailMode
  //      * no more change to local memory lines => just update the OSD overlay with the current local memory lines (error info)
  //
  //    *ErrorMode
  //      * TopRow: LastPPS/Field1/Field2 times 
  //      * BottomRow: Update field count only (leave existing error message)
  //
  //    *WaitingForGPS
  //      * TopRow: LastPPS / Field1/Field2 times 
  //      * BottomRow: Update field count only (leave existing status message)
  //
  //    *Syncing
  //      * TopRow: LastPPS / Field1/Field2 times 
  //      * BottomRow: Update field count only (leave existing status message)
  //
  //    *TimeValid
  //      * TopRow: data field rotation 
  //      * BottomRow: Update times & field count
  //  
  //  
  if (CurrentMode != FailMode)
  {
     
    // *** Update field count in lower right of BottomRow (display field count in all non-fail modes with an active display)
    //
    // clear the field count portion of the bottom row
    //
    for( int i = FIELDTOT_COL; i < 30; i++ )
    {
      BottomRow[i] = 0x00;
    }
    ultodec(BottomRow + FIELDTOT_COL,fieldTotal,0);     // no padding, left justified
  
    // now update the rest of the display lines
    //  
    if ((CurrentMode == ErrorMode) || (CurrentMode == WaitingForGPS) || (CurrentMode == Syncing))
    {
      // clear the current top display line
      //
      for( int i = 0; i < 30; i++ )
      {
        TopRow[i] = 0x00;
      }

      // Top Row - Last PPS count (hex)
      //
      ultohex(TopRow + 1, tk_PPS_valid);
      
      // Top Row = update Field1/Field2 counts
      //
      if (fieldParity == 1)
      {
        // field 1
        ultohex(TopRow + 10, tk_VSYNC);  // write field 1 tick count
      }
      else
      {
        // field 2
        //
        ultohex(TopRow + 20, tk_VSYNC);  // write field 2 tick count
      }
      
    }
    else if (CurrentMode == TimeValid)
    {
      // Nominal timing mode , update all timing info
      //
      unsigned long tDiff;

      // clear the top line data
      //
      for( int i = 0; i < 30; i++ )
      {
        TopRow[i] = 0x00;
      }

      // OSD Top Row
      // Rotate through 
      //    Date DD-MM-YY
      //    Lat/Long
      //    MSL/N/Datum
      //    Version/UTC offset
      //
      switch (osdRotation)
      {
        
        case 0:
          // Date dd-mm-yyyy
          //
          if (gpsRMC.valid)
          {
            bytetodec2(TopRow + 1,gpsRMC.day);
            TopRow[3] = 0x49;   // '-'
            bytetodec2(TopRow + 4,gpsRMC.mon);
            TopRow[6] = 0x49;   // '-'
            TopRow[7] = 0x02;   // '2'
            TopRow[8] = 0x0A;   // '0'
            bytetodec2(TopRow + 9,gpsRMC.yr);
          }
          break;

        case 1:
          // Lat/Long
          //
          if (gpsGGA.valid)
          {
            TopRow[1] = (gpsGGA.NS == 'N')? 0x18 : 0x1D;   // N or S
            OSD.atomax(TopRow + 3, gpsGGA.lat,MAX_LATLONG);
            TopRow[3 + MAX_LATLONG + 1] = (gpsGGA.EW == 'E')? 0x0F : 0x21;   // E or W
            OSD.atomax(TopRow + 3 + MAX_LATLONG + 2,gpsGGA.lng, MAX_LATLONG);
          }
          break;

        case 2:      
          // altitude 
          //    MSL & geoid separation
          if (gpsGGA.valid)
          {
            TopRow[1] = 0x17;   // M
            TopRow[2] = 0x1D;   // S
            TopRow[3] = 0x16;   // L
            OSD.atomax(TopRow + 5,gpsGGA.alt_msl,MAX_ALT);

            TopRow[5 + MAX_ALT + 1] = 0x18;    // N
            OSD.atomax(TopRow + 5 + MAX_ALT + 3, gpsGGA.geoid_sep, MAX_ALT);
          }
          break;       

        case 3:
          // software version
          //
          TopRow[1] = 0x20;   // V
          TopRow[3] = 0x05;   // 5
          TopRow[4] = 0x41;   // '.'
          TopRow[5] = 0x0A;   // 0

          // CRC
          //
          ustohex(TopRow + 7, pgmCRC);

          // error codes
          //
          OSD.atomax(TopRow + 12, (uint8_t*) msgErrorCodes, len_msgErrorCodes);

          // leap seconds
          //
          OSD.atomax(TopRow + 20, gpsPUBX04.cLeap, 3);
          
          break;

        default:
          break;
      }


      // OSD Bottom Row ...
      //  ** use "local" values saved at start of ISR to match state at that time
      //

      // compute time since PPS in 0.1ms intervals
      //
      tDiff = timeDiff * 1000;           // upcoonvert to avoid loss of precision in next divide
      tDiff = tDiff / Timer_100ms;       // time difference in 0.1ms intervals

      // clear the bottom row up to the field count
      //
      for( int i = 0; i < FIELDTOT_COL; i++ )
      {
        BottomRow[i] = 0x00;
      }   

      //
      // time base UTC or GPS
      //
      BottomRow[1] = (vs_blnUTC? 0x1F : 0x11);       // U (UTC) or G (GPS)
      
      //    - pps "mark" on first field after PPS
      //
      if ((pps_now) && (tk_VSYNC > tk_PPS))
      {
        BottomRow[0] = 0xFA;
        pps_now = false;
      }
      else
      {
        BottomRow[0] = 0x00;
      }
    
      // OSD Bottom Row
      //    - field times
      //
      if (fieldParity == 1)
      {
        // field 1
        //
        ultodec(BottomRow + FIELD1TS_COL, tDiff, 4);  // write field 1 stamp, 4 chars, zero padded
        
      }
      else
      {
        // field 2
        //
        ultodec(BottomRow + FIELD2TS_COL, tDiff, 4);  // write field 2 stamp, 4 chars, zero padded
     
      }
  
      // OSD Bottom Row
      //  - HH:MM:SS time
      //  - protect from interrupts to maintain consistency of all values
      //  
      bytetodec2(BottomRow + 3,vs_hh);
      
      BottomRow[5] = 0x44;
  
      bytetodec2(BottomRow + 6,vs_mm);
      
      BottomRow[8] = 0x44;
  
      bytetodec2(BottomRow + 9,vs_ss);     
    }
    else
    {
      // What??? - shouldn't be here
      //
      CurrentMode = FailMode;
      // clear the current display lines
      //
      for( int i = 0; i < 30; i++ )
      {
        TopRow[i] = 0x00;
        BottomRow[i] = 0x00;
      }
      
      //  failure message
      //
      OSD.atomax(&BottomRow[1],(uint8_t*)msgUnknownError,len_msgUnknownError);
      
    } // end of handling non-failure modes
    
  }  // end of mode checks


  //**************************
  // Update OSD
  //  - send updated info to Max7456
  //
  if (EnableOverlay)
  {    
    OSD.sendArray(osdTop_RowOffset,TopRow,30);
  
    OSD.sendArray(osdBottom_RowOffset,BottomRow,30);
  
    #if (TESTROW == 1)
    OSD.sendArray(5*30,TestRow,30); // testing...
    #endif
    
  }

} // end of VSYNC_ISR

//===============================
//  PPS ISR = ICP4
//===============================
PPS_ISR()
{
  unsigned long timeCurrent;
  unsigned long timePrev;
  unsigned long timeDiff;
  unsigned long ppsDiff;

  // get the PPS time
  // 
  //
  timeCurrent = GetTicks(CR4);
  
  //*******************
  //   if Init, or Fail , just leave
  //
  if ( (CurrentMode == InitMode) || (CurrentMode == FailMode))
  {
    return;
  }

  //**********************
  //  echo PPS time to USB port
  //
  if (blnEchoPPS)
  {
    ultohexA((uint8_t *)msgEchoPPS + 3, timeCurrent);
    for (int i = 0; i < len_msgEchoPPS; i++)
    {
      Serial.write(msgEchoPPS[i]);
    }
  }
  

  //************************************
  //  Validate PPS interval
  //    if too long or too short => Error Mode (PPS error)
  //   

  // save the previous value for compare
  //
  timePrev = tk_PPS;
  tk_PPS = timeCurrent;

  // delay from last PPS
  //
  if (timeCurrent > timePrev)
  {
    timeDiff = timeCurrent - timePrev;
  }
  else
  {
    timeDiff = 0 - (timePrev - timeCurrent);
  }
  
  // PPS interval between now and the last PPS
  //   save this to be used by averaging code
  //
  ppsDiff = timeDiff;       
  
  // Check delay since last PPS pulse...
  //  if too short or too long and we are not in WaitingForGPS mode
  //      go to Error mode
  //
  if ( (timeDiff < (Timer_Second - PPS_TOLERANCE)) || (timeDiff > (Timer_Second + PPS_TOLERANCE)) && (CurrentMode != WaitingForGPS) )
  {

    // PPS issue!
    //   set error mode and leave
    //
    CurrentMode = ErrorMode;

    // clear the PPS interval average
    //
    tk_pps_interval_total = 0;
    tk_pps_interval_count = 0;
   
    // write error message
    //
    for( int i = 0; i < FIELDTOT_COL; i++ )   // clear all but the field count
    {
      BottomRow[i] = 0x00;
    }   
    OSD.atomax(&BottomRow[1], (uint8_t*)msgNoPPS, len_msgNoPPS);
    bytetodec2(BottomRow + len_msgNoPPS + 1,1);

    // set error countdown
    //
    ErrorCountdown = ERROR_DISPLAY_SECONDS;
    return;
  }

  // ***** PPS validated
  //
  tk_PPS_valid = tk_PPS;
   
  // rotation counter
  //
  osdRotation++;
  if (osdRotation == 4)
    osdRotation = 0;

  //  flag for "flash" on PPS
  //
  //
  pps_now = true;

 
  // OK to enable ints
  //
  interrupts();

  //********************************
  // Other Modes
  //  *Error Mode
  //    - If (error has displayed for two seconds)
  //        move to Waiting for GPS mode
  //    - else
  //        return (no mode change)
  //
  //  *WaitingForGPS Mode
  //    - if GPS data good
  //        move to Syncing mode
  //    - else
  //        leave (keep waiting)
  //
  //  *Syncing Mode
  //    - validate data and restart if necessary
  //
  //  *TimeValid Mode
  //    - check for GPS/UTC switch
  //
  //
  //
  if ( CurrentMode == ErrorMode )
  {
    // ERROR MODE
    // decrement counter and see if we are done displaying the message
    //
    ErrorCountdown--;
    if (ErrorCountdown <= 0)
    {
      // we can move to WaitingForGPS now
      //
      CurrentMode = WaitingForGPS;
      
      // write message
      //
      for( int i = 0; i < FIELDTOT_COL; i++ )   // clear all but the field count
      {
        BottomRow[i] = 0x00;
      }   
      OSD.atomax(&BottomRow[1], (uint8_t*)msgWaiting, len_msgWaiting);
      
    }
  }
  else if ( CurrentMode == TimeValid )
  {
    int ErrorFound;
    
    // TimeValid Mode
    //

    // update PPS interval statistics
    //
    tk_pps_interval_total += ppsDiff;
    tk_pps_interval_count++;

    // check for switch from GPS to UTC time
    //
    if (!time_UTC)
    {
      // time has been GPS based
      //  - do we have a valid almanac now?
      //
      if (gpsPUBX04.valid && gpsPUBX04.blnLeapValid)
      {
        // aha... we now have a current value for of GPS-UTC offset
        //  decrement time by this number of seconds and mark it as UTC time now...
        //
        for( int i = 0; i < gpsPUBX04.usLeapSec; i++)
        {
          SecDec();            
        }
        time_UTC = true;      // UTC now
      }
    }

    // NOW increment the time by one second for this PPS
    //
    SecInc();
    
  }
  else if ( CurrentMode == WaitingForGPS )
  {
    // WaitingForGPS mode
    //
    
    // we are waiting to synchronize
    //  if we have a valid time from the serial data, start the process
    //
    if ((!gpsRMC.valid) || (!gpsPUBX04.valid))
    {
      // no valid RMC or PUBX04, keep waiting
      return;
    }
    else if ((gpsRMC.mode != 'A') && (gpsRMC.mode != 'D'))
    {
      // wait for a good fix
      return;
    }
    else
    {
      // RMC time should correspond to the PREVIOUS second
      //
      noInterrupts();       // update hh:mm:ss as one operation
      sec_ss = gpsRMC.ss;
      sec_mm = gpsRMC.mm;
      sec_hh = gpsRMC.hh;
      SecInc();               // bump the count by one to match the second for THIS PPS signal    
      interrupts();
    }

    // GPS data looks good => move to Syncing mode (we will check the next five seconds for consistency)
    //
    CurrentMode = Syncing;
    TimeSync = SYNC_SECONDS;
    
    // update sync count & field count (already written)
    //
    for( int i = 0; i < FIELDTOT_COL; i++ )   // clear all but the field count
    {
      BottomRow[i] = 0x00;
    }   
    OSD.atomax(&BottomRow[1],(uint8_t*)msgSync,len_msgSync);
    ultodec(&BottomRow[1 + len_msgSync + 1],(unsigned long)TimeSync,0);
    
  }
  else if ( CurrentMode == Syncing )
  {
    int ErrorFound;
    
    // SYNCING mode
    //
    ErrorFound = 0;
            
    // validation 1 : was last PPS about one second away?  we checked this earlier
    //

    // validation 2: gpsRMC, PUBX04 good?
    //
    if ((!gpsRMC.valid) || (!gpsPUBX04.valid))
    {
      ErrorFound = 1;
    }
    else if ((gpsRMC.mode != 'A') && (gpsRMC.mode != 'D'))
    {
      ErrorFound = 3;
    }
    else
    {
      // compare time of RMC, PUBX04 vs time of PPS
      //
      if (timeCurrent > tk_GPSRMC)
      {
        timeDiff = timeCurrent - tk_GPSRMC;
      }
      else
      {
        timeDiff = 0 - (tk_GPSRMC - timeCurrent);
      }
      
      if (timeDiff > Timer_Second)
      {
        // oops... this RMC was sent MORE than one second befor this PPS
        ErrorFound = 4;    
      }
      else
      {
        if (timeCurrent > tk_PUBX04)
        {
          timeDiff = timeCurrent - tk_PUBX04;
        }
        else
        {
          timeDiff = 0 - (tk_PUBX04 - timeCurrent);
        }
        
        if (timeDiff > Timer_Second)
        {
          // oops... this RMC was sent MORE than one second befor this PPS
          ErrorFound = 5;
        }
        
      } // end of PUBX04 check 

      // ok... both RMC and PUBX04 look good
      
    } // end of checking timing to RMC/PUBX sentence
    
    
    // validation 3 : check the time stamp
    //   we have not yet incremented the time, so it should match the current NMEA value
    //   * note... if UTC offset updates in this timeframe it will break the sync ... but that is OK... it just restarts...
    //
    if (ErrorFound == 0)
    {
      noInterrupts();     // protect hh:mm:ss check
      if ((gpsRMC.hh != sec_hh) || (gpsRMC.mm != sec_mm) || (gpsRMC.ss != sec_ss))
      {
        ErrorFound = 2;
      }
      else if ((gpsRMC.mode != 'A') && (gpsRMC.mode != 'D'))
      {
        ErrorFound = 3;
      }
      interrupts();          
    }
    
    // if error, report it and return
    //
    if (ErrorFound > 0)
    {
      // failed the test - report error 
      //
      CurrentMode = ErrorMode;
      ErrorCountdown = ERROR_DISPLAY_SECONDS;

      noInterrupts();                 // clear the ave interval computation
      tk_pps_interval_total = 0;
      tk_pps_interval_count = 0;
      interrupts();
      
      // Error message
      //
      for( int i = 0; i < FIELDTOT_COL; i++ )   // clear all but the field count
      {
        BottomRow[i] = 0x00;
      }   
      OSD.atomax(&BottomRow[1],(uint8_t*)msgSyncFailed,len_msgSyncFailed);
      bytetodec2(BottomRow + len_msgSyncFailed + 2,(byte)ErrorFound + 3);
      return;
    }
    
    // this pps passed the test: bump the time and decrement the count
    //
    SecInc();  
    TimeSync --;
    
    // update sync count & field count (already written)
    //
    ultodec(&BottomRow[1 + len_msgSync + 1],(unsigned long)TimeSync,2);

    tk_pps_interval_total += ppsDiff;
    tk_pps_interval_count++;
#if (TESTROW == 1)
    if (tk_pps_interval_ave != 0)
    {
      ultodec(&TestRow[1], tk_pps_interval_ave,10);
    }
#endif    

    // are we all done with the sync?
    //
    if (TimeSync == 0)
    {
      // Set internal time according to most recent PPS and Leap Second status
      //

      // RMC time should correspond to the PREVIOUS second
      //
      noInterrupts();       // protect volatiles from changing ...
      sec_ss = gpsRMC.ss;
      sec_mm = gpsRMC.mm;
      sec_hh = gpsRMC.hh;
      SecInc();               // bump the count by one to match the second for THIS PPS signal
      
      // check for leap second status of the time from RMC
      //
      if (gpsPUBX04.blnLeapValid)
      {
        // we have an almanac => UTC time
        //
        time_UTC = true;
      }
      else
      {
        // using default leap seconds... increment time by leap seconds to match GPS time
        //
        time_UTC = false;
        for(int i = 0; i < gpsPUBX04.usLeapSec; i++)
        {
          SecInc();
        }
        
      } // end of check for leap second status

      // update PPS statistics
      //
      tk_pps_interval_ave = tk_pps_interval_total / tk_pps_interval_count;      // compute the average delay between PPS intervals
      Timer_Second = tk_pps_interval_ave;                                       // use this average as the new definition of a second
      Timer_100ms = Timer_Second / 10;
      
      tk_pps_interval_total = 0;                                                // reset
      tk_pps_interval_count = 0;

      
      interrupts();     // int back on again

      // We now have a valid time base... 
      //
      CurrentMode = TimeValid;
      
    }  // end of TimeSync = 0 section

  }
  else
  {
    // WHAT???
    //
    CurrentMode = FailMode;    
    
    // clear the current display lines
    //
    for( int i = 0; i < 30; i++ )
    {
      TopRow[i] = 0x00;
      BottomRow[i] = 0x00;
    }
    
    //  failure message
    //
    OSD.atomax(&BottomRow[1],(uint8_t*)msgUnknownError,len_msgUnknownError);
    
  }  // end of check for current mode
    
} // end of PPS_ISR

//========================================
// ISR for Timer 4 overflow
//=========================================
ISR( TIMER4_OVF_vect)
{
  
  timer4_ov++;   // just increment overflow count
  
} // end of Timer1 overflow vector

//========================================
// GetTicks() - get timer tick count from either TCNT4, TCNT5, ICP4 or ICP5
//    *** call with interrupts OFF
//    *** CNT4 & CNT5 should return the same value (timers are sync'd)
//=========================================
unsigned long GetTicks(CountSource src)
{
  unsigned long tickCount;

  if (src == CNT4)
  {
    tickCount = ((unsigned long)timer4_ov << 16) + (unsigned long)TCNT4;
  }
  else if (src == CNT5)
  {
    tickCount = ((unsigned long)timer4_ov << 16) + (unsigned long)TCNT5;
  }
  else if (src == CR4)
  {
    tickCount = ((unsigned long)timer4_ov << 16) + (unsigned long)ICR4;
  }
  else if (src == CR5)
  {
    tickCount = ((unsigned long)timer4_ov << 16) + (unsigned long)ICR5;
  }
  else
  {
    tickCount = 0;    // ** should be an ERROR!
  }
  
  // watch for overflow pending
  //
  if ( TIFR4 & (1 << TOV4) )
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

//=================================================
//  SecInc - increment second 
//   note: interrupts are disabled while making this change across multiple values
//=================================================
void SecInc()
{
  uint8_t savSREG;

  savSREG = SREG;
  noInterrupts();       // disable all interrupts to protect this operation
  sec_ss++;
  if (sec_ss == 60)
  {
    sec_ss = 0;
    sec_mm++;
    if (sec_mm == 60)
    {
      sec_mm = 0;
      sec_hh++;
      if (sec_hh == 24)
      {
        sec_hh = 0;
      }
    }
  }
  SREG = savSREG;     // restore interrupt status
  
} // end of SecInc

//=================================================
//  SecDec - decrement second 
//   note: interrupts are disabled while making this change across multiple values
//=================================================
void SecDec()
{
  uint8_t savSREG;

  savSREG = SREG;
  noInterrupts();       // disable all interrupts to protect this operation
  sec_ss--;
  if (sec_ss == -1)
  {
    sec_ss = 59;
    sec_mm--;
    if (sec_mm == -1)
    {
      sec_mm = 59;
      sec_hh--;
      if (sec_hh == -1)
      {
        sec_hh = 23;
      }
    }
  }
  SREG = savSREG;     // restore interrupt status
  
} // end of SecDec

//**********************************************************************************************************
// NEO 6 GPS routines
//**********************************************************************************************************
#define gpsSerial Serial1
#define gps_OK        0
#define gps_E_RMC     1
#define gps_E_GGA     2
#define gps_E_DTM     3
#define gps_E_PUBX04  4
#define gps_E_CFGTP   5
#define gps_E_CFGDTM  6

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

  // configure timepulse
  //
  ubxSend(configTimepulse,sizeof(configTimepulse)/sizeof(uint8_t));
  if (!ubxGetAck(configTimepulse))
  {
    return gps_E_CFGTP;
  }
 
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

// NMEA return codes
//
#define NMEA_ERROR    0
#define NMEA_UNKNOWN  1
#define NMEA_RMC      2
#define NMEA_GGA      3
#define NMEA_DTM      4
#define NMEA_PUBX04   5

//=============================================================
//  ReadGPS - gather and parse any pending serial data from GPS
//    returns false if error parsing data
//=============================================================
bool ReadGPS()
{
  char c;
  int resultParse;
  long internalSec;
  long ubxSec;
  int ErrorFound;

  //************
  // Read/process all currently available characters
  //  nmeaCount == 0 => not in a sentence now
  //

  while (gpsSerial.available() > 0)
  {
    // get the char
    //
    c = gpsSerial.read();

    // echo it to the USB port
    //
    Serial.write(c);

    // Watch for beginning/ending of a sentence
    //

    if (c == '$')
    {
      // start of a sentence
      //
      noInterrupts();
      tk_NMEAStart = GetTicks(CNT4);
      interrupts();
      
      // save the current hh:mm:ss time
      //
      noInterrupts();
      n_blnUTC = time_UTC;
      n_hh = sec_hh;
      n_mm = sec_mm;
      n_ss = sec_ss;
      interrupts();
      
      // are we in the right place?
      //
      if (nmeaCount > 0)
      {
        // oops! - currently in a sentence, should not be here
        //
        return false;
      }

      //  ok, save the start char and get ready for next one
      //
      nmeaSentence[0] = (uint8_t)c;
      nmeaCount = 1;
      
    }
    else
    {
      // we are somewhere IN a sentence
      //

      // are we in the right place?
      //
      if (nmeaCount <= 0)
      {
        // oops! - not in a sentence now
        //
        return false;
      }

      // ok, save the end of the sentence and call the parser
      //
      nmeaSentence[nmeaCount] = (uint8_t)c;
      nmeaCount++;                      

      // too many?
      //
      if (nmeaCount >= NMEA_MAX)
      {
        nmeaCount = 0;      // drop all of it 
        continue;
      }

      // if we just ended a sentence, call the parser
      //
      if (c == '\n')
      {
        nmeaSentence[nmeaCount] = 0;      // null terminate the sentence

        // call the parser
        //
        resultParse = ParseNMEA();
        if (resultParse == NMEA_ERROR)
        {
          nmeaCount = 0;   // restart
          return false;   // exit on parsing error
        }

        // save time of RMC and PUBX04 sentences
        //
        if (resultParse == NMEA_RMC)
        {
          
          // NMEA_RMC sentence...
          //
          tk_GPSRMC = tk_NMEAStart;     // save start time
          
        }
        else if (resultParse == NMEA_PUBX04)
        {
          tk_PUBX04 = tk_NMEAStart;     // save start time

          // update current or default GPS-UTC offset value
          //
          if (gpsPUBX04.blnLeapValid)
          {
            offsetUTC_Current = gpsPUBX04.usLeapSec;
          }
          else
          {
            offsetUTC_Default = gpsPUBX04.usLeapSec;
          }
          
          // now check PUBX04 time against internal time to catch errors...
          //
          ErrorFound = 0;
          if ( CurrentMode == TimeValid )
          {
            
            internalSec = n_hh*3600 + n_mm*60 + n_ss;   // internal seconds of the day
            
            if (!n_blnUTC)
            {
              // internal time is GPS
              //
              ubxSec = gpsPUBX04.hh*3600 + gpsPUBX04.mm * 60 + gpsPUBX04.ss;
              if (gpsPUBX04.blnLeapValid )
              {
                ubxSec += offsetUTC_Current;      // move to GPS time
              }
              else
              { 
                ubxSec += offsetUTC_Default;      // move to GPS time
              }
              
              if (ubxSec != internalSec)
              {
                ErrorFound = 10;
              }            
              
            }
            else
            {
              // internal time is UTC
              //   UBX time should be UTC also
              //
              ubxSec = gpsPUBX04.hh*3600 + gpsPUBX04.mm * 60 + gpsPUBX04.ss;
              if (ubxSec != internalSec)
              {
                ErrorFound = 11;
              }
             
            } // end of checking internal time for UTC
          
            // Did we find an error?
            //
            if (ErrorFound > 0)
            {
              CurrentMode = ErrorMode;
              ErrorCountdown = ERROR_DISPLAY_SECONDS;

              noInterrupts();                 // clear the ave interval computation
              tk_pps_interval_total = 0;
              tk_pps_interval_count = 0;
              interrupts();
              
              // Error message
              //
              for( int i = 0; i < FIELDTOT_COL; i++ )   // clear all but the field count
              {
                BottomRow[i] = 0x00;
              }   
              OSD.atomax(&BottomRow[1],(uint8_t*)msgNoPPS,len_msgSyncFailed);
              bytetodec2(BottomRow + len_msgNoPPS + 2,(byte)ErrorFound);
            }
              
          } // end of RMC check against internal time
          
        } // end of processing for RMC,PUBX04 data
        
        // looking for new sentence now...
        //
        nmeaCount = 0;
       
      } // found the end of a sentence
      
    }  // end of check for start/non-start char
    
  } // end of loop through available characters
  
  
} // end of ReadGPS

//=============================================================
//  ParseNMEA - parse the current NMEA sentence
//    currently supports the following sentences
//        RMC, GGA, DTM, and PUBX,04
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    nmeaCount = # of chars in sentence (including terminating CRLF)
//=============================================================
int ParseNMEA()
{
  int iField;       // current field #
  int fieldCount;   // # of fields in sentence (including CRLF)
  int iPos;         // current position in nmea string

  int fStart;
  int fLen;
  
  //********
  // find the start,end of each field
  //
  iField = 0;
  fieldStart[iField] = 0;
  iPos = 1;                 // next char to be tested

  while (iPos < nmeaCount)
  {
    // start of a new field?
    //
    if ((char)nmeaSentence[iPos] == ',')
    {
      //  end of this field inside the sentence
      //
      iField++;           // new field start
      iPos++;             // start with position past ','
      if (iPos >= nmeaCount)
      {
        return NMEA_ERROR;     // no start of next field => unexpected end of sentence
      }
      fieldStart[iField] = iPos;

    }
    else if ((char)nmeaSentence[iPos] == '\r')
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
  if ((char)nmeaSentence[iPos] != '\n')
  {
    return NMEA_ERROR;       // terminated loop without finding CRLF
  }
  fieldCount = iField + 1;
  if (fieldCount < 3)
  {
    return NMEA_ERROR;     // all sentences have at least three fields
  }
  
  //************
  // found the fields, now parse sentence data
  //  
  fStart = fieldStart[0];                 // start of message ID field
  fLen = fieldStart[1] - fStart - 1;    // length of message ID field

  if ( ((char)nmeaSentence[fStart] == '$') &&
        ((char)nmeaSentence[fStart+1] == 'G') &&
        ((char)nmeaSentence[fStart+2] == 'P') )
  {
    // this is a standard NMEA sentence starting with $GP
    //
    if (fLen < 6)
    {
      // unknown message - just return no error for now
      return NMEA_UNKNOWN;
    }
    
    if ( ((char)nmeaSentence[fStart+3] == 'R') &&
          ((char)nmeaSentence[fStart+4] == 'M') &&
          ((char)nmeaSentence[fStart+5] == 'C') )
    {
      return ParseRMC(fieldCount);
    }
    else if ( ((char)nmeaSentence[fStart+3] == 'G') &&
          ((char)nmeaSentence[fStart+4] == 'G') &&
          ((char)nmeaSentence[fStart+5] == 'A') )
    {
      return ParseGGA(fieldCount);
    }
    else if ( ((char)nmeaSentence[fStart+3] == 'D') &&
          ((char)nmeaSentence[fStart+4] == 'T') &&
          ((char)nmeaSentence[fStart+5] == 'M') )
    {
      return NMEA_DTM;
    }
    else
    {
        return NMEA_UNKNOWN;
    } // end of if/else block parsing standard NMEA sentences
  }
  else if ( ((char)nmeaSentence[fStart] == '$') &&
          ((char)nmeaSentence[fStart+1] == 'P') &&
          ((char)nmeaSentence[fStart+2] == 'U') &&
          ((char)nmeaSentence[fStart+3] == 'B') &&
          ((char)nmeaSentence[fStart+4] == 'X'))
  {
    // this is a UBX proprietary sentence
    //  check field 1 for the sentence type
    //
    fStart = fieldStart[1];
    fLen = fieldStart[2] - fStart - 1;
    if ( ((char)nmeaSentence[fStart] == '0') &&
          ((char)nmeaSentence[fStart+1] == '4') )
    {
      // PUBX,04 sentence
      //
      return ParsePUBX04(fieldCount);
    } 
    else
    {
      // unknown PUBX sentence => do nothing
      //
      return NMEA_UNKNOWN;
      
    }// end of check for PUBX sentence type
  }
  else
  {
    // unknown sentence type
    //
    //  DO NOTHING and no error
    return NMEA_UNKNOWN;
  }
  
} // end of ParseNMEA

//=============================================================
//  ParseGGA - parse & save the GGA data of interest
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    fieldStart[] - array of starting indicies for fields
//    fieldCount = # of fields (including CRLF)
//=============================================================
int ParseGGA(int fieldCount)
{
  int iStart;
  int iLen;
  char cTmp;

  gpsGGA.valid = false;

#if 0
  // should be 17 fields including the CRLF at the end
  //
  if (fieldCount < 17)
  {
    return NMEA_ERROR;
  }
#endif

  //******************************
  // field 2 = Latitude
  //
  iStart = fieldStart[2];
  iLen = fieldStart[3] - iStart - 1;

  if ((iLen < 5) || (iLen > MAX_LATLONG))
  {
    return NMEA_ERROR; 
  }

  for( int i = 0; i < MAX_LATLONG; i++ )
  {
    if (i < iLen)
    {
      gpsGGA.lat[i] = nmeaSentence[iStart + i];    
    }
    else
    {
      gpsGGA.lat[i] = ' ';  // pad with spaces on the end
    }
  }

  //**************************
  // field 3 = N/S for latitude
  //
  iStart = fieldStart[3];
  iLen = fieldStart[4] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR; 
  }
  gpsGGA.NS = (char)nmeaSentence[iStart];  
  if ( ((char)gpsGGA.NS != 'N') && ((char)gpsGGA.NS != 'n') && 
          ((char)gpsGGA.NS != 'S') && ((char)gpsGGA.NS != 's') )
  {
    return NMEA_ERROR;
  }

  //******************************
  // field 4 = Longitude
  //
  iStart = fieldStart[4];
  iLen = fieldStart[5] - iStart - 1;

  if ((iLen < 5) || (iLen > MAX_LATLONG))
  {
    return NMEA_ERROR; 
  }

  for( int i = 0; i < MAX_LATLONG; i++ )
  {
    if (i < iLen)
    {
      gpsGGA.lng[i] = nmeaSentence[iStart + i];    
    }
    else
    {
      gpsGGA.lng[i] = ' ';  // pad with spaces on the end
    }
  }

  //**************************
  // field 5 = E/W for Longitude
  //
  iStart = fieldStart[5];
  iLen = fieldStart[6] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR; 
  }
  gpsGGA.EW = (char)nmeaSentence[iStart];
  if ( ((char)gpsGGA.EW != 'E') && ((char)gpsGGA.EW != 'e') && 
          ((char)gpsGGA.EW != 'W') && ((char)gpsGGA.EW != 'w') )
  {
    return NMEA_ERROR;
  }

  //******************************
  // field 9 = MSL altitude
  //
  iStart = fieldStart[9];
  iLen = fieldStart[10] - iStart - 1;

  if ((iLen < 1) || (iLen > MAX_ALT))
  {
    return NMEA_ERROR; 
  }

  for( int i = 0; i < MAX_ALT; i++ )
  {
    if (i < iLen)
    {
      gpsGGA.alt_msl[i] = nmeaSentence[iStart + i];    
    }
    else
    {
      gpsGGA.alt_msl[i] = ' ';  // pad with spaces on the end
    }
  }

  //**************************
  // field 10 = units for altitude
  //
  iStart = fieldStart[10];
  iLen = fieldStart[11] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR; 
  }
  gpsGGA.alt_units = (char)nmeaSentence[iStart];
  if ((gpsGGA.alt_units != 'M') && (gpsGGA.alt_units != 'm'))      // must be "m"
  {
    return NMEA_ERROR;
  }

  //******************************
  // field 11 = geoid separation
  //
  iStart = fieldStart[11];
  iLen = fieldStart[12] - iStart - 1;

  if ((iLen < 1) || (iLen > MAX_ALT))
  {
    return NMEA_ERROR; 
  }

  for( int i = 0; i < MAX_ALT; i++ )
  {
    if (i < iLen)
    {
      gpsGGA.geoid_sep[i] = nmeaSentence[iStart + i];    
    }
    else
    {
      gpsGGA.geoid_sep[i] = ' ';  // pad with spaces on the end
    }
  }

  //**************************
  // field 12 = units for altitude
  //
  iStart = fieldStart[12];
  iLen = fieldStart[13] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR; 
  }
  gpsGGA.sep_units = (char)nmeaSentence[iStart];
  if ((gpsGGA.sep_units != 'M') && (gpsGGA.sep_units != 'm'))      // must be "m"
  {
    return NMEA_ERROR;
  }
  
  //**********
  // all done
  //
  gpsGGA.valid = true;
  
  return NMEA_GGA;
  
} // end of parseGGA

//=============================================================
//  ParseRMC - parse & save the RMC data of interest
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    fieldStart[] - array of starting indicies for fields
//    fieldCount = # of fields (including CRLF)
//=============================================================
int ParseRMC(int fieldCount)
{
  int iStart;
  int iLen;

  gpsRMC.valid = false;

#if 0
  // should be 14 fields including the CRLF at the end
  //
  if (fieldCount < 14)
  {
    return NMEA_ERROR;
  }
#endif

  //******************************
  // field 1 = HH:MM:SS time
  //
  iStart = fieldStart[1];
  iLen = fieldStart[2] - iStart - 1;

  if (iLen < 6)
  {
    return NMEA_ERROR; 
  }

  //*** protect from interrupts ***
  //  hh,mm,ss from the RMC sentence may be used by ISR for the PPS signal
  //  we should keep these changes "atomic"
  //
  noInterrupts();
  gpsRMC.hh = d2i( &nmeaSentence[iStart]);
  if (gpsRMC.hh < 0)
  {
     interrupts();
    return NMEA_ERROR;
  }
  gpsRMC.mm = d2i( &nmeaSentence[iStart+2]);
  if (gpsRMC.mm < 0)
  {
    interrupts();
    return NMEA_ERROR;
  }
  gpsRMC.ss = d2i( &nmeaSentence[iStart+4]);
  if (gpsRMC.ss < 0)
  {
    interrupts();
    return NMEA_ERROR;
  }
  interrupts();
  
  //****************************
  // field 9 = ddmmyy Date 
  //
  iStart = fieldStart[9];
  iLen = fieldStart[10] - iStart - 1;

  if (iLen < 6)
  {
    return NMEA_ERROR; 
  }
  
  gpsRMC.day = d2i( &nmeaSentence[iStart]);
  if (gpsRMC.day < 0)
  {
    return NMEA_ERROR;
  }
  gpsRMC.mon = d2i( &nmeaSentence[iStart+2]);
  if (gpsRMC.mon < 0)
  {
    return NMEA_ERROR;
  }
  gpsRMC.yr = d2i( &nmeaSentence[iStart+4]);
  if (gpsRMC.yr < 0)
  {
    return NMEA_ERROR;
  }

  //**************
  // field 12 - mode indicator
  //
  iStart = fieldStart[12];
  iLen = fieldStart[13] - iStart - 1;

  if (iLen < 1)
  {
    return NMEA_ERROR; 
  }
  gpsRMC.mode = (char)nmeaSentence[iStart];     // mode char
  
  //**********
  // all done
  //
  gpsRMC.valid = true;
  
  return NMEA_RMC;
  
} // end of parseRMC

//=============================================================
//  ParsePUBX04 - parse & save the PUBX04 data of interest
//  INPUTS:
//    nmeaSentence[] - array of chars containing sentence
//    fieldStart[] - array of starting indicies for fields
//    fieldCount = # of fields (including CRLF)
//=============================================================
int ParsePUBX04(int fieldCount)
{
  int iStart;
  int iLen;
  int iTmp;

  gpsPUBX04.valid = false;

#if 0
  // should be 12 fields including the CRLF at the end
  //
  if (fieldCount < 12)
  {
    return NMEA_ERROR;
  }
#endif

  //******************************
  // field 2 = hhmmss.ss
  //
  iStart = fieldStart[2];
  iLen = fieldStart[3] - iStart - 1;

  if (iLen < 6)
  {
    return NMEA_ERROR; 
  }

  gpsPUBX04.hh = d2i( &nmeaSentence[iStart]);
  if (gpsPUBX04.hh < 0)
  {
    return NMEA_ERROR;
  }
  gpsPUBX04.mm = d2i( &nmeaSentence[iStart+2]);
  if (gpsPUBX04.mm < 0)
  {
    return NMEA_ERROR;
  }
  gpsPUBX04.ss = d2i( &nmeaSentence[iStart+4]);
  if (gpsPUBX04.ss < 0)
  {
    return NMEA_ERROR;
  }
  
  //******************************
  // field 6 = LEAP seconds
  //
  iStart = fieldStart[6];
  iLen = fieldStart[7] - iStart - 1;

  // this field should always be two digits with an optional 'D' at the end
  //
  if ((iLen < 2) || (iLen > 3))
  {
    return NMEA_ERROR; 
  }
  gpsPUBX04.cLeap[0] = nmeaSentence[iStart];
  gpsPUBX04.cLeap[1] = nmeaSentence[iStart+1];

  // decode the two digit leap second count
  //
  iTmp = d2i(&nmeaSentence[iStart]);
  if (iTmp < 0)
  {
    return NMEA_ERROR;
  }
  gpsPUBX04.usLeapSec = (uint8_t)iTmp;

  // is this terminated with a 'D' to indicate no almanac yet?
  //
  if (iLen == 2)
  {
    // no terminating 'D' => alamanac is up to date and time is UTC
    //
    gpsPUBX04.blnLeapValid = true;
    gpsPUBX04.cLeap[2] = 0x20;        // space at end
  }
  else
  {
    // terminatinting D => using firmware default leap seconds
    //
    if (nmeaSentence[iStart + 2] == 'D')
    {
      gpsPUBX04.blnLeapValid = false;
      gpsPUBX04.cLeap[2] = 0x44;        // 'D'
    }
    else
    {
      return NMEA_ERROR;
    }
  }
   
  //**********
  // all done
  //
  gpsPUBX04.valid = true;
  
  return NMEA_PUBX04;
  
} // end of parsePUBX04

//*****************************************************************************************
// Utility routines
//*****************************************************************************************

//===========================================================================
// ultohex - convert unsigned long to 8 hex MAX7456 characters in a character array
//
//===========================================================================
uint8_t hex[16] = {0x0A,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0B,0x0C,0x0D,0x0E,0x0F,0x10};
void ultohex(uint8_t *dest, unsigned long ul)
{


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
// ultohexA - convert unsigned long to 8 hex ASCII characters in a character array
//
//===========================================================================
uint8_t hexA[16] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x41,0x42,0x43,0x44,0x45,0x46};
void ultohexA(uint8_t *dest, unsigned long ul)
{


  uint8_t *pn;
  unsigned long nibble;

  pn= dest + 7;
  
  for(int i = 0; i < 8; i++)
  {

    // get nibble 
    //
    nibble = (ul & 0xF);
    *pn = hexA[nibble];

    // move to next nibble
    //
    ul = ul >> 4;
    pn--;
    
  } // end of for loop through the nibbles
  
} // end of ultohex

//===========================================================================
// ustohex - convert unsigned short to 4 hex MAX7456 characters in a character array
//
//===========================================================================
void ustohex(uint8_t *dest, unsigned short us)
{


  uint8_t *pn;
  unsigned short nibble;

  pn= dest + 3;
  
  for(int i = 0; i < 4; i++)
  {

    // get nibble 
    //
    nibble = (us & 0xF);
    *pn = hex[nibble];

    // move to next nibble
    //
    us = us >> 4;
    pn--;
    
  } // end of for loop through the nibbles
  
} // end of ustohex

//===========================================================================
// bytetohex - convert byte to 2 hex MAX7456 characters in a character array
//
//===========================================================================
void bytetohex(uint8_t *dest, uint8_t byt)
{

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
//    total = field length, 0 => left aligned with no padding (10 char max)
//                        , > 0 => pad with zeros on left to field length 
//
//    ** note: if total > 0 and ul doesn't fit, then writes only least significant digits
//===========================================================================
unsigned long powers[10] = {
  1000000000,
  100000000,
  10000000,
  1000000,
  100000,
  10000,
  1000,
  100,
  10,
  1
};
void ultodec(uint8_t *dest, unsigned long ul, int total)
{
  unsigned long pwr10;
  uint8_t digit;
  uint8_t count = 0;      // > 0 => write zeros
  uint8_t start = 0;      // # start writing at this index into powers

  // check for fixed length zero padding
  //
  if ((total > 0) && (total <= 10))
  {
    count = 1;     // => pad with leading zeros
    start = 10 - total;
  }
  
  // check each power from high to low - find the divisor at each level
  //    
  //
  for(int i = 0; i < 10; i++)
  {
    pwr10 = powers[i];
    if (pwr10 > ul)
    {
      if (count > 0)
      {
        // we already have some digits (or we are zero padding) => add a zero in this position
        //
        if ( i >= start)
        {
          *dest = hex[0x00];
          dest++;
          count++;
        }
      }
      // current number is lower than this...
      continue;
    }
    else
    {
      // current number (ul) is higher or equal to the current power of 10
      //
      digit = 0;
      while( pwr10 <= ul)
      {
        ul -= pwr10;
        digit++;
      }
      // current number is now less than current power (pwr10)
      // AND ul was digit * pwr10
      //   set the destination char
      //
      if ( i >= start)
      {
        *dest = hex[digit];
        dest++;
        count++;              // # of digits written so far
      }
    }
  } // end of loop through powers
  
} // end of ultodec

//===========================================================================
// bytetodec2 - convert unsigned byte to zero padded two decimal MAX7456 characters in a character array
//    dest = ptr to destination of most significant char of decimal number
//    us = uint8_t value to convert
//
//    ** note: if us > 99, nothing is written
//===========================================================================
void bytetodec2(uint8_t *dest, uint8_t us)
{
  unsigned long pwr10;
  uint8_t digit;
  uint8_t count = 0;
  uint8_t start = 0;

  // sanity check
  //
  if (us > 99)
  {
    return;     // do nothing
  }

  // how many 10s?
  //
  digit = 0;
  while( 10 <= us)
  {
    us -= 10;
    digit++;
  }
  *dest = hex[digit];   // set the char for the 10s
  dest++;

  // now the 1east significan digit
  //
  *dest = hex[us];
  

} // end of bytetodec2

//===========================================================================
// d2i - decode two POSITIVE ascii digits to int value
//          * on error or negative value, returns negative value for error
//
//===========================================================================
int d2i(uint8_t *src)
{
    int val;

    //  first char
    // 0x20 = ASCII space
    if (*src == 0x20)
    {
      // space for left digit
      //
      val = 0;
      src++;
    }
    else
    {
      // non-space char
      if ((*src < 0x30) || (*src > 0x39))   // 0x30 = '0'
      {
        return -1;
      }
  
      val = (*src - 0x30)*10;
  
      src++;
    }

    // right most digit
    //
    if ((*src < 0x30) || (*src > 0x39))
    {
      return -1;
    }
    val += (*src - 0x30);

    return val;
    
} // end of atob2

//================================================================
//  getFlashCRC
//   - compute CRC code for all program memory up to the specified last address
//================================================================
uint16_t getFlashCRC( uint32_t lastAddr )
{
  uint32_t addr;
  uint8_t  byte_u8;
  uint16_t crc_u16;


  /* Compute the CRC */
  crc_u16 = 0;
  for( addr = 0; addr < lastAddr; addr++)
  {
    byte_u8 = pgm_read_byte_far( addr );
    crc_u16 = _crc16_update( crc_u16, byte_u8 );
  }

  return( crc_u16 );

} // end of getFlashCRC

