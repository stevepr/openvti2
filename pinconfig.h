#ifndef PINCONFIG_H_
#define PINCONFIG_H_

// Pin Mapping /////////////////////////////////////////////////////////////////
  
  // pinValue = 0 means "not connected"

  //  FDTI Basic 5V                   ---  Arduino  VCC      (AVCC,VCC)
  //  FDTI Basic GND                  ---  Arduino  GND      (AGND,GND)
  //  FDTI Basic CTS                  ---  Arduino  GND      (AGND,GND)
  //  FDTI Basic DTR                  ---  Arduino  GRN
  //  FDTI Basic TXO                  ---> Arduino  0        [RXD, PD0]
  //  FDTI Basic RXI                 <---  Arduino  1        [TXD, PD1]
  
  
  //  Max7456 +5V   [DVDD,AVDD,PVDD]  ---  Arduino  VCC      (AVCC,VCC)
  //  Max7456 GND   [DGND,AGND,PGND]  ---  Arduino  GND      (AGND,GND)
  //  Max7456 CS    [~CS]            <---  Arduino  10  [PB2](SS/OC1B)
  //  Max7456 CS    [~CS]            <---  Mega2560 43  [PL6]
  const byte osdChipSelect             =            10;
  
  //  Max7456 DIN   [SDIN]           <---  Arduino  11  [PB3](MOSI/OC2)
  //  Max7456 DIN   [SDIN]           <---  Mega2560 51  [PB2](MOSI)
  const byte masterOutSlaveIn          =                      MOSI;
  
  //  Max7456 DOUT  [SDOUT]           ---> Arduino  12  [PB4](MISO)
  //  Max7456 DOUT  [SDOUT]           ---> Mega2560 50  [PB3](MISO)
  const byte masterInSlaveOut          =                      MISO;
  
  //  Max7456 SCK   [SCLK]           <---  Arduino  13  [PB5](SCK)
  //  Max7456 SCK   [SCLK]           <---  Mega2560 52  [PB1](SCK)
  const byte slaveClock                =                      SCK;
  
  //  Max7456 RST   [~RESET]          ---  Arduino  RST      (RESET)
  const byte osdReset                  =            0;

  //  **********  need to solder connections for ~VSYNC and ~HSYNC
  //  Max7456 VSYNC [~VSYNC]         ---> Arduino   8   [INT0, PCINT18, PD2]
  //  Max7456 HSYNC [~HSYNC]         ---> Arduino   2   [PCINT0, PB0, ICP1]
  //  Max7456 LOS   [LOS]             -X-

  // GPS mapping

  // GPS VCC [3.3v]                  <---  Arduino  3.3V power
  // GPS GND                         <-->  Arduino  GND
  // GPS PPS [3.3v] [INPUT]          --->  Arduino  3   [INT1, PD3] 
  //   **** NOTE: GPS serial is on h/w serial which is shared with USB port, disconnect during programming
  // GPS TXD [3.3v]                  --->  Arduino  0   [RXD, PD0]
  // GPS RXD [3.3v]                  <---  Arduino  1   [TXD, PD1] 

/////////////////////////  defines for init and access /////////////////////////

  // startup mode
  //    GND / 0 => programming mode
  //    GND / 1 => run mode
  //
#define STARTUP   7           // digital pin 7
#define STARTUP_PIN   PD7     //
#define STARTUP_DDR   DDRD
#define STARTUP_PINR  PIND
#define STARTUP_CFG_INPUT() (STARTUP_DDR &= ~_BV(STARTUP_PIN))
#define STARTUP_READ() (STARTUP_PINR & _BV(STARTUP_PIN))
 

  // VSYNC 
  //  connected to ICP for Timer 1 = ICP1
  //
#define VSYNC 8
#define VSYNC_PIN     PB0     // aka PCINT0 & ICP1
#define VSYNC_DDR     DDRB
#define VSYNC_PINR    PINB
#define VSYNC_CFG_INPUT() (VSYNC_DDR &= ~_BV(VSYNC_PIN))
#define VSYNC_READ() (VSYNC_PINR & _BV(VSYNC_PIN))

#define VSYNC_ISR() ISR(TIMER1_CAPT_vect)

  // HSYNC
  //  connected to digital pin 2 
  //
#define HSYNC 2
#define HSYNC_PIN   PD2       // aka PCINT18
#define HSYNC_DDR   DDRD
#define HSYNC_PINR  PIND
#define HSYNC_CFG_INPUT() (HSYNC_DDR &= ~_BV(HSYNC_PIN))
#define HSYNC_READ() (HSYNC_PINR & _BV(HSYNC_PIN))

  // PPS
  //  Connected to edge triggered INT1 = digital pin 3
  //
#define PPS         3
#define PPS_PIN     PD3       // aka INT1 / PCINT19
#define PPS_DDR     DDRD
#define PPS_PINR    PIND
#define PPS_CFG_INPUT()   (PPS_DDR &= ~_BV(PPS_PIN))
#define PPS_READ()        (PPS_PINR & _BV(PPS_PIN))

// set these macros to enable the RISING edge interrupt for the 
// pin select for PPS = PD3 = INT1 = PCINT19
//   - enable the EICRA ints for INT1
//   - disable the PCMSK for PCINT19
//
#define PPS_CFG_EICRA()     (EICRA = (EICRA | 0b00001100))
#define PPS_CFG_EIMSK()     (EIMSK |= (1 << INT1))
#define PPS_CFG_PCMSK()     (PCMSK2 &= ~(1 << PCINT19))
#define PPS_ISR()           ISR(INT1_vect)


#endif // PINCONFIG_H_

