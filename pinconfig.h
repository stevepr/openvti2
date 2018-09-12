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
  //  Max7456 CS    [~CS]            <---  Mega2560 53  [PB0](SS/PCINT0)
  const byte osdChipSelect             =            53;
  
  //  Max7456 DIN   [SDIN]           <---  Arduino  11  [PB3](MOSI/OC2)
  //  Max7456 DIN   [SDIN]           <---  Mega2560 51  [PB2](MOSI)
  const byte masterOutSlaveIn          =            51;
  
  //  Max7456 DOUT  [SDOUT]           ---> Arduino  12  [PB4](MISO)
  //  Max7456 DOUT  [SDOUT]           ---> Mega2560 50  [PB3](MISO)
  const byte masterInSlaveOut          =            50;
  
  //  Max7456 SCK   [SCLK]           <---  Arduino  13  [PB5](SCK)
  //  Max7456 SCK   [SCLK]           <---  Mega2560 52  [PB1](SCK)
  const byte slaveClock                =            52;
  
  //  Max7456 RST   [~RESET]          ---  Arduino  RST      (RESET)
  const byte osdReset                  =            0;

  //  **********  need to solder connections for ~VSYNC and ~HSYNC
  //  Max7456 VSYNC [~VSYNC]         ---> Arduino   48   [ICP5]
  //  Max7456 HSYNC [~HSYNC]         ---> Arduino   44   [PL5]
  //  Max7456 LOS   [LOS]             -X-

  // GPS mapping

  // GPS VCC [3.3v]                  <---  Arduino  3.3V power
  // GPS GND                         <-->  Arduino  GND
  // GPS PPS [3.3v] [INPUT]          --->  Arduino  49   [ICP4] 
  //  
  // GPS TXD [3.3v]                  --->  Arduino  0   [RXD, PD0]
  // GPS RXD [3.3v]                  <---  Arduino  1   [TXD, PD1] 

/////////////////////////  defines for init and access /////////////////////////



  // VSYNC 
  //  connected to ICP for Timer 5 = ICP5
  //
#define VSYNC         48
#define VSYNC_PIN     PL1    
#define VSYNC_DDR     DDRL
#define VSYNC_PINR    PINL
#define VSYNC_CFG_INPUT() (VSYNC_DDR &= ~_BV(VSYNC_PIN))
#define VSYNC_READ() (VSYNC_PINR & _BV(VSYNC_PIN))

#define VSYNC_ISR() ISR(TIMER5_CAPT_vect)

  // HSYNC
  //  connected to digital pin 2 
  //
#define HSYNC       44
#define HSYNC_PIN   PL5       
#define HSYNC_DDR   DDRL
#define HSYNC_PINR  PINL
#define HSYNC_CFG_INPUT() (HSYNC_DDR &= ~_BV(HSYNC_PIN))
#define HSYNC_READ() (HSYNC_PINR & _BV(HSYNC_PIN))

  // PPS
  //  Connected to edge triggered ICP for Timer 4 = ICP4
  //
#define PPS         49
#define PPS_PIN     PL0       
#define PPS_DDR     DDRL
#define PPS_PINR    PINL
#define PPS_CFG_INPUT()   (PPS_DDR &= ~_BV(PPS_PIN))
#define PPS_READ()        (PPS_PINR & _BV(PPS_PIN))

#define PPS_ISR()           ISR(TIMER4_CAPT_vect)

#endif // PINCONFIG_H_

