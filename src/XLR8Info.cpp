/*--------------------------------------------------------------------
  Copyright (c) 2015 Alorium Technology.  All right reserved.
  This library queries information from an XLR8 board.
  Written by Matt Weber (support@aloriumtech.com) of
  Alorium Technology (info@aloriumtech.com)
  
  
  This library is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library.  If not, see
  <http://www.gnu.org/licenses/>.
  --------------------------------------------------------------------*/


#include "XLR8Info.h"
#include <avr/pgmspace.h>

#define XLR8VERS      _SFR_MEM16(0xd4)
#define XLR8VERSL     _SFR_MEM8(0xd4)
#define XLR8VERSH     _SFR_MEM8(0xd5)
#define XLR8VERST     _SFR_MEM8(0xd6)
#define FCFGCID       _SFR_MEM8(0xcf)
#define CLKSPD        _SFR_IO8(0x29)
#define XLR8VERS_OLD  _SFR_MEM16(0x90)
#define FCFGCID_OLD   _SFR_MEM8(0xd8)
#define XBINFO        _SFR_MEM8(0xFF)

XLR8Info::XLR8Info(void) : designConfig(0),XBEnables(0) { // constructor
  // xlr8Version = (uint16_t)XLR8VERSL | ((uint16_t)XLR8VERSH << 8);
  xlr8Version = XLR8VERS;
  // Get the config information from the FPGA design
  if (xlr8Version < 2161) { // Pre-Hinj version of flashload
    initializeCidStateMachine();
    chipId = getNextConfigWord();// Chip id comes out first from the CID register
    designConfig = FCFGCID;  // Only 1 byte in older version
    XBEnables = (uint32_t)FCFGCID | ((uint32_t)FCFGCID << 8); // two bytes
  } else { // Post Hinj version
    initializeCidStateMachine();
    chipId = getNextConfigWord();// Chip id comes out first from the CID register
    designConfig = getNextConfigWord(); // four bytes
    XBEnables = getNextConfigWord(); // four bytes
  }
}

XLR8Info::~XLR8Info() {} // nothing to destruct
  
uint16_t XLR8Info::getXLR8Version(void) {
  return xlr8Version;
}

// Don't worry about reporting version flags for older designs
bool  XLR8Info::isVersionClean(void) {return (XLR8VERST == 0);}
bool  XLR8Info::isVersionMixed(void) {return XLR8VERST & _BV(0);}
bool  XLR8Info::isVersionModified(void) {return XLR8VERST & _BV(1);}

// The functions initializeCidStateMachine() and getNextConfigWord()
// use a hardware function built in to the xlr8_flashload verilog. The
// sequence calls for first writing a zero to FCFGCID, the FPGA
// Configuration Chip ID register. This initializes a state
// machine. The next 12 reads to the FCFGCID location will return the
// four bytes of the CID, then the four bytes of the Design
// Configuration, and then the four bytes of the XB Enables.
//
// Thus, to get the correct values, one must first call
// initializeCidStateMachine(), then getNextConfigWord() for ChipID,
// again for Design Config and again for XBEnables. In that order and
// preferably one right after the other. The Constructor for XLR8Info
// does that.

bool  XLR8Info::initializeCidStateMachine(void) {return (FCFGCID = 0);}
//
// Function: getNextConfigWord()
//
// Returns the next 32bit value for chip configuration. Does not
// initialize the statemachine by writing a zero to FCFGCID, assumes
// this has been done.

uint32_t XLR8Info::getNextConfigWord(void) {
  uint8_t i;
  uint8_t temp[4];
  uint32_t temp2 = 0;
  for (i = 0; i < 4; ++i) {temp[i] = FCFGCID;} // read low byte first
  for (i = 0; i < 4; ++i) {temp2 = (temp2<<8 | temp[3-i]);} // construct 32b value
  return temp2;
};

uint32_t XLR8Info::getChipId(void) {return chipId;}
uint32_t XLR8Info::getDesignConfig(void) {return designConfig;}
uint32_t XLR8Info::getXBEnables(void) {return XBEnables;}

uint8_t XLR8Info::getImageNum(void)     {return !(designConfig & 1);} // factory=1 is on image=0
uint8_t XLR8Info::getClockMHz(void) { // in bits[2:1], 0=16MHz, 1=32MHz, 2=64MHz, 3=reserved
  if (designConfig & _BV(2) ) {return 64;}
  if (designConfig & _BV(1) ) {return 32;}
  return 16;
}
bool     XLR8Info::hasFullProgMem(void)     {return (designConfig >> 3) & 1;}
bool     XLR8Info::hasFastPLL(void)         {return (designConfig >> 4) & 1;}
bool     XLR8Info::hasSnoADCSwizzle(void)   {return (designConfig >> 5) & 1;}
bool     XLR8Info::hasM16Max10(void)        {
  if (xlr8Version < 2161) { // Pre-Hinj version of flashload
    return (designConfig >> 6) & 1;
  } else { // Post Hinj version
    if (((designConfig >> 6) & 0xff) == 0x10) {
      return 1;
    } else {
      return 0;
    }
  }
}
uint8_t  XLR8Info::getFPGASize(void)        {
  if (xlr8Version < 2161) { // Pre-Hinj version of flashload
    if ((designConfig >> 6) & 1) {
      return 16;  // Sno board
    } else {
      return 8; // XLR8 One board
    }
  } else { // Post Hinj version
    return (designConfig >> 6) & 0xff;
  }
}
uint8_t XLR8Info::getBoardType(void) {
  return (designConfig >> 24);
}

uint8_t  XLR8Info::getUBRR115200(void)      {return CLKSPD;}
bool     XLR8Info::hasXLR8FloatAddSubMult(void) {return (XBEnables >> 0) & 1;}

bool     XLR8Info::hasXLR8FloatDiv(void) {
  return ((XBEnables >> 0) & 1) && ((getXLR8Version() < 363) || (getXLR8Version() >= 815));
}
bool     XLR8Info::hasXLR8Servo(void)       {return (XBEnables >> 1) & 1;}
bool     XLR8Info::hasXLR8NeoPixel(void)    {return (XBEnables >> 2) & 1;}
bool     XLR8Info::hasXLR8Quad(void)        {return (XBEnables >> 3) & 1;}
bool     XLR8Info::hasXLR8PID(void)         {return (XBEnables >> 4) & 1;}

bool     XLR8Info::hasICSPVccGndSwap(void)  {
  // List of chip IDs from boards that have Vcc and Gnd swapped on the ICSP header
  //   Chip ID of affected parts are 0x????6E00. Store the ???? part
  const static char cidTable[] PROGMEM =
    {0xC88F,  0x08B7,  0xA877,  0xF437,
     0x94BF,  0x88D8,  0xB437,  0x94D7,  0x38BF,  0x145F,  0x288F,  0x28CF,
     0x543F,  0x0837,  0xA8B7,  0x748F,  0x8477,  0xACAF,  0x14A4,  0x0C50,
     0x084F,  0x0810,  0x0CC0,  0x540F,  0x1897,  0x48BF,  0x285F,  0x8C77,
     0xE877,  0xE49F,  0x2837,  0xA82F,  0x043F,  0x88BF,  0xF48F,  0x88F7,
     0x1410,  0xCC8F,  0xA84F,  0xB808,  0x8437,  0xF4C0,  0xD48F,  0x5478,
     0x080F,  0x54D7,  0x1490,  0x88AF,  0x2877,  0xA8CF,  0xB83F,  0x1860,
     0x38BF};
  uint32_t chipId = getChipId();
  for (int i=0;i< sizeof(cidTable)/sizeof(cidTable[0]);i++) {
    uint32_t cidtoTest = (cidTable[i] << 16) + 0x6E00;
    if (chipId == cidtoTest) {return true;}
  }
  return false;
}

void  XLR8Info::enableInternalOscPin(void) {CLKSPD |= 1;} // send internal oscillator to pin
void  XLR8Info::disableInternalOscPin(void) {CLKSPD &= ~1;}

// XB Info methods
bool XLR8Info::checkXBInfoValid(void) {
  XBINFO = 0xFC; // Set the XBINFO read address to XB_INFO_VALID_ADDR
  // Check to see if the value returned is "XLR8"
  if (XBINFO == 0x88) {       // "X" at address 0xFC
    if (XBINFO == 0x76) {     // "L" at address 0xFD
      if (XBINFO == 0x82) {   // "R" at address 0xFE
        if (XBINFO == 0x56) { // "8" at address 0xFF
          return true;}}}}
    return false;
}

uint8_t XLR8Info::getXBInfoNumRegs(void) {
  XBINFO = 0x00; // Set the read address
  return XBINFO;
}

uint8_t XLR8Info::getXBInfoVal(uint8_t addr) {
  XBINFO = addr; // Set the read address
  return XBINFO;
}

uint8_t XLR8Info::getXBInfoNextVal(void) {return XBINFO;}



