/*--------------------------------------------------------------------
 Copyright (c) 2015 Alorim Technology.  All right reserved.
 This library queries information from an XLR8 board.
 Written by Matt Weber (linkedin.com/in/mattweberdesign) of
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

#define XLR8VERS      _SFR_MEM16(0xd4)
#define XLR8VERSL     _SFR_MEM8(0xd4)
#define XLR8VERSH     _SFR_MEM8(0xd5)
#define XLR8VERST     _SFR_MEM8(0xd6)
#define FCFGCID       _SFR_MEM8(0xcf)
#define CLKSPD        _SFR_IO8(0x29)
#define XLR8VERS_OLD  _SFR_MEM16(0x90)
#define FCFGCID_OLD   _SFR_MEM8(0xd8)

XLR8Info::XLR8Info(void) : designConfig(0),XBEnables(0) { // constructor
  // The XLR8VERS register moved. Handle either old or new location
  oldRegisters = (XLR8VERS == 0);
  // Find out which XLR8 blocks this design has
  getChipId();// Chip id comes out first from the CID register
  uint8_t i;
  uint8_t temp[4];
  if (oldRegisters) {
    designConfig = FCFGCID_OLD;
    XBEnables = (uint16_t)FCFGCID_OLD | ((uint16_t)FCFGCID_OLD << 8); // this should leave 1st byte read in lower byte
    if (getXLR8Version() < 756) { // older designs had design config and XB enables mixed together
      XBEnables = (XBEnables << 6) | (designConfig >> 2);
      designConfig &= 1;
    }
  } else {
    designConfig = FCFGCID;
    XBEnables = (uint16_t)FCFGCID | ((uint16_t)FCFGCID << 8); // this should leave 1st byte read in lower byte
  }
}
XLR8Info::~XLR8Info() {} // nothing to destruct
uint16_t XLR8Info::getXLR8Version(void) {
  // The XLR8VERS register moved. Handle either old or new location
  return oldRegisters ? XLR8VERS_OLD : XLR8VERS;
}
// Don't worry about reporting version flags for older designs
bool  XLR8Info::isVersionClean(void) {return ~oldRegisters && (XLR8VERST == 0);}
bool  XLR8Info::isVersionMixed(void) {return XLR8VERST & _BV(0);}
bool  XLR8Info::isVersionModified(void) {return XLR8VERST & _BV(1);}
uint32_t XLR8Info::getChipId(void) {
  uint8_t i;
  uint8_t temp[4];
  uint32_t temp2 = 0;
  if (oldRegisters) {
    FCFGCID_OLD = 0;  // trigger capture
    for (i = 0; i < 4; ++i) {temp[i] = FCFGCID_OLD;} // read low byte first
    if (getXLR8Version() < 756) { // older designs had full 64 bit chip id, we only want top half, so overwrite
      for (i = 0; i < 4; ++i) {temp[i] = FCFGCID_OLD;}
    }
  } else {
    FCFGCID = 0;  // trigger capture
    for (i = 0; i < 4; ++i) {temp[i] = FCFGCID;} // read low byte first
  }
  for (i = 0; i < 4; ++i) {temp2 = (temp2<<8 | temp[3-i]);} // construct 32b value
  return temp2;
};
uint8_t XLR8Info::getDesignConfig(void) {return designConfig;}
uint8_t XLR8Info::getImageNum(void) {return !(designConfig & 1);} // factory=1 is on image=0
uint8_t XLR8Info::getClockMHz(void) { // in bits[2:1], 0=16MHz, 1=32MHz, 2=64MHz, 3=reserved
  if (designConfig & _BV(2) ) {return 64;}
  if (designConfig & _BV(1) ) {return 32;}
  return 16;
}
uint8_t XLR8Info::getUBRR115200(void) {return CLKSPD;}
bool XLR8Info::hasFullProgMem(void) {return (designConfig >> 3) & 1;}
uint32_t XLR8Info::getXBEnables(void)  {return XBEnables;}
bool  XLR8Info::hasXLR8FloatAddSubMult(void) {return (XBEnables >> 0) & 1;}
bool  XLR8Info::hasXLR8FloatDiv(void) {
  return ((XBEnables >> 0) & 1) && ((getXLR8Version() < 363) || (getXLR8Version() >= 815));
}
bool  XLR8Info::hasXLR8Servo(void) {return (XBEnables >> 1) & 1;}
bool  XLR8Info::hasXLR8NeoPixel(void) {return (XBEnables >> 2) & 1;}

void  XLR8Info::enableInternalOscPin(void) {CLKSPD |= 1;} // send internal oscillator to pin
void  XLR8Info::disableInternalOscPin(void) {CLKSPD &= ~1;}

