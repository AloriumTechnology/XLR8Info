/*--------------------------------------------------------------------
 Copyright (c) 2015 Alorim Technology.  All right reserved.
 This library queries information from an XLR8 board.
 Written by Matt Weber (Matthew.D.Weber@ieee.org) of
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
 License along with XLR8 NeoPixel.  If not, see
 <http://www.gnu.org/licenses/>.
 --------------------------------------------------------------------*/

#include "XLR8Info.h"

XLR8Info::XLR8Info(void) : designConfig(0),XBEnables(0) { // constructor
  // Find out which XLR8 blocks this design has
  getChipId();// Chip id comes out first from the CID register
  uint8_t i;
  uint8_t temp[4];
  designConfig = XLR8_CID;
  XBEnables = (uint16_t)XLR8_CID | ((uint16_t)XLR8_CID << 8); // this should leave 1st byte read in lower byte
  if (XLR8_VERS < 756) { // older designs had design config and XB enables mixed together
    XBEnables = (XBEnables << 6) | (designConfig >> 2);
    designConfig &= 1;
  }
}
XLR8Info::~XLR8Info() {} // nothing to destruct
bool  XLR8Info::isVersionClean(void) {return XLR8_VERST == 0;}
bool  XLR8Info::isVersionMixed(void) {return XLR8_VERST & _BV(0);}
bool  XLR8Info::isVersionModified(void) {return XLR8_VERST & _BV(1);}
uint32_t XLR8Info::getChipId(void) {
  XLR8_CID = 0;  // trigger capture
  uint8_t i;
  uint8_t temp[4];
  uint32_t temp2 = 0;
  for (i = 0; i < 4; ++i) {temp[i] = XLR8_CID;} // read low byte first
  if (XLR8_VERS < 756) { // older designs had full 64 bit chip id, we only want top half, so overwrite
    for (i = 0; i < 4; ++i) {temp[i] = XLR8_CID;}
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
bool XLR8Info::hasFullProgMem(void) {return (designConfig >> 3) & 1;}
uint32_t XLR8Info::getXBEnables(void)  {return XBEnables;}
bool  XLR8Info::hasXLR8FloatAddSubMult(void) {return (XBEnables >> 0) & 1;}
bool  XLR8Info::hasXLR8FloatDiv(void) {
  return ((XBEnables >> 0) & 1) && ((XLR8_VERS < 363) || (XLR8_VERS >= 815));
}
bool  XLR8Info::hasXLR8Servo(void) {return (XBEnables >> 1) & 1;}
bool  XLR8Info::hasXLR8NeoPixel(void) {return (XBEnables >> 2) & 1;}
