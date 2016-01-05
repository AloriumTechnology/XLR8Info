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

XLR8Info::XLR8Info(void) : XBEnables(0) { // constructor
  // Find out which XLR8 blocks this design has
  getChipId();// Chip id comes out first from the CID register
  uint8_t i;
  uint8_t temp[4];
  for (i = 0; i < 4; ++i) {temp[i] = XLR8_CID;} // get XBEnables, read low byte first
  for (i = 0; i < 4; ++i) {XBEnables = (XBEnables<<8 | temp[3-i]);} // construct 32b value
}
XLR8Info::~XLR8Info() {} // nothing to destruct
bool  XLR8Info::isVersionClean(void) {return XLR8_VERST == 0;}
bool  XLR8Info::isVersionMixed(void) {return XLR8_VERST & _BV(0);}
bool  XLR8Info::isVersionModified(void) {return XLR8_VERST & _BV(1);}
uint64_t XLR8Info::getChipId(void) {
  XLR8_CID = 0;  // trigger capture
  uint8_t i;
  uint8_t temp[8];
  uint64_t temp2 = 0;
  for (i = 0; i < 8; ++i) {temp[i] = XLR8_CID;} // read low byte first
  for (i = 0; i < 8; ++i) {temp2 = (temp2<<8 | temp[7-i]);} // construct 64b value
  return temp2;
};
bool  XLR8Info::hasBootRestore(void) {return (XBEnables >> 0) & 1;} // Boot restore is in bit 0
bool  XLR8Info::hasXLR8FloatAddSubMult(void) {return (XBEnables >> 1) & 1;}
bool  XLR8Info::hasXLR8FloatDiv(void) {
  return ((XBEnables >> 1) & 1) && ((XLR8_VERS < 363) || (0)); // update with XLR8_VERS>=?? when it's back in
}
bool  XLR8Info::hasXLR8FloatConvert(void) {
  return ((XBEnables >> 1) & 1) && (0); // Update with XLR8_VERS>=?? when it's in
}
bool  XLR8Info::hasXLR8Servo(void) {return (XBEnables >> 1) & 1;}
bool  XLR8Info::hasXLR8NeoPixel(void) {return (XBEnables >> 2) & 1;}
