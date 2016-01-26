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

#include <Arduino.h>

#ifndef XLR8INFO_h
#define XLR8INFO_h

#if defined (__AVR_ATmega328P__)
#  define XLR8_VERS      _SFR_MEM16(0x90)
#  define XLR8_VERSL     _SFR_MEM8(0x90)
#  define XLR8_VERSH     _SFR_MEM8(0x91)
#  define XLR8_VERST     _SFR_MEM8(0x92)
#  define XLR8_CID       _SFR_MEM8(0xd8)
#  define XLR8_CLKSPD    _SFR_IO8(0x29)
#else
#   warning "XLR8 Hardware not implemented for selected device"
#endif

class XLR8Info {
  public:
  // Constructor sets XBEnables, Destructor doesn't do anything
  XLR8Info(void);
  ~XLR8Info();
  // Return information about the XLR8 board.
  inline uint16_t getXLR8Version(void) {return XLR8_VERS;}
  bool  isVersionClean(void);
  bool  isVersionMixed(void);
  bool  isVersionModified(void);
  uint32_t getChipId(void);
  uint8_t  getDesignConfig(void);
  uint8_t  getImageNum(void);
  uint8_t  getClockMHz(void);
  bool  hasFullProgMem(void);
  uint32_t getXBEnables(void);
  bool  hasXLR8FloatAddSubMult(void);
  bool  hasXLR8FloatDiv(void);
  bool  hasXLR8Servo(void);
  bool  hasXLR8NeoPixel(void);
  private:
  uint8_t  designConfig;
  uint16_t XBEnables;
  
};

#endif
