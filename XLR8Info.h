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

#include <Arduino.h>

#ifndef XLR8INFO_h
#define XLR8INFO_h

// #ARDUINO_XLR8 is passed from IDE to the compiler if XLR8 is selected properly
#ifdef ARDUINO_XLR8

class XLR8Info {
  public:
  // Constructor sets XBEnables, Destructor doesn't do anything
  XLR8Info(void);
  ~XLR8Info();
  // Return information about the XLR8 board.
  uint16_t getXLR8Version(void);
  bool  isVersionClean(void);
  bool  isVersionMixed(void);
  bool  isVersionModified(void);
  uint32_t getChipId(void);
  uint8_t  getDesignConfig(void);
  uint8_t  getImageNum(void);
  uint8_t  getClockMHz(void);
  uint8_t  getUBRR115200(void);
  bool  hasFullProgMem(void);
  uint32_t getXBEnables(void);
  bool  hasXLR8FloatAddSubMult(void);
  bool  hasXLR8FloatDiv(void);
  bool  hasXLR8Servo(void);
  bool  hasXLR8NeoPixel(void);
  bool  hasICSPVccGndSwap(void);
  void  enableInternalOscPin(void);
  void  disableInternalOscPin(void);
  private:
  uint8_t  designConfig;
  uint16_t XBEnables;
  bool     oldRegisters;
  
};

#else
#error "XLR8Info library requires Tools->Board->XLR8xxx selection. Install boards from https://github.com/AloriumTechnology/Arduino_Boards"
#endif
#endif
