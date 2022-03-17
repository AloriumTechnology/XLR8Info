#include <XLR8Info.h>
/* GetXLR8Version
 Copyright (c) 2015-2016 Alorium Technology.  All rights reserved.
 by Matt Weber (support@aloriumtech.com) of
 Alorium Technology (info@aloriumtech.com)
 Reports information about the FPGA design currently
  loaded on the XLR8 board
 Set serial monitor to 115200 baud
*/
// The DESIGN_CONFIG parameter/reg is used to configure many aspects
// of the design and can be read by a sketch and inspected to
// determine what type of board and what type of image is being
// used. The definition of the fields in the DESIGN_CONFIG is shown
// below.

// DESIGN_CONFIG = {
//     8'd0,  // [31:24] - Board type      0 = Not specified, figure out via other fields
//     9'd0,  // [23:15] - reserved
//     8'h2,  //  [14]   - Compact,        0 = Analog/Flash,    1 = Compact
//     8'h8,  // [13:6]  - MAX10 Size,     ex: 0x8 = M08, 0x32 = M50
//     1'b0,  //   [5]   - ADC_SWIZZLE,    0 = XLR8,            1 = Sno
//     1'b0,  //   [4]   - PLL Speed,      0 = 16MHz PLL,       1 = 50Mhz PLL
//     1'b1,  //   [3]   - Force 16K PMEM, 0 = FPGA Dependent,  1 = 16K
//     2'd0,  //  [2:1]  - Clock Speed,    0 = 16MHZ,           1 = 32MHz, 2 = 64MHz, 3=na
//     1'b0   //   [0]   - FPGA Image,     0 = CFM Application, 1 = CFM Factory            
// };
//
//   |------------+------------+-----------------------------------------|
//   | Board Code | Board Type | Description                             |
//   |------------+------------+-----------------------------------------|
//   |     0      | Legacy     | use old methods to determine board type |
//   |     1      | XLR8       | Original UNo compatible board           |
//   |     2      | SNO        | First small form factor board           |
//   |     3      | HINJ       | Big prototyping board                   |
//   |     4      | BTBEE      | XBee form factor                        |
//   |     5      | DE10LITE   | Terasic DE10-Lite board                 |
//   |     6      | SNOM2      | Digikey MicroMod form factor            |
//   |     7      | SNOEDGE    | Sno with DDR2 Edge connector, M25 FPGA  |
//   |     8      | SNOEDGE50  | Sno with DDR2 Edge connector, M50 FPGA  |
//   |------------+------------+-----------------------------------------|
  
#define SAMPLES 65

// SIM_SAMPLES for simulation only
#define SIM_SAMPLES 10
volatile uint8_t index;
volatile uint16_t timestamp[SAMPLES];
XLR8Info myXLR8;

// GPIOR0 != 0: simulation: dont print out as much
void setup() {
  boolean verbose = (GPIOR0 == 0);
  uint8_t fsize, xbnum, xbi;
  uint32_t chipid, designconfig, xbenables;
  Serial.begin(115200);
  // Check if it looks like the correct MHz setting is chosen under Tools->FPGA Image
  uint8_t ubrr0lCurrent = UBRR0L;
  uint8_t ubrr115200baud = myXLR8.getUBRR115200();
  if (ubrr115200baud < 3) {ubrr115200baud = UBRR0L;} // Handle early hardware that didn't have getUBRR115200() function
  UBRR0L = ubrr115200baud; // Adjust speed if it wasn't set correctly so we don't see gibberish on serial monitor
  if (verbose) Serial.println("============================================================");
  if (myXLR8.hasICSPVccGndSwap()) {
    Serial.println("Warning: Do not use ICSP header on this board. The ICSP Vcc and Gnd pins are swapped");
  }

  if (verbose) {
    if (myXLR8.getBoardType()) { // DESIGN_CONFIG contains BOARD_TYPE field
      switch (myXLR8.getBoardType()) {
      case 1:
        Serial.println("getBoardType(): XLR8");
        break;
      case 2:
        Serial.println("Board Type: Sno");
        break;
      case 3:
        Serial.println("Board Type: Hinj");
        break;
      case 4:
        // Known as BTBEE in the verilog, the product name is XGZ
        Serial.println("Board Type: XGZ");
        break;
      case 5:
        Serial.println("Board Type: DE10-Lite");
        break;
      case 6:
        Serial.println("Board Type: Sno M2");
        break;
      case 7:
        Serial.println("Board Type: Sno Edge");
        break;
      case 8:
        Serial.println("Board Type: Sno Edge 50");
        break;
      default:
        Serial.print("Board Type: Unknown Board Type: ");
        Serial.println(myXLR8.getBoardType());
        break;
      }
    } else { // DESIGN_CONFIG does not contain BOARD_TYPE field
      // Try to determine Board type from other fields
      if (myXLR8.hasSnoADCSwizzle()) {Serial.println("Board Type: Sno");}
      else {
        if (myXLR8.getFPGASize() == 8) {
          Serial.println("Board Type: XLR8");
        } else {
          Serial.println("Board Type: Hinj");
        }
      }
    }
  }
  if (verbose) printImageName(Serial);
  if (verbose) Serial.println("============================================================");
  Serial.flush();
  Serial.print("XLR8 Hardware Version = ");
  Serial.println(myXLR8.getXLR8Version());
  if (myXLR8.isVersionMixed()) {Serial.println("  Mixed version, lowest reported");}
  if (myXLR8.isVersionModified()) {Serial.println("  Modified working copy");}
  Serial.print("XLR8 CID              = 0x");
  Serial.println(myXLR8.getChipId(),HEX);

  if (verbose) Serial.println("------------------------------------------------------------");
  Serial.print("Design Configuration  = 0x");Serial.println(myXLR8.getDesignConfig(),HEX);
  Serial.print("  Image       = ");Serial.println(myXLR8.getImageNum());
  Serial.print("  Clock       = ");Serial.print(myXLR8.getClockMHz());Serial.println("MHz");
  if (myXLR8.hasFastPLL()) {
    Serial.println("  PLL Speed   = 50MHz");}
  else  {
    Serial.println("  PLL Speed   = 16MHz");}
  if (myXLR8.hasSnoADCSwizzle()) {Serial.println("  ADC Swizzle = True" );}
  Serial.print("  FPGA Size   = M"); Serial.println(myXLR8.getFPGASize());

  if (verbose) Serial.println("------------------------------------------------------------");
  if (myXLR8.getXBEnables() == 0) {
    Serial.println("No Builtin XB Enabled");  
  } else {
    Serial.print("Builtin XB ENABLE             = 0x");Serial.println(myXLR8.getXBEnables(),HEX);
    if (verbose) {
      if (myXLR8.hasXLR8FloatAddSubMult()) {Serial.println(F("  Has Floating Point Add, Subtract, and Multiply"));}
      if (myXLR8.hasXLR8FloatDiv())        {Serial.println(F("  Has Floating Point Divide"));}
      if (myXLR8.hasXLR8Servo())           {Serial.println(F("  Has Servo XB"));}
      if (myXLR8.hasXLR8NeoPixel())        {Serial.println(F("  Has NeoPixel XB"));}
      if (myXLR8.hasXLR8Quad())            {Serial.println(F("  Has Quadrature XB"));}
      if (myXLR8.hasXLR8PID())             {Serial.println(F("  Has PID XB"));}
    }
  }

  if (verbose) Serial.println("------------------------------------------------------------");
  if (!myXLR8.checkXBInfoValid()) {
    Serial.println("OpenXLR8 Info Regs    = None");
    //    Serial.print("myXLR8.getXBInfoVal(0xA5) = ");
    //Serial.println(myXLR8.getXBInfoVal(0xA5));
  } else {
    xbnum = myXLR8.getXBInfoNumRegs();
    Serial.print("OpenXLR8 Info Regs    = ");Serial.println(xbnum);
    if (verbose) {
      for (xbi=0; xbi<xbnum; xbi++) {
        Serial.print("  Info Reg ");
        Serial.print(xbi+1);
        Serial.print(" = 0x");
        Serial.println(myXLR8.getXBInfoNextVal(),HEX);
      }
    }
  }
  if (verbose) Serial.println("------------------------------------------------------------");

  // XLR8 has an internal oscillator that doesn't get used for anything and varies with
  //  voltage, temperature, and transistor characteristics. Still, it may be interesting
  //  to see how fast it is running. It comes out on pin 8 so we can use timer/counter1
  //  input capture function to measure it accurately.
  PRR &= ~(1 << 4); // Enable the oscillator (turn off power reduction on bit 4)
  myXLR8.enableInternalOscPin(); // send internal osc divide by 1024 to pin 8
  delay(10); // Let oscillator get started
  //ICP1 enable
  TCCR1A = 0;
  TCCR1B =(1<<ICNC1)|(1<<ICES1)|(1<<CS10); //Noise canceller, rising edge, with no prescaler
  index = 0;
  TIFR1 |= (1<<ICF1); //clear interrupt-flag
  TCNT1 = 0;
  TIMSK1 |=(1<<ICIE1); //enable input capture interrupt
  uint8_t sample_limit = GPIOR0 ? SIM_SAMPLES : SAMPLES;
  while (index < sample_limit);
  TIMSK1 &= ~(1<<ICIE1); //disable input capture interrupt
  
  
  float intOscSpeed;
  uint8_t sample_idx = sample_limit - 1;

  intOscSpeed =  myXLR8.getClockMHz()*1024.0*(sample_idx)/(timestamp[sample_idx] - timestamp[0]);
  if (verbose) Serial.println("------------------------------------------------------------");
  Serial.print("Int Osc = ");
  Serial.print(intOscSpeed);
  Serial.println(" MHz");
  if (verbose) Serial.println("------------------------------------------------------------");
  Serial.flush();
  myXLR8.disableInternalOscPin(); // send internal osc divide by 1024 to pin 8
  PRR |= (1 << 4); // power down the internal oscillator
  
  // Post results to our product improvement page
  Serial.println();
  if (verbose) Serial.println(F("To help improve our products, please paste the followig URL into a web browser, add any notes, and click submit"));
  Serial.print(F("https://docs.google.com/forms/d/1djbu8L3VNO3RdnVh2VHpkj0YPG3BeW8nmZxYzvHM9jc/viewform?"));
  Serial.print(F("&entry.451752966=XLR8"));
  Serial.print(F("&entry.1215707535="));Serial.print("generated by GetXLR8Version"); 
  Serial.print(F("&entry.110701079=0x"));Serial.print(myXLR8.getChipId(),HEX); // Chip ID
  Serial.print(F("&entry.328279444="));Serial.print(myXLR8.getXLR8Version()); // subversion
  Serial.print(F("&entry.1881676735="));Serial.print(myXLR8.getDesignConfig()); // Design Config
  Serial.print(F("&entry.821741284="));Serial.print(myXLR8.getXBEnables()); // XB Enable
  Serial.print(F("&entry.545646289="));Serial.print(intOscSpeed); // Speed Test
  Serial.println();
  Serial.println();
  Serial.println(F("GetXLRVersion Complete"));
  Serial.flush();
  GPIOR1 = 0xc0;

}

void loop() {
}

ISR(TIMER1_CAPT_vect) { 
  if (index < SAMPLES) {
    timestamp[index++] = ICR1;
  }
} 

// Print FPGA image name in format that matches Tools->FPGA Image selection
void printImageName(Stream &s) {
  s.print("FPGA Image: ");
  s.print(myXLR8.getClockMHz());
  s.print(" MHz ");
  if (myXLR8.hasXLR8FloatAddSubMult()) {s.print(F("Float "));}
  if (myXLR8.hasXLR8Servo())           {s.print(F("Servo "));}
  if (myXLR8.hasXLR8NeoPixel())        {s.print(F("NeoPixel "));}
  if (myXLR8.hasXLR8Quad())            {s.print(F("Quadrature "));}
  if (myXLR8.hasXLR8PID())             {s.print(F("PID "));}
  s.print("r");
  s.println(myXLR8.getXLR8Version());
}

