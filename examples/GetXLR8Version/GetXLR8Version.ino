#include <XLR8Info.h>
/* GetXLR8Version
 Copyright (c) 2015-2016 Alorium Technology.  All rights reserved.
 by Matt Weber (support@aloriumtech.com) of
 Alorium Technology (info@aloriumtech.com)
 Reports information about the FPGA design currently
  loaded on the XLR8 board
 Set serial monitor to 115200 baud
*/
  
#define SAMPLES 65

// SIM_SAMPLES for simulation only
#define SIM_SAMPLES 10
volatile uint8_t index;
volatile uint16_t timestamp[SAMPLES];
XLR8Info myXLR8;

// GPIOR0 != 0: simulation: dont print out as much
void setup() {
  boolean verbose = (GPIOR0 == 0);
  uint8_t fsize;
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
  //sjp// Commented this out for production since Console output would be garbage anyway if the Baud
  //sjp// rate wasn't set correctly antway. Its just confusing.
  //  if (abs(ubrr0lCurrent - ubrr115200baud) > 1) { // +/- 1 on UBRR setting usually still works
  //    Serial.println("Error: Change Tools->FPGA Image to a selection with the following MHz");
  //  }
  if (verbose) {
    if (myXLR8.hasSnoADCSwizzle()) {Serial.println("Board Type: Sno");}
    else {
      if (myXLR8.getFPGASize() == 8) {
        Serial.println("Board Type: XLR8");
      } else {
        Serial.println("Board Type: Hinj");
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
  Serial.print("XB_ENABLE             = 0x");Serial.println(myXLR8.getXBEnables(),HEX);
  if (verbose) {
    if (myXLR8.hasXLR8FloatAddSubMult()) {Serial.println(F("  Has Floating Point Add, Subtract, and Multiply"));}
    if (myXLR8.hasXLR8FloatDiv())        {Serial.println(F("  Has Floating Point Divide"));}
    if (myXLR8.hasXLR8Servo())           {Serial.println(F("  Has Servo XB"));}
    if (myXLR8.hasXLR8NeoPixel())        {Serial.println(F("  Has NeoPixel XB"));}
    if (myXLR8.hasXLR8Quad())            {Serial.println(F("  Has Quadrature XB"));}
    if (myXLR8.hasXLR8PID())             {Serial.println(F("  Has PID XB"));}
  }
  
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

