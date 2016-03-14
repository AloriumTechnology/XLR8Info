#include <XLR8Info.h>
/* GetXLR8Version
 Copyright (c) 2015-2016 Alorim Technology.  All right reserved.
 by Matt Weber (linkedin.com/in/mattweber0) of
 Alorium Technology (info@aloriumtech.com)
 Reports information about the FPGA design currently
  loaded on the XLR8 board
 Set serial monitor to 115200 baud
*/
  

#define SAMPLES 129
volatile uint8_t index;
volatile uint16_t timestamp[SAMPLES];
XLR8Info myXLR8;

void setup() {
  Serial.begin(115200);
  // Check if it looks like the correct MHz setting is chosen under Tools->FPGA Image
  uint8_t ubrr0lCurrent = UBRR0L;
  uint8_t ubrr115200baud = myXLR8.getUBRR115200();
  UBRR0L = ubrr115200baud; // Adjust speed if it wasn't set correctly so we don't see gibberish on serial monitor
  Serial.println("***************");
  if (abs(ubrr0lCurrent - ubrr115200baud) > 1) { // +/- 1 on UBRR setting usually still works
    Serial.println("Error: Change Tools->FPGA Image to a selection with the following MHz");
  }
  printImageName(Serial);
  Serial.println("***************");
  Serial.flush();
  Serial.print("XLR8 Hardware Version Number = ");
  Serial.println(myXLR8.getXLR8Version());
  if (myXLR8.isVersionMixed()) {Serial.println("  Mixed version, lowest reported");}
  if (myXLR8.isVersionModified()) {Serial.println("  Modified working copy");}
  Serial.print("XLR8 CID = 0x");
  Serial.println(myXLR8.getChipId(),HEX);
  Serial.print("DesignConfig = 0x");
  Serial.println(myXLR8.getDesignConfig());
  Serial.print("Image = ");
  Serial.println(myXLR8.getImageNum());
  Serial.print("Clock = ");
  Serial.print(myXLR8.getClockMHz());
  Serial.println(" MHz");
  Serial.print("XB_ENABLE = 0x");
  Serial.println(myXLR8.getXBEnables(),HEX);
  if (myXLR8.hasXLR8FloatAddSubMult()) {Serial.println(F("Has Floating Point Add, Subtract, and Multiply"));}
  if (myXLR8.hasXLR8FloatDiv())        {Serial.println(F("Has Floating Point Divide"));}
  if (myXLR8.hasXLR8Servo())           {Serial.println(F("Has Servo XB"));}
  if (myXLR8.hasXLR8NeoPixel())        {Serial.println(F("Has NeoPixel XB"));}
  
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
  while (index < SAMPLES);
  TIMSK1 &= ~(1<<ICIE1); //disable input capture interrupt
  float intOscSpeed = F_CPU*1024.0*(SAMPLES-1)/(timestamp[SAMPLES-1] - timestamp[0])/1000000.0;
  Serial.print("Int Osc = ");
  Serial.print(intOscSpeed);
  Serial.println(" MHz");
  Serial.flush();
  myXLR8.disableInternalOscPin(); // send internal osc divide by 1024 to pin 8
  PRR |= (1 << 4); // power down the internal oscillator
  GPIOR1 = 0xc0;
  
  // Post results to our product improvement page
  Serial.println();
  Serial.println(F("To help improve our products, please paste the followig URL into a web browser, add any notes, and click submit"));
  Serial.print(F("https://docs.google.com/forms/d/1GCmN3hRF-fnkr0J8K8HBrYvmuhmz6y94ViadHgobeRQ/viewform?"));
  Serial.print(F("entry.882185102="));Serial.print(F("Log+GetXLR8Version+Results")); // Board Number
  Serial.print(F("&entry.451752966=XLR8"));
  Serial.print(F("&entry.110701079=0x"));Serial.print(myXLR8.getChipId(),HEX); // Chip ID
  Serial.print(F("&entry.328279444="));Serial.print(myXLR8.getXLR8Version()); // subversion
  Serial.print(F("&entry.1881676735="));Serial.print(myXLR8.getDesignConfig()); // Design Config
  Serial.print(F("&entry.821741284="));Serial.print(myXLR8.getXBEnables()); // XB Enable
  //Serial.print(F("&entry.1215707535="));Serial.print(); // Notes
  Serial.print(F("&entry.545646289="));Serial.print(intOscSpeed); // Speed Test
  Serial.println();
  Serial.flush();
   
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
  s.print(" 1.0.");
  s.println(myXLR8.getXLR8Version());
}
