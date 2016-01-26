#include <XLR8Info.h>

#define SAMPLES 129
volatile uint8_t index;
volatile uint16_t timestamp[SAMPLES];

void setup() {
  XLR8Info myXLR8;
  Serial.begin(115200);
  Serial.print("XLR8 Hardware Version Number = ");
  Serial.println(myXLR8.getXLR8Version());
  if (myXLR8.isVersionMixed()) {Serial.println("  Mixed version, lowest reported");}
  if (myXLR8.isVersionModified()) {Serial.println("  Modified working copy");}
  if (myXLR8.isVersionClean()) {Serial.println("  Clean working copy");}
  Serial.print("XLR8 CID = 0x");
  Serial.println(myXLR8.getChipId());
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
  XLR8_CLKSPD |= 1; // send internal osc divide by 1024 to pin 8
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
  GPIOR1 = 0xc0;
}

void loop() {
}

ISR(TIMER1_CAPT_vect) { 
  if (index < SAMPLES) {
    timestamp[index++] = ICR1;
  }
} 

