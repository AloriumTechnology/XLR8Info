/* XLR8SelfTest
 Copyright (c) 2015-2016 Alorim Technology.  All right reserved.
 by Matt Weber (linkedin.com/in/mattweber0) of
 Alorium Technology (info@aloriumtech.com)
 Does a simple self test of an alorium XLR8 board which should
  detect many (not all) possible manufacturing flaws
 Specifically,
  On the digital pins, they can be set to outputs driving low, 
   then switch to inputs and measure the time needed before the
   input reads high. If the pullups are missing it seems to take more than 100 clock cycles and if the pullups are there, less than 10. So those get tested pretty completely except for the header pin itself.
  On the analog pins, we probably can set the digital I/O side
   to output high, and output low, and do an ADC reading at each.
 Things that are not covered by the two things above are:
  The header pins themselves
  I2C pins, pullups, and pullup enable
  AREF (although wiring 3.3V to it and repeating the analog pins test could cover it)
  LEDs manually observed instead of auto-tested
  Reset push button
  Switch between USB and barrel power, and barrel diode
  JTAG pins that aren’t used by USB Blaster
  ISP header
  EEPROM signals
  IOREF
  decoupling caps are probably untestable
 Wiring is really simple. Just wire from 3.3V to Aref and that's it.
*/

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 
#ifndef NOP
#define NOP __asm__ __volatile__ ("nop"  "\n\t" ::);
#endif

#include <XLR8Info.h>

int testnum=0;
int passnum=0;
int failnum=0;
int lastfail=-1;
const uint8_t maxPullupTimeB = 7; // for ports D and B that have pullups
const uint8_t maxPullupTimeD = 7; // for ports D and B that have pullups
const uint8_t minPullupTimeC  = 200; // for port C without any pullups
const uint8_t portBpinMask = 0x3F; // digital 8-13
const uint8_t portCpinMask = 0x3F; // analog 0-5
const uint8_t portDpinMask = 0xFF; // digital 0-7
// Expected values are 3.3/5 * 1024 = 676, 0, 1023, and 0, but actual is a bit different
const int maxAreadih = 704; // max expected analog read value with internal reference and digital driving high
const int minAreadih = 684; // min expected analog read value with internal reference and digital driving high
const int maxAreadil = 6; // max expected analog read value with internal reference and digital driving low
const int minAreadil = 0; // min expected analog read value with internal reference and digital driving low
const int maxAreadeh = 1023; // max expected analog read value with external reference and digital driving high
const int minAreadeh = 1011; // min expected analog read value with external reference and digital driving high
const int maxAreadel = 12; // max expected analog read value with external reference and digital driving low
const int minAreadel = 0; // min expected analog read value with external reference and digital driving low
uint8_t fastPinD = 255;
uint8_t fastValD = 255;
uint8_t slowPinD = 254;
uint8_t slowValD = 0;
uint8_t fastPinB = 255;
uint8_t fastValB = 255;
uint8_t slowPinB = 254;
uint8_t slowValB = 0;
uint8_t highPinC = 255;
uint8_t lowPinC  = 254;
uint16_t highValC = 0;
uint16_t lowValC = 1023;
uint16_t highXrefh = 0;
uint16_t lowXrefh = 1023;
uint16_t highXrefl = 0;
uint16_t lowXrefl = 1023;

//#define INCLUDE_DEBUG
#ifdef INCLUDE_DEBUG
const uint8_t numTestDebug = 40;
uint8_t testDebug[numTestDebug];
const uint8_t numFailFlags = 128;
bool failFlags[numFailFlags];
int areadih[6];
int areadil[6];
int areadeh[6];
int areadel[6];
#endif

const int SAMPLES = 129;  // Set to 0 if trying this sketch on non-XLR8 board
volatile uint8_t index;
volatile uint16_t timestamp[SAMPLES];

void setup() {
 #ifdef INCLUDE_DEBUG
  for (uint8_t i = 0; i<numFailFlags; i++) {
    failFlags[i]=0;
  }
  for (uint8_t i = 0; i<numTestDebug; i++) {
    testDebug[i]=0;
  }  
  for (uint8_t i = 0; i<6; i++) {
    areadih[i]=0;
    areadil[i]=0;
    areadeh[i]=0;
    areadel[i]=0;
  }
 #endif 

  // Disable Arduino's timer0 interrupt handling so that it doesn't impact the timing measurements
  //  we're taking here
  cbi(TIMSK0, TOIE0);
  cbi(TIMSK2, TOIE2); // disable overflow interrupt until we need it again  
  
  // Measure time using timer/counter 2
  uint8_t elapsedTime;
  TCCR2A = 0;
  TCCR2B = (1 << CS20) ; // no prescaler
  TCNT2 = 0;
  uint8_t pinSelect,npinSelect;
  // Port D, pins 0-7
  //  Don't do Serial.begin() until later so that pins 0 and 1 can be tested as ordinary IO
  //  Use avr register names instead of Arduino pinMode, digitalWrite, etc functions otherwise
  //   the while loop where we wait won't be fast enough
  for (uint8_t i = PD0; i<=PD7; i++) {
    pinSelect = (1 << i);
    npinSelect = ~pinSelect;
    DDRD  = portDpinMask; // Set to outputs
    PORTD = npinSelect; // Set pin being tested to driving low, pins not being tested driving high
    NOP // figure 14-4 in ATmega328 datasheet
    // watch for a blog post that explains why the cast is needed in the next line
    //assert(PIND == (uint8_t)~pinSelect);  // Read pins to confirm correct value is driven
    assert(((PIND ^ npinSelect) & portDpinMask) == 0); 
    sbi(TIFR2,TOV2); // write-1-to clear overflow flag
    sbi(TIMSK2, TOIE2); // enable overflow interrupt so we timeout if never get pulled up  
    TCNT2 = 0; // start timer
    //DDRD  = 0; // Switch to input (could also do DDRD = ~pinSelect)
    DDRD = npinSelect; // pin being tested goes to input, others remain outputs
    NOP  // We've observed that we never make 5 cycles, sometimes 6, and always 7. This NOP lets us set the limit at 7 instead of 10
    PORTD = pinSelect; // Enable pullup. Optional on XLR8 because pullups are always on ports D and B
    // Following while loop could be made tighter by getting rid of for loop and hardcoding pin selects
    while (!(PIND & pinSelect)); // Wait for pullup to work. This becomes a 4 cycles loop
    elapsedTime = TCNT2;
    cbi(TIMSK2, TOIE2); // disable overflow interrupt until we need it again  
    #ifdef INCLUDE_DEBUG
    if (testnum < numTestDebug) {testDebug[testnum-1] = PIND;}
    if (testnum < numTestDebug) {testDebug[testnum] = elapsedTime;}
    #endif
    assert(elapsedTime <= maxPullupTimeD); // Check that pullup was fast enough
    if (elapsedTime < fastValD) {
      fastValD = elapsedTime;
      fastPinD = i;
    }
    if (elapsedTime > slowValD) {
      slowValD = elapsedTime;
      slowPinD = i;
    }
  }
  // Port B, pins 8-13
  for (uint8_t i = PB0; i<=PB5; i++) {
    pinSelect = (1 << i);
    npinSelect = ~pinSelect;
    DDRB  = portBpinMask; // Set to outputs
    PORTB = npinSelect; // Set pin being tested to driving low, pins not being tested driving high
    NOP // figure 14-4 in ATmega328 datasheet
    //assert(PINB == (uint8_t)~pinSelect);  // Read pins to confirm correct value is driven
    assert(((PINB ^ npinSelect) & portBpinMask) == 0); 
    sbi(TIFR2,TOV2); // write-1-to clear overflow flag
    sbi(TIMSK2, TOIE2); // enable overflow interrupt so we timeout if never get pulled up  
    TCNT2 = 0; // start timer
    //DDRB  = 0; // Switch to input (could also do DDRD = ~pinSelect)
    DDRB = npinSelect; // pin being tested goes to input, others remain outputs
    NOP
    PORTB = pinSelect; // Enable pullup. Optional on XLR8 because pullups are always on ports D and B
    // Following while loop could be made tighter by getting rid of for loop and hardcoding pin selects
    while (!(PINB & pinSelect)); // Wait for pullup to work
    elapsedTime = TCNT2;
    cbi(TIMSK2, TOIE2); // disable overflow interrupt until we need it again  
    #ifdef INCLUDE_DEBUG
    if (testnum < numTestDebug) {testDebug[testnum-1] = PINB;}
    if (testnum < numTestDebug) {testDebug[testnum] = elapsedTime;}
    #endif
    assert(elapsedTime <= maxPullupTimeB); // Check that pullup was fast enough
    if (elapsedTime < fastValB) {
      fastValB = elapsedTime;
      fastPinB = i;
    }
    if (elapsedTime > slowValB) {
      slowValB = elapsedTime;
      slowPinB = i;
    }
  }
  // Port C, pins A0-A5 check digitalWrite and digitalRead
  for (uint8_t i = PC0; i<=PC5; i++) {
    pinSelect = (1 << i);
    npinSelect = ~pinSelect;
    DDRC  = portCpinMask; // Set to outputs
    PORTC = npinSelect; // Set pin being tested to driving low, pins not being tested driving high
    NOP // figure 14-4 in ATmega328 datasheet
    //assert(PINC == (uint8_t)~pinSelect);  // Read pins to confirm correct value is driven
    assert(((PINC ^ npinSelect) & portCpinMask) == 0); 
    // On port C we don't have pullups, so expect either long time (>100 cycles) or timeout
    sbi(TIFR2,TOV2); // write-1-to clear overflow flag
    sbi(TIMSK2, TOIE2); // enable overflow interrupt so we timeout if never get pulled up  
    TCNT2 = 0; // start timer
    //DDRC  = 0; // Switch to input (could also do DDRD = ~pinSelect)
    DDRC = npinSelect; // pin being tested goes to input, others remain outputs
    //PORTC = pinSelect; // Enable pullup. Optional on XLR8 because port C never has pullups
    // Following while loop could be made tighter by getting rid of for loop and hardcoding pin selects
    while (!(PINC & pinSelect)); // Wait for pullup to work (it shouldn't)
    elapsedTime = TCNT2;
    cbi(TIMSK2, TOIE2); // disable overflow interrupt until we need it again  
   #ifdef INCLUDE_DEBUG
    if (testnum < numTestDebug) {testDebug[testnum-1] = PINC;}
    if (testnum < numTestDebug) {testDebug[testnum] = elapsedTime;}
   #endif
    assert(elapsedTime >= minPullupTimeC); // Check that pullup was slow (as expected)
  }
  // Port C, pins A0-A5 check analogRead
  //  speed not as important, go ahead an use Arduino functions
  for (uint8_t i = A0; i<=A5; i++) {
    pinMode(i,OUTPUT);
    digitalWrite(i,HIGH);
    int analogVal=analogRead(i);
   #ifdef INCLUDE_DEBUG
    areadih[i-A0]=analogVal;
   #endif
    assert(analogVal >= minAreadih);
    assert(analogVal <= maxAreadih);
    if (analogVal < lowValC) {
      lowValC = analogVal;
      lowPinC = i;
    }
    if (analogVal > highValC) {
      highValC = analogVal;
      highPinC = i;
    }
    digitalWrite(i,LOW);
    analogVal=analogRead(i);
   #ifdef INCLUDE_DEBUG
    areadil[i-A0]=analogVal;
   #endif
    assert(analogVal >= minAreadil);
    assert(analogVal <= maxAreadil);
  }
  analogReference(EXTERNAL);
  for (uint8_t i = A0; i<=A5; i++) {
    pinMode(i,OUTPUT);
    digitalWrite(i,HIGH);
    int analogVal=analogRead(i);
   #ifdef INCLUDE_DEBUG
    areadeh[i-A0]=analogVal;
   #endif
    assert(analogVal >= minAreadeh);
    assert(analogVal <= maxAreadeh);
    if (analogVal > highXrefh) {
      highXrefh = analogVal;
    }
    if (analogVal < lowXrefh) {
      lowXrefh = analogVal;
    }
    digitalWrite(i,LOW);
    analogVal=analogRead(i);
   #ifdef INCLUDE_DEBUG
    areadel[i-A0]=analogVal;
   #endif
    assert(analogVal >= minAreadel);
    assert(analogVal <= maxAreadel);
    if (analogVal > highXrefl) {
      highXrefl = analogVal;
    }
    if (analogVal < lowXrefl) {
      lowXrefl = analogVal;
    }
  }
  analogReference(DEFAULT);
  uint16_t vccRead=analogRead(7); // Not a real pin, but tied to divider on 5V supply
  
   
  // Set everything back to inputs
  DDRB = 0;
  DDRC = 0;
  DDRD = 0;

  // If running in simulation, the testbench watches the GPIOs to see when we're done
  GPIOR2 = testnum;
  GPIOR2 = lastfail;
  GPIOR1 = 0x80 | ((lastfail == -1) << 6); // bit 6 is pass/fail indicator

  
  // Report results across serial
  XLR8Info myXLR8;
  Serial.begin(115200);
  Serial.print("XLR8 Hardware Version Number = ");
  Serial.println(myXLR8.getXLR8Version());
  if (myXLR8.isVersionMixed()) {Serial.println("  Mixed version, lowest reported");}
  if (myXLR8.isVersionModified()) {Serial.println("  Modified working copy");}
  if (myXLR8.isVersionClean()) {Serial.println("  Clean working copy");}
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
  Serial.print("5V supply =~ ");
  Serial.print(5.0*vccRead/1024.0);
  Serial.print("V  assumed to be ");
  if (vccRead > 1021) {Serial.println("Barrel power");}
  else  {Serial.println("USB power");}

  Serial.println(F("******* TEST RESULTS **********"));
  Serial.print(testnum);Serial.println(" tests run");
  Serial.print(passnum);Serial.println(" tests pass");
  Serial.print(failnum);Serial.println(" tests fail");
  if (failnum) {
    Serial.print(F("  Last failure at test number "));Serial.println(lastfail);
    Serial.println(F("********** FAIL ***************"));
  } else if (passnum == testnum) {
    Serial.println(F("********** PASS ***************"));
  } else {
    Serial.println(F("***** SOME TESTS NOT RUN ******"));
  } 
  Serial.println();
  
  // Post results to our product improvement page
  Serial.println();
  Serial.println(F("To help improve our products, please paste the followig URL into a web browser, add any notes, and click submit"));
  Serial.print(F("https://docs.google.com/forms/d/1GCmN3hRF-fnkr0J8K8HBrYvmuhmz6y94ViadHgobeRQ/viewform?"));
  Serial.print(F("entry.498366088="));Serial.print(F("(optional)")); // Board Number
  Serial.print(F("&entry.110701079=0x"));Serial.print(myXLR8.getChipId(),HEX); // Chip ID
  Serial.print(F("&entry.328279444="));Serial.print(myXLR8.getXLR8Version()); // subversion
  Serial.print(F("&entry.1881676735="));Serial.print(myXLR8.getDesignConfig()); // Design Config
  Serial.print(F("&entry.821741284="));Serial.print(myXLR8.getXBEnables()); // XB Enable
  Serial.print(F("&entry.1215707535="));Serial.print("XLR8SelfTest"); // Notes
  Serial.print(F("&entry.545646289="));Serial.print(intOscSpeed); // Speed Test
  Serial.print(F("&entry.1561823473="));Serial.print(vccRead); // 5V supply measurement
  Serial.print(F("&entry.882185102=Record Test Results")); // Add test results
  Serial.print(F("&entry.1879785912=")); // Pass/Fail/Status
  if (failnum) {Serial.print("FAIL");}
  else if (passnum == testnum) {Serial.print("PASS");}
  else {Serial.print("INCOMPLETE");}
  Serial.print(F("&entry.1656234522=("));Serial.print(fastPinD); // Port D measurements
  Serial.print(",");Serial.print(fastValD);
  Serial.print(",");Serial.print(slowPinD);
  Serial.print(",");Serial.print(slowValD);Serial.print(")");
  Serial.print(F("&entry.1272501077=("));Serial.print(fastPinB); // Port B measurements
  Serial.print(",");Serial.print(fastValB);
  Serial.print(",");Serial.print(slowPinB);
  Serial.print(",");Serial.print(slowValB);Serial.print(")");
  Serial.print(F("&entry.525544221=("));Serial.print(highPinC); // Port C measurements
  Serial.print(",");Serial.print(highValC);
  Serial.print(",");Serial.print(lowPinC);
  Serial.print(",");Serial.print(lowValC);Serial.print(")");
  Serial.print(F("&entry.51020830=("));Serial.print(highXrefh); // Ext Ref measurements
  Serial.print(",");Serial.print(lowXrefh);
  Serial.print(",");Serial.print(highXrefl);
  Serial.print(",");Serial.print(lowXrefl);Serial.print(")");
  Serial.println();
  Serial.flush();


 #ifdef INCLUDE_DEBUG
  for (uint8_t i = 0; i<numTestDebug && i<numFailFlags; i++) {
    if (failFlags[i]) {Serial.print("*");}
    else {Serial.print(" ");}
    Serial.print(i);Serial.print("  ");Serial.println(testDebug[i]);
  }
  for (uint8_t i = numTestDebug; i<numFailFlags; i++) {
    if (failFlags[i]) {Serial.print("*");Serial.println(i);}
  }
  for (uint8_t i = numFailFlags; i<numTestDebug; i++) {
    Serial.print(" ");
    Serial.print(i);Serial.print("  ");Serial.println(testDebug[i]);
  }  
  Serial.println("pin  IH   IL   EH   EL");
  for (uint8_t i = 0; i<6; i++) {
    Serial.print(i);Serial.print("  ");
    if ((failFlags[40+4*i]) || (failFlags[41+4*i])) {Serial.print("*");} // check failFlags index
    else {Serial.print(" ");}
    Serial.print(areadih[i]);Serial.print("   ");
    if ((failFlags[42+4*i]) || (failFlags[43+4*i])) {Serial.print("*");} // check failFlags index
    else {Serial.print(" ");}
    Serial.print(areadil[i]);Serial.print(" ");
    if ((failFlags[64+4*i]) || (failFlags[65+4*i])) {Serial.print("*");} // check failFlags index
    else {Serial.print(" ");}
    Serial.print(areadeh[i]);Serial.print(" ");
    if ((failFlags[66+4*i]) || (failFlags[67+4*i])) {Serial.print("*");} // check failFlags index
    else {Serial.print(" ");}
    Serial.print(areadel[i]);Serial.println();
  }
  Serial.flush();
 #endif 
}

void loop() {
}

void assert(bool r) {
  if (r) {
    passnum++;
  } else {
    failnum++;
    lastfail = testnum;
   #ifdef INCLUDE_DEBUG
    if (testnum < numFailFlags) {failFlags[testnum] = true;}
    GPIOR2 = testnum;
   #endif
  }
  testnum++;
}  

// if we don't get response we're looking for in time, call it an error
//   and set pins to force us to exit the while loop that we're spinning in
ISR(TIMER2_OVF_vect) {
    DDRD  = 0xFF; // Set to outputs
    PORTD = 0xFF; // Driving High
    DDRD  = 0;    // Set back to inputs, they should remain high
    DDRB  = 0x3F; // Set to outputs
    PORTB = 0x3F; // Driving High
    DDRB  = 0;    // Set back to inputs, they should remain high
    DDRC  = 0x3F; // Set to outputs
    PORTC = 0x3F; // Driving High
    DDRC  = 0;    // Set back to inputs, they should remain high
    TCNT2 = -32;  // Big enough to fail, but small enough to to get checked before overflowing again
}

// Timer 1 used for speed test
ISR(TIMER1_CAPT_vect) { 
  if (index < SAMPLES) {
    timestamp[index++] = ICR1;
  }
} 

