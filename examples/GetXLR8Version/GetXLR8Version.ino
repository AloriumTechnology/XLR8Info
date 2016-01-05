#include <XLR8Info.h>

void setup() {
  XLR8Info myXLR8;
  Serial.begin(115200);
  Serial.print("XLR8 Hardware Version Number = ");
  Serial.println(myXLR8.getXLR8Version());
  if (myXLR8.isVersionMixed()) {Serial.println("  Mixed version, lowest reported");}
  if (myXLR8.isVersionModified()) {Serial.println("  Modified working copy");}
  if (myXLR8.isVersionClean()) {Serial.println("  Clean working copy");}
  Serial.print("XLR8 CID = 0x");
  // Serial.print doesn't seem to do HEX format with 64 bit values, so create our own
  uint64_t temp64 = myXLR8.getChipId();
  uint8_t i;
  for (i = 0; i < 16; i++) { // Do a nibble at a time otherwise zero padding gets thrown away.
    uint8_t temp_nibble = (temp64 >> 60) & 15; // Get top nibble first
    Serial.print(temp_nibble,HEX);
    temp64 = temp64 << 4; // move next nibble up to top
  }
  Serial.println("");
  Serial.print("XB_ENABLE[6:0] = 0x");
  Serial.println(myXLR8.getXBEnables(),HEX);
  if (myXLR8.hasBootRestore())     {Serial.println(F("Has Boot Restore"));}
  if (myXLR8.hasXLR8FloatAddSubMult()) {Serial.println(F("Has Floating Point Add, Subtract, and Multiply"));}
  if (myXLR8.hasXLR8FloatDiv())        {Serial.println(F("Has Floating Point Divide"));}
  if (myXLR8.hasXLR8FloatConvert())    {Serial.println(F("Has Floating Point <-> Fixed Point Conversion"));}
  if (myXLR8.hasXLR8Servo())           {Serial.println(F("Has Servo XB"));}
  if (myXLR8.hasXLR8NeoPixel())        {Serial.println(F("Has NeoPixel XB"));}
  Serial.flush();
  GPIOR1 = 0xc0;
}

void loop() {
}

