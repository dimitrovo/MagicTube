/* Arduino Nano Code for MagicElectronTube project MET3
 * http://
 * Copyright (c) 2023 Serhii Zaitsev <dimitrovo@gmail.com>
 *
 * Using library: PinChangeInterrupt, FreqCount, SSD1306Ascii
 * Version 1.0
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */



#include <Wire.h>
#include <FreqCount.h>
#include <SPI.h>
#include <EEPROM.h>




//-------Begin hardware definition block---------------------------------------

#define SWITCHER1 10
#define SWITCHER2 11
#define SWITCHER3 12
int buttonState1 = 0;
int buttonState2 = 0;
int buttonState3 = 0;
//---------------------OLED----------------------------------------------------
//#define I2C_ADDRESS 0x3C
//SSD1306AsciiWire oled;
//--------------------RDA5807 -------------------------------------------------
#define RDA5807M_RANDOM_ACCESS_ADDRESS 0x11
// Registers
#define RDA5807M_REG_CONFIG 0x02
#define RDA5807M_REG_TUNING 0x03
#define RDA5807M_REG_VOLUME 0x05
#define RDA5807M_REG_RSSI   0x0B
// FLAGS
#define RDA5807M_FLG_DHIZ 0x8000 //15 DHIZ
#define RDA5807M_FLG_DMUTE 0x4000  //14 DMUTE 
#define RDA5807M_FLG_BASS 0x1000// 12 BASS
#define RDA5807M_FLG_ENABLE word(0x0001)  //0  ENABLE
#define RDA5807M_FLG_TUNE word(0x0010)
#define RDA5807M_FLG_MONO 0x2000 
#define RDA5807M_BIT_MUTE 14
// MASKS
#define RDA5807M_CHAN_MASK 0xFFC0
#define RDA5807M_CHAN_SHIFT 6
#define RDA5807M_VOLUME_MASK word(0x000F)
#define RDA5807M_VOLUME_SHIFT 0
#define RDA5807M_RSSI_MASK 0xFE00
#define RDA5807M_RSSI_SHIFT 9
//-----------------------AD9833----------------------------------------------
//#define FNC_PIN 10 

const byte numberOfDigits = 6; // number of digits in the frequense
byte freqSGLo[numberOfDigits] = {0, 0, 0, 1, 0, 0}; // 1000Hz 
byte freqSGHi[numberOfDigits] = {0, 0, 0, 0, 2, 0}; // 20kHz  

const int wSine     = 0b0000000000000000;
//const int wTriangle = 0b0000000000000010;
//const int wSquare   = 0b0000000000101000;

int waveType = wSine;


int SG_iSweep,SG_nSweep;


const int SG_fsyncPin = 7;
const int SG_CLK = 8;
const int SG_DATA = 9;

long lOutFrequency = 0;
//-------End hardware defenition block--------------------------------------------------






//------------------------Global Frequencies defenitions ---------------
#define FM_MIN 875 //FM radio range
#define FM_MAX 1080

const unsigned long c_MIN_OUT_FREQUENCY = 100000; //Low boundary of oscillator

unsigned long g_lFrequency; // oscillating circuit frequency of tube  receiver
const uint8_t g_FF=4; //frequency factor

const unsigned long c_lMinPossibleFrq = 100000; //Max and Min possible generation range to restrict bad frequencies
const unsigned long c_lMaxPossibleFrq = 1200000;

unsigned long g_lMinFrq = 0; //Current range input frequency to map on FM range
unsigned long g_lMaxFrq = 0;

unsigned int g_iFMFreq; //FM frequency in Mhz*10
unsigned int g_iFMFreq_old = 1050;

//Customizations store
struct TCust{ 
  unsigned long lMinFreq;
  unsigned long lMaxFreq;
} cust, old_cust;

bool bRangeChanged = false;
const unsigned int cFixRange = 0;
const unsigned int cDynRange = 1;
unsigned int bMode = 0;
bool bModeSaved = false;


void setup() {



 Serial.begin(115200);


 //Read and proceed saved customisation
  EEPROM.get( 0, cust );
  if(cust.lMinFreq < c_lMinPossibleFrq || cust.lMinFreq > c_lMaxPossibleFrq || cust.lMaxFreq < c_lMinPossibleFrq || cust.lMaxFreq > c_lMaxPossibleFrq){
     cust.lMinFreq = 0;
     cust.lMaxFreq = 0;
  } //checkin read values for trash

  if (cust.lMinFreq ==0 && cust.lMaxFreq == 0) { //no saved range, generate it in process
   g_lMaxFrq = c_lMinPossibleFrq; //cross-initialize to fill data that are ready to compare operations (see loop)
   g_lMinFrq = c_lMaxPossibleFrq;
   bMode = cDynRange;
  } 
  else { //range exists, use it
    g_lMaxFrq = cust.lMaxFreq;
    g_lMinFrq = cust.lMinFreq;
    bMode = cFixRange;
  }
   old_cust.lMaxFreq =0;
   old_cust.lMinFreq =0;
   EEPROM.put(0, old_cust); //if we swith off the device less 3 sec, next mode will be DYNAMIC range.
   Serial.println("Next start:Dynamics");
  
 //Prepare hardware
  Wire.begin();
 // oled.begin(&Adafruit128x64, I2C_ADDRESS);
  FreqCount.begin(1000/g_FF);
  setupRDA5807();
  setRDA5807frequency(1036);
  setRDA5807volume(15); 

  pinMode(SWITCHER1,INPUT_PULLUP);
  pinMode(SWITCHER2,INPUT_PULLUP);
  pinMode(SWITCHER3,INPUT_PULLUP);
  

  
  //setAD9833frequency(465000);
    InitSigGen();
    
   
 Serial.println("Started");

   buttonState1 = digitalRead(SWITCHER1);
   buttonState2 = digitalRead(SWITCHER2);
   buttonState3 = digitalRead(SWITCHER3);

  if (buttonState1 == HIGH && buttonState2 == HIGH && buttonState3 == HIGH) lOutFrequency = 0;
  if (buttonState1 == LOW && buttonState2 == HIGH && buttonState3 == HIGH) lOutFrequency = 465000;
  if (buttonState1 == LOW && buttonState2 == LOW && buttonState3 == HIGH) lOutFrequency =  455000;
  if (buttonState1 == HIGH && buttonState2 == LOW && buttonState3 == HIGH) lOutFrequency = 460000;
  if (buttonState1 == HIGH && buttonState2 == LOW && buttonState3 == LOW) lOutFrequency = 470000;
  if (buttonState1 == HIGH && buttonState2 == HIGH && buttonState3 == LOW) lOutFrequency = 1000000;

  Serial.print("Output Frequency: ");  Serial.println(lOutFrequency);
    
  SG_freqSet(lOutFrequency,wSine);
  
}



void loop() {

//Serial.println(millis());
//delay(100);

 //if we swith off the device less 3 sec, next mode will be DYNAMIC range.
 if(millis() > 3000 && bMode == cFixRange && ! bModeSaved) {
  EEPROM.put(0, cust); 
  bModeSaved = true;
  Serial.println("Next start:Fixed");
 }
 

 

 if (FreqCount.available()) {
    g_lFrequency = g_FF*FreqCount.read();

    if (g_lFrequency > c_lMinPossibleFrq &&  g_lFrequency < c_lMaxPossibleFrq ) {  //0607
    
    if (g_lFrequency > c_lMinPossibleFrq && g_lFrequency < g_lMinFrq ) { g_lMinFrq = g_lFrequency; bRangeChanged = true; }
    if (g_lFrequency < c_lMaxPossibleFrq && g_lFrequency > g_lMaxFrq ) { g_lMaxFrq = g_lFrequency; bRangeChanged = true; }
    g_iFMFreq = FM_MIN + (g_lFrequency - g_lMinFrq ) * (float(FM_MAX-FM_MIN)/float(g_lMaxFrq-g_lMinFrq));  //Set FM freq corresponds to circuit frequency
    if(g_iFMFreq != g_iFMFreq_old && g_lFrequency >= g_lMinFrq && g_lFrequency <= g_lMaxFrq  ) {  //to prevent parasit switching
      setRDA5807frequency(g_iFMFreq);
  //    Serial.println(g_iFMFreq);
      g_iFMFreq_old = g_iFMFreq;
     // g_bUpdated = true;
     } 

    }  //0607
    Serial.print(millis()); Serial.print(" m: ");Serial.print(bMode);
    Serial.print(" fr: ");Serial.print(g_lFrequency);Serial.print(" min:");Serial.print(g_lMinFrq);;Serial.print(" max:");Serial.print(g_lMaxFrq);
    Serial.print(" iFMF:");Serial.print(g_iFMFreq);Serial.println("=>");
  }

 //Ranges changes
 if( bRangeChanged  && bMode == cDynRange ){
   bRangeChanged = false;  
   cust.lMinFreq = g_lMinFrq;
   cust.lMaxFreq = g_lMaxFrq;
   EEPROM.put(0, cust); 
 }  
}





//-----------------------------------------------------------------------------
//returns 10^y
//-----------------------------------------------------------------------------
unsigned long Power(int y) {
  unsigned long t = 1;
  for (byte i = 0; i < y; i++)
    t = t * 10;
  return t;
}

//-----------------------------------------------------------------------------
//calculate the frequency from the array.
//-----------------------------------------------------------------------------
unsigned long calcFreq(byte* freqSG) {
  unsigned long i = 0;
  for (byte x = 0; x < numberOfDigits; x++)
    i = i + freqSG[x] * Power(x);
  return i;
}

//-----------------------------------------------------------------------------
// SG_WriteRegister
//-----------------------------------------------------------------------------
void SG_WriteRegister(word dat) {
  digitalWrite(SG_CLK, LOW);
  digitalWrite(SG_CLK, HIGH);

  digitalWrite(SG_fsyncPin, LOW);
  for (byte i = 0; i < 16; i++) {
    if (dat & 0x8000)
      digitalWrite(SG_DATA, HIGH);
    else
      digitalWrite(SG_DATA, LOW);
    dat = dat << 1;
    digitalWrite(SG_CLK, HIGH);
    digitalWrite(SG_CLK, LOW);
  }
  digitalWrite(SG_CLK, HIGH);
  digitalWrite(SG_fsyncPin, HIGH);
}

//-----------------------------------------------------------------------------
// SG_Reset
//-----------------------------------------------------------------------------
void SG_Reset() {
  delay(100);
  SG_WriteRegister(0x100);
  delay(100);
}

//-----------------------------------------------------------------------------
// SG_freqReset
//    reset the SG regs then set the frequency and wave type
//-----------------------------------------------------------------------------
void SG_freqReset(long frequency, int wave) {
  long fl = frequency * (0x10000000 / 25000000.0);
  SG_WriteRegister(0x2100);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
  SG_WriteRegister(0xC000);
  SG_WriteRegister(wave);
  waveType = wave;
}

//-----------------------------------------------------------------------------
// SG_freqSet
//    set the SG frequency regs 
//-----------------------------------------------------------------------------
void SG_freqSet(long frequency, int wave) {
  long fl = frequency * (0x10000000 / 25000000.0);
  SG_WriteRegister(0x2000 | wave);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
}

void InitSigGen(void) {
 // pinMode(SG_POWER, OUTPUT);
 // digitalWrite(SG_POWER, HIGH);

  pinMode(SG_DATA, OUTPUT);
  pinMode(SG_CLK, OUTPUT);
  pinMode(SG_fsyncPin, OUTPUT);
  digitalWrite(SG_fsyncPin, HIGH);
  digitalWrite(SG_CLK, HIGH);
  SG_Reset();
  SG_freqReset(calcFreq(freqSGLo), waveType);
}


/////RDA5807

void setRegister(uint8_t reg, const uint16_t value) {
  Wire.beginTransmission(0x11);
  Wire.write(reg);
  Wire.write(highByte(value));
  Wire.write(lowByte(value));
  Wire.endTransmission(true);
}
 
uint16_t getRegister(uint8_t reg) {
  uint16_t result;
  Wire.beginTransmission(RDA5807M_RANDOM_ACCESS_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(0x11, 2, true);
  result = (uint16_t)Wire.read() << 8;
  result |= Wire.read();
  return result;
}

void setupRDA5807()
{
uint16_t reg02h, reg03h, reg05h, reg0Bh;
// Register 02h is settings
  reg02h = RDA5807M_FLG_ENABLE | RDA5807M_FLG_DHIZ | RDA5807M_FLG_DMUTE;
  setRegister(RDA5807M_REG_CONFIG, reg02h);
  reg02h |= RDA5807M_FLG_BASS; //bass
  setRegister(RDA5807M_REG_CONFIG, reg02h);
  reg02h |= RDA5807M_FLG_MONO; //Mono mode
  setRegister(RDA5807M_REG_CONFIG, reg02h);
}

void setRDA5807frequency(uint16_t freq)
{
uint16_t reg02h, reg03h, reg05h, reg0Bh;
  // Register 03h is radio station selection
  // After reset the default value in register 03h is 0
  // So BAND = 00 (87..108MHz), STEP = 00 (100kHz).
  reg03h = (freq - 870) << RDA5807M_CHAN_SHIFT; // chan = (freq - band) / space
  setRegister(RDA5807M_REG_TUNING, reg03h | RDA5807M_FLG_TUNE);
}


void setRDA5807volume(uint8_t v)
{
  uint16_t reg02h, reg03h, reg05h, reg0Bh;
  // Register 05h. Set the volume, do not change the other bits
  reg05h = getRegister(RDA5807M_REG_VOLUME); // read the current value
  reg05h &= ~RDA5807M_VOLUME_MASK; // Reset the VOLUME bits
  reg05h |= v << RDA5807M_VOLUME_SHIFT; // set a new volume
  setRegister(RDA5807M_REG_VOLUME, reg05h);
 
}
