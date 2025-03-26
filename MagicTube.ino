
/* Arduino Nano Code for MagicElectronTube project MET3
 * https://github.com/dimitrovo/MagicTube
 * Copyright (c) 2025 Serhii Zaitsev <dimitrovo@gmail.com>
 *
 * Using library:  FreqCount
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


//--------------------RDA5807 -------------------------------------------------
#define RDA5807M_RANDOM_ACCESS_ADDRESS 0x11
// Registers
#define RDA5807M_REG_CONFIG 0x02
#define RDA5807M_REG_TUNING 0x03
#define RDA5807M_REG_VOLUME 0x05
#define RDA5807M_REG_RSSI   0x0B
// FLAGS
#define RDA5807M_FLG_DHIZ 0x8000 
#define RDA5807M_FLG_DMUTE 0x4000  
#define RDA5807M_FLG_BASS 0x1000
#define RDA5807M_FLG_ENABLE word(0x0001) 
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

const byte numberOfDigits = 6; 
byte freqSGLo[numberOfDigits] = {0, 0, 0, 1, 0, 0}; 
const int wSine     = 0b0000000000000000;
int waveType = wSine;


const int SG_fsyncPin = 7;
const int SG_CLK = 8;
const int SG_DATA = 9;

long lOutFrequency = 0;
//-------End hardware defenition block--------------------------------------------------
//------------------------Global Frequencies defenitions ---------------
#define FM_MIN 875
#define FM_MAX 1080

const unsigned long c_MIN_OUT_FREQUENCY = 100000; 
unsigned long g_lFrequency; 
const uint8_t g_FF=10; 
const unsigned long c_lMinPossibleFrq = 100000;
const unsigned long c_lMaxPossibleFrq = 1200000;
unsigned long g_lMinFrq = 0; 
unsigned long g_lMaxFrq = 0;
unsigned int g_iFMFreq; 
unsigned int g_iFMFreq_old = 1050;

struct TCust{ 
  byte mode;
  unsigned long lMinFreq;
  unsigned long lMaxFreq;
} cust;

bool bRangeChanged = false;
const unsigned int cFixRange = 0;
const unsigned int cDynRange = 1;
unsigned int bMode = 0;

const unsigned long threshold = 5000; 
unsigned long startTime;             
const int ledPin = 13;       
        
bool updated = false; 

void setup() {
 

 pinMode(ledPin, OUTPUT); 

 EEPROM.get( 0, cust );
  if(cust.lMinFreq < c_lMinPossibleFrq || cust.lMaxFreq > c_lMaxPossibleFrq ||  cust.mode < 0 || cust.mode > 1 ){
     cust.lMinFreq = 0;
     cust.lMaxFreq = 0;
     cust.mode = cDynRange;
     bMode = cDynRange;
  } 
  EEPROM.put(0, cust);

  
 if (  cust.mode == cDynRange  ) { 
   g_lMaxFrq = c_lMinPossibleFrq; 
   g_lMinFrq = c_lMaxPossibleFrq;
   bMode = cDynRange;
  } 
  else { 
    g_lMaxFrq = cust.lMaxFreq;
    g_lMinFrq = cust.lMinFreq;
    bMode = cFixRange;
  }

 
   cust.mode = cDynRange;
   EEPROM.put(0, cust);
   digitalWrite(ledPin, bMode == cDynRange ? HIGH : LOW);

 
   startTime = millis();
  
  Wire.begin();
  FreqCount.begin(1000/g_FF);
  setupRDA5807();
  setRDA5807frequency(1036);
  setRDA5807volume(15); 
  pinMode(SWITCHER1,INPUT_PULLUP);
  pinMode(SWITCHER2,INPUT_PULLUP);
  pinMode(SWITCHER3,INPUT_PULLUP);
  
    
  int buttonState1 = 0;
  int buttonState2 = 0;
  int buttonState3 = 0;
  buttonState1 = digitalRead(SWITCHER1);
  buttonState2 = digitalRead(SWITCHER2);
  buttonState3 = digitalRead(SWITCHER3);

  if (buttonState1 == HIGH && buttonState2 == HIGH && buttonState3 == HIGH) lOutFrequency = 0;
  if (buttonState1 == LOW && buttonState2 == HIGH && buttonState3 == HIGH) lOutFrequency = 465000;
  if (buttonState1 == LOW && buttonState2 == LOW && buttonState3 == HIGH) lOutFrequency =  455000;
  if (buttonState1 == HIGH && buttonState2 == LOW && buttonState3 == HIGH) lOutFrequency = 460000;
  if (buttonState1 == HIGH && buttonState2 == LOW && buttonState3 == LOW) lOutFrequency = 470000;
  if (buttonState1 == HIGH && buttonState2 == HIGH && buttonState3 == LOW) lOutFrequency = 1000000;
  if (buttonState1 == LOW && buttonState2 == LOW && buttonState3 == LOW) lOutFrequency = 0;//Please enter your value
  
  InitSigGen();
  SG_freqSet(lOutFrequency,wSine);
}



void loop() {

 unsigned long elapsedTime = millis() - startTime;
 digitalWrite(ledPin, bMode == cDynRange ? HIGH : LOW);
  if (elapsedTime > threshold && !updated) {
    cust.mode = cFixRange;
    EEPROM.put(0, cust);
    updated = true;
  }

 if (FreqCount.available()) {
    g_lFrequency = g_FF*FreqCount.read();

    if (g_lFrequency > c_lMinPossibleFrq &&  g_lFrequency < c_lMaxPossibleFrq ) {  //only valid value
    if(bMode == cDynRange) {
        if (g_lFrequency > c_lMinPossibleFrq && g_lFrequency < g_lMinFrq ) { g_lMinFrq = g_lFrequency; bRangeChanged = true; }
        if (g_lFrequency < c_lMaxPossibleFrq && g_lFrequency > g_lMaxFrq ) { g_lMaxFrq = g_lFrequency; bRangeChanged = true; }
    } 
    g_iFMFreq = FM_MIN + (g_lFrequency - g_lMinFrq ) * (float(FM_MAX-FM_MIN)/float(g_lMaxFrq-g_lMinFrq));  
    
    if(g_iFMFreq != g_iFMFreq_old && g_lFrequency >= g_lMinFrq && g_lFrequency <= g_lMaxFrq  ) {  
      setRDA5807frequency(g_iFMFreq);
      g_iFMFreq_old = g_iFMFreq;
     } 
    }  
  }

 
 if( bRangeChanged  && bMode == cDynRange ){
   bRangeChanged = false;  
   cust.lMinFreq = g_lMinFrq;
   cust.lMaxFreq = g_lMaxFrq;
   EEPROM.put(0, cust); 
 }  
}


unsigned long Power(int y) {
  unsigned long t = 1;
  for (byte i = 0; i < y; i++)
    t = t * 10;
  return t;
}

unsigned long calcFreq(byte* freqSG) {
  unsigned long i = 0;
  for (byte x = 0; x < numberOfDigits; x++)
    i = i + freqSG[x] * Power(x);
  return i;
}

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

void SG_Reset() {
  delay(100);
  SG_WriteRegister(0x100);
  delay(100);
}

void SG_freqReset(long frequency, int wave) {
  long fl = frequency * (0x10000000 / 25000000.0);
  SG_WriteRegister(0x2100);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
  SG_WriteRegister(0xC000);
  SG_WriteRegister(wave);
  waveType = wave;
}

void SG_freqSet(long frequency, int wave) {
  long fl = frequency * (0x10000000 / 25000000.0);
  SG_WriteRegister(0x2000 | wave);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
}

void InitSigGen(void) {
  pinMode(SG_DATA, OUTPUT);
  pinMode(SG_CLK, OUTPUT);
  pinMode(SG_fsyncPin, OUTPUT);
  digitalWrite(SG_fsyncPin, HIGH);
  digitalWrite(SG_CLK, HIGH);
  SG_Reset();
  SG_freqReset(calcFreq(freqSGLo), waveType);
}


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
  reg02h = RDA5807M_FLG_ENABLE | RDA5807M_FLG_DHIZ | RDA5807M_FLG_DMUTE;
  setRegister(RDA5807M_REG_CONFIG, reg02h);
  reg02h |= RDA5807M_FLG_BASS; 
  setRegister(RDA5807M_REG_CONFIG, reg02h);
  reg02h |= RDA5807M_FLG_MONO; 
  setRegister(RDA5807M_REG_CONFIG, reg02h);
}

void setRDA5807frequency(uint16_t freq)
{
uint16_t reg02h, reg03h, reg05h, reg0Bh;
  reg03h = (freq - 870) << RDA5807M_CHAN_SHIFT; 
  setRegister(RDA5807M_REG_TUNING, reg03h | RDA5807M_FLG_TUNE);
}


void setRDA5807volume(uint8_t v)
{
  uint16_t reg02h, reg03h, reg05h, reg0Bh;
  reg05h = getRegister(RDA5807M_REG_VOLUME); 
  reg05h &= ~RDA5807M_VOLUME_MASK; 
  reg05h |= v << RDA5807M_VOLUME_SHIFT; 
  setRegister(RDA5807M_REG_VOLUME, reg05h);
}
