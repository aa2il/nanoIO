//======================================================================
//
//  nanoIO paddle keyer (c) 2018, David Freese, W1HKJ
//
//  based on code from Iambic Keyer Code Keyer Sketch
//  Copyright (c) 2009 Steven T. Elliott
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details:
//
//  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
//  Boston, MA  02111-1307  USA
//
//1.0.0:  Initial release
//1.0.1:  MORTTY Version 3 board support
//1.1.0:  added support for WPM potentiometer; wiper on A0

//======================================================================

#include "Arduino.h"

// Timer One only works on the arduino
#if not defined(ESP32)
#include "TimerOne.h"
#endif

#include "Morse.h"
#include "Keyer.h"

//======================================================================
//  keyerControl bit definitions
//
#define     DIT_L      0x01     // Dit latch
#define     DAH_L      0x02     // Dah latch
#define     DIT_PROC   0x04     // Dit is being processed
#define     PDLSWAP    0x08     // 0 for normal, 1 for swap
//======================================================================
//
//  State Machine Defines

enum KSTYPE {IDLE, CHK_DIT, CHK_DAH, KEYED_PREP, KEYED, INTER_ELEMENT };

Keyer::Keyer(int wpm, float weight)
{
  ptt_pin_ = PTT_PIN;
  cw_pin_ = CW_PIN;

  // Setup inputs & outputs - also enable internal pull-up resistors on inputs
  pinMode(LP_in, INPUT_PULLUP);            // sets Left Paddle digital pin as input
  pinMode(RP_in, INPUT_PULLUP);            // sets Right Paddle digital pin as input
#ifdef SK_in  
  pinMode(SK_in, INPUT_PULLUP);            // sets Straight Key digital pin as input
#endif  
#ifdef SIDETONE
  pinMode(ST_Pin, OUTPUT);          // Sets the Sidetone digital pin as output
#endif  
  
  keyerState = IDLE;
  keyerControl = 0;
  key_mode = IAMBICA;
  _weight = weight;

  _speed = wpm;
  calc_ratio();
}

// Calculate the length of dot, dash and silence
void Keyer::calc_ratio()
{
  float w = (1 + _weight) / (_weight -1);
  _space_len = 9 * (1200 / _speed) / 10;
  _dotlen = _space_len * (w - 1);
  _dashlen =  (1 + w) * _space_len;
}

void Keyer::cw_pin(int pin)
{
	cw_pin_ = pin;
}

void Keyer::ptt_pin(int pin)
{
	ptt_pin_ = pin;
}

void Keyer::set_mode(int md)
{
  key_mode = md;
}

void Keyer::wpm(int wpm)
{
  _speed = wpm;
  calc_ratio();
}
//======================================================================
//    Latch paddle press
//======================================================================

void Keyer::update_PaddleLatch()
{
	if (digitalRead(RP_in) == LOW) {
		keyerControl |= DIT_L;
	}
	if (digitalRead(LP_in) == LOW) {
		keyerControl |= DAH_L;
	}
}


#ifdef ECHO_PRACTICE
static unsigned long tup=0,ch=0;
static int element,bit=1;

#define MAX_TIMES 0  
//#define MAX_TIMES 100
#if MAX_TIMES>0
//static long times[MAX_TIMES],ntimes=0;
static int times[MAX_TIMES],ntimes=0;

void print_times()
{
  int i;
  
  Serial.write(" \n ");
  for(i=0;i<ntimes; i++) {
    Serial.print(times[i]);
    Serial.print(' ');
  }
  //Serial.write("\n ");
  ntimes=0;
}  
#endif


void Keyer::echo_timing(int flush)
{
  unsigned long t0,dt;
  static int need_space=0,need_eol=0;

  // Determine time since the key up from the last element
  dt = millis()-tup;
#if MAX_TIMES>0
  if(flush==0) {
    if(ntimes<MAX_TIMES)
      times[ntimes++] = dt;
    if(ntimes>=MAX_TIMES) {
      print_times();       // Too slow
      ntimes=0;
    }
  }
#endif

  // Element spacing is 1 dit
  // Char spacing is 3 dits
  // Word spacing is 7 dits
  // If the spacing is over 10 dits, we throw in a linefeed for good measure
  
  // Are we past the element spacing? 
  if( dt>2*_space_len) {

    // Have any elements been sent?
    if( bit>1 ) {
      ch |= bit;
      char c=elements2char(ch);
      //Serial.write("\nSENT="); Serial.print(ch,BIN); 
      //Serial.write("\t");
      Serial.print(c);
      need_space=1;
      
      // Get ready for next char
      ch=0;
      bit=1;
    }
    
    // Are we past the word spacing and not yet sent a space?
    if( dt>5*_space_len & need_space) {
      //Serial.write("\nSENT="); Serial.print(1,BIN); 
      //Serial.write("\t");
      Serial.print(' ');
      need_space=0;
      need_eol=1;
    }
      
    // Are we past the line spacing and not yet sent a line feed?
    if( dt>10*_space_len & need_eol) {
      need_eol=0;
      if(flush) {
#if MAX_TIMES>0
        print_times();
#else          
        Serial.write("\n ");
#endif
      }
    }
    
  }
  
}
#endif      



bool Keyer::do_paddles()
{

  // Handle straight key  
#ifdef SK_in
  static int sk=0;
  
  // Check Straight Key connected to its own pin
  // Doing it this way allows both paddles and straight key to work together
  if (digitalRead(SK_in) == LOW) {
    sk=1;
    digitalWrite(ptt_pin_, HIGH);
    digitalWrite(cw_pin_, HIGH);
#ifdef SIDETONE      
    tone(ST_Pin, ST_Freq);      // Turn the Sidetone on
#endif      
    return true;
  } else if(sk) {
    sk=0;
    digitalWrite(ptt_pin_, LOW);
    digitalWrite(cw_pin_, LOW);
#ifdef SIDETONE      
    noTone(ST_Pin);      // Turn the Sidetone off
#endif      
    return true;
  }

#else

  // Original nano IO connected straight key to either of the paddle pins
  // and had to put into STRAIGHT mode 
  if (key_mode == STRAIGHT) { // Straight Key
    if ((digitalRead(LP_in) == LOW) || (digitalRead(RP_in) == LOW)) {
      // Key from either paddle
      digitalWrite(ptt_pin_, HIGH);
      digitalWrite(cw_pin_, HIGH);
#ifdef SIDETONE      
      tone(ST_Pin, ST_Freq);      // Turn the Sidetone on
#endif      
      return true;
    } else {
      digitalWrite(ptt_pin_, LOW);
      digitalWrite(cw_pin_, LOW);
#ifdef SIDETONE      
      noTone(ST_Pin);      // Turn the Sidetone off
#endif      
    }
    return false;
  }
  
#endif  

  // keyerControl contains processing flags and keyer mode bits
  // Supports Iambic A and B
  // State machine based, uses calls to millis() for timing.
  switch (keyerState) {
    case IDLE:      // Wait for direct or latched paddle press
#ifdef ECHO_PRACTICE
      echo_timing(1);
#endif      
      if ((digitalRead(LP_in) == LOW) || (digitalRead(RP_in) == LOW) || (keyerControl & 0x03)) {
        update_PaddleLatch();
        keyerState = CHK_DIT;
        return true;
      }
      return false;
      
    case CHK_DIT:      // See if the dit paddle was pressed
      if (keyerControl & DIT_L) {
        keyerControl |= DIT_PROC;
        ktimer = _dotlen;
        keyerState = KEYED_PREP;
#ifdef ECHO_PRACTICE
        element=0;
#endif      
        return true;
      }  // fall through
        keyerState = CHK_DAH;
        
    case CHK_DAH:      // See if dah paddle was pressed
      if (keyerControl & DAH_L) {
        ktimer = _dashlen;
        keyerState = KEYED_PREP;
#ifdef ECHO_PRACTICE
        element=1;
#endif      
        return true;
      } else {
        keyerState = IDLE;
        return false;
      }
      
    case KEYED_PREP:                     // Assert key down, start timing
                                         // state shared for dit or dah
      if (CWstruc.ptt_enable)
        digitalWrite(ptt_pin_, HIGH);    // Enable PTT
      digitalWrite(cw_pin_, HIGH);       // Key the CW line
#ifdef ECHO_PRACTICE
      echo_timing(0);
      ch |= element*bit;
      bit*=2;
      //Serial.write("\tch="); Serial.print(ch);
#endif      
#ifdef SIDETONE      
      tone(ST_Pin, ST_Freq,ktimer);      // Turn the Sidetone on
#endif      
      ktimer += millis();                // set ktimer to interval end time
      keyerControl &= ~(DIT_L + DAH_L);  // clear both paddle latch bits
      keyerState = KEYED;                // next state
      return true;

    case KEYED:                          // Wait for timer to expire
      if (millis() > ktimer) {           // are we at end of key down ?
#ifdef ECHO_PRACTICE
        tup = millis();
#endif      
        digitalWrite(ptt_pin_, LOW);     // Disable PTT 
        digitalWrite(cw_pin_, LOW);      // Unkey the CW line
        ktimer = millis() + _space_len;  // inter-element time
        keyerState = INTER_ELEMENT;      // next state
        return true;
      }
      else if (key_mode == IAMBICB)     // Iambic B Mode ?
        update_PaddleLatch();           // yes, early paddle latch in Iambic B mode
      return true;

    case INTER_ELEMENT:                 // Insert time between dits/dahs
      update_PaddleLatch();             // latch paddle state
      if (millis() > ktimer) {          // are we at end of inter-space ?
        if (keyerControl & DIT_PROC) {  // was it a dit or dah ?
          keyerControl &= ~(DIT_L + DIT_PROC);   // clear two bits
          keyerState = CHK_DAH;                  // dit done, check for dah
          return true;
        } else {
          keyerControl &= ~(DAH_L);     // clear dah latch
          keyerState = IDLE;            // go idle
          return false;
        }
      }
      return true;

  }
}
