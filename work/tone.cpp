///////////////////////////////////////////////////////////////////////
//
// tone(pin,frequency[,duration]) generate a tone on a given pin
//
// noTone(pin)                    switch of the tone on the pin
//
///////////////////////////////////////////////////////////////////////

//#include "Arduino.h"
//#include <HardwareTimer.h>

#ifndef TONE_TIMER
  #define TONE_TIMER 2
#endif

HardwareTimer tone_timer(TONE_TIMER);

bool tone_state = true;             // last pin state for toggling
short tone_pin = -1;                // pin for outputting sound
short tone_freq = 444;              // tone frequency (0=pause)
unsigned tone_micros = 500000/444;  // tone have wave time in usec
int tone_counts = 0;                // tone duration in units of half waves

// timer handler for tone with no duration specified, 
// will keep going until noTone() is called
void tone_handler_1(void) {
  tone_state = !tone_state;
  digitalWrite(tone_pin,tone_state);
}

// timer handler for tone with a specified duration,
// will stop automatically when duration time is up.
void tone_handler_2(void) {   // check duration
  if(tone_freq > 0) {
   tone_state = !tone_state;
   digitalWrite(tone_pin,tone_state);
  }
  if(!--tone_counts){
   tone_timer.pause();
   pinMode(tone_pin, INPUT);
  }
}

//  play a tone on given pin with given frequency and optional duration in msec
void tone(uint8_t pin, unsigned short freq, unsigned duration = 0) {
  tone_pin = pin;
  tone_freq = freq;
  tone_micros = 500000/(freq>0?freq:1000);
  tone_counts = 0;

  tone_timer.pause();

  if(freq >= 0){
    if(duration > 0)tone_counts = ((long)duration)*1000/tone_micros;
    pinMode(tone_pin, OUTPUT);

    // set timer to half period in microseconds
    tone_timer.setPeriod(tone_micros);

    // Set up an interrupt on channel 1
    tone_timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    tone_timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
    tone_timer.attachCompare1Interrupt(tone_counts?tone_handler_2:tone_handler_1);

    // Refresh the tone timer
    tone_timer.refresh();

    // Start the timer counting
    tone_timer.resume();
  } else {
    pinMode(tone_pin, INPUT);
  }
}

// disable tone on specified pin, if any
void noTone(uint8_t pin){
  tone(pin,-1);
}

