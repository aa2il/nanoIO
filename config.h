/***********************************************************************

  nanoIO

  CW / FSK interface sketch for use either stand alone or with fldigi.

  It is designed to fit within the RAM and I/O constraints of an Arduino 
  Nano circuit board.  It generates either CW 5 to 100 WPM Morse code, 
  or FSK Baudot hardline signals of 45.45, 50 or 75 Baud.

  Copyright (C) 2018 David Freese W1HKJ
  Derived from tinyFSK by Andrew T. Flowers K0SM

  nanoIO is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with tinyCWFSK.ino.  If not, see <http://www.gnu.org/licenses/>.

  Revisions:

  //1.0.0:  Initial release
  //1.0.1:  MORTTY Version 3 board support
  //1.1.0:  added support for WPM potentiometer; wiper on A0
  //1.3.0:  added additional FSK echo of control strings
  //1.3.1:  enabled keyline inputs
  //1.4.0:  MORTTY Version 4 board support
  //1.4.1:  Modified usec corr usage

***********************************************************************/
#ifndef _CONFIG_H_
#define _CONFIG_H_

#define VERSION "1.4.1"

// Configure for target h/w by removing the comment on the appropriate line
// ONLY one of the following:

//#define MORTTY_V2
//#define MORTTY_V3
//#define MORTTY_V4
//#define HA2OS
//#define W1HKJ
//#define N1MM
#if defined(ESP32)
#define AA2IL_NODE_MCU
#else
#define AA2IL
#endif

// Configure for target h/w by removing the comment if your Morrty has a 
// speed pot:

// #define WITH_SPEED_POT

//----------------------------------------------------------------------
// values which may need to be changed depending on specific hardware
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//Arduino pins for FSK, CW and PTT to control transmitter
// uncomment the desired h/w configuration
// comment the others

#ifdef MORTTY_V2
//----------------------------------------------------------------------
// Configuration for MORTTY Version 2 circuit board
//----------------------------------------------------------------------
#  define FSK_PIN 11
#  define CW_PIN  11
#  define PTT_PIN 13
#  define ST_Pin 4      // Sidetone Output Pin on Pin 4
// paddle input pins compatible with MORTTY board
#  define LP_in 2       // Left Paddle Input on Pin 2
#  define RP_in 5       // Right Paddle Input on Pin 5
#  define DEFAULT_MODE CW_MODE
#endif

#ifdef MORTTY_V3
//----------------------------------------------------------------------
// Configuration for MORTTY Version 3 circuit board
//----------------------------------------------------------------------
#  define FSK_PIN 11
#  define CW_PIN  12
#  define PTT_PIN 13
#  define ST_Pin 4      // Sidetone Output Pin on Pin 4
// paddle input pins compatible with MORTTY board
#  define RP_in 2       // Right Paddle Input on Pin 2
#  define LP_in 5       // Left Paddle Input on Pin 5
#  define DEFAULT_MODE CW_MODE
#  define WPM_POT 0
#endif

#ifdef MORTTY_V4
//----------------------------------------------------------------------
// Configuration for MORTTY Version 4 circuit board
//----------------------------------------------------------------------
#  define FSK_PIN 12
#  define CW_PIN  11
#  define PTT_PIN 13
#  define ST_Pin 4      // Sidetone Output Pin on Pin 4
// paddle input pins compatible with MORTTY board
#  define RP_in 2       // Right Paddle Input on Pin 2
#  define LP_in 5       // Left Paddle Input on Pin 5
#  define DEFAULT_MODE CW_MODE
#  define WPM_POT 0
#endif

#ifdef HA2OS
//----------------------------------------------------------------------
// Configuration for HA2OS quad opto-isolator circuit board design
//----------------------------------------------------------------------
#    define FSK_PIN 12
#    define CW_PIN  10
#    define PTT_PIN 11
#    define ST_Pin 4      // Sidetone Output Pin on Pin 4
//   paddle input pins
#    define LP_in 5       // Left Paddle Input on Pin 2
#    define RP_in 2       // Right Paddle Input on Pin 5
#    define DEFAULT_MODE CW_MODE
#endif

#ifdef W1HKJ
//----------------------------------------------------------------------
// Configuration for W1HKJ quad opto-isolator circuit board design
//----------------------------------------------------------------------
#    define FSK_PIN 12
#    define CW_PIN  11
#    define PTT_PIN 10
#    define ST_Pin 4      // Sidetone Output Pin on Pin 4
//   paddle input pins
#    define LP_in 2       // Left Paddle Input on Pin 2
#    define RP_in 5       // Right Paddle Input on Pin 5
#    define DEFAULT_MODE CW_MODE
#endif

#ifdef AA2IL
//----------------------------------------------------------------------
// Configuration for AA2IL - Of course I have to be different!
//----------------------------------------------------------------------
#    define FSK_PIN 12
#    define CW_PIN  13    // CW keying pin to rig same as red LED
#    define PTT_PIN 10

#    define SK_in 9       // Straight Key Input Pin 
#    define LP_in 5       // Left Paddle Input Pin 
#    define RP_in 3       // Right Paddle Input Pin
#    define DEFAULT_MODE CW_MODE

#    define SIDETONE 1    // Enable sidetone osc - eventually need to turn this into a run time param
#    define ST_Pin 7      // Sidetone Output Pin
#    define ST_Freq 650   // Set the Sidetone Frequency to 600 Hz
// #    define BUZZER_Pin 11      // Buzzer Output Pin

#    undef WITH_SPEED_POT
#    define DEFAULT_WPM 20

#    define ECHO_PRACTICE

//#    define LOCK_SPEEDS              // Lock speed of keyer and paddles
#endif

#ifdef AA2IL_NODE_MCU
//----------------------------------------------------------------------
// Configuration for ESP32 NODE MCU - In Progress
//----------------------------------------------------------------------
#    define FSK_PIN 17
#    define CW_PIN  2      // CW keying pin to rig - same as blue LED
#    define PTT_PIN 19

#    define SK_in 23       // Straight Key Input Pin 
#    define LP_in 25       // Left Paddle Input Pin 
#    define RP_in 27       // Right Paddle Input Pin
#    define DEFAULT_MODE CW_MODE

#    define SIDETONE 1    // Enable sidetone osc - eventually need to turn this into a run time param
#    define ST_Pin 33      // Sidetone Output Pin
#    define ST_Freq 700   // Set the Sidetone Frequency to 600 Hz

#    undef WITH_SPEED_POT
#    define DEFAULT_WPM 20

#    define ECHO_PRACTICE

//#    define LOCK_SPEEDS              // Lock speed of keyer and paddles
#endif

#endif // __CONFIG_H_
