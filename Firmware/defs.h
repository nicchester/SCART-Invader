/* Pin definitions */

#define IR_REMOTE   8 
#define FP_BUTTON   10
#define STATUS_LED  13

#define SIG_DETECT  A0
#define SYNCMUX_A   A3 
#define SYNCMUX_B   A2
#define SYNCMUX_C   A1

#define _LO_SYNC_OE 2
#define _RL_OE      3
#define _HI_SYNC_OE 4
#define RCLK        5
#define SRCK        6
#define DATA        7

/* EEPROM Stuff */

// If the magic number 42 is not at this address, default states will be written to the EEPROM
#define EEPROM_MAGIC_ADDR 0x0
#define EEPROM_MAGIC 42 

// Base address of EEPROM aspect ratio/RGB/CVBS/SYNC configuration bytes
#define EEPROM_CHANNEL_MODES_BASE 0x01

/* Define remote control codes */

#define BUTTON_1 0x10 
#define BUTTON_2 0x810
#define BUTTON_3 0x410
#define BUTTON_4 0xC10
#define BUTTON_5 0x210 
#define BUTTON_6 0xA10
#define BUTTON_7 0x610 
#define BUTTON_8 0xE10 
#define BUTTON_9 0x110
#define BUTTON_0 0x910

#define BUTTON_RED      0x338 // use to toggle 16:9, 4:3 etc 
#define BUTTON_GREEN    0xB38 // use to toggle CVBS, RGB
#define BUTTON_YELLOW   0x738 // use to toggle CSYNC or CVBS 
#define BUTTON_BLUE     0xF38 // dunno 

#define CH_UP   0x90
#define CH_DOWN 0x890

/* Define bytes for relay driver shift register */
const byte rlBytes[] = 
{
  0b00000001, // No Selection
  0b00000000, // Ch1 
  0b00000011, // Ch2
  0b00000101, // Ch3
  0b00001001, // Ch4
  0b10000001, // Ch5
  0b01000001, // Ch6
  0b00100001, // Ch7
  0b00010001, // Ch8
}; 

/* Aspect ratio and RGB switching shift register bytes */
const byte ASP_WIDE   = 0b00000010; // Force 16:9 aspect ratio 
const byte ASP_NORMAL = 0b00000001; // Force 4:3 aspect ratio 
const byte STATUS_OFF = 0b00000000; // No output at all
const byte MODE_RGB   = 0b00001000; // Fast RGB switching enabled, forces TV to RGB video
const byte MODE_CVBS  = 0b00000000; // No fast RGB switching; may switch TV to composite video
const byte MODE_CSYNC = 0b10000000; // Switch CVBS via to LM1881 sync cleaner 

// Default states that will be written to EEPROM on first boot
const byte defaultChannelStates[] = 
{
  STATUS_OFF, 
  ASP_NORMAL | MODE_CSYNC | MODE_RGB, 
  ASP_NORMAL | MODE_CSYNC | MODE_RGB,
  ASP_NORMAL | MODE_CSYNC | MODE_RGB,
  ASP_NORMAL | MODE_CSYNC | MODE_RGB,
  ASP_NORMAL | MODE_CSYNC | MODE_RGB,
  ASP_NORMAL | MODE_CSYNC | MODE_RGB,
  ASP_NORMAL | MODE_CSYNC | MODE_RGB,
  ASP_NORMAL | MODE_CSYNC | MODE_RGB,
}; 

// Debounce period for select button 
const unsigned long DEBOUNCE_TIME = 100; 

// Timeout for an auto AV signal (milliseconds)
const unsigned long SIG_TIMEOUT = 3000; 

// Timeout after manual switching 
const unsigned long MANUAL_TIMEOUT = 5000; 