#include <EEPROM.h>
#include <IRremote.h> 

/*** Pin definitions ***/

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

/* Define bytes for relay driver shift register */
const byte rlBytes[] = 
{
0b00000001, 
0b00000000,
0b00000011, 
0b00000101, 
0b00001001, 
0b10000001, 
0b01000001, 
0b00100001, 
0b00010001, 


}; 

// Represents the state of the first shift register (relay driver) 
byte output = 0x0; 

/* Aspect ratio and RGB switching shift register bytes */

const byte ASP_WIDE   = 0b00000010; // Force 16:9 aspect ratio 
const byte ASP_NORMAL = 0b00000001; // Force 4:3 aspect ratio 
const byte STATUS_OFF = 0b00000000; // No output at all
const byte MODE_RGB   = 0b00001000; // Fast RGB switching enabled, forces TV to RGB video
const byte MODE_CVBS  = 0b00000000; // No fast RGB switching; may switch TV to composite video
const byte MODE_CSYNC = 0b10000000; // Switch CVBS via to LM1881 sync cleaner 

const byte DEFAULT_STATE = ASP_NORMAL | MODE_CSYNC | MODE_RGB; 

const byte USE_RGB  = ASP_NORMAL | MODE_CSYNC | MODE_RGB; 
const byte USE_CVBS = ASP_NORMAL | MODE_CVBS; 

const byte channelStates[] = 
{
  STATUS_OFF, 
  USE_RGB, 
  USE_RGB, 
  USE_RGB, 
  USE_RGB, 

  USE_CVBS,
  USE_CVBS,
  USE_CVBS,
  USE_CVBS  
}; 

// Byte representing state of the second shift register (Status & RGB signal via a resistor ladder) 
byte statusMode = STATUS_OFF; 

// Time since the front panel button was pressed 
unsigned long lastManualSwitch = 0; 

// Time to ignore the button presses after a press is initially detected 
const unsigned long DEBOUNCE_TIME = 100; 

// Channel selected on the solid state muxes 
byte muxChannel = 0; 

// Channel selected on the AV multiplexer 
byte selectedChannel = 0; 

// Whether an input is locked on 
bool lock = false; 

// Whether the device is in 'auto' mode 
bool autoMode = true;

// Last time a 'valid' voltage was seen on the current auto-detected channel 
unsigned long lastSignalDetect; 

// Timeout for an auto AV signal (milliseconds)
const unsigned long SIG_TIMEOUT = 3000; 

// Timeout after manual switching 
const unsigned long MANUAL_TIMEOUT = 5000; 

unsigned long lastBlinkMillis = 0; 
unsigned long statusLedOn = true; 
unsigned long currentMillis;
bool signalDetected = false; 

/* Selects input on sync muxes */
void syncSelect(byte channel)
{  
  digitalWrite(SYNCMUX_A,     channel & 0b0001); 
  digitalWrite(SYNCMUX_B,     channel & 0b0010); 
  digitalWrite(SYNCMUX_C,     channel & 0b0100); 
  digitalWrite(_HI_SYNC_OE, !(channel & 0b1000)); 
  digitalWrite(_LO_SYNC_OE,   channel & 0b1000);
}

// Detect an incoming analog signal (muxes must be set first)
bool detectSignal()
{
  // 40 millisecond detection period 
  unsigned long endtime = millis() + 40; 
  bool result = false; 
  int adcVal = 0; 

  while(millis() < endtime)
  {
    adcVal = analogRead(SIG_DETECT);
    if(adcVal > 5)
      result = true; 
  }

  return result; 
}


// Shifts out data to relay driver and aspect ratio/RGB switching registers 
void updateOutput()
{
  digitalWrite(RCLK, LOW);
  shiftOut(DATA, SRCK, MSBFIRST, statusMode); 
  shiftOut(DATA, SRCK, MSBFIRST, output); 
  digitalWrite(RCLK, HIGH); 
}

// Flip the output enable 
void enableOutput(bool enable)
{
  digitalWrite(_RL_OE, !enable); 
}

void selectChannel(byte channel)
{
  statusMode = channelStates[channel]; 
  output = rlBytes[channel]; 

  updateOutput(); 
}

void setup() 
{
  Serial.begin(115200); 
  
  /* Set mode of all I/O pins */
  pinMode(RCLK, OUTPUT); 
  pinMode(SRCK, OUTPUT); 
  pinMode(DATA, OUTPUT); 
  pinMode(_RL_OE, OUTPUT);

  pinMode(_LO_SYNC_OE, OUTPUT); 
  pinMode(_HI_SYNC_OE, OUTPUT); 

  pinMode(IR_REMOTE, INPUT); 
  pinMode(FP_BUTTON, INPUT_PULLUP); 
  pinMode(STATUS_LED, OUTPUT); 

  pinMode(SIG_DETECT, INPUT); 
  pinMode(SYNCMUX_A, OUTPUT); 
  pinMode(SYNCMUX_B, OUTPUT); 
  pinMode(SYNCMUX_C, OUTPUT); 

  /* Set output enable/inhibit pins initially high */
  digitalWrite(_RL_OE, HIGH); 
  digitalWrite(_LO_SYNC_OE, HIGH); 
  digitalWrite(_HI_SYNC_OE, HIGH); 

  output = rlBytes[0]; 
  statusMode = STATUS_OFF; 
  updateOutput(); 
  enableOutput(true); 

  lastManualSwitch = millis(); 

  digitalWrite(STATUS_LED, LOW); 

}

// Blink the status led when not locked to an input, or steady when locked 
void blinkStatus()
{
  // If the device is set to an input then show a steady status light 
  if(!autoMode || lock)
  {
    digitalWrite(STATUS_LED, HIGH); 
  }
  // Otherwise it blinks 
  else
  {
    currentMillis = millis();

    if (currentMillis - lastBlinkMillis >= 1000) 
    {
      lastBlinkMillis = currentMillis;
  
      statusLedOn = !statusLedOn;
      digitalWrite(STATUS_LED, statusLedOn);
    }
    
  }
}

void handleFPButton()
{
    currentMillis = millis(); 
    if(currentMillis - lastManualSwitch >= DEBOUNCE_TIME)
    {
      delay(100); 

      // Switch out of auto mode and remove lock
      autoMode = false; 
      lock = false; 

      // Increment the target channel 
      selectedChannel++; 

      // If we overflow the number of channels, go back to zero indicating no selection on the AV mux 
      if(selectedChannel > 8) selectedChannel = 0; 

      // If channel is zero then set output off and enable 'auto' mode, 
      // else update and enable the output 
      if(selectedChannel == 0) 
      {
        autoMode = true; 
      }

      selectChannel(selectedChannel); 

      delay(100); 
      lastManualSwitch = millis(); 
    }
}

void handleAutoMode()
{
  currentMillis = millis(); 

  // Return if we're not outside of the grace period after user manually switching
  if(currentMillis - lastManualSwitch <= MANUAL_TIMEOUT)
  {
    return; 
  }
  
  // Select channel using the solid state muxes
  syncSelect(muxChannel); 

  // If the device is currently auto-locked to an input, 
  // check if the signal has disappeared after a given timeout 
  if(lock)
  {
    currentMillis = millis(); 
    // Check if a signal is still detected, if so store the time it was seen
    signalDetected = detectSignal(); 
    if(signalDetected) lastSignalDetect = millis(); 
    
    // If we no longer detect a signal, and the grace period is satisfied, then remove the lock and disable output 
    if(!signalDetected && currentMillis - lastSignalDetect >= SIG_TIMEOUT)
    {      
      lock = false; 
      delay(100); 
      selectedChannel = 0; 
      selectChannel(selectedChannel); 
    }
  }
  else
  {
    // Else, check for a valid signal on the current input of the sync mux. 
    currentMillis = millis(); 
    if(detectSignal())
    {
      lastSignalDetect = millis(); 
      lock = true; 
      selectedChannel = (muxChannel / 2) + 1; 
      selectChannel(selectedChannel); 
      delay(100); 
    }
    else
    {
      muxChannel++; 

      if(muxChannel > 15) muxChannel = 0; 
    }
  } 
}

void loop() 
{
  blinkStatus(); 
  
  if(autoMode)
  {
    handleAutoMode(); 
  }

  // If the front panel button is pressed
  if(!digitalRead(FP_BUTTON))
  {
    handleFPButton(); 
  }
  
}
