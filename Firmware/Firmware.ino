#include <EEPROM.h>
#include <IRremote.h> 
#include "defs.h"

// AVRDude Command I used to burn firmware with USBasp (for future reference)
// sudo avrdude -c usbasp -p m328p -U flash:w:Firmware.ino.with_bootloader.standard.hex:i 

// Represents the state of the first shift register (relay driver) 
byte output = 0x0; 

// Byte representing state of the second shift register (Status & RGB signal via a resistor ladder) 
byte statusMode = STATUS_OFF; 

// Time since the front panel button was pressed 
unsigned long lastManualSwitch = 0; 

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


// Last time the status LED blinked in 'search' mode 
unsigned long lastBlinkMillis = 0; 

// state of the status LED
unsigned long statusLedOn = true; 

// IR decoder stuff 
IRrecv irrecv(IR_REMOTE);
decode_results results;

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
    if(adcVal > 10)
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
  // Get the status byte out of the EEPROM 
  statusMode = EEPROM.read(EEPROM_CHANNEL_MODES_BASE + channel); 

  // Update the output byte 
  output = rlBytes[channel]; 

  // Write out the data 
  updateOutput(); 
}

// Check EEPROM for magic number indicating first run
void checkEEPROM()
{
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) 
  {

    for(int i = 0; i <= 8; i++)
    {
      EEPROM.write(EEPROM_CHANNEL_MODES_BASE + i, defaultChannelStates[i]); 
    }

    // Write magic number 
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  }
}

void setup() 
{
  Serial.begin(115200); 
  Serial.println("Play SCART Invaders! Happy Gaming!"); 

  checkEEPROM(); 
  
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
  unsigned long currentMillis; 
  
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

void stepChannel(bool up)
{
    unsigned long currentMillis = millis(); 
    if(currentMillis - lastManualSwitch >= DEBOUNCE_TIME)
    {
      delay(100); 

      // Switch out of auto mode and remove lock
      autoMode = false; 
      lock = false; 

      if(up)
      {
        // Increment the target channel 
        selectedChannel++; 

        // If we overflow the number of channels, 
        // go back to zero indicating no selection on the AV mux 
        if(selectedChannel > 8) selectedChannel = 0; 
      }
      else
      {
        if(selectedChannel == 0)
        {
          selectedChannel = 8; 
        }
        else
        {
          selectedChannel--; 
        }
      }

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

void changeChannelManual(byte channel)
{
    unsigned long currentMillis = millis(); 
    if(currentMillis - lastManualSwitch >= DEBOUNCE_TIME)
    {
      delay(100); 

      // Switch out of auto mode and remove lock
      autoMode = false; 
      lock = false; 

      selectedChannel = channel; 

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

// Cycle aspect ratio 
void changeAspectRatio()
{
  // Return if we're 
  if(selectedChannel == 0)
    return; 

  // if wide then switch to 4:3 
  if(statusMode & ASP_WIDE)
  {
    statusMode &= ~ASP_WIDE;
    statusMode |= ASP_NORMAL; 
  }
  // if 4:3 then switch to off
  else if(statusMode & ASP_NORMAL)
  {
    statusMode &= ~ASP_WIDE; 
    statusMode &= ~ASP_NORMAL; 
  }
  // if off then switch to wide 
  else
  {
    statusMode |= ASP_WIDE; 
  }

  updateOutput(); 

  // Update the EEPROM with the channel mode 
  EEPROM.update(EEPROM_CHANNEL_MODES_BASE + selectedChannel, statusMode); 
}

// Toggle RGB/CVBS
void changeRGB()
{
  if(selectedChannel == 0)
    return; 

    // switch between RGB and no RGB 
    statusMode ^= MODE_RGB;

    updateOutput(); 

    EEPROM.update(EEPROM_CHANNEL_MODES_BASE + selectedChannel, statusMode); 
}

// Toggle LM1881 sync stripper 
void changeCSYNC()
{
  if(selectedChannel == 0)
    return; 

  // Switch CSYNC on/off 
  statusMode ^= MODE_CSYNC;

  updateOutput(); 

  EEPROM.update(EEPROM_CHANNEL_MODES_BASE + selectedChannel, statusMode); 
}

void handleAutoMode()
{
  unsigned long currentMillis = millis();
  bool signalDetected = false;  

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

void handleIR()
{
  unsigned long currentMillis = millis(); 

  if (irrecv.decode(&results))
  {
    if(currentMillis - lastManualSwitch >= DEBOUNCE_TIME)
    {
      switch(results.value)
      {
        case BUTTON_1:
          changeChannelManual(1);
          break; 

        case BUTTON_2:
          changeChannelManual(2);
          break; 

        case BUTTON_3:
          changeChannelManual(3);
          break; 

        case BUTTON_4:
          changeChannelManual(4);
          break; 

        case BUTTON_5:
          changeChannelManual(5);
          break; 

        case BUTTON_6:
          changeChannelManual(6);
          break; 

        case BUTTON_7:
          changeChannelManual(7);
          break; 

        case BUTTON_8:
          changeChannelManual(8);
          break; 

        case BUTTON_0:
          changeChannelManual(0);
          break; 

        case CH_UP:
          stepChannel(true);
          break; 

        case CH_DOWN:
          stepChannel(false);
          break;

        case BUTTON_RED: 
          changeAspectRatio(); 
          break; 

        case BUTTON_GREEN: 
          changeRGB(); 
          break;

        case BUTTON_YELLOW:
          changeCSYNC(); 
          break; 

        default:
          Serial.print("Unknown IR command: 0x");
          Serial.println(results.value, HEX); 
          break; 
      }
    }

    lastManualSwitch = millis();
    irrecv.resume();
  }
}

void loop() 
{
  blinkStatus(); 
  
  if(autoMode)
  {
    handleAutoMode(); 
  }

  // If the front panel button is pressed then increment the current channel 
  if(!digitalRead(FP_BUTTON))
  {
    stepChannel(true); 
  }
  
}
