/*******************************************************************************
  CYCLOP++ with OSD brings manual channel selection and onscreen information to
  the HobbyKing Quanum Cyclops FPV googles.

  The rx5808-pro and rx5808-pro-diversity projects served as a starting
  point for the code base, even if little of the actual code remains.
  Without those projects CYCLOP++ would not have been created. All possible
  credit goes to the two mentioned projects and their contributors.

  The max7456 library by theboredengineers with a few changes by Dvogonen is
  used to control the video circuit.

  Copyright (c) 2017 Kjell Kernen (Dvogonen)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
********************************************************************************/
// Application includes
#include "cyclop_plus_plus.h"
#include "rtc6715.h"

// Library includes
#include <avr/pgmspace.h>
#include <string.h>
#include <EEPROM.h>
#include <EnableInterrupt.h>

/*******************************************************************************
  Minimosd Display Protocol Definition v1.0
   Formal definition for the display protocol: (Token[Command[Param]])*
   Tokens,  commands and params are all bytes.
   All tokens except CMD_CMD results in the corresponding character in the Max7456
   charmap being displayed onscreen  att current position using current params.
   The token CMD_CMD signals that the next byte is a command.
   Some commands have a single param.
   Invalid characters are ignored (e.g. unknown commands, params that are out of
   bounds, etc).
 *******************************************************************************
  Protocol Commands:
 *******************************************************************************/
#define CMD_CMD             255 /* The start command token.                    */
#define CMD_LOAD_CHARS      1   /* Load charset to EEPROM. Needed only once    */
#define CMD_CLEAR_SCREEN    2   /* Empties the screen                          */
#define CMD_ENABLE_OSD      3   /* Shows OSD on video out                      */
#define CMD_DISABLE_OSD     4   /* Hides OSD on video out                      */
#define CMD_ENABLE_VIDEO    5   /* Shows video in on video out                 */
#define CMD_DISABLE_VIDEO   6   /* Hides video in from video out               */
#define CMD_ENABLE_BLINK    7   /* All upcomming characters will blink         */
#define CMD_DISABLE_BLINK   8   /* All upcoming characters will be stable      */
#define CMD_ENABLE_INVERSE  9   /* All upcoming characters are inversed        */
#define CMD_DISABLE_INVERSE 10  /* All upcoming characters without inverse     */
#define CMD_ENABLE_FILL     11  /* Fill background of upcoming characters      */
#define CMD_DISABLE_FILL    12  /* No background fill for upcoming characters  */
#define CMD_NEWLINE         13  /* Moves cursor to start of the next line      */
#define CMD_SET_X           14  /* Position X cursor (next char is a parameter)*/
#define CMD_SET_Y           15  /* Position Y cursor (next char is a parameter)*/
#define CMD_SET_OFFSET      16  /* Display Offset XY cursor (next 2 chars are parameters)*/

//******************************************************************************
//* Character constants

#define OSD_BATTERY_100     0x01
#define OSD_BATTERY_75      0x02
#define OSD_BATTERY_50      0x03
#define OSD_BATTERY_25      0x04
#define OSD_BATTERY_0       0x05
#define OSD_METER_START     0x06
#define OSD_METER_100       0x07
#define OSD_METER_66        0x08
#define OSD_METER_33        0x09
#define OSD_METER_0         0x0A
#define OSD_METER_END       0x0B
#define OSD_ANTENNA         0x0C
#define OSD_MHZ             0x0D
#define OSD_DEGREE          0x0E
#define OSD_FILLED          0x0F
#define OSD_BAR_0010        0x10
#define OSD_BAR_1010        0x11
#define OSD_BAR_0001        0x12
#define OSD_BAR_0101        0x13
#define OSD_BAR_0011        0x14
#define OSD_BAR_1011        0x15
#define OSD_BAR_0111        0x16
#define OSD_BAR_1111        0x17
#define OSD_BAR_EMPTY       0x18
#define OSD_SPACE           0x20
#define OSD_LOGO            0x80

//******************************************************************************
//* File scope function declarations

unsigned int  autoScan( unsigned int frequency );
unsigned int  averageAnalogRead( unsigned char pin );
void          batteryMeter(unsigned char x, unsigned char y);
unsigned char bestChannelMatch( unsigned int frequency );
void          buttonPressInterrupt();
void          drawAutoScanScreen(void);
void          drawBattery(unsigned char xPos, unsigned char yPos, unsigned char value, bool showNumbers = false );
void          drawLeftInfoLine( void );
void          drawLogo( unsigned char xPos, unsigned char yPos);
void          drawOptionsScreen(unsigned char option, unsigned char in_edit_state );
void          drawRightInfoLine( void );
void          drawScannerScreen( void );
void          drawStartScreen(void);
unsigned char getClickType(unsigned char buttonPin);
unsigned int  getVoltage( void );
unsigned int  graphicScanner( unsigned int frequency );
char         *longNameOfChannel(unsigned char channel, char *name);
unsigned char nextChannel( unsigned char channel);
void          osd( unsigned char command );
void          osd( unsigned char command, unsigned char param );
void          osd( unsigned char command, unsigned char param1, unsigned char param2 );
void          osd_char( unsigned char token );
void          osd_int( unsigned int integer );
void          osd_string( const char *str );
unsigned char previousChannel( unsigned char channel);
bool          readEeprom(void);
void          resetOptions(void);
char         *shortNameOfChannel(unsigned char channel, char *name);
void          setOptions( void );
void          updateScannerScreen(unsigned char position, unsigned char value1, unsigned char value2 );
void          applyDisplayOffset(void);

//******************************************************************************
//* Positions in the frequency table for the 48 channels
//* Direct access via array operations does not work since data is stored in
//* flash, not in RAM. Use getPosition to retrieve data

const unsigned char positions[] PROGMEM = {
  40, 41, 42, 43, 44, 45, 46, 47, 19, 32, 18, 17, 33, 16,  7, 34,
  8, 24,  6,  9, 25,  5, 35, 10, 26,  4, 11, 27,  3, 36, 12, 28,
  2, 13, 29, 37,  1, 14, 30,  0, 15, 31, 38, 20, 21, 39, 22, 23
};

unsigned int getPosition( unsigned char channel ) {
  return pgm_read_byte_near(positions + channel);
}

//******************************************************************************
//* Frequencies for the 48 channels
//* Direct access via array operations does not work since data is stored in
//* flash, not in RAM. Use getFrequency to retrieve data

const unsigned int channelFrequencies[] PROGMEM = {
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A - Boscam A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B - Boscam B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E - DJI
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F - FatShark \ Immersion
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // Band R - Raceband
  5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621  // Band L - Lowband
};

unsigned int getFrequency( unsigned char channel ) {
  return pgm_read_word_near(channelFrequencies + getPosition(channel));
}
//******************************************************************************
//* Reverse lookup for frequencies in position table
//* Direct access via array operations does not work since data is stored in
//* flash, not in RAM. Use getRevesePositions to retrieve data

const unsigned int reversePositions[] PROGMEM = {
  39, 36, 32, 28, 25, 21, 18, 14, // Band A - Boscam A
  16, 19, 23, 26, 30, 33, 37, 40, // Band B - Boscam B
  13, 11, 10,  8, 43, 44, 46, 47, // Band E - DJI
  17, 20, 24, 27, 31, 34, 38, 41, // Band F - FatShark \ Immersion
  9,  12, 15, 22, 29, 35, 42, 45, // Band R - Raceband
  0,   1,  2,  3,  4,  5,  6,  7  // Band L - Lowband
};

unsigned int getReversePosition( unsigned char position ) {
  return pgm_read_word_near(reversePositions + position);
}

//******************************************************************************
//* Other file scope variables
unsigned char lastClick = NO_CLICK;
unsigned char clickType = NO_CLICK;
unsigned char currentChannel = 0;
unsigned char lastChannel = 0;
unsigned char ledState = LED_ON;
unsigned long pauseStart = 0;
unsigned long displayUpdateTimer = 0;
unsigned long eepromSaveTimer = 0;
unsigned long forceDisplayTimer = 0;
unsigned long pulseTimer = 0;
unsigned long alarmTimer = 0;
unsigned char alarmSoundOn = 0;
unsigned int  alarmOnPeriod = 0;
unsigned int  alarmOffPeriod = 0;
unsigned char options[MAX_OPTIONS];
unsigned char saveScreenActive = 0;
unsigned char screenCleaning = 0;
rtc6715 receiver( SPI_CLOCK_PIN, SLAVE_SELECT_PIN, SPI_DATA_PIN );
unsigned char softPositions[48];

//******************************************************************************
//* function: setup
//******************************************************************************
void setup()
{
  unsigned int i;

  // initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);

  // initialize button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  enableInterrupt(BUTTON_PIN, buttonPressInterrupt, CHANGE);

  // initialize alarm
  pinMode(ALARM_PIN, OUTPUT );

  // Read current channel and options data from EEPROM
  if (!readEeprom()) {
    currentChannel = CHANNEL_MIN;
    resetOptions();
  }
  // Start receiver
  receiver.setFrequency(getFrequency(currentChannel));

  // Initialize the display
  Serial.begin(57600);

  // Set Options
  if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED ) {
    setOptions();
    writeEeprom();
  }
  else { // Show Startscreen
    if (options[SHOW_STARTSCREEN_OPTION]) {
      long redrawTimer = 0;
      long stopTimer = millis() + 7000;
      while (( millis() < stopTimer ) && (digitalRead(BUTTON_PIN) != BUTTON_PRESSED )) {
        if ( millis() > redrawTimer) {
          redrawTimer = millis() + 500;
          drawStartScreen();
        }
      }
    }
  }
  osd( CMD_CLEAR_SCREEN );

  // Set delay time before entering screen save mode
  forceDisplayTimer = millis() + FORCED_SCREEN_UPDATE_MS;
}

//******************************************************************************
//* function: loop
//******************************************************************************
void loop()
{
  switch (lastClick = getClickType( BUTTON_PIN ))
  {
    case NO_CLICK: // do nothing
      break;

    case WAKEUP_CLICK: // do nothing
      break;

    case LONG_CLICK:
      switch (selectFunction())
      {
        case 1:
          osd( CMD_CLEAR_SCREEN );
          currentChannel = bestChannelMatch(graphicScanner(getFrequency(currentChannel)));
          break;
        case 2:
          osd( CMD_CLEAR_SCREEN );
          drawAutoScanScreen();
          currentChannel = bestChannelMatch(autoScan(getFrequency(currentChannel)));
          break;
        case 3:
          osd( CMD_CLEAR_SCREEN );
          setOptions();
          writeEeprom();
          break;
      }
      screenCleaning = 1;
      break;

    case SINGLE_CLICK: // up the frequency
      currentChannel = nextChannel( currentChannel );
      receiver.setFrequency(getFrequency(currentChannel));
      displayUpdateTimer = 0;
      screenCleaning = 1;
      break;

    case DOUBLE_CLICK:  // down the frequency
      currentChannel = previousChannel( currentChannel );
      receiver.setFrequency(getFrequency(currentChannel));
      displayUpdateTimer = 0;
      screenCleaning = 1;
      break;
  }
  // Reset screensaver timer after each key click
  if  (lastClick != NO_CLICK )
    forceDisplayTimer = millis() + FORCED_SCREEN_UPDATE_MS;

  if (screenCleaning)
  {
    screenCleaning = 0;
    osd(CMD_CLEAR_SCREEN);
    displayUpdateTimer = 0;
  }

  // Check if it is time for a display update
  if ((displayUpdateTimer < millis())) {
    displayUpdateTimer = millis() + 1000;
    if ( options[INFO_LINE_OPTION] || (forceDisplayTimer > millis()))
    {
      if (options[INFO_LINE_POS_OPTION])
        drawLeftInfoLine();
      else
        drawRightInfoLine();
    }
    else
    {
      osd(CMD_CLEAR_SCREEN);
      saveScreenActive = 1;
    }
  }

  // Check if EEPROM needs a save. Reduce EEPROM writes by not saving too often
  if ((currentChannel != lastChannel) && (millis() > eepromSaveTimer))
  {
    writeEeprom();
    lastChannel = currentChannel;
    eepromSaveTimer = millis() + 10000;
  }

  // Check if it is time to switch LED state for the pulse led
  if ( millis() > pulseTimer ) {
    ledState = !ledState;
    pulseTimer = millis() + 500;
  }
  digitalWrite(LED_PIN, ledState);

  // Toggle alarm on or off
  if (options[BATTERY_ALARM_OPTION] && alarmOnPeriod) {
    if (millis() > alarmTimer) {
      alarmSoundOn = !alarmSoundOn;
      if (alarmSoundOn) {
        analogWrite( ALARM_PIN, (1 << options[ALARM_LEVEL_OPTION]) - 1 );
        alarmTimer = millis() + alarmOnPeriod;
      }
      else {
        analogWrite( ALARM_PIN, 0 );
        alarmTimer = millis() + alarmOffPeriod;
      }
    }
  }
  else
    analogWrite( ALARM_PIN, 0 );
}

//******************************************************************************
//* function: writeEeprom
//*         : Writes configuration settings to nonvolatile memory
//******************************************************************************
void writeEeprom(void) {
  unsigned char i;
  EEPROM.write(EEPROM_CHANNEL, currentChannel);
  for (i = 0; i < MAX_OPTIONS; i++)
    EEPROM.write(EEPROM_OPTIONS + i, options[i]);
  EEPROM.write(EEPROM_CHECK, VER_EEPROM);
}

//******************************************************************************
//* function: readEeprom
//*         : Reads configuration settings from nonvolatile memory
//******************************************************************************
bool readEeprom(void) {
  unsigned char i;
  if (EEPROM.read(EEPROM_CHECK) != VER_EEPROM)
    return false;
  currentChannel =   EEPROM.read(EEPROM_CHANNEL);
  for (i = 0; i < MAX_OPTIONS; i++)
    options[i] = EEPROM.read(EEPROM_OPTIONS + i);
  updateSoftPositions();
  return true;
}

//******************************************************************************
//* function: buttonPressInterrupt
//******************************************************************************
void buttonPressInterrupt() {
  static long clickStart = 0;

  if ( digitalRead(BUTTON_PIN) == BUTTON_PRESSED ) {// Button was pressed
    clickStart = millis();
  }
  else {   // Button was released
    if ( pauseStart) {
      clickType = DOUBLE_CLICK;
      clickStart = 0;
      pauseStart = 0;
    }
    else
    {
      if ( saveScreenActive ) {
        clickType = WAKEUP_CLICK;
        clickStart = 0;
        saveScreenActive = 0;
      }
      else if (( millis() - clickStart) > 350 )
        clickType = LONG_CLICK;
      else
        clickType = SINGLE_CLICK;
      clickStart = 0;
      pauseStart = millis();
    }
  }
}

//******************************************************************************
//* function: getClickType
//******************************************************************************
uint8_t getClickType(uint8_t buttonPin) {
  uint8_t tempClickType = NO_CLICK;

  if (pauseStart && (millis() - pauseStart) < 350)
    return NO_CLICK;

  pauseStart = 0;

  if ( clickType ) {
    tempClickType = clickType;
    clickType = NO_CLICK;
  }
  return tempClickType;
}

//******************************************************************************
//* function: nextChannel
//******************************************************************************
unsigned char incrementChannel(unsigned char channel)
{
  if (channel > (CHANNEL_MAX - 1))
    return CHANNEL_MIN;
  else
    return channel + 1;
}

unsigned char nextChannel(unsigned char channel)
{
  do {
    channel = incrementChannel( channel);
  } while (softPositions[channel] == 255);
  return channel;
}

//******************************************************************************
//* function: previousChannel
//******************************************************************************
unsigned char decrementChannel(unsigned char channel)
{
  if (channel > CHANNEL_MAX)
    return CHANNEL_MAX;

  if ( channel == CHANNEL_MIN )
    return CHANNEL_MAX;

  return channel - 1;
}

unsigned char previousChannel(unsigned char channel)
{
  do {
    channel = decrementChannel( channel );
  } while (softPositions[channel] == 255);
  return channel;
}

//******************************************************************************
//* function: bestChannelMatch
//*         : finds the best matching standard channel for a given frequency
//******************************************************************************
unsigned char bestChannelMatch( unsigned int frequency )
{
  int16_t comp;
  int16_t bestComp = 300;
  unsigned char bestChannel = CHANNEL_MIN;
  unsigned char i;

  for (i = CHANNEL_MIN; i <= CHANNEL_MAX; i++) {
    comp = abs( (int16_t)getFrequency(i) - (int16_t)frequency );
    if (comp < bestComp)
    {
      bestComp = comp;
      bestChannel = i;
    }
  }
  return bestChannel;
}

//******************************************************************************
//* function: graphicScanner
//*         : scans the 5.8 GHz band and draws a graphical representation.
//*         : when the button is pressed the current frequency is returned.
//******************************************************************************
unsigned int graphicScanner( unsigned int frequency ) {
  unsigned char i;
  unsigned int scanRssi;
  unsigned int bestRssi = 0;
  unsigned int scanFrequency = frequency;
  unsigned int bestFrequency = frequency;
  unsigned char clickType;
  unsigned char rssiDisplayValue1;
  unsigned char rssiDisplayValue2;

  // Draw screen frame etc
  drawScannerScreen();

  // Disable video
  osd(CMD_DISABLE_VIDEO);

  // Cycle through the band
  while (digitalRead(BUTTON_PIN) != BUTTON_PRESSED) {
    for (i = 0; i < 2; i++) {
      scanFrequency += SCANNING_STEP;
      if (scanFrequency > FREQUENCY_MAX)
        scanFrequency = FREQUENCY_MIN;
      receiver.setFrequency(scanFrequency);
      delay( RSSI_STABILITY_DELAY_MS );
      scanRssi = averageAnalogRead(RSSI_PIN);
      if (!i)
        rssiDisplayValue1 = (scanRssi - 140) / 20;    // Roughly 1 - 23
      else
        rssiDisplayValue2 = (scanRssi - 140) / 20;    // Roughly 1 - 23
    }
    updateScannerScreen(29 - ((FREQUENCY_MAX - scanFrequency) / FREQUENCY_DIVIDER), rssiDisplayValue1, rssiDisplayValue2 );
  }
  // Fine tuning
  scanFrequency = scanFrequency - SCANNING_STEP*4;
  for (i = 0; i < SCANNING_STEP*4; i++, scanFrequency += 2) {
    receiver.setFrequency(scanFrequency);
    delay( RSSI_STABILITY_DELAY_MS );
    scanRssi = averageAnalogRead(RSSI_PIN);
    if (bestRssi < scanRssi) {
      bestRssi = scanRssi;
      bestFrequency = scanFrequency;
    }
  }
  // Enable Video
  osd(CMD_ENABLE_VIDEO);

  // Return the best frequency
  receiver.setFrequency(bestFrequency);
  return (bestFrequency);
}

//******************************************************************************
//* function: autoScan
//******************************************************************************
unsigned int autoScan( unsigned int frequency ) {
  unsigned char i;
  unsigned int scanRssi = 0;
  unsigned int bestRssi = 0;
  unsigned int scanFrequency;
  unsigned int bestFrequency;

  // Disable video
  osd(CMD_DISABLE_VIDEO);

  // Skip 10 MHz forward to avoid detecting the current channel
  scanFrequency = frequency + 10;
  if (!(scanFrequency % 2))
    scanFrequency++;        // RTC6715 can only generate odd frequencies

  // Coarse tuning
  bestFrequency = scanFrequency;
  for (i = 0; i < 60 && (scanRssi < RSSI_TRESHOLD); i++) {
    if ( scanFrequency <= (FREQUENCY_MAX - SCANNING_STEP))
      scanFrequency += SCANNING_STEP;
    else
      scanFrequency = FREQUENCY_MIN;
    receiver.setFrequency(scanFrequency);
    delay( RSSI_STABILITY_DELAY_MS );
    scanRssi = averageAnalogRead(RSSI_PIN);
    if (bestRssi < scanRssi) {
      bestRssi = scanRssi;
      bestFrequency = scanFrequency;
    }
  }
  // Fine tuning
  scanFrequency = bestFrequency - SCANNING_STEP*4;
  bestRssi = 0;
  for (i = 0; i < SCANNING_STEP*4; i++, scanFrequency += 2) {
    receiver.setFrequency(scanFrequency);
    delay( RSSI_STABILITY_DELAY_MS );
    scanRssi = averageAnalogRead(RSSI_PIN);
    if (bestRssi < scanRssi) {
      bestRssi = scanRssi;
      bestFrequency = scanFrequency;
    }
  }
  // Enable Video
  osd(CMD_ENABLE_VIDEO);

  // Return the best frequency
  receiver.setFrequency(bestFrequency);
  return (bestFrequency);
}
//******************************************************************************
//* function: averageAnalogRead
//*         : used to read from an anlog pin
//*         : returns an averaged value between (in theory) 0 and 1024
//*         : this function is called often, so it is speed optimized
//******************************************************************************
unsigned int averageAnalogRead( unsigned char pin)
{
  unsigned int rssi = 0;
  unsigned char i = 32;

  for ( ; i ; i--) {
    rssi += analogRead(pin);
  }
  return (rssi >> 5);
}

//******************************************************************************
//* function: shortNameOfChannel
//******************************************************************************
char *shortNameOfChannel(unsigned char channel, char *name)
{
  unsigned char channelIndex = getPosition(channel);
  if (channelIndex < 8)
    name[0] = 'a';
  else if (channelIndex < 16)
    name[0] = 'b';
  else if (channelIndex < 24)
    name[0] = 'e';
  else if (channelIndex < 32)
    name[0] = 'f';
  else if (channelIndex < 40)
    name[0] = 'c';
  else
    name[0] = 'l';
  name[1] = (channelIndex % 8) + '0' + 1;
  name[2] = 0;
  return name;
}

//******************************************************************************
//* function: longNameOfChannel
//******************************************************************************
char *longNameOfChannel(unsigned char channel, char *name)
{
  unsigned char len;
  unsigned char channelIndex = getPosition(channel);
  if (channelIndex < 8)
    strcpy(name, "Boscam A");
  else if (channelIndex < 16)
    strcpy(name, "Boscam B");
  else if (channelIndex < 24)
    strcpy(name, "Foxtech/DJI ");
  else if (channelIndex < 32)
    strcpy(name, "FatShark ");
  else if (channelIndex < 40)
    strcpy(name, "RaceBand ");
  else
    strcpy(name, "LowBand  ");
  len = strlen( name );
  name[len] = (channelIndex % 8) + '0' + 1;
  name[len + 1] = 0;
  return name;
}

//******************************************************************************
//* function: getVoltage
//*         : returns battery voltage as an unsigned integer.
//*         : The value is multiplied with 10, 12volts => 120, 7.2Volts => 72
//*
//*         : Measured voltage values:
//*         : 12.6v = 639   10.8v = 546   8.4v = 411   7.2v = 359
//*         : The result is not linear...
//*         : A rough estimation is that 5 steps correspond to 0.1 volts
//******************************************************************************
unsigned int getVoltage( void )
{
  return ( 50 + (((averageAnalogRead(VOLTAGE_METER_PIN) - 250 + (options[BATTERY_CALIB_OPTION] - 128))) / 5 ));
}

//******************************************************************************
//* function: batteryMeter
//******************************************************************************
void batteryMeter( unsigned char x, unsigned char y )
{
  unsigned int voltage;
  unsigned char value;
  unsigned int minV;
  unsigned int maxV;

  if (options[BATTERY_TYPE_OPTION])
  { /* 2s lipo battery*/
    minV = 68;
    maxV = 84;
  }
  else
  { /* 3s lipo battery */
    minV = 102;
    maxV = 126;
  }
  voltage = getVoltage();
  if (voltage >= maxV)
    value = 99;
  else if (voltage <= minV)
    value = 0;
  else
    value = (unsigned char)((voltage - minV) / (float)(maxV - minV) * 100.0);

  // Set alarm periods
  if (value < 5)
  {
    alarmOnPeriod = ALARM_MAX_ON;
    alarmOffPeriod = ALARM_MAX_OFF;
  }
  else if (value < 15)
  {
    alarmOnPeriod = ALARM_MED_ON;
    alarmOffPeriod = ALARM_MED_OFF;
  }
  else if (value < 25)
  {
    alarmOnPeriod = ALARM_MIN_ON;
    alarmOffPeriod = ALARM_MIN_OFF;
  }
  else
  {
    alarmOnPeriod = 0;
    alarmOffPeriod = 0;
  }
  drawBattery(x, y, value, options[BATTERY_TEXT_OPTION]);
}

//******************************************************************************
//* function: updateSoftPositions
//******************************************************************************
void updateSoftPositions( void ) {
  unsigned char i;

  // Mark blocked channels in the softPositions array
  for (i = 0; i < 48; i++)
    softPositions[i] = positions[i];

  if (!options[A_BAND_OPTION]) {
    for (i = 0; i < 8; i++)
      softPositions[getReversePosition( i )] = 255;
  }
  if (!options[B_BAND_OPTION]) {
    for (i = 8; i < 16; i++)
      softPositions[getReversePosition( i )] = 255;
  }
  if (!options[E_BAND_OPTION]) {
    for (i = 16; i < 24; i++)
      softPositions[getReversePosition( i )] = 255;
  }
  if (!options[F_BAND_OPTION]) {
    for (i = 24; i < 32; i++)
      softPositions[getReversePosition( i )] = 255;
  }
  if (!options[R_BAND_OPTION]) {
    for (i = 32; i < 40; i++)
      softPositions[getReversePosition( i )] = 255;
  }
  if (!options[L_BAND_OPTION]) {
    for (i = 40; i < 48; i++)
      softPositions[getReversePosition( i )] = 255;
  }
}

//******************************************************************************
//* function: resetOptions
//*         : Resets all configuration settings to their default values
//******************************************************************************
void resetOptions(void) {
  options[BATTERY_ALARM_OPTION]    = BATTERY_ALARM_DEFAULT;
  options[ALARM_LEVEL_OPTION]      = ALARM_LEVEL_DEFAULT;
  options[BATTERY_TYPE_OPTION]     = BATTERY_TYPE_DEFAULT;
  options[BATTERY_CALIB_OPTION]    = BATTERY_CALIB_DEFAULT;
  options[SHOW_STARTSCREEN_OPTION] = SHOW_STARTSCREEN_DEFAULT;
  options[INFO_LINE_OPTION]        = INFO_LINE_DEFAULT;
  options[INFO_LINE_POS_OPTION]    = INFO_LINE_POS_DEFAULT;
  options[A_BAND_OPTION]           = A_BAND_DEFAULT;
  options[B_BAND_OPTION]           = B_BAND_DEFAULT;
  options[E_BAND_OPTION]           = E_BAND_DEFAULT;
  options[F_BAND_OPTION]           = F_BAND_DEFAULT;
  options[R_BAND_OPTION]           = R_BAND_DEFAULT;
  options[L_BAND_OPTION]           = L_BAND_DEFAULT;
  options[BATTERY_TEXT_OPTION]     = BATTERY_TEXT_DEFAULT;
  options[OFFSET_X_OPTION]         = OFFSET_X_DEFAULT;
  options[OFFSET_Y_OPTION]         = OFFSET_Y_DEFAULT;

  updateSoftPositions();
}

//******************************************************************************
//* function: setOptions
//******************************************************************************
void setOptions()
{
  unsigned char exitNow = false;
  unsigned char menuSelection = 0;
  unsigned char click = NO_CLICK;
  unsigned char in_edit_state = 0;
  long redrawTimer = 0;

  // Display option screen
  drawOptionsScreen( menuSelection, in_edit_state );

  // Let the user release the button
  getClickType( BUTTON_PIN );

  while ( !exitNow )
  {
    click = getClickType( BUTTON_PIN );

    if (in_edit_state)
      switch ( click )
      {
        case NO_CLICK:        // do nothing
          break;

        case SINGLE_CLICK:    // increase option
          if (menuSelection == ALARM_LEVEL_OPTION)
          {
            if (options[ALARM_LEVEL_OPTION] < 8 )
              options[ALARM_LEVEL_OPTION]++;
          }
          else if (menuSelection == BATTERY_CALIB_OPTION)
          {
            if (options[BATTERY_CALIB_OPTION] < 250)
              options[BATTERY_CALIB_OPTION] += 5;
          }
          else if (menuSelection == OFFSET_X_OPTION)
          {
            if (options[OFFSET_X_OPTION] <= 63)
              options[OFFSET_X_OPTION]++;
          }
          else if (menuSelection == OFFSET_Y_OPTION)
          {
            if (options[OFFSET_Y_OPTION] <= 31)
              options[OFFSET_Y_OPTION]++;
          }
          else
            options[menuSelection] = !options[menuSelection];
          break;

        case DOUBLE_CLICK:    // decrease option
          if (menuSelection == ALARM_LEVEL_OPTION)
          {
            if (options[ALARM_LEVEL_OPTION] > 1)
              options[ALARM_LEVEL_OPTION]--;
          }
          else if (menuSelection == BATTERY_CALIB_OPTION)
          {
            if (options[BATTERY_CALIB_OPTION] > 5)
              options[BATTERY_CALIB_OPTION] -= 5;
          }
          else if (menuSelection == OFFSET_X_OPTION)
          {
            if (options[OFFSET_X_OPTION] >= 0)
              options[OFFSET_X_OPTION]--;
          }
          else if (menuSelection == OFFSET_Y_OPTION)
          {
            if (options[OFFSET_Y_OPTION] >= 0)
              options[OFFSET_Y_OPTION]--;
          }
          else
            options[menuSelection] = !options[menuSelection];
          break;

        case LONG_CLICK:     // stop editing
          in_edit_state = 0;
      }
    else
      switch ( click )
      {
        case NO_CLICK:        // do nothing
          break;

        case SINGLE_CLICK:    // Move to next option
          menuSelection++;
          if (menuSelection >= MAX_OPTIONS + MAX_COMMANDS)
            menuSelection = 0;
          break;

        case DOUBLE_CLICK:    // Move to previous option
          if (menuSelection == 0)
            menuSelection = MAX_OPTIONS + MAX_COMMANDS - 1;
          else
            menuSelection--;
          break;

        case LONG_CLICK:     // Execute command or toggle option
          if (menuSelection == EXIT_COMMAND)
            exitNow = true;
          else if (menuSelection == RESET_SETTINGS_COMMAND)
            resetOptions();
          else if (menuSelection == TEST_ALARM_COMMAND) {
            testAlarm();
          }
          else if (menuSelection == SET_OFFSET_COMMAND) {
            applyDisplayOffset();
          }
          else
            in_edit_state = 1;
          break;
      }
    if (( click != NO_CLICK ) || ( millis() > redrawTimer )) {
      redrawTimer = millis() + 1000;
      drawOptionsScreen( menuSelection, in_edit_state );
    }
  }
  updateSoftPositions();
}

void applyDisplayOffset(void) {
  unsigned char offsetX = options[OFFSET_X_OPTION];
  unsigned char offsetY = options[OFFSET_Y_OPTION];
  osd( CMD_SET_OFFSET, offsetX, offsetX );
}

//******************************************************************************
//* function: testAlarm
//*         : Cycles through alarms, regardless of alarm settings
//******************************************************************************
void testAlarm( void ) {
  unsigned char i;

  while (getClickType(BUTTON_PIN) == NO_CLICK) {
    for (i = 0; i < 3; i++) {
      analogWrite( ALARM_PIN, (1 << options[ALARM_LEVEL_OPTION]) - 1 );
      delay(ALARM_MAX_ON);
      analogWrite( ALARM_PIN, 0 );
      delay(ALARM_MAX_OFF);
    }
  }
}

//******************************************************************************
//* function: selectFunction
//******************************************************************************
uint8_t selectFunction(void)
{
  uint8_t function = 0;
  uint8_t lastClick = NO_CLICK;
  do
  {
    drawFunctionScreen( function );
    lastClick = getClickType( BUTTON_PIN );
    if (lastClick == SINGLE_CLICK)
      function == 3 ? function = 0 : function++;
    if (lastClick == DOUBLE_CLICK)
      function == 0 ? function = 3 : function--;
  }
  while ( lastClick != LONG_CLICK );
  return ( function );
}

//******************************************************************************
//* function: osd
//******************************************************************************
void osd( unsigned char command )
{
  while (Serial.availableForWrite() < 2) ;
  Serial.write( CMD_CMD );
  Serial.write( command );
}

//******************************************************************************
//* function: osd
//******************************************************************************
void osd( unsigned char command, unsigned char param )
{
  while (Serial.availableForWrite() < 3) ;
  Serial.write( CMD_CMD );
  Serial.write( command );
  Serial.write( param );
}

//******************************************************************************
//* function: osd
//******************************************************************************
void osd( unsigned char command, unsigned char param1, unsigned char param2 )
{
  while (Serial.availableForWrite() < 4) ;
  Serial.write( CMD_CMD );
  Serial.write( command );
  Serial.write( param1 );
  Serial.write( param2 );
}

//******************************************************************************
//* function: osd_char
//******************************************************************************
void osd_char( unsigned char token )
{
  while (Serial.availableForWrite() == 0) ;
  Serial.write( token );
}

//******************************************************************************
//* function: osd_int
//******************************************************************************
void osd_int( unsigned int integer )
{
  char buff[17];
  itoa(integer, buff, 10);
  while (Serial.availableForWrite() < strlen(buff)) ;
  Serial.write(buff);
}

//******************************************************************************
//* function: osd_string
//******************************************************************************
void osd_string( const char *str )
{
  while (Serial.availableForWrite() < strlen(str)) ;
  Serial.write(str);
}



//******************************************************************************
//* function: drawLogo
//*         : displays the program Logo
//******************************************************************************
void drawLogo( unsigned char xPos, unsigned char yPos) {
  // Set Position
  osd( CMD_SET_X, xPos );
  osd( CMD_SET_Y, yPos );

  // Display Logo
  osd(CMD_ENABLE_INVERSE);
  osd_char( OSD_LOGO );
  osd_char( OSD_LOGO + 1 );
  osd_char( OSD_LOGO + 2 );
  osd_char( OSD_LOGO + 3 );
  osd_char( OSD_LOGO + 4 );
  osd_char( OSD_LOGO + 5 );
  osd_char( OSD_LOGO + 6 );
  osd_char( OSD_LOGO + 7 );
  osd( CMD_SET_X, xPos );
  osd( CMD_SET_Y, yPos + 1 );
  osd_char( OSD_LOGO + 8 );
  osd_char( OSD_LOGO + 9 );
  osd_char( OSD_LOGO + 10 );
  osd_char( OSD_LOGO + 11);
  osd_char( OSD_LOGO + 12);
  osd_char( OSD_LOGO + 13 );
  osd_char( OSD_LOGO + 14 );
  osd_char( OSD_LOGO + 15 );
  osd(CMD_DISABLE_INVERSE);
}

//******************************************************************************
//* function: drawStartScreen
//*         : displays the boot screen
//******************************************************************************
void drawStartScreen( void ) {
  unsigned char i;

  // Display Logo
  drawLogo( 1, 0 );

  // Display version date
  osd( CMD_SET_X, 10 );
  osd( CMD_SET_Y, 0 );
  osd_string(VER_DATE_STRING);

  // Display version information
  osd( CMD_SET_X, 10 );
  osd( CMD_SET_Y, 1 );
  osd_string(VER_INFO_STRING);

  // Display battery status
  batteryMeter( 25, 0 );
}

//******************************************************************************
//* function: drawFunctionScreen
//******************************************************************************
#define XPOS  5
#define YPOS  4
uint8_t drawFunctionScreen( uint8_t function )
{
  drawLogo(1, 0);
  batteryMeter(25, 0);

  osd(CMD_SET_X, XPOS);
  osd(CMD_SET_Y, YPOS);
  function == 0 ? osd(CMD_ENABLE_INVERSE) : osd(CMD_DISABLE_INVERSE);
  function == 0 ? osd(CMD_ENABLE_FILL) : osd(CMD_DISABLE_FILL);
  osd_string(" Exit            ");

  osd(CMD_SET_X, XPOS);
  osd(CMD_SET_Y, YPOS + 1);
  function == 1 ? osd(CMD_ENABLE_INVERSE) : osd(CMD_DISABLE_INVERSE);
  function == 1 ? osd(CMD_ENABLE_FILL) : osd(CMD_DISABLE_FILL);
  osd_string(" Graphic Scanner ");

  osd(CMD_SET_X, XPOS);
  osd(CMD_SET_Y, YPOS + 2);
  function == 2 ? osd(CMD_ENABLE_INVERSE) : osd(CMD_DISABLE_INVERSE);
  function == 2 ? osd(CMD_ENABLE_FILL) : osd(CMD_DISABLE_FILL);
  osd_string(" Auto Scanner    ");

  osd(CMD_SET_X, XPOS);
  osd(CMD_SET_Y, YPOS + 3);
  function == 3 ? osd(CMD_ENABLE_INVERSE) : osd(CMD_DISABLE_INVERSE);
  function == 3 ? osd(CMD_ENABLE_FILL) : osd(CMD_DISABLE_FILL);
  osd_string(" Options         ");
}

//******************************************************************************
//* function: drawAutoScanScreen
//******************************************************************************
void drawAutoScanScreen( void ) {
  char buffer[22];
  unsigned char answer;

  drawLogo(1, 0);
  batteryMeter(25, 0);

  osd( CMD_ENABLE_FILL );
  osd( CMD_ENABLE_INVERSE );

  osd( CMD_SET_X, 1 );
  osd( CMD_SET_Y, 5 );
  osd_string("      Auto Scanning      ");

  osd( CMD_DISABLE_INVERSE );
  osd( CMD_DISABLE_FILL );
}

//******************************************************************************
//* function: drawScannerScreen
//******************************************************************************
void drawScannerScreen( void ) {
  osd(CMD_SET_X, 0);
  osd(CMD_SET_Y, 11);
  for (int i = 0; i < 30 ; i++) {
    osd_char(OSD_BAR_EMPTY);
  }
  osd(CMD_SET_X, 0);
  osd(CMD_SET_Y, 12);
  if ( options[L_BAND_OPTION] )
    osd_string(" 5.35       5.60       5.95");
  else
    osd_string(" 5.65       5.80       5.95");
}

//******************************************************************************
//* function: updateScannerScreen
//*         : position = 0 to 29
//*         : value1 = 0 to 24
//*         : value2 = 0 to 24
//*         : must be fast since there are frequent updates
//******************************************************************************
void updateScannerScreen(unsigned char position, unsigned char value1, unsigned char value2 ) {
  unsigned char i;
  static unsigned char last_position = 0;
  static unsigned char last_value1 = 0;
  static unsigned char last_value2 = 0;
  bool barCells[4];

  for (i = 0; i < 12; i++)
  {
    // Errase the scan line character from last call
    osd(CMD_SET_X, last_position);
    osd(CMD_SET_Y, 11 - i);

    barCells[0] = (last_value1 >= ((i * 2) + 2));
    barCells[1] = (last_value2 >= ((i * 2) + 2));
    barCells[2] = (last_value1 >= ((i * 2) + 1));
    barCells[3] = (last_value2 >= ((i * 2) + 1));

    if (barCells[0] && barCells[1] && barCells[2] && barCells[3])
      osd_char(OSD_BAR_1111);
    else if ((!barCells[0]) && barCells[1] && barCells[2] && barCells[3])
      osd_char(OSD_BAR_0111);
    else if (barCells[0] && (!barCells[1]) && barCells[2] && barCells[3])
      osd_char(OSD_BAR_1011);
    else if ((!barCells[0]) && (!barCells[1]) && barCells[2] && barCells[3])
      osd_char(OSD_BAR_0011);
    else if ((!barCells[0]) && (!barCells[1]) && (!barCells[2]) && barCells[3])
      osd_char(OSD_BAR_0001);
    else if ((!barCells[0]) && (!barCells[1]) && barCells[2] && (!barCells[3]))
      osd_char(OSD_BAR_0010);
    else if (barCells[0] && !barCells[1] && barCells[2] && !barCells[3])
      osd_char(OSD_BAR_1010);
    else if (!barCells[0] && barCells[1] && !barCells[2] && barCells[3])
      osd_char(OSD_BAR_0101);
    else
      osd_char(i == 0 ? OSD_BAR_EMPTY : ' ');

    // Draw the current scan line character
    osd(CMD_SET_X, position);
    osd(CMD_SET_Y, 11 - i);
    osd_char(i == 0 ? OSD_FILLED : ' ');
  }
  // Save position and values for the next pass
  last_position = position;
  last_value1 = value1;
  last_value2 = value2;
  if (value1 > 24 )
    value1 = 24;
  if (value2 > 24 )
    value2 = 24;
}

//******************************************************************************
//* function: drawBattery
//*         : value = 0 to 100
//******************************************************************************
void drawBattery(unsigned char xPos, unsigned char yPos, unsigned char value, bool showNumbers ) {

  osd(CMD_SET_X, xPos);
  osd(CMD_SET_Y, yPos);
  if (value > 87)
    osd_char(OSD_BATTERY_100);
  else if (value > 62)
    osd_char(OSD_BATTERY_75);
  else if (value > 37)
    osd_char(OSD_BATTERY_50);
  else if (value > 12)
    osd_char(OSD_BATTERY_25);
  else
    osd_char(OSD_BATTERY_0);

  if (showNumbers) {
    osd_string(" ");
    osd_int(value);
    osd_string("%");
  }
}

//******************************************************************************
//* function: drawOptionsScreen
//******************************************************************************
void drawOptionsScreen(unsigned char option, unsigned char in_edit_state ) {
  unsigned char i, j;
  unsigned int voltage = getVoltage();

  drawStartScreen();
  if (option != 0)
    j = option - 1;
  else
    j = MAX_OPTIONS + MAX_COMMANDS - 1;

  for (i = 0; i < MAX_OPTION_LINES; i++, j++)
  {
    osd( CMD_SET_Y, i + 3 );
    osd( CMD_SET_X, 0 );
    if (j >= (MAX_OPTIONS + MAX_COMMANDS))
      j = 0;

    if ((j == option) && !in_edit_state) {
      osd( CMD_ENABLE_FILL );
      osd( CMD_ENABLE_INVERSE );
    }
    else
    {
      osd( CMD_DISABLE_FILL );
      osd( CMD_DISABLE_INVERSE );
    }
    switch (j) {
      case BATTERY_ALARM_OPTION:     osd_string("battery alarm      "); break;
      case ALARM_LEVEL_OPTION:       osd_string("alarm sound level  "); break;
      case BATTERY_TYPE_OPTION:      osd_string("battery type       "); break;
      case BATTERY_CALIB_OPTION:     osd_string("volt calibration   "); break;
      case SHOW_STARTSCREEN_OPTION:  osd_string("show start screen  "); break;
      case INFO_LINE_OPTION:         osd_string("constant info line "); break;
      case INFO_LINE_POS_OPTION:     osd_string("info line position "); break;
      case A_BAND_OPTION:            osd_string("boscam a band      "); break;
      case B_BAND_OPTION:            osd_string("boscam b band      "); break;
      case E_BAND_OPTION:            osd_string("foxtech/dji band   "); break;
      case F_BAND_OPTION:            osd_string("fatshark band      "); break;
      case R_BAND_OPTION:            osd_string("race band          "); break;
      case L_BAND_OPTION:            osd_string("low band           "); break;
      case BATTERY_TEXT_OPTION:      osd_string("show bat percentage"); break;
      case OFFSET_X_OPTION:          osd_string("disp offset X      "); break;
      case OFFSET_Y_OPTION:          osd_string("disp offset Y      "); break;
      case RESET_SETTINGS_COMMAND:   osd_string("reset settings     "); break;
      case TEST_ALARM_COMMAND:       osd_string("test alarm         "); break;
      case SET_OFFSET_COMMAND:       osd_string("apply disp offset  "); break;
      case EXIT_COMMAND:             osd_string("exit               "); break;
    }
    if ((j == option) && in_edit_state) {
      osd( CMD_ENABLE_FILL );
      osd( CMD_ENABLE_INVERSE );
    }
    else
    {
      osd( CMD_DISABLE_FILL );
      osd( CMD_DISABLE_INVERSE );
    }
    if (j < MAX_OPTIONS) {
      switch (j) {
        case BATTERY_ALARM_OPTION:    osd_string(options[j] ? "yes    " : "no     "); break;
        case ALARM_LEVEL_OPTION:      osd_int(options[j]);      osd_string("      "); break;
        case BATTERY_TYPE_OPTION:     osd_string(options[j] ? "2s lipo" : "3s lipo"); break;
        case BATTERY_CALIB_OPTION:    osd_int(voltage / 10); osd_string("."); osd_int(voltage % 10);  osd_string("   "); break;
        case SHOW_STARTSCREEN_OPTION: osd_string(options[j] ? "yes    " : "no     "); break;
        case INFO_LINE_OPTION:        osd_string(options[j] ? "yes    " : "no     "); break;
        case INFO_LINE_POS_OPTION:    osd_string(options[j] ? "left   " : "right  "); break;
        case A_BAND_OPTION:           osd_string(options[j] ? "on     " : "off    "); break;
        case B_BAND_OPTION:           osd_string(options[j] ? "on     " : "off    "); break;
        case E_BAND_OPTION:           osd_string(options[j] ? "on     " : "off    "); break;
        case F_BAND_OPTION:           osd_string(options[j] ? "on     " : "off    "); break;
        case R_BAND_OPTION:           osd_string(options[j] ? "on     " : "off    "); break;
        case L_BAND_OPTION:           osd_string(options[j] ? "on     " : "off    "); break;
        case BATTERY_TEXT_OPTION:     osd_string(options[j] ? "on     " : "off    "); break;
        case OFFSET_X_OPTION:         osd_int(options[j]);      osd_string("px    "); break;
        case OFFSET_Y_OPTION:         osd_int(options[j]);      osd_string("px    "); break;
      }
    }
    else
      osd_string("       ");
  }
  // Make sure that the inverse is disabled even if option was on last line
  osd( CMD_DISABLE_INVERSE );
}

//******************************************************************************
//* function: drawLeftInfoLine
//******************************************************************************
void drawLeftInfoLine( void )
{
  char buffer[3];
  batteryMeter(1, 0);
  osd( CMD_SET_X, 3 );
  osd( CMD_SET_Y, 0);
  osd_string(shortNameOfChannel(currentChannel, buffer));
  osd_char(OSD_SPACE);
  osd_char(OSD_MHZ);
  osd_int(getFrequency(currentChannel));
  osd_char(OSD_SPACE);
  osd_char(OSD_ANTENNA);
  osd_int(averageAnalogRead(RSSI_PIN));
}

//******************************************************************************
//* function: drawRightInfoLine
//******************************************************************************
void drawRightInfoLine( void )
{
  char buffer[3];

  osd( CMD_SET_X, 12 );
  osd( CMD_SET_Y, 0);
  osd_string(shortNameOfChannel(currentChannel, buffer));
  osd_char(OSD_SPACE);
  osd_char(OSD_MHZ);
  osd_int(getFrequency(currentChannel));
  osd_char(OSD_SPACE);
  osd_char(OSD_ANTENNA);
  osd_int(averageAnalogRead(RSSI_PIN));
  batteryMeter(25, 0);
}
