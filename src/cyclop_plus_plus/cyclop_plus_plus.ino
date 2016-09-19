/*******************************************************************************
  CYCLOP+ with OSD brings manual channel selection and onscreen information to
  the HobbyKing Quanum Cyclops FPV googles.

  The rx5808-pro and rx5808-pro-diversity projects served as a starting
  point for the code base, even if little of the actual code remains.
  Without those projects CYCLOP+ would not have been created. All possible
  credit goes to the two mentioned projects and their contributors.

  The OSD_Max7456 library from the night-ghost minimosd-extra project was used
  as a starting point for the MAX7456 library

  Copyright (c) 2016 Kjell Kernen (Dvogonen)

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

// Library includes
#include <avr/pgmspace.h>
#include <string.h>
#include <EEPROM.h>

/*******************************************************************************
  Minimosd Display Protocol Definition v1.0
   Formal definition for the display protocol: (Token[Command[Param]])*
   Tokens,  commands and params are all bytes.
   All tokens except 0 results in the corresponding character in the Max7456
   charmap being displayed onscreen  att current position using current paramss.
   The token 0 signals that the next byte is a command.
   Some commands have a single param.
   Invalid characters are ignored (e.g. unknown commands, params that are out of
   bounds, etc).
 *******************************************************************************
  Protocol Commands:
 *******************************************************************************/
#define CMD_CMD             0   /* The start command token.                    */
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
void          batteryMeter(unsigned char x, unsigned char y, bool showNumbers = false);
unsigned char bestChannelMatch( unsigned int frequency );
void          dissolveDisplay(void);
void          drawAutoScanScreen(void);
void          drawBattery(unsigned char xPos, unsigned char yPos, unsigned char value, bool showNumbers = false );
void          drawBottomInfoLine( void );
void          drawChannelScreen( unsigned char channel, unsigned int rssi);
void          drawOptionsScreen(unsigned char option );
void          drawScannerScreen( void );
void          drawStartScreen(void);
void          drawTopInfoLine( void );
unsigned char getClickType(unsigned char buttonPin);
unsigned int  graphicScanner( unsigned int frequency );
char         *longNameOfChannel(unsigned char channel, char *name);
unsigned char nextChannel( unsigned char channel);
void          osd( unsigned char command );
void          osd( unsigned char command, unsigned char param );
void          osd_char( unsigned char token );
void          osd_int( unsigned int integer );
void          osd_string( char *str );
unsigned char previousChannel( unsigned char channel);
bool          readEeprom(void);
void          resetOptions(void);
char         *shortNameOfChannel(unsigned char channel, char *name);
void          setRTC6715Frequency(unsigned int frequency);
void          setOptions( void );
void          testAlarm( void );
void          updateScannerScreen(unsigned char position, unsigned char value );
void          writeEeprom(void);

//******************************************************************************
//* Positions in the frequency table for the 40 channels
//* Direct access via array operations does not work since data is stored in
//* flash, not in RAM. Use getPosition to retrieve data

const unsigned char positions[] PROGMEM = {
  19, 18, 32, 17, 33, 16,  7, 34,  8, 24,  6,  9, 25,  5, 35, 10, 26,  4, 11, 27,
  3, 36, 12, 28,  2, 13, 29, 37,  1, 14, 30,  0, 15, 31, 38, 20, 21, 39, 22, 23
};

unsigned int getPosition( unsigned char channel ) {
  return pgm_read_byte_near(positions + channel);
}

//******************************************************************************
//* Frequencies for the 40 channels
//* Direct access via array operations does not work since data is stored in
//* flash, not in RAM. Use getFrequency to retrieve data

const unsigned int channelFrequencies[] PROGMEM = {
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A - Boscam A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B - Boscam B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E - DJI
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F - FatShark \ Immersion
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Band C - Raceband
};

unsigned int getFrequency( unsigned char channel ) {
  return pgm_read_word_near(channelFrequencies + getPosition(channel));
}

//******************************************************************************
//* Other file scope variables
unsigned char lastClick = NO_CLICK;
unsigned char currentChannel = 0;
unsigned char lastChannel = 0;
unsigned int  currentRssi = 0;
unsigned char ledState = LED_ON;
unsigned long saveScreenTimer;
unsigned long displayUpdateTimer = 0;
unsigned long eepromSaveTimer = 0;
unsigned long pulseTimer = 0;
unsigned long alarmTimer = 0;
unsigned char alarmSoundOn = 0;
unsigned int  alarmOnPeriod = 0;
unsigned int  alarmOffPeriod = 0;
unsigned char options[MAX_OPTIONS];
unsigned char saveScreenActive = 0;

//******************************************************************************
//* function: setup
//******************************************************************************
void setup()
{
  // initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);

  // initialize button pin
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, INPUT_PULLUP);

  // initialize alarm
  pinMode(ALARM_PIN, OUTPUT );

  // SPI pins for RX control
  pinMode (SLAVE_SELECT_PIN, OUTPUT);
  pinMode (SPI_DATA_PIN, OUTPUT);
  pinMode (SPI_CLOCK_PIN, OUTPUT);

  // Read current channel and options data from EEPROM
  if (!readEeprom()) {
    currentChannel = CHANNEL_MIN;
    resetOptions();
  }
  
  // Start receiver
  setRTC6715Frequency(getFrequency(currentChannel));

  // Initialize the display
  Serial.begin(115200);
  osd( CMD_CMD, CMD_CLEAR_SCREEN );

  // Set Options
  if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED ) {
    setOptions();
    writeEeprom();
  }
  // Show start screen
  if (options[SHOW_STARTSCREEN_OPTION])
    drawStartScreen();

  // Wait at least the delay time before entering screen save mode
  saveScreenTimer = millis() + SAVE_SCREEN_DELAY_MS;

  return;
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

    case LONG_LONG_CLICK: // graphical band scanner
      currentChannel = bestChannelMatch(graphicScanner(getFrequency(currentChannel)));
      drawChannelScreen(currentChannel, 0);
      displayUpdateTimer = millis() +  RSSI_STABILITY_DELAY_MS ;
      break;

    case LONG_CLICK:      // auto search
      drawAutoScanScreen();
      currentChannel = bestChannelMatch(autoScan(getFrequency(currentChannel)));
      drawChannelScreen(currentChannel, 0);
      displayUpdateTimer = millis() +  RSSI_STABILITY_DELAY_MS ;
      break;

    case SINGLE_CLICK: // up the frequency
      currentChannel = nextChannel( currentChannel );
      setRTC6715Frequency(getFrequency(currentChannel));
      drawChannelScreen(currentChannel, 0);
      break;

    case DOUBLE_CLICK:  // down the frequency
      currentChannel = previousChannel( currentChannel );
      setRTC6715Frequency(getFrequency(currentChannel));
      drawChannelScreen(currentChannel, 0);
      break;
  }
  // Reset screensaver timer after each key click
  if  (lastClick != NO_CLICK )
    saveScreenTimer = millis() + SAVE_SCREEN_DELAY_MS;

  // Check if the display needs updating
  if ( millis() > displayUpdateTimer ) {
    if ( saveScreenTimer < millis())
    {
      if ( options[TOP_INFO_LINE_OPTION])
        drawTopInfoLine();
      if ( options[BOTTOM_INFO_LINE_OPTION])
        drawBottomInfoLine();
      if (!options[TOP_INFO_LINE_OPTION] && ! options[BOTTOM_INFO_LINE_OPTION])
        osd(CMD_CLEAR_SCREEN);
    }
    else
    {
      currentRssi = averageAnalogRead(RSSI_PIN);
      drawChannelScreen(currentChannel, currentRssi);
      displayUpdateTimer = millis() + 1000;
    }
  }
  // Check if EEPROM needs a save. Reduce EEPROM writes by not saving to often
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
        analogWrite( ALARM_PIN, 32 );
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
//* function: resetOptions
//*         : All options are reset to their default values
//******************************************************************************
void resetOptions(void) {
  options[LIPO_2S_METER_OPTION]    = LIPO_2S_METER_DEFAULT;
  options[LIPO_3S_METER_OPTION]    = LIPO_3S_METER_DEFAULT;
  options[BATTERY_ALARM_OPTION]    = BATTERY_ALARM_DEFAULT;
  options[SHOW_STARTSCREEN_OPTION] = SHOW_STARTSCREEN_DEFAULT;
  options[TOP_INFO_LINE_OPTION]    = TOP_INFO_LINE_DEFAULT;
  options[BOTTOM_INFO_LINE_OPTION] = BOTTOM_INFO_LINE_DEFAULT;
}

//******************************************************************************
//* function: writeEeprom
//******************************************************************************
void writeEeprom(void) {
  unsigned char i;
  EEPROM.write(EEPROM_CHANNEL, currentChannel);
  for (i = 0; i < MAX_OPTIONS; i++)
    EEPROM.write(EEPROM_OPTIONS + i, options[i]);
  EEPROM.write(EEPROM_CHECK, 238);
}

//******************************************************************************
//* function: readEeprom
//******************************************************************************
bool readEeprom(void) {
  unsigned char i;
  if (EEPROM.read(EEPROM_CHECK) != 238)
    return false;
  currentChannel =   EEPROM.read(EEPROM_CHANNEL);
  for (i = 0; i < MAX_OPTIONS; i++)
    options[i] = EEPROM.read(EEPROM_OPTIONS + i);
  return true;
}

//******************************************************************************
//* function: get_click_type
//*         : Polls the specified pin and returns the type of click that was
//*         : performed NO_CLICK, SINGLE_CLICK, DOUBLE_CLICK, LONG_CLICK
//*         : or LONG_LONG_CLICK
//******************************************************************************
unsigned char getClickType(unsigned char buttonPin) {
  unsigned int timer = 0;
  unsigned char click_type = NO_CLICK;

  // check if the key has been pressed
  if (digitalRead(buttonPin) == !BUTTON_PRESSED)
    return ( NO_CLICK );

  // If the screen saver is active the key press is just a wakeup call
  if (saveScreenActive) {
    saveScreenActive = 0;
    while (digitalRead(buttonPin) == BUTTON_PRESSED) ;
    return ( WAKEUP_CLICK );
  }

  while (digitalRead(buttonPin) == BUTTON_PRESSED) {
    timer++;
    delay(5);
  }
  if (timer < 120)                  // 120 * 5 ms = 0.6s
    click_type = SINGLE_CLICK;
  if (timer >= 80 && timer < 300 )  // 300 * 5 ms = 1.5s
    click_type = LONG_CLICK;
  if (timer >= 300)
    click_type = LONG_LONG_CLICK;

  // Check if there is a second click
  timer = 0;
  while ((digitalRead(buttonPin) == !BUTTON_PRESSED) && (timer++ < 40)) {
    delay(5);
  }
  if (timer >= 40)                  // 40 * 5 ms = 0.2s
    return click_type;

  if (digitalRead(buttonPin) == BUTTON_PRESSED ) {
    click_type = DOUBLE_CLICK;
    while (digitalRead(buttonPin) == BUTTON_PRESSED) ;
  }
  return (click_type);
}

//******************************************************************************
//* function: nextChannel
//******************************************************************************
unsigned char nextChannel(unsigned char channel)
{
  if (channel > (CHANNEL_MAX - 1))
    return CHANNEL_MIN;
  else
    return channel + 1;
}

//******************************************************************************
//* function: previousChannel
//******************************************************************************
unsigned char previousChannel(unsigned char channel)
{
  if ((channel > CHANNEL_MAX) || (channel == CHANNEL_MIN))
    return CHANNEL_MAX;
  else
    return channel - 1;
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
//*         : scans the 5.8 GHz band in 5 MHz increments and draws a graphical
//*         : representation. when the button is pressed the currently
//*         : scanned frequency is returned.
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

  // Cycle through the band in 5MHz steps
  while ((clickType = getClickType(BUTTON_PIN)) == NO_CLICK) {
    for (i = 0; i < 2; i++) {
      scanFrequency += 5;
      if (scanFrequency > FREQUENCY_MAX)
        scanFrequency = FREQUENCY_MIN;
      setRTC6715Frequency(scanFrequency);
      delay( RSSI_STABILITY_DELAY_MS );
      scanRssi = averageAnalogRead(RSSI_PIN);
      if (!i)
        rssiDisplayValue1 = (scanRssi - 140) / 20;    // Roughly 1 - 23
      else
        rssiDisplayValue2 = (scanRssi - 140) / 20;    // Roughly 1 - 23
    }
    updateScannerScreen(100 - ((FREQUENCY_MAX - scanFrequency) / 10), rssiDisplayValue1, rssiDisplayValue2 );
  }
  // Fine tuning
  scanFrequency = scanFrequency - 20;
  for (i = 0; i < 20; i++, scanFrequency += 2) {
    setRTC6715Frequency(scanFrequency);
    delay( RSSI_STABILITY_DELAY_MS );
    scanRssi = averageAnalogRead(RSSI_PIN);
    if (bestRssi < scanRssi) {
      bestRssi = scanRssi;
      bestFrequency = scanFrequency;
    }
  }
  // Return the best frequency
  setRTC6715Frequency(bestFrequency);
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

  // Skip 10 MHz forward to avoid detecting the current channel
  scanFrequency = frequency + 10;
  if (!(scanFrequency % 2))
    scanFrequency++;        // RTC6715 can only generate odd frequencies

  // Coarse tuning
  bestFrequency = scanFrequency;
  for (i = 0; i < 60 && (scanRssi < RSSI_TRESHOLD); i++) {
    if ( scanFrequency <= (FREQUENCY_MAX - 5))
      scanFrequency += 5;
    else
      scanFrequency = FREQUENCY_MIN;
    setRTC6715Frequency(scanFrequency);
    delay( RSSI_STABILITY_DELAY_MS );
    scanRssi = averageAnalogRead(RSSI_PIN);
    if (bestRssi < scanRssi) {
      bestRssi = scanRssi;
      bestFrequency = scanFrequency;
    }
  }
  // Fine tuning
  scanFrequency = bestFrequency - 20;
  bestRssi = 0;
  for (i = 0; i < 20; i++, scanFrequency += 2) {
    setRTC6715Frequency(scanFrequency);
    delay( RSSI_STABILITY_DELAY_MS );
    scanRssi = averageAnalogRead(RSSI_PIN);
    if (bestRssi < scanRssi) {
      bestRssi = scanRssi;
      bestFrequency = scanFrequency;
    }
  }
  // Return the best frequency
  setRTC6715Frequency(bestFrequency);
  return (bestFrequency);
}
//******************************************************************************
//* function: averageAnalogRead
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
    name[0] = 'A';
  else if (channelIndex < 16)
    name[0] = 'B';
  else if (channelIndex < 24)
    name[0] = 'E';
  else if (channelIndex < 32)
    name[0] = 'F';
  else
    name[0] = 'R';
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
  else
    strcpy(name, "Raceband ");
  len = strlen( name );
  name[len] = (channelIndex % 8) + '0' + 1;
  name[len + 1] = 0;
  return name;
}

//******************************************************************************
//* function: calcFrequencyData
//*         : calculates the frequency value for the syntheziser register B of
//*         : the RTC6751 circuit that is used within the RX5808/RX5880 modules.
//*         : this value is inteded to be loaded to register at adress 1 via SPI
//*         :
//*  Formula: frequency = ( N*32 + A )*2 + 479
//******************************************************************************
unsigned int calcFrequencyData( unsigned int frequency )
{
  unsigned int N;
  unsigned char A;
  frequency = (frequency - 479) / 2;
  N = frequency / 32;
  A = frequency % 32;
  return (N << 7) |  A;
}

//******************************************************************************
//* function: setRTC6715Frequency
//*         : for a given frequency the register setting for synth register B of
//*         : the RTC6715 circuit is calculated and bitbanged via the SPI bus
//*         : please note that the synth register A is assumed to have default
//*         : values.
//*
//* SPI data: 4  bits  Register Address  LSB first
//*         : 1  bit   Read or Write     0=Read 1=Write
//*         : 13 bits  N-Register Data   LSB first
//*         : 7  bits  A-Register        LSB first
//******************************************************************************
void setRTC6715Frequency(unsigned int frequency)
{
  unsigned int sRegB;
  unsigned char i;

  sRegB = calcFrequencyData(frequency);

  // Bit bang the syntheziser register

  // Clock
  spiEnableHigh();
  delayMicroseconds(1);
  spiEnableLow();

  // Address (0x1)
  spi_1();
  spi_0();
  spi_0();
  spi_0();

  // Read/Write (Write)
  spi_1();

  // Data (16 LSB bits)
  for (i = 16; i; i--, sRegB >>= 1 ) {
    (sRegB & 0x1) ? spi_1() : spi_0();
  }
  // Data zero padding
  spi_0();
  spi_0();
  spi_0();
  spi_0();

  // Clock
  spiEnableHigh();
  delayMicroseconds(1);

  digitalWrite(SLAVE_SELECT_PIN, LOW);
  digitalWrite(SPI_CLOCK_PIN, LOW);
  digitalWrite(SPI_DATA_PIN, LOW);
}

//******************************************************************************
//* function: spi_1
//******************************************************************************
void spi_1()
{
  digitalWrite(SPI_CLOCK_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(SPI_DATA_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SPI_CLOCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SPI_CLOCK_PIN, LOW);
  delayMicroseconds(1);
}

//******************************************************************************
//* function: spi_0
//******************************************************************************
void spi_0()
{
  digitalWrite(SPI_CLOCK_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(SPI_DATA_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(SPI_CLOCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SPI_CLOCK_PIN, LOW);
  delayMicroseconds(1);
}

//******************************************************************************
//* function: spiEnableLow
//******************************************************************************
void spiEnableLow()
{
  delayMicroseconds(1);
  digitalWrite(SLAVE_SELECT_PIN, LOW);
  delayMicroseconds(1);
}

//******************************************************************************
//* function: spiEnableHigh
//******************************************************************************
void spiEnableHigh()
{
  delayMicroseconds(1);
  digitalWrite(SLAVE_SELECT_PIN, HIGH);
  delayMicroseconds(1);
}

//******************************************************************************
//* function: batteryMeter
//*         : Measured voltage values
//*         : 3s LiPo
//*         : max = 4.2v * 3 = 12.6v = 643
//*         : min = 3.6v * 3 = 10.8v = 551
//*         : 2s LiPo
//*         : max = 4.2v * 2 = 8.4v = 429
//*         : min = 3.6v * 2 = 7.2v = 367
//******************************************************************************
void batteryMeter( unsigned char x, unsigned char y, bool showNumbers )
{
  unsigned int voltage;
  unsigned char value;
  unsigned int minV;
  unsigned int maxV;

  // Do no calculations if the meter should not be displayed
  if (!options[LIPO_3S_METER_OPTION] && !options[LIPO_2S_METER_OPTION])
    return;

  if (options[LIPO_3S_METER_OPTION]) {
    minV = 551;
    maxV = 643;
  }
  if (options[LIPO_2S_METER_OPTION]) {
    minV = 367;
    maxV = 429;
  }
  voltage = averageAnalogRead(VOLTAGE_METER_PIN);

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
  drawBattery(x, y, value, showNumbers);
}

//******************************************************************************
//* function: setOptions
//******************************************************************************
void setOptions()
{
  unsigned char exitNow = false;
  unsigned char menuSelection = 0;
  unsigned char click = NO_CLICK;

  // Display option screen
  drawOptionsScreen( menuSelection );

  // Let the user release the button
  getClickType( BUTTON_PIN );

  while ( !exitNow )
  {
    drawOptionsScreen( menuSelection );
    click = getClickType( BUTTON_PIN );
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
      case LONG_LONG_CLICK:
        if (menuSelection == EXIT_COMMAND)
          exitNow = true;
        else if (menuSelection == RESET_SETTINGS_COMMAND)
          resetOptions();

        else if (menuSelection == TEST_ALARM_COMMAND) {
          testAlarm();
        }
        else
          options[menuSelection] = !options[menuSelection];
        break;
    }
  }
}

//******************************************************************************
//* function: testAlarm
//*         : Cycles through alarms, regardless of alarm settings
//******************************************************************************
void testAlarm( void ) {
  unsigned char i;

  for ( i = 0; i < 3; i++) {
    analogWrite( ALARM_PIN, 32 ); delay(ALARM_MIN_ON);
    analogWrite( ALARM_PIN, 0 );  delay(ALARM_MIN_OFF);
  }
  for (i = 0; i < 3; i++) {
    analogWrite( ALARM_PIN, 32 ); delay(ALARM_MED_ON);
    analogWrite( ALARM_PIN, 0 );  delay(ALARM_MED_OFF);
  }
  for (i = 0; i < 3; i++) {
    analogWrite( ALARM_PIN, 32 ); delay(ALARM_MAX_ON);
    analogWrite( ALARM_PIN, 0 );  delay(ALARM_MAX_OFF);
  }
  analogWrite( ALARM_PIN, 0 );
}

//******************************************************************************
//* Screen functions
//******************************************************************************
//******************************************************************************
//* function: osd
//******************************************************************************
void osd( unsigned char command )
{
  while (Serial.availableForWrite() < 2) ;
  Serial.write( CMD_CMD );
  Serial.write( command );
}

void osd( unsigned char command, unsigned char param )
{
  while (Serial.availableForWrite() < 3) ;
  Serial.write( CMD_CMD );
  Serial.write( command );
  Serial.write( param );
}

void osd_char( unsigned char token )
{
  while (Serial.availableForWrite() == 0) ;
  Serial.write( token );
}

void osd_int( unsigned int integer )
{
  char buff[17];
  itoa(integer, buff, 10);
  while (Serial.availableForWrite() < strlen(buff)) ;
  Serial.write(buff);
}

void osd_string( char *str )
{
  while (Serial.availableForWrite() < strlen(str)) ;
  Serial.write(str);
}
//******************************************************************************
//* function: dissolveDisplay
//*         : fancy graphics stuff that dissolves the screen into black
//*         : unnecessary, but fun
//******************************************************************************
void dissolveDisplay(void)
{
  unsigned char x, y, i = 10;
  unsigned int j;
  while (i--) {
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED) // Return if button pressed
      return;
    j = 250;
    while (j--) {
      x = random(30);
      y = random(13);
      osd( CMD_SET_X, x );
      osd( CMD_SET_Y, y );
      osd_string(" ");
    }
  }
  osd( CMD_CLEAR_SCREEN );
}

//******************************************************************************
//* function: drawStartScreen
//*         : displays a boot image for a short time
//******************************************************************************
void drawStartScreen( void ) {
  unsigned char i;

  osd( CMD_CLEAR_SCREEN );
  osd( CMD_SET_X, 0 );
  osd( CMD_SET_Y, 0 );
  osd(CMD_ENABLE_INVERSE);
  osd_char( OSD_LOGO );
  osd_char( OSD_LOGO + 1 );
  osd_char( OSD_LOGO + 2 );
  osd_char( OSD_LOGO + 3 );
  osd_char( OSD_LOGO + 4 );
  osd_char( OSD_LOGO + 5 );
  osd_char( OSD_LOGO + 6 );
  osd_char( OSD_LOGO + 7 );
  osd( CMD_SET_X, 0 );
  osd( CMD_SET_Y, 1 );
  osd_char( OSD_LOGO + 8 );
  osd_char( OSD_LOGO + 9 );
  osd_char( OSD_LOGO + 10 );
  osd_char( OSD_LOGO + 11);
  osd_char( OSD_LOGO + 12);
  osd_char( OSD_LOGO + 13 );
  osd_char( OSD_LOGO + 14 );
  osd_char( OSD_LOGO + 15 );
  osd(CMD_DISABLE_INVERSE);
  osd( CMD_SET_X, 11 );
  osd( CMD_SET_Y, 0 );  
  osd_string(VER_INFO_STRING);
  osd( CMD_SET_X, 14 );
  osd( CMD_SET_Y, 1 );  
  osd_string(VER_DATE_STRING);
  batteryMeter( 29, 0 );
  
  // Return after 2000 ms or when button is pressed
  for (i = 200; i; i--)
  {
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED )
      return;
    delay(10);
  }
  dissolveDisplay();
  return;
}

//******************************************************************************
//* function: drawChannelScreen
//*         : draws the standard screen with channel information
//******************************************************************************
void drawChannelScreen( unsigned char channel, unsigned int rssi) {
  char buffer[22];
  unsigned char i;

  osd(CMD_CLEAR_SCREEN );
  osd(CMD_SET_X, 0);
  osd(CMD_SET_Y, 3);

  osd_string(" Frequency: ");
  osd(CMD_ENABLE_FILL);
  osd_int(getFrequency(channel));
  osd(CMD_DISABLE_FILL);
  osd_string(" MHz");
  osd(CMD_NEWLINE);

  osd_string(" Channel  : ");
  osd(CMD_ENABLE_FILL);
  osd_string(shortNameOfChannel(channel, buffer));
  osd(CMD_DISABLE_FILL);
  osd(CMD_NEWLINE);

  osd_string(" Name     : ");
  osd(CMD_ENABLE_FILL);
  osd_string( longNameOfChannel(channel, buffer));
  osd(CMD_DISABLE_FILL);
  osd(CMD_NEWLINE);

  osd_string(" RSSI     : ");
  osd(CMD_ENABLE_FILL);
  osd_int(rssi);
  osd(CMD_DISABLE_FILL);

  batteryMeter(29, 0);
}

//******************************************************************************
//* function: drawAutoScanScreen
//******************************************************************************
void drawAutoScanScreen( void ) {

  osd(CMD_CLEAR_SCREEN );
  osd(CMD_SET_X, 0);
  osd(CMD_SET_Y, 3);

  osd_string(" Frequency: ");
  osd_string(" Scanning...");
  osd(CMD_NEWLINE);

  osd_string(" Channel  : ");
  osd(CMD_NEWLINE);

  osd_string(" Name     : ");
  osd(CMD_NEWLINE);

  osd_string(" RSSI     : ");

  batteryMeter(29, 0);
}

//******************************************************************************
//* function: drawScannerScreen
//******************************************************************************
void drawScannerScreen( void ) {

  osd(CMD_CLEAR_SCREEN );
  osd(CMD_SET_X, 0);
  osd(CMD_SET_Y, 12);
  osd(CMD_ENABLE_FILL);
  osd_string("5.65         5.80         5.95");
  osd(CMD_DISABLE_FILL);
}

//******************************************************************************
//* function: updateScannerScreen
//*         : position = 0 to 59
//*         : value1 = 0 to 24
//*         : value2 = 0 to 24
//*         : must be fast since there are frequent updates
//******************************************************************************
void updateScannerScreen(unsigned char position, unsigned char value1, unsigned char value2 ) {
  unsigned char i;
  static unsigned char last_position = 0;
  static unsigned char last_value1 = 0;
  static unsigned char last_value2 = 0;
  char barCells[4];

  // Errase the scan line from the last pass
  for (i = 0; i < 12; i++) {
    osd(CMD_SET_X, last_position);
    osd(CMD_SET_Y, i);

    barCells[0] = value1 >= ((i * 2) + 2);
    barCells[1] = value2 >= ((i * 2) + 2);
    barCells[2] = value1 >= ((i * 2) + 1);
    barCells[3] = value2 >= ((i * 2) + 1);

    if (barCells[0] && barCells[1] && barCells[2] && barCells[4])
      osd_char(OSD_BAR_1111);
    else if (!barCells[0] && barCells[1] && barCells[2] && barCells[4])
      osd_char(OSD_BAR_0111);
    else if (barCells[0] && !barCells[1] && barCells[2] && barCells[4])
      osd_char(OSD_BAR_1011);
    else if (!barCells[0] && !barCells[1] && barCells[2] && barCells[4])
      osd_char(OSD_BAR_0011);
    else if (!barCells[0] && !barCells[1] && !barCells[2] && barCells[4])
      osd_char(OSD_BAR_0001);
    else if (!barCells[0] && !barCells[1] && barCells[2] && !barCells[4])
      osd_char(OSD_BAR_0010);
    else
      osd_char(OSD_BAR_EMPTY);
  }
  // Draw the current scan line
  osd(CMD_ENABLE_INVERSE);
  for (i = 0; i < 12; i++) {
    osd(CMD_SET_X, position);
    osd(CMD_SET_Y, i);
    osd_char(OSD_BAR_1111);
  }
  osd(CMD_DISABLE_INVERSE);

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
    osd(OSD_BATTERY_25);
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
void drawOptionsScreen(unsigned char option ) {
  unsigned char i, j;

  osd( CMD_CLEAR_SCREEN );
  osd( CMD_SET_X, 0 );
  osd( CMD_SET_Y, 2 );

  if (option != 0)
    j = option - 1;
  else
    j = MAX_OPTIONS + MAX_COMMANDS - 1;

  for (i = 0; i < MAX_OPTION_LINES; i++, j++)
  {
    if (j >= (MAX_OPTIONS + MAX_COMMANDS))
      j = 0;
    if (j == option) {
      osd( CMD_ENABLE_INVERSE );
    }
    else {
      osd( CMD_DISABLE_INVERSE );
    }
    switch (j) {
      case LIPO_2S_METER_OPTION:     osd_string("LiPo 2s Meter   "); break;
      case LIPO_3S_METER_OPTION:     osd_string("LiPo 3s Meter   "); break;
      case BATTERY_ALARM_OPTION:     osd_string("Battery Alarm   "); break;
      case SHOW_STARTSCREEN_OPTION:  osd_string("Show Startscreen"); break;
      case TOP_INFO_LINE_OPTION:     osd_string("Top Line Info   "); break;
      case BOTTOM_INFO_LINE_OPTION:  osd_string("Bottom Line Info"); break;
      case RESET_SETTINGS_COMMAND:   osd_string("Reset Settings  "); break;
      case TEST_ALARM_COMMAND:       osd_string("Test Alarm      "); break;
      case EXIT_COMMAND:             osd_string("Exit            "); break;
    }
    if (j < MAX_OPTIONS) {
      if (options[j])
        osd_string(" ON ");
      else
        osd_string(" OFF");
    }
    else
      osd_string("    ");

    osd( CMD_NEWLINE );
  }
}

//******************************************************************************
//* function: drawTopInfoLine
//******************************************************************************
void drawTopInfoLine( void )
{
  char buffer[3];

  osd( CMD_CLEAR_SCREEN );
  osd( CMD_SET_Y, 0);
  osd( CMD_SET_X, 15 );
  osd_char(OSD_MHZ);
  osd_string(shortNameOfChannel(currentChannel, buffer));
  osd_char(OSD_SPACE);
  osd_int(getFrequency(currentChannel));
  osd_char(OSD_SPACE);
  osd_char(OSD_ANTENNA);
  osd_int(currentRssi);
  osd_char(OSD_SPACE);
  batteryMeter(29, 0);
}
//******************************************************************************
//* function: drawBottomInfoLine
//******************************************************************************
void drawBottomInfoLine( void )
{
  char buffer[3];

  osd( CMD_CLEAR_SCREEN );
  osd( CMD_SET_Y, 12);
  osd( CMD_SET_X, 15 );
  osd_char(OSD_MHZ);
  osd_string(shortNameOfChannel(currentChannel, buffer));
  osd_char(OSD_SPACE);
  osd_int(getFrequency(currentChannel));
  osd_char(OSD_SPACE);
  osd_char(OSD_ANTENNA);
  osd_int(currentRssi);
  osd_char(OSD_SPACE);
  batteryMeter(29, 12);
}
