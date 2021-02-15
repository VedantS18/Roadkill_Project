 

#include <SD.h>

#include <ArduinoSound.h>

#include "wiring_private.h"

#include <Wire.h>

#include <Adafruit_Sensor.h>

 

#include <Adafruit_TSL2591.h>

#define TCAADDR 0x70

 

TwoWire myWire(&sercom3, 4, 5);

 

 

int rightLux = 0;

int leftLux = 0;

 

const int rightSensorPin = 2;

const int leftSensorPin = 1;

const int ledPin = 7;

const double instantMomentum = 0.8;

const double baselineGrowth = 0.05;

const double baselineDecay = 0.8;

const double velocityThreshold = 1.25;

 

//dynamic variables

double rightInstantLightLevel = 0;

double rightBaselineLightLevel = 0;

double leftInstantLightLevel = 0;

double leftBaselineLightLevel = 0;

int rightSensorValue;

int leftSensorValue;

int rightiInstant;

int rightiBaseline;

int leftiInstant;

int leftiBaseline;

 

char printBuffer[256];

bool isBootup = true;

int loudness = 5;

int timesPlayed = 0;

static const int audioRepetitionsPerTrigger = 2;

// variable representing the Wave File

static SDWaveFile wavefiles[32];

static int numberOfAudioFiles = 0;

File root;

 

void loadAudioFiles(File dir) {

  File entry =  dir.openNextFile();

  while (entry) {

    wavefiles[numberOfAudioFiles] = SDWaveFile(entry.name());

    entry.close();

    numberOfAudioFiles++;

    entry =  dir.openNextFile();

  }

}

 

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

void configureSensor(void)

{

  // You can change the gain on the fly, to adapt to brighter/dimmer light situations

  // tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)

   tsl.setGain(TSL2591_GAIN_MED);      // 25x gain

  // tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

 

 // Changing the integration time gives you a longer time over which to sense light

  // longer timelines are slower, but are good in very low light situtations!

  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)

 

 

  /* Display the gain and integration time for reference sake */

  Serial.println("------------------------------------");

  Serial.print  ("Gain:         ");

  tsl2591Gain_t gain = tsl.getGain();

  switch (gain)

  {

    case TSL2591_GAIN_LOW:

      Serial.println("1x (Low)");

      break;

    case TSL2591_GAIN_MED:

      Serial.println("25x (Medium)");

      break;

    case TSL2591_GAIN_HIGH:

      Serial.println("428x (High)");

      break;

    case TSL2591_GAIN_MAX:

      Serial.println("9876x (Max)");

      break;

  }

  Serial.print  ("Timing:       ");

  Serial.print((tsl.getTiming() + 1) * 100, DEC);

  Serial.println(" ms");

  Serial.println("------------------------------------");

  Serial.println("");

 

  /* Setup the SW interrupt to trigger between 100 and 1500 lux */

  /* Threshold values are defined at the top of this sketch */

 

 

  /* Display the interrupt threshold window */

 

}

 

void displaySensorDetails(void)

{

  sensor_t sensor;

  tsl.getSensor(&sensor);

  uint32_t lum = tsl.getFullLuminosity();

  uint16_t ir, full;

  ir = lum >> 16;

  full = lum & 0xFFFF;

 

  //Serial.println("------------------------------------");

  //Serial.print  ("Sensor:       "); Serial.println(sensor.name);

  //Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);

  //Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);

  //Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");

  //Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");

  //Serial.print  ("Curr Value:   "); Serial.print(tsl.calculateLux(full, ir)); Serial.println(" lux");

  //Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");

  //Serial.println("------------------------------------");

  //Serial.println("");

  delay(500);

}

void tcaselect(uint8_t i) {

  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);

  Wire.write(1 << i);

  Wire.endTransmission(); 

}

 

void setup() {

 

 

  // register event

  analogWrite(ledPin,255);

 

  //Setup the pin mappings

  //if (!SD.begin()) {

    //Serial.println("SD card initialization failed!");

    //return;

  //}

  Serial.println("SD card is valid.");

  pinMode(ledPin, OUTPUT);    // Set up the LED pin to be an output.

  Serial.begin(9600);

  while (!Serial);

  SD.begin();

  root = SD.open("/Sounds/");

  loadAudioFiles(root);

 

  // check if the I2S output can play the wave file

  delay(500);

  Wire.begin();

  tcaselect(2);

  if(tsl.begin())

  {

    Serial.println("Found a TSL2591 sensor!");

  }

  else

  {

    Serial.println("No sensor found ...");

  }

  displaySensorDetails();

  configureSensor();

   //AudioOutI2S.volume(loudness);

   //AudioOutI2S.play(waveFile);

  

}

 

void loop() {

  // put your main code here, to run repeatedly:

  tcaselect(2);

  uint32_t lum = tsl.getFullLuminosity();

  uint16_t ir, full;

  ir = lum >> 16;

  full = lum & 0xFFFF;

  rightLux = tsl.calculateLux(full, ir);

  //displaySensorDetails();

 

  tcaselect(7);

  uint32_t lumLeft = tsl.getFullLuminosity();

  uint16_t irLeft, fullLeft;

  irLeft = lumLeft >> 16;

  fullLeft = lumLeft & 0xFFFF;

  leftLux = tsl.calculateLux(fullLeft, irLeft);

 

  rightSensorValue = rightLux; 

  leftSensorValue = leftLux; 

  if (isBootup == true)

  {

    Serial.println("Bootup Mode");

    rightBaselineLightLevel = rightSensorValue;

    rightInstantLightLevel = rightSensorValue;

    leftInstantLightLevel = leftSensorValue;

    leftBaselineLightLevel = leftSensorValue;

    isBootup = false;

  }

  else

  {

      Serial.println("Right");

      Serial.print("rightSensorValue "); Serial.println(rightSensorValue);

      Serial.print("Right Instant "); Serial.println(rightInstantLightLevel);

      Serial.print("Right Baseline "); Serial.println(rightBaselineLightLevel);

      Serial.println("Left");

      Serial.print("leftSensorValue "); Serial.println(leftSensorValue);

      Serial.print("Left Instant "); Serial.println (leftInstantLightLevel);

      Serial.print("Left Baseline "); Serial.println(leftBaselineLightLevel);

 

    //Noise filter- configurable depending on sensor parameters

    if(rightSensorValue < rightBaselineLightLevel){

      rightBaselineLightLevel = (rightBaselineLightLevel + (rightSensorValue - rightBaselineLightLevel) * baselineDecay);

      Serial.println("One");

    } else {

    rightBaselineLightLevel = (rightBaselineLightLevel + (rightSensorValue - rightBaselineLightLevel) * baselineGrowth);

      Serial.println("Two");

    }

    if(leftSensorValue < leftBaselineLightLevel){

      leftBaselineLightLevel = (leftBaselineLightLevel + (leftSensorValue - leftBaselineLightLevel) * baselineDecay);

      Serial.println("Three");

    } else {

      leftBaselineLightLevel = (leftBaselineLightLevel + (leftSensorValue - leftBaselineLightLevel) * baselineGrowth);

      Serial.println("Four");

    }

 

    rightInstantLightLevel = (rightInstantLightLevel + (rightSensorValue - rightInstantLightLevel) * instantMomentum);

    leftInstantLightLevel = (leftInstantLightLevel + (leftSensorValue - leftInstantLightLevel) * instantMomentum);

 

  }

 

  rightiInstant = int(rightInstantLightLevel);

  rightiBaseline = int(rightBaselineLightLevel);

  leftiInstant = int(leftInstantLightLevel);

  leftiBaseline = int(leftBaselineLightLevel);

   

  //Conditional based on light value and impulse momentum

  //Configurable parameter is velocity threshold

  if(numberOfAudioFiles > 0)

  {

    if ((rightiInstant > (rightiBaseline * velocityThreshold)) ||

        (leftiInstant > (leftiBaseline * velocityThreshold)))  {

        int fileIndex = random(numberOfAudioFiles);

        Serial.println("Mountain Lion go away");

        analogWrite(ledPin, 255);

        delay(300);

        analogWrite(ledPin, 0);       

        delay(100);

        if (AudioOutI2S.isPlaying()!= 1 && timesPlayed < audioRepetitionsPerTrigger){

          Serial.println("playing WAV file using I2S!");

          AudioOutI2S.play(wavefiles[fileIndex]);

          timesPlayed = timesPlayed + 1;

        }

        else

        {

          analogWrite(ledPin, 255);

          delay(100);

          analogWrite(ledPin, 0);

 

        }

    }

    else {

      timesPlayed = 0;

    }

  }

  delay(100);

  analogWrite(ledPin,0);

  delay(100);

}
