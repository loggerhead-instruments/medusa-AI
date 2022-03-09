//
// Medusa AI Drifter
//
// Loggerhead Instruments
// c 2022
// David Mann

// There must be an interval between files to work properly

// Compile 96 MHz Fastest

// To Do:
// - add accelerometer support
// - check failure scenarios
// -     pi doesn't boot
// -     sd doesn't connect

// Power Consumptions
// Startup: 200 mA
// Recording: 48 mA 
// Transmit: 150-280 (timeout at 120 s)
// Sleep: start at 5.0 mA goes down to 3.9 mA (prob supercapcitor discharging on Iridium)


// Hydrophone connector
// Red: Power
// Black: GND
// Yellow: LEFT
// Green: RIGHT

#define codeVersion 20220311
#define MQ 100 // to be used with LHI record queue (modified local version)

#include "input_i2s.h"
#include <i2c_t3.h>  //https://github.com/nox771/i2c_t3; Teensy Audio: control_sgtl5000.cpp needs to have Wire.h commented
#include <SPI.h>
#include "SdFat.h"
#include <Snooze.h>  //using https://github.com/duff2013/Snooze; uncomment line 62 #define USE_HIBERNATE
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <AltSoftSerial.h>
#include "IridiumSBD.h"
#include "LHI_record_queue.h"
#include "control_sgtl5000.h"
#include <analyze_fft1024.h>
#include <analyze_fft256.h>
// 
// Dev settings
//
#define IRIDIUM_MODEM
// #define SWARM_MODEM
#define PI_PROCESSING

int runMode = 1; // 0 = dev mode (power on Pi and give microSD access); 1 = deployment mode
boolean sendIridium = 1;
boolean useGPS = 1;
static boolean printDiags = 1;  // 1: serial print diagnostics; 0: no diagnostics 2=verbose
#define I_SAMP 6   // 0 is 8 kHz; 1 is 16 kHz; 2 is 32 kHz; 3 is 44.1 kHz; 4 is 48 kHz; 5 is 96 kHz; 6 is 192 kHz
boolean imuFlag = 1;
int moduloSeconds = 10; // round to nearest start time
#define SENDBINARY 1 // send Iridium message as binary; set to 1
float hydroCalLeft = -180;
float hydroCalRight = -180;
#define FFT1024 1024
//#define FFT256 256
#define NBANDS 25  // number of frequency bands to calculate and send. MAX = 25
#define NPEAKBANDS 8 // must be same or lower than NBANDS; this is number of peak bands to send in satellite packet

uint32_t nbufs_noise_stats;
int bandLow[NBANDS] = {1,2,3,4,5,6,7,9,11,14,18,23,29,36,46,58,73,92,116,146,184,232,292,368,463}; // band low frequencies
int bandHigh[NBANDS]= {2,3,4,5,6,7,9,11,14,18,23,29,36,46,58,73,92,116,146,184,232,292,368,463,512};
//int bandLow[NBANDS] = {1,2,3,4,5,6,7,9,10,11,12,13,14,15,16,17,18,19,20,30,40,50,70,90,110}; // band low frequencies
//int bandHigh[NBANDS]= {2,3,4,5,6,7,9,10,11,12,13,14,15,16,17,18,19,20,30,40,50,70,90,110,128};
#define NCHAN 1
#define ONECHAN 1
//#define TWOCHAN 2

long piTimeout = 300 ; // timeout Pi processing in seconds
boolean cardFailed = 0; // if sd card fails, skip Pi processing
char piPayload[200];  // payload to send from Pi/Coral detector

//
// EEPROM SETTINGS -- THESE ONLY TAKE EFFECT FOR NEW MEDUSA
//
int isf = 2; // index sampling frequency
long rec_dur = 600; // seconds
long rec_int = 600;  // seconds Maximum = 600 s when using watchdog timer
long msg_int = rec_dur + rec_int; // send every message
long analyze_dur; // must be same or less than record duration; default is same as rec_dur
int gainSetting = 4; // SG in script file
//
// ********************************************************************************************************//
//


// Pin Assignments
#define hydroPowPin 8
#define vSense A14  // moved to Pin 21 for X1
#define iridiumAv 2 // High when Iridium network available 
#define iridiumRi 3 // Ring Indicator: active low; inactive high
#define iridiumSleep 4 // Sleep active low
#define RECV_PIN 6
#define gpsEnable 5

#define audioPowEnable 8
#define POW_5V 15
#define SD_POW 16
#define SD_SWITCH 17
#define PI_STATUS A10
#define PI_STATUS2 A11

#define SD_TEENSY LOW
#define SD_PI HIGH

AltSoftSerial gpsSerial;  // RX 20; Tx: 21

IridiumSBD modem(Serial1, iridiumSleep);
int sigStrength;
uint8_t rxBuffer[100];


#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);
#define displayLine1 0
#define displayLine2 9
#define displayLine3 18
#define displayLine4 27
#define BOTTOM 55


static uint8_t myID[8];

unsigned long baud = 115200;

#ifdef TWOCHAN
  #ifdef FFT1024
    int fftPoints = 1024;
    #define FFT1 fft1024_1
    #define FFT2 fft1024_2
    
    // GUItool: begin automatically generated code
    AudioInputI2S            i2s2;           //xy=262,190
    AudioAnalyzeFFT1024       FFT1;       //xy=518,130
    AudioAnalyzeFFT1024       FFT2;       //xy=518,130
    LHIRecordQueue           queue1;         //xy=281,63
    LHIRecordQueue           queue2;         //xy=281,63
    AudioConnection          patchCord1(i2s2, 0, FFT1, 0);
    AudioConnection          patchCord2(i2s2, 0, queue1, 0);
    AudioConnection          patchCord3(i2s2, 1, FFT2, 0);
    AudioConnection          patchCord4(i2s2, 1, queue2, 0);
    AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
    // GUItool: end automatically generated code
  #endif
  
  #ifdef FFT256
  int fftPoints = 256;
    // GUItool: begin automatically generated code
    #define FFT1 fft256_1
    #define FFT2 fft256_2
    AudioInputI2S            i2s2;           //xy=262,190
    AudioAnalyzeFFT256       fft256_1;       //xy=518,130
    AudioAnalyzeFFT256       fft256_2;       //xy=518,130
    LHIRecordQueue           queue1;         //xy=281,63
    LHIRecordQueue           queue2;         //xy=281,63
    AudioConnection          patchCord1(i2s2, 0, FFT1, 0);
    AudioConnection          patchCord2(i2s2, 0, queue1, 0);
    AudioConnection          patchCord3(i2s2, 1, FFT2, 0);
    AudioConnection          patchCord4(i2s2, 1, queue2, 0);
    AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
    // GUItool: end automatically generated code
  #endif
#endif


#ifdef ONECHAN
  #ifdef FFT1024
  int fftPoints = 1024;
    #define FFT1 fft1024_1
    // GUItool: begin automatically generated code
    AudioInputI2S            i2s2;           //xy=262,190
    AudioAnalyzeFFT1024       FFT1;       //xy=518,130
    LHIRecordQueue           queue1;         //xy=281,63
    AudioConnection          patchCord1(i2s2, 0, FFT1, 0);
    AudioConnection          patchCord2(i2s2, 0, queue1, 0);
    AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
    // GUItool: end automatically generated code
  #endif
  
  #ifdef FFT256
    int fftPoints = 256;
    // GUItool: begin automatically generated code
    #define FFT1 fft256_1
    AudioInputI2S            i2s2;           //xy=262,190
    AudioAnalyzeFFT256       FFT1;       //xy=518,130
    LHIRecordQueue           queue1;         //xy=281,63
    AudioConnection          patchCord1(i2s2, 0, FFT1, 0);
    AudioConnection          patchCord2(i2s2, 0, queue1, 0);
    AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
    // GUItool: end automatically generated code
  #endif
#endif

const int myInput = AUDIO_INPUT_LINEIN;
float gainDb;
int noDC = 1; // 0 = freezeDC offset; 1 = remove DC offset

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording audio, 2=recprding sensors
time_t startTime;
time_t stopTime;
time_t iridiumSendTime;
time_t t;
time_t sensorStartTime;
time_t packetStartTime;

boolean audioFlag = 1;
volatile boolean LEDSON = 1;
boolean introPeriod=1;  //flag for introductory period; used for keeping LED on for a little while

int update_rate = 10;  // rate (Hz) at which interrupt to read sensors
float imu_srate = 10.0;

int32_t lhi_fsamps[7] = {8000, 16000, 32000, 44100, 48000, 96000, 192000};

float audio_srate;

int accel_scale = 16; //full scale on accelerometer [2, 4, 8, 16] (example cmd code: AS 8)

int wakeahead = 15;
int snooze_hour;
int snooze_minute;
int snooze_second;
volatile long buf_count;
float total_hour_recorded = 0.0;
long nbufs_per_file;
boolean settingsChanged = 0;

long file_count;
char filename[25];
char dirname[8];
int folderMonth;
//SnoozeBlock snooze_config;
SnoozeAlarm alarm;
SnoozeAudio snooze_audio;
SnoozeBlock config_teensy32(snooze_audio, alarm);

// The file where data is recorded
File frec;
SdFat sd;
float recDays;
boolean sdGood = 1;

typedef struct {
    char    rId[4];
    unsigned int rLen;
    char    wId[4];
    char    fId[4];
    unsigned int    fLen;
    unsigned short nFormatTag;
    unsigned short nChannels;
    unsigned int nSamplesPerSec;
    unsigned int nAvgBytesPerSec;
    unsigned short nBlockAlign;
    unsigned short  nBitsPerSamples;
    char    dId[4];
    unsigned int    dLen;
} HdrStruct;

HdrStruct wav_hdr;
unsigned int rms;

// Sensor descriptions
#define STR_MAX 8
#define SENSOR_MAX 5
struct SENSOR{
  char chipName[STR_MAX]; // name of sensor e.g. MPU9250
  uint16_t nChan;       //number of channels used (e.g. MPU9250 might have 9 for accel, mag, and gyro)
  uint16_t NU;
  char name[12][STR_MAX]; //name of each channel (e.g. accelX, gyroZ). Max of 12 channels per chip.
  char units[12][STR_MAX];// units of each channel (e.g. g, mGauss, degPerSec)
  float cal[12];     //calibration coefficient for each channel when multiplied by this value get data in specified units
};
SENSOR sensor[SENSOR_MAX]; //structure to hold sensor specifications. e.g. MPU9250, MS5837

unsigned char prev_dtr = 0;

// IMU
int FIFOpts;
#define IMUBUFFERSIZE 900 // store 10 s at 10 Hz (9 axes)
volatile int16_t imuBuffer[IMUBUFFERSIZE]; // buffer used to store IMU sensor data before writes in samples
volatile int bufferposIMU = 0;
volatile boolean bufferImuFull;
volatile uint8_t imuTempBuffer[20];
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
int16_t mag_x;
int16_t mag_y;
int16_t mag_z;
int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;
float gyro_temp;
int magXoffset;
int magYoffset;
int magZoffset;
float pitch, roll, yaw;
volatile float sdAccelZmg;

float voltage;

IntervalTimer slaveTimer;

// GPS
#define GPS_UBLOX 1
#define GPS_GTOP 2
volatile int gpsType = GPS_UBLOX; //if detects GTOP from Adafruit getGpsType will switch
volatile int goodGpsType = 0;
volatile float latitude = 0.0;
volatile float longitude = 0.0;
char latHem, lonHem;
int gpsYear = 19, gpsMonth = 2, gpsDay = 4, gpsHour = 22, gpsMinute = 5, gpsSecond = 0;
volatile int goodGPS = 0;
long gpsTimeOutThreshold = 600000;  //milliseconds; 10 minutes

// define bands to measure acoustic signals
// these need to be recalculated if the sample rate changes from settings
float binwidth = audio_srate / fftPoints; //256 point FFT; = 172.3 Hz for 44.1kHz
float fftDurationMs = 1000.0 / binwidth;
long fftCount;

float meanBand[NBANDS][2]; // mean band values for left and right channel
float peakBand[NBANDS][2]; // peak band values for left and right channel

double sumOfSquares; // For running RMS calculation
uint32_t sumOfSquaresCount;
int16_t posPeak, negPeak; // For peak tracking

int nBins[NBANDS]; // number of FFT bins in each band
volatile unsigned int whistleCount = 0;

String dataPacket; // data packed for transmission after each file

void setup() {
  setupWdt();
  read_myID();
  pinMode(POW_5V, OUTPUT);
  digitalWrite(POW_5V, LOW);
  pinMode(SD_POW, OUTPUT);
  digitalWrite(SD_POW, HIGH);
  pinMode(SD_SWITCH, OUTPUT);
  digitalWrite(SD_SWITCH, SD_TEENSY); 
  pinMode(PI_STATUS, INPUT);
  pinMode(PI_STATUS2, INPUT);
  pinMode(hydroPowPin, OUTPUT);
  pinMode(vSense, INPUT);
  analogReference(DEFAULT); 
  pinMode(iridiumAv, INPUT);
  pinMode(iridiumRi, INPUT);
  pinMode(iridiumSleep, OUTPUT);
  digitalWrite(iridiumSleep, LOW); // HIGH = enabled; LOW = sleeping
  pinMode(gpsEnable, OUTPUT);

  digitalWrite(hydroPowPin, HIGH);
  digitalWrite(gpsEnable, HIGH);  // HIGH = enabled; LOW = Sleep
  for(int i=0; i<NBANDS; i++){
    nBins[i] = bandHigh[i] - bandLow[i];
  }
  
  Serial.begin(baud);

  delay(500);
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire.setDefaultTimeout(10000);

  RTC_CR = 0; // disable RTC
  delay(100);
  Serial.println(RTC_CR,HEX);
  // change capacitance to 26 pF (12.5 pF load capacitance)
  RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P; 
  delay(100);
  RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P | RTC_CR_OSCE;
  delay(100);

  if(printDiags > 0){
      Serial.println("YY-MM-DD HH:MM:SS ");
      // show 3 ticks to know crystal is working
      for (int n=0; n<3; n++){
        printTime(getTeensy3Time());
        delay(1000);
      }
   }
  
  readVoltage();
  displayOn();
  cDisplay();
  display.println("Loggerhead");
  display.display();

  // Check if runMode = 0 for Pi dev
  if (runMode == 0){
    digitalWrite(SD_POW, LOW); // switch off power to microSD (Pi will use SD mode, so card needs to reset)
    digitalWrite(SD_SWITCH, SD_PI); // switch control to Pi
    digitalWrite(POW_5V, HIGH); // power on Pi
    delay(1000);
    digitalWrite(SD_POW, HIGH); // power on microSD
    display.println("Pi Dev Mode");
    display.display();
    while(1);
    
  }

  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(sd.begin(10))) {
    sdGood = 0;
    Serial.println("Unable to access the SD card");
    cDisplay();
    for (int flashMe=0; flashMe<50; flashMe++){
      display.println("");
      display.println("SD error");
      display.display();
      delay(400);
      cDisplay();
      display.display();
      delay(400);
    }
    delay(400);    
  }
  readEEPROM();  // read settings stored in EEPROM
  analyze_dur = rec_dur;  
  if(sdGood) LoadScript();
  if (msg_int < (rec_dur + rec_int)) msg_int = rec_dur + rec_int;
  writeEEPROM(); // update settings changed from script

  binwidth = audio_srate / fftPoints; //256 point FFT; = 172.3 Hz for 44.1kHz
  fftDurationMs = 1000.0 / binwidth;

  updatePowerDuration();
 
  sensorInit(); // initialize and test sensors; GPS and Iridium should be after this
  resetSignals();

  uint16_t failTimes = 0;
  if(useGPS){
    display.println("GPS");
    display.println("Getting type....");
    display.display();
    getGpsType();
    cDisplay();
    display.println("GPS");
    if(gpsType==GPS_UBLOX) display.println("UBLOX");
    else
      display.println("GTOP");
      display.println("Searching....");
      display.display();
    
    while(!goodGPS){
      if(failTimes > 6) break;  // in case restarted don't get stuck here
      gpsGetTimeLatLon(); // this has 10 minute timeout
      if(!goodGPS){
        Serial.println("Unable to get GPS");
        cDisplay();
        display.println();
        display.println("Wait for GPS");
        display.println("Do not deploy");
        display.display();
        delay(10000);
        failTimes++;
      }
    }
  }

  setTeensyTime(gpsHour, gpsMinute, gpsSecond, gpsDay, gpsMonth, gpsYear + 2000);

  cDisplay();
  display.println("Medusa AI");
  #ifdef IRIDIUM_MODEM
    display.println("Initialize modem");
  #endif
  display.setCursor(0,30);
  display.print("Lat: ");
  display.println(latitude);
  display.print("Lon: ");
  display.print(longitude);
  display.display();

  #ifdef IRIDIUM_MODEM
    if(sendIridium){
      Serial1.begin(19200, SERIAL_8N1);  //Iridium
      modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
      int result = modem.begin();
      display.print("Iridium:"); display.println(result);
      display.display();
      modem.getSignalQuality(sigStrength); // update Iridium modem strength
      modem.sleep();
    }
  #endif
  delay(5000);
  if(sdGood) logFileHeader();
  setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time
  
// ULONG newtime;
 
  // Power down USB if not using Serial monitor
  if (printDiags==0){
    //  usbDisable();
  }
  
  //SdFile::dateTimeCallback(file_date_time);

  // if(imuFlag) mpuInit(1); // update MPU with new settings
  setupDataStructures();

  t = getTeensy3Time();
  startTime = t;
  startTime -= startTime % moduloSeconds;  //modulo to nearest modulo seconds
  startTime += moduloSeconds; //move forward
  stopTime = startTime + rec_dur;  // this will be set on start of recording
  iridiumSendTime = stopTime; // make sure send first recording
 

  audio_srate = lhi_fsamps[isf];
  nbufs_per_file = (long) (rec_dur * audio_srate / 256.0) * NCHAN;
  //nbufs_noise_stats = (uint32_t) (analyze_dur * audio_srate / 256.0) * NCHAN;
  binwidth = audio_srate / fftPoints; //256 point FFT; = 172.3 Hz for 44.1kHz
  fftDurationMs = 1000.0 / binwidth;

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  
  AudioMemory(MQ+10);

  AudioInit(isf); // this calls Wire.begin() in control_sgtl5000.cpp
  mode = 0;

  // create first folder to hold data
  folderMonth = -1;  //set to -1 so when first file made will create directory
  packetStartTime = startTime;
}

//
// MAIN LOOP
//

int recLoopCount;  //for debugging when does not start record

void loop() {
  t = getTeensy3Time();

  // Standby mode
  if(mode == 0)
  {
    delay(100);
    resetWdt();

    // if voltage too low sleep until gets to good voltage
    bool sleepFlag = 0;
    while(readVoltage()<3.4){
      resetWdt();
      displayOff();
      Serial1.println("$SL S=300*62"); // sleep Swarm 5 minutes
      sleepFlag = 1;
      alarm.setRtcTimer(0, 5, 0); // sleep Teensy 5 minutes
      delay(10);
      Snooze.sleep(config_teensy32);
      delay(5000); // give time for Swarm to wake up
      if(File logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
        logFile.print('low power sleep');
        logFile.print(',');
        logFile.print(codeVersion);
        logFile.print(',');  
        logFile.print(',');
        logFile.print(readVoltage()); 
        logFile.print(',');
        logFile.print(latitude, 4);
        logFile.print(',');
        logFile.print(longitude, 4);
        logFile.println();
        logFile.close();
      }
    }

    // had been asleep because of low voltage
    if(sleepFlag==1){
      t = getTeensy3Time();
      if(startTime < t) startTime = t + 300; // new next startTime will be current time plus 300 seconds
    }
      
    if(introPeriod){
      cDisplay();
      displaySettings();
      displayClock(BOTTOM, t);
      display.display();
    }
      
    if(t >= startTime){      // time to start?
      Serial.println("Record Start.");

      if(noDC==0) {
        audio_freeze_adc_hp(); // this will lower the DC offset voltage, and reduce noise
        noDC = -1;
      }

      packetStartTime = startTime;
      stopTime = startTime + rec_dur;
      startTime = stopTime + rec_int;  // next start time

      Serial.print("Current Time: ");
      printTime(getTeensy3Time());
      Serial.print("Stop Time: ");
      printTime(stopTime);
      Serial.print("Next Start:");
      printTime(startTime);
      Serial.print("Next Message:");
      printTime(iridiumSendTime);
      Serial.println();

      displayOff();

      mode = 1;
      startRecording();
    }
  }


  // Record mode
  if (mode == 1) {
    resetWdt();
    continueRecording();  // download data  
    //
    // Automated signal processing
    //
    if(FFT1.available()){  
         noiseBandCalc();
    }

    if(buf_count >= nbufs_per_file){       // time to stop?
      total_hour_recorded += (float) rec_dur / 3600.0;
      if(total_hour_recorded > 0.1) introPeriod = 0;  //LEDS on for first file
      stopRecording();
      resetWdt();
      if(introPeriod) displayOn();
      digitalWrite(gpsEnable, HIGH); // turn on so GPS can get fix while processing data and recording motion
      //
      // Process audio with Pi
      //
      #ifdef PI_PROCESSING
        cDisplay();
        display.println("Booting");
        display.display();
        digitalWrite(SD_SWITCH, SD_PI); // switch control to Pi
        digitalWrite(SD_POW, LOW); // switch off power to microSD (Pi will use SD mode, so card needs to reset)
        digitalWrite(POW_5V, HIGH); // power on Pi
        delay(1000);
        digitalWrite(SD_POW, HIGH); // power on microSD
  
        // Pi pin values around 900 when booting
        // value low 0-3
        // value high 1014-1017
        
        // wait for Pi to boot
        // PI_STATUS goes to 0-2 when Python program starts
        time_t startPiTime = getTeensy3Time();
        t = startPiTime;
        int piStatus = analogRead(PI_STATUS);
        Serial.println("PI_STATUS "); 
        while((t - startPiTime < piTimeout) & (piStatus > 200)){
          t = getTeensy3Time();
          piStatus = analogRead(PI_STATUS);
          Serial.println(piStatus);
          delay(2000);
          resetWdt();
        }
        boolean piTimedOut = 0;
        if (t - startPiTime > piTimeout) piTimedOut = 1;
  
        if(introPeriod){
          displayOn();
          cDisplay();
          display.println("Processing");
          if(piTimedOut) display.println("Pi timeout");
          display.display();
        }
  
        // wait for Pi to finish processing or timeout
        // PI_STATUS2 around 1016-1017 during processing. 0-2 when done and Pi about to shut down.
        startPiTime = getTeensy3Time();
        t = startPiTime;
        int piStatus2 = analogRead(PI_STATUS2);
        Serial.println("PI_STATUS2 "); 
        while((t - startPiTime < piTimeout) & (piStatus2 > 200) & (piTimedOut == 0)){
          t = getTeensy3Time();
          piStatus2 = analogRead(PI_STATUS2);
          Serial.println(piStatus2);
          delay(2000);
          resetWdt();
        }
        if (t - startPiTime > piTimeout) piTimedOut = 1;
  
        if(introPeriod){
          displayOn();
          cDisplay();
          display.println("Wait on Pi");
          if(piTimedOut) display.println("Pi timeout");
          display.display();
        }
  
        // wait for Pi to power down
        // values will be 100 when Pi off
        startPiTime = getTeensy3Time();
        t = startPiTime;
        Serial.println("Wait for PI to power down");
        do{
          piStatus = analogRead(PI_STATUS);
          delay(1000);
          Serial.print("Pi Status:"); Serial.println(piStatus);
          t = getTeensy3Time();
          resetWdt();
        }while(((piStatus<20) | (piStatus>1000)) & (t - startPiTime < piTimeout)  & (piTimedOut == 0));
        digitalWrite(POW_5V, LOW); // power off Pi   
        digitalWrite(SD_POW, LOW); // switch off power to microSD (Pi will use SD mode, so card needs to reset)
        digitalWrite(SD_SWITCH, SD_TEENSY); // switch control to Teensy
        delay(1000);
        digitalWrite(SD_POW, HIGH); // power on microSD
        delay(100);

        int sdAttempts = 0;
        if (!(sd.begin(10) & sdAttempts < 2000)) {
          digitalWrite(SD_POW, LOW);
          delay(1000);
          digitalWrite(SD_POW, HIGH);
          delay(1000);
          Serial.println("SD restart failed");
        }
        if(sdAttempts>=10) cardFailed = 1;
  
        // read detections file
        if(!cardFailed) readDetections();
      #endif

      resetWdt();
//      // Record motion data
//      if(imuFlag){
//        if(introPeriod){
//          cDisplay();
//          display.println();
//          display.println("IMU");
//          display.display();
//        }
//        
//         startSensors();
//         long startImuMillis = millis();
//          while(bufferImuFull==0){
//             // timeout 30 s
//             if(millis() - startImuMillis > 30000) break; 
//          }
//          stopSensors();
//          writeSensors();
//          calcSensorStats();
//          Serial.print("Z:");
//          Serial.println(sdAccelZmg);
//          if (imuFlag) mpuInit(0);  //gyro to sleep
//      }

      // get GPS
      if(useGPS){
          if(introPeriod){
            cDisplay();
            display.println();
            display.println("GPS");
            display.display();
          }
          gpsTimeOutThreshold = 30000;//give 20 seconds to read
          gpsGetTimeLatLon();  
          Serial.print("Lat:"); Serial.println(latitude, 6);
          Serial.print("Lon:"); Serial.println(longitude, 6);
          // tweak Teensy time if close to GPS time
          if(goodGPS & (gpsDay==day(t)) & (gpsMonth==month(t)) & (gpsHour==hour(t)) & (gpsMinute=minute(t))) {
            setTeensyTime(gpsHour, gpsMinute, gpsSecond, gpsDay, gpsMonth, gpsYear + 2000);
            Serial.println("RTC update");
          }
      }
      digitalWrite(gpsEnable, LOW); // turn off once have good GPS

    // IRIDIUM
      if(sendIridium==0){
        if(SENDBINARY==1) makeSendBinaryDataPacket();  //just so can see output for testing
        else{
          makeDataPacket();
        }
      }
      if(sendIridium & (stopTime >= iridiumSendTime)){
        if(introPeriod) {
          cDisplay();
          display.println();
          display.println("Send Iridium");
          display.display();
        }
        iridiumSendTime = stopTime + msg_int;  // next time to send
        modem.begin();
        if(SENDBINARY==1) makeSendBinaryDataPacket();
        else{
          makeDataPacket();
          sendDataPacket();
        }
        modem.sleep();
      }
//          if((sendIridium) & (stopTime >= iridiumSendTime)){
//            modem.getSignalQuality(sigStrength);
//            iridiumSendTime = stopTime + msg_int;  // next time to send
//            modem.begin();  // wake Iridium
//            modem.adjustSendReceiveTimeout(120);  // timeout in 120 seconds
//            if(introPeriod) displayOn();
//            int err = sendDataPacket(0);    
//            modem.sleep();
//          }

      resetSignals();

      if(introPeriod) delay(1000); // time to read display
      displayOff();
      
      long ss = startTime - getTeensy3Time() - wakeahead;
      if (ss<0) ss=0;
      snooze_hour = floor(ss/3600);
      ss -= snooze_hour * 3600;
      snooze_minute = floor(ss/60);
      ss -= snooze_minute * 60;
      snooze_second = ss;
      if((snooze_hour * 3600) + (snooze_minute * 60) + snooze_second >=15){
          digitalWrite(hydroPowPin, LOW); //hydrophone off          
          audio_power_down();
          
          if(printDiags > 0){
            Serial.print("Time: ");
            Serial.print(getTeensy3Time());
            Serial.print("  Next: ");
            Serial.println(startTime);
            printTime(getTeensy3Time());
            Serial.print("Snooze HH MM SS ");
            Serial.print(snooze_hour);
            Serial.print(snooze_minute);
            Serial.println(snooze_second);
          }
          delay(100);
          resetWdt();

          alarm.setRtcTimer(snooze_hour, snooze_minute, snooze_second);
          Snooze.sleep(config_teensy32);
     
          /// ... Sleeping ....
          
          // Waking up
          
         if(printDiags>0) printTime(getTeensy3Time());
          digitalWrite(hydroPowPin, HIGH); // hydrophone on 
          delay(100);
          AudioInit(isf);
       }
      if(introPeriod) displayOn();
      mode = 0;
      digitalWrite(gpsEnable, HIGH); // wake up to get GPS in standby mode
      
    }
  }
  asm("wfi"); // reduce power between interrupts
}


void startRecording() {
  Serial.println("startRecording");
  FileInit();
  buf_count = 0;
  queue1.begin();
  #ifdef TWOCHAN
    queue2.begin();
  #endif
  Serial.println("Queue Begin");
}


void continueRecording() {
  byte sampleBuffer[512]; // data to write
    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
    // one buffer is 512 bytes = 256 samples
    // readBuffer returns an int16 *
    if(queue1.available() >= 2) {
      #ifdef ONECHAN
        buf_count += 1;
        memcpy(sampleBuffer, queue1.readBuffer(), 256);
        queue1.freeBuffer();
        memcpy(sampleBuffer+256, queue1.readBuffer(), 256);
        queue1.freeBuffer();
      #endif
      #ifdef TWOCHAN
          buf_count += 1;
          mxLR(sampleBuffer, queue1.readBuffer(), queue2.readBuffer()); // interleave 
          queue1.freeBuffer(); 
          queue2.freeBuffer();  // free buffer
      #endif

      // time domain processing of audio buffer
      for(int i=0; i<512; i+=2){
        int16_t sample = (int16_t) sampleBuffer[i+1]<<8 | sampleBuffer[i];
        if(sample > posPeak) posPeak = sample;
        if(sample < negPeak) negPeak = sample;
        float floatSample = sample / 32768.0;
        sumOfSquares += (floatSample * floatSample);
      }
      sumOfSquaresCount += 256;
      
      if(sdGood) frec.write(sampleBuffer, 512); //audio to .wav file
    }
}

inline void mxLR(byte *dst, const int16_t *srcL, const int16_t *srcR)
  {
    byte cnt = 128;
    int16_t *d = (int16_t *)dst;
    const int16_t *l = srcL;
    const int16_t *r = srcR;

    while (cnt--)
    {
      *(d++) = *l++;
      *(d++) = *r++;
    }
  }

void stopRecording() {
  Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  Serial.print("Audio Memory Max");
  Serial.println(maxblocks);
  queue1.end();
  #ifdef TWOCHAN
    queue2.end();
  #endif

  AudioMemoryUsageMaxReset();
  if(sdGood) frec.close();
  delay(100);
}

void setupDataStructures(void){
  // setup sidSpec and sidSpec buffers...hard coded for now
  
  // audio
  strncpy(sensor[0].chipName, "SGTL5000", STR_MAX);
  sensor[0].nChan = 1;
  strncpy(sensor[0].name[0], "audio1", STR_MAX);
  strncpy(sensor[0].name[1], "audio2", STR_MAX);
  strncpy(sensor[0].name[2], "audio3", STR_MAX);
  strncpy(sensor[0].name[3], "audio4", STR_MAX);
  strncpy(sensor[0].units[0], "Pa", STR_MAX);
  strncpy(sensor[0].units[1], "Pa", STR_MAX);
  strncpy(sensor[0].units[2], "Pa", STR_MAX);
  strncpy(sensor[0].units[3], "Pa", STR_MAX);
  sensor[0].cal[0] = gainDb + hydroCalLeft; // this needs to be set based on hydrophone sensitivity + chip gain
  sensor[0].cal[1] = gainDb + hydroCalRight;
  sensor[0].cal[2] = gainDb + hydroCalLeft;
  sensor[0].cal[3] = gainDb + hydroCalLeft;

  // IMU
  strncpy(sensor[2].chipName, "MPU9250", STR_MAX);
  sensor[2].nChan = 9;
  strncpy(sensor[2].name[0], "accelX", STR_MAX);
  strncpy(sensor[2].name[1], "accelY", STR_MAX);
  strncpy(sensor[2].name[2], "accelZ", STR_MAX);
  //strncpy(sensor[2].name[3], "temp-21C", STR_MAX);
  strncpy(sensor[2].name[3], "gyroX", STR_MAX);
  strncpy(sensor[2].name[4], "gyroY", STR_MAX);
  strncpy(sensor[2].name[5], "gyroZ", STR_MAX);
  strncpy(sensor[2].name[6], "magX", STR_MAX);
  strncpy(sensor[2].name[7], "magY", STR_MAX);
  strncpy(sensor[2].name[8], "magZ", STR_MAX);
  strncpy(sensor[2].units[0], "g", STR_MAX);
  strncpy(sensor[2].units[1], "g", STR_MAX);
  strncpy(sensor[2].units[2], "g", STR_MAX);
  //strncpy(sensor[2].units[3], "degreesC", STR_MAX);
  strncpy(sensor[2].units[3], "degPerS", STR_MAX);
  strncpy(sensor[2].units[4], "degPerS", STR_MAX);
  strncpy(sensor[2].units[5], "degPerS", STR_MAX);
  strncpy(sensor[2].units[6], "uT", STR_MAX);
  strncpy(sensor[2].units[7], "uT", STR_MAX);
  strncpy(sensor[2].units[8], "uT", STR_MAX);
  
  float accelFullRange = (float) accel_scale; //ACCEL_FS_SEL 2g(00), 4g(01), 8g(10), 16g(11)
  int gyroFullRange = 1000.0;  // FS_SEL 250deg/s (0), 500 (1), 1000(2), 2000 (3)
  int magFullRange = 4800.0;  // fixed
  
  sensor[2].cal[0] = accelFullRange / 32768.0;
  sensor[2].cal[1] = accelFullRange / 32768.0;
  sensor[2].cal[2] = accelFullRange / 32768.0;
  //sensor[2].cal[3] = 1.0 / 337.87;
  sensor[2].cal[3] = gyroFullRange / 32768.0;
  sensor[2].cal[4] = gyroFullRange / 32768.0;
  sensor[2].cal[5] = gyroFullRange / 32768.0;
  sensor[2].cal[6] = magFullRange / 32768.0;
  sensor[2].cal[7] = magFullRange / 32768.0;
  sensor[2].cal[8] = magFullRange / 32768.0;
}

void logFileHeader(){
  if(File logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      logFile.println("filename,ID,version,gain (dB),Voltage,Latitude,Longitude,GPS status");
      logFile.close();
  }

  if(imuFlag){
      if(File sensorFile = sd.open("IMU.csv",  O_CREAT | O_APPEND | O_WRITE)){
      sensorFile.println("date,seconds,accel_x (g),accel_y (g),accel_z (g),gyro_x (deg/s),gyro_y (deg/s),gyro_z (deg/s),mag_x (uT),mag_y (uT),mag_z (uT)");
      sensorFile.close();
    } 
  }

}

float offsetTime, samplePeriod;
void writeSensors(){
  // IMU sensors
  offsetTime = 0.0;
  samplePeriod = 1.0 / imu_srate;
  if(File sensorFile = sd.open("IMU.csv",  O_CREAT | O_APPEND | O_WRITE)){
      //sensorFile.println("date,seconds,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,gyro_x,gyro_y,gyro_z");
      for(int i=0; i<IMUBUFFERSIZE-1; i+=9){
        sensorFile.print(sensorStartTime);
        sensorFile.print(",");
        sensorFile.print(offsetTime);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i] * sensor[2].cal[0]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+1] * sensor[2].cal[1]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+2] * sensor[2].cal[2]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+3] * sensor[2].cal[3]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+4] * sensor[2].cal[4]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+5] * sensor[2].cal[5]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+6] * sensor[2].cal[6]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+7] * sensor[2].cal[7]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+8] * sensor[2].cal[8]);
        sensorFile.print("\n");
        offsetTime += samplePeriod;
      }
      sensorFile.close();
  }
  else{
    Serial.println("Can't open IMU.csv");
  }
}

void calcSensorStats(){
  long sumAccelZ;
  float sumSquaresAccelZ;
  int counter;

  // mean of Z
  for(int i=2; i<IMUBUFFERSIZE-1; i+=9){
    sumAccelZ += imuBuffer[i];
    counter++;
  }
  float meanAccelZ = sumAccelZ / counter;

  // calc RMS of Z
  for(int i=2; i<IMUBUFFERSIZE-1; i+=9){
    sumSquaresAccelZ += ((float) imuBuffer[i] - meanAccelZ) * ((float) imuBuffer[i] - meanAccelZ);
  }
  sdAccelZmg = (sqrt(sumSquaresAccelZ / counter) * sensor[2].cal[2] * 1000.0); // mg
  
}

void FileInit()
{
   t = getTeensy3Time();
   // open file 
   sprintf(filename,"%04d%02d%02dT%02d%02d%02d.wav", year(t), month(t), day(t), hour(t), minute(t), second(t));  //filename is YYYYMMDDTHHMMSS

   // log file
   SdFile::dateTimeCallback(file_date_time);

   voltage = readVoltage();
   
   if(File logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      logFile.print(filename);
      logFile.print(',');
      for(int n=0; n<8; n++){
        logFile.print(myID[n]);
      }

      logFile.print(',');
      logFile.print(codeVersion);
      
      logFile.print(',');
      logFile.print(gainDb); 
      
      logFile.print(',');
      logFile.print(voltage); 
      
      logFile.print(',');
      logFile.print(latitude, 4);

      logFile.print(',');
      logFile.print(longitude, 4);

      logFile.print(',');
      logFile.print(goodGPS);
      
      logFile.println();
      
//      if(voltage < 3.0){
//        logFile.println("Stopping because Voltage less than 3.0 V");
//        logFile.close();  
//        // low voltage hang but keep checking voltage
//        while(readVoltage() < 3.0){
//            delay(30000);
//        }
//      }
      logFile.close();
   }
   else{
    if(printDiags) Serial.print("Log open fail.");
   }
    
   frec = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);

   if(printDiags > 0){
     Serial.println(filename);
     Serial.print("Hours rec:"); Serial.println(total_hour_recorded);
     Serial.print(voltage); Serial.println("V");
   }

   if (!frec){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
    frec = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
   }
   if(!frec) sdGood = 0; // give up on trying to write file

//    //intialize .wav file header
//    sprintf(wav_hdr.rId,"RIFF");
//    wav_hdr.rLen=36;
//    sprintf(wav_hdr.wId,"WAVE");
//    sprintf(wav_hdr.fId,"fmt ");
//    wav_hdr.fLen=0x10;
//    wav_hdr.nFormatTag=1;
//    wav_hdr.nChannels=1;
//    wav_hdr.nSamplesPerSec=audio_srate;
//    wav_hdr.nAvgBytesPerSec=audio_srate*2;
//    wav_hdr.nBlockAlign=2;
//    wav_hdr.nBitsPerSamples=16;
//    sprintf(wav_hdr.dId,"data");
//    wav_hdr.rLen = 36 + nbufs_per_file * 256 * 2;
//    wav_hdr.dLen = nbufs_per_file * 256 * 2;


    //intialize stereo .wav file header
    sprintf(wav_hdr.rId,"RIFF");
    wav_hdr.rLen=36;
    sprintf(wav_hdr.wId,"WAVE");
    sprintf(wav_hdr.fId,"fmt ");
    wav_hdr.fLen=0x10;
    wav_hdr.nFormatTag=1;
    wav_hdr.nChannels = NCHAN;
    wav_hdr.nSamplesPerSec=audio_srate;
    wav_hdr.nAvgBytesPerSec = audio_srate * 2 * NCHAN;
    wav_hdr.nBlockAlign = 2 * NCHAN;
    wav_hdr.nBitsPerSamples=16;
    sprintf(wav_hdr.dId,"data");
    wav_hdr.dLen = nbufs_per_file * 256 * 2;
    wav_hdr.rLen = 36 + wav_hdr.dLen;
  
    frec.write((uint8_t *)&wav_hdr, 44);


  if(printDiags > 0){
    Serial.print("Buffers: ");
    Serial.println(nbufs_per_file);
    Serial.print("Analyze Bufs: ");
    Serial.println(nbufs_noise_stats);
    Serial.print(audio_srate);
    Serial.println(" Hz");
    Serial.print("NCHAN: ");
    Serial.println(NCHAN);
  }
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  t = getTeensy3Time();
  *date=FAT_DATE(year(t),month(t),day(t));
  *time=FAT_TIME(hour(t),minute(t),second(t));
}


void AudioInit(int ifs){
 // Instead of using audio library enable; do custom so only power up what is needed in sgtl5000_LHI
  I2S_modification(lhi_fsamps[ifs], 16);
  Wire.begin();
  audio_enable(ifs);
 
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.0);
  sgtl5000_1.lineInLevel(gainSetting);  //default = 4
  sgtl5000_1.autoVolumeDisable();
  sgtl5000_1.audioProcessorDisable();

  setGain(); 
}

void setGain(){
   sgtl5000_1.lineInLevel(gainSetting);  //default = 4
  calcGain();
}

void calcGain(){
    switch(gainSetting){
    case 0: gainDb = -20 * log10(3.12 / 2.0); break;
    case 1: gainDb = -20 * log10(2.63 / 2.0); break;
    case 2: gainDb = -20 * log10(2.22 / 2.0); break;
    case 3: gainDb = -20 * log10(1.87 / 2.0); break;
    case 4: gainDb = -20 * log10(1.58 / 2.0); break;
    case 5: gainDb = -20 * log10(1.33 / 2.0); break;
    case 6: gainDb = -20 * log10(1.11 / 2.0); break;
    case 7: gainDb = -20 * log10(0.94 / 2.0); break;
    case 8: gainDb = -20 * log10(0.79 / 2.0); break;
    case 9: gainDb = -20 * log10(0.67 / 2.0); break;
    case 10: gainDb = -20 * log10(0.56 / 2.0); break;
    case 11: gainDb = -20 * log10(0.48 / 2.0); break;
    case 12: gainDb = -20 * log10(0.40 / 2.0); break;
    case 13: gainDb = -20 * log10(0.34 / 2.0); break;
    case 14: gainDb = -20 * log10(0.29 / 2.0); break;
    case 15: gainDb = -20 * log10(0.24 / 2.0); break;
  }
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1451606400; // Jan 1 2016
} 

void sampleSensors(void){  //interrupt at update_rate
  readImu();
  calcImu();
  if(bufferImuFull==0) incrementIMU();  
}

void incrementIMU(){
  imuBuffer[bufferposIMU] = accel_x;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = accel_y;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = accel_z;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = gyro_x;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = gyro_y;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = gyro_z;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = mag_x;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = mag_y;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = mag_z;
  bufferposIMU++;


  if(bufferposIMU>=IMUBUFFERSIZE) bufferImuFull = 1;
}
void resetFunc(void){
  CPU_RESTART
}


void read_EE(uint8_t word, uint8_t *buf, uint8_t offset)  {
  noInterrupts();
  FTFL_FCCOB0 = 0x41;             // Selects the READONCE command
  FTFL_FCCOB1 = word;             // read the given word of read once area

  // launch command and wait until complete
  FTFL_FSTAT = FTFL_FSTAT_CCIF;
  while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF))  // don't want to get stuck here
  
  *(buf+offset+0) = FTFL_FCCOB4;
  *(buf+offset+1) = FTFL_FCCOB5;       
  *(buf+offset+2) = FTFL_FCCOB6;       
  *(buf+offset+3) = FTFL_FCCOB7;       
  interrupts();
}

    
void read_myID() {
  read_EE(0xe,myID,0); // should be 04 E9 E5 xx, this being PJRC's registered OUI
  read_EE(0xf,myID,4); // xx xx xx xx

}

float readVoltage(){
   float vDivider = 2.2; //when using 3.3 V ref R9 100K
   //float vDivider = 4.5;  // when using 1.2 V ref R9 301K
   float vRef = 3.3;
   pinMode(vSense, INPUT);  // get ready to read voltage
   if (vRef==1.2) analogReference(INTERNAL); //1.2V ref more stable than 3.3 according to PJRC
   int navg = 32;
   voltage = 0;
   for(int n = 0; n<navg; n++){
    voltage += (float) analogRead(vSense);
   }
   voltage = vDivider * vRef * voltage / 1024.0 / navg;  
   pinMode(vSense, OUTPUT);  // done reading voltage
   return voltage;
}

void sensorInit(){
  Serial.println("Sensor Init");
  // IMU
  if(imuFlag){
    mpuInit(1);

    int i = 0;
    while(i<5){
      readImu();
      calcImu();
      euler();
      i++;
      Serial.print("a/g/m/t:\t");
      Serial.print(accel_x); Serial.print("\t");
      Serial.print(accel_y); Serial.print("\t");
      Serial.print(accel_z); Serial.print("\t");
      Serial.print(gyro_x); Serial.print("\t");
      Serial.print(gyro_y); Serial.print("\t");
      Serial.print(gyro_z); Serial.print("\t");
      Serial.print(mag_x); Serial.print("\t");
      Serial.print(mag_y); Serial.print("\t");
      Serial.print(mag_z); Serial.print("\t");
      Serial.println(gyro_temp);

      cDisplay();
      display.println("IMU");
      if(i<7){
        display.print("A:");
        display.print( accel_x); display.print(" ");
        display.print( accel_y); display.print(" ");
        display.println( accel_z); 
        display.print("G:");
        display.print(gyro_x); display.print(" ");
        display.print(gyro_y); display.print(" ");
        display.println(gyro_z); 
        display.print("M:");
        display.print(mag_x); display.print(" ");
        display.print(mag_y); display.print(" ");
        display.print(mag_z); display.print(" ");
      }
      else{
        display.print("P: ");
        display.println(pitch);
        display.print("R: ");
        display.println(roll);
        display.print("Y: ");
        display.print(yaw);
      }
      display.display();
      delay(200);
      }
  }
  else{
    mpuInit(0);
  }
  cDisplay();
  display.display();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void startSensors(){
  Serial.println("Start Sensors");
  bufferImuFull = 0;
  bufferposIMU = 0;
  t = getTeensy3Time();
  sensorStartTime = t;
  if(imuFlag) mpuInit(1);
  slaveTimer.begin(sampleSensors, 1000000 / update_rate); 
  slaveTimer.priority(255);
}

void stopSensors(){
  slaveTimer.end();
}

void setupWdt(){
  noInterrupts();                   // don't allow interrupts while setting up WDOG
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;   // unlock access to WDOG registers
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1);             // Need to wait a bit..

  // can only set this once
  // 10 s (72 million) = 0x44AA200 with WDOG_PRESC = 0x400
  // 596 s (9.9 minutes) = 0xFFFFFFFF with WDOG_PRESC = 0x400
  // 954 s (15.9 minutes) = 0xFFFFFFFF with WDOG_PRESC = 0x700
  WDOG_TOVALH = 0xFFFF;
  WDOG_TOVALL = 0xFFFF;

  // Tick at 7.2 MHz 0x400
  // Tick at 4.5 MHz 0x700
  WDOG_PRESC  = 0x400;

  // Set options to enable WDT. You must always do this as a SINGLE write to WDOG_CTRLH
  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
      WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
      WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  interrupts();
}

void resetWdt(){
  noInterrupts();  //   reset WDT
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}
