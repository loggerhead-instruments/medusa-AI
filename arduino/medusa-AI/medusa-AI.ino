// Loggerhead Instruments
// c 2021, David Mann

// To do:
// - remote: start recording, stop, Pi download mode (so can SSH in and see card), reboot
// - measure power consumption
// - measure waves with accelerometer
// - go through all fail scenarios and reboot contingency, including low battery power
// - WDT

// Iridium ISU module needs to be configured for 3-wire (UART) operation
// Baud 19200
// Configuration is done using serial connection (e.g. FTDI board)
// Connections: TX-TX, RX-RX, DTR-DTR, CTS-CTS, GND-SG (signal ground)
// Can use Rockblock board with their USB cable and Serial Monitor of Arduino IDE set to Carriage return

// AT&D0   (ignore DTR)
// AT&K0   (ignore CTS)
// AT&W0   (store active configuration to memory)
// AT&Y0   (designate as default reset profile)

// Commands must have a carriage return \r, not a line feed
// "AT\r"


#include <Audio.h>  //this also includes SD.h from lines 89 & 90
#include <analyze_fft256.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Snooze.h>  //using https://github.com/duff2013/Snooze; uncomment line 62 #define USE_HIBERNATE
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include <AltSoftSerial.h>
#include "IridiumSBD.h"

// 
// Dev settings
//
#define codeVersion 20210321
// #define IRIDIUM_MODEM
#define SWARM_MODEM
#define PI_PROCESSING

boolean sendSatellite = 1;
boolean useGPS = 0;  // Tile has it's own GPS, this is Ublox separate GPS module
static boolean printDiags = 1;  // 1: serial print diagnostics; 0: no diagnostics 2=verbose
long rec_dur = 30; // 3000 seconds = 50 minutes
long rec_int = 600;  // miminum is time needed for audio processing

int moduloSeconds = 10; // round to nearest start time
float hydroCal = -170;
int systemGain = 4; // SG in script file
long piTimeout = 600 ; // timeout Pi processing in seconds
boolean cardFailed = 0; // if sd card fails, skip Pi processing
char piPayload[200];  // payload to send from Pi/Coral detector

// Pin Assignments
#define hydroPowPin 8
#define vSense A14


#define iridiumAv 2 // High when Iridium network available 
#define iridiumRi 3 // Ring Indicator: active low; inactive high
#define iridiumSleep 4 // Sleep active low

#define TILE_ENABLE 3
#define TILE1 2

#define gpsEnable 5
#define infraRed 6

#define POW_5V 15
#define SD_POW 16
#define SD_SWITCH 17
#define PI_STATUS A10
#define PI_STATUS2 A11

#define SD_TEENSY LOW
#define SD_PI HIGH

AltSoftSerial gpsSerial;  // RX 20; Tx: 21

#ifdef IRIDIUM_MODEM
  IridiumSBD modem(Serial1, iridiumSleep);
#endif
int sigStrength;

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);
#define BOTTOM 55

static uint8_t myID[8];
unsigned long baud = 115200;

// GUItool: begin automatically generated code
AudioInputI2S            i2s2;           //xy=105,63
AudioAnalyzeFFT256       fft256_1; 
AudioRecordQueue         queue1;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioConnection          patchCord2(i2s2, 0, fft256_1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

const int myInput = AUDIO_INPUT_LINEIN;
float gainDb;
int noDC = 0; // 0 = freezeDC offset; 1 = remove DC offset

// Pins used by audio shield
// https://www.pjrc.com/store/teensy3_audio.html
// MEMCS 6
// MOSI 7
// BCLK 9
// SDCS 10
// MCLK 11
// MISO 12
// RX 13
// SCLK 14
// VOL 15
// SDA 18
// SCL 19
// TX 22
// LRCLK 23

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording audio, 2=recprding sensors
time_t startTime;
time_t stopTime;
time_t t;
time_t sensorStartTime;


boolean audioFlag = 1;
volatile boolean LEDSON = 1;
boolean introPeriod=1;  //flag for introductory period; used for keeping LED on for a little while

int update_rate = 10;  // rate (Hz) at which interrupt to read P/T sensors will run, so sensor_srate needs to <= update_rate
float sensor_srate = 10.0;
float imu_srate = 10.0;
volatile int updateSensorCounter = 0;
float audio_srate = 44100.0;

int accel_scale = 16; //full scale on accelerometer [2, 4, 8, 16] (example cmd code: AS 8)

float audioIntervalSec = 256.0 / audio_srate; //buffer interval in seconds
unsigned int audioIntervalCount = 0;

int wakeahead = 30;  //wake from snooze to give hydrophone time to power up; needs to be longer than sensor sampling duration
int snooze_hour;
int snooze_minute;
int snooze_second;
volatile long buf_count;
float total_hour_recorded = 0.0;
unsigned long nbufs_per_file;
boolean settingsChanged = 0;

uint16_t file_count;
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
#define IMUBUFFERSIZE 300 // store 10 s at 10 Hz (3 axes)
volatile int16_t imuBuffer[IMUBUFFERSIZE]; // buffer used to store IMU sensor data before writes in samples
volatile int bufferposIMU = 0;
volatile boolean bufferImuFull;
volatile uint8_t imuTempBuffer[20];
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;

float voltage;

IntervalTimer slaveTimer;

// GPS
float latitude = 0.0;
float longitude = 0.0;
char latHem, lonHem;
int gpsYear = 19, gpsMonth = 2, gpsDay = 4, gpsHour = 22, gpsMinute = 5, gpsSecond = 0;
int goodGPS = 0;
long gpsTimeOutThreshold = 120000;

// define bands to measure acoustic signals
int fftPoints = 256; // 5.8 ms at 44.1 kHz
float binwidth = audio_srate / fftPoints; //256 point FFT; = 172.3 Hz for 44.1kHz
float fftDurationMs = 1000.0 / binwidth;
long fftCount;
#define NBANDS 10
float meanBand[NBANDS]; // mean band valuesd
int bandLow[NBANDS]; // band low frequencies
int bandHigh[NBANDS];
int nBins[NBANDS]; // number of FFT bins in each band
String dataPacket; // data packed for transmission after each file

void setup() {
  read_myID();

  bandLow[0] = 1; // start at 172 Hz to eliminate wave noise
  bandHigh[0] = 2;
  
  bandLow[1] = 2; // 344
  bandHigh[1] = 3;
  
  bandLow[2] = 3; // 516
  bandHigh[2] = 4;
  
  bandLow[3] = 4; // 688
  bandHigh[3] = 5;

  bandLow[4] = 5; // 860
  bandHigh[4] = 6;

  bandLow[5] = 6; // 1032
  bandHigh[5] = 7;

  bandLow[6] = 7; // 1204
  bandHigh[6] = 8;
  
  bandLow[7] = bandHigh[6]; // 1376
  bandHigh[7] = (int) 2000 / binwidth;
  
  bandLow[8] = (int) bandHigh[7];
  bandHigh[8] = (int) 5000 / binwidth;
  
  bandLow[9] = bandHigh[8];
  bandHigh[9] = (int) 20000 / binwidth;
  
  for(int i=0; i<NBANDS; i++){
    nBins[i] = bandHigh[i] - bandLow[i];
  }
  
  Serial.begin(baud);

  pinMode(POW_5V, OUTPUT);
  digitalWrite(POW_5V, LOW);
  pinMode(SD_POW, OUTPUT);
  digitalWrite(SD_POW, HIGH);
  pinMode(SD_SWITCH, OUTPUT);
  digitalWrite(SD_SWITCH, SD_TEENSY); 
  pinMode(PI_STATUS, INPUT);
  pinMode(PI_STATUS2, INPUT);

  pinMode(TILE_ENABLE, OUTPUT);
  digitalWrite(TILE_ENABLE, HIGH);

  delay(500);
  Wire.begin();
 // Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
 // Wire.setDefaultTimeout(10000);

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
  display.display();

  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(sd.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
      cDisplay();
      display.println();
      display.println("SD error");
      display.display();
      while(1);
      
    }
  sensorInit(); // initialize and test sensors; GPS and Iridium should be after this


  #ifdef IRIDIUM_MODEM
  if(sendSatellite){
    Serial1.begin(19200, SERIAL_8N1);  //Iridium
    modem.getSignalQuality(sigStrength); // update Iridium modem strength
    modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
    int result = modem.begin();
  }
  #endif

  #ifdef SWARM_MODEM
    Serial.println("SWARM Get GPS");
    display.println("Get Swarm GPS");
    display.display();
    Serial1.begin(115200, SERIAL_8N1);
    delay(1000);
    while(!goodGPS){
      delay(1000);
      pollTile(); // print tile messages 
      Serial1.println("$DT @*70");  // get dt
      delay(100);
      pollTile();
      Serial1.println("$GN @*69");
      delay(100);
      pollTile(); // print tile messages
    }
    setTeensyTime(gpsHour, gpsMinute, gpsSecond, gpsDay, gpsMonth, gpsYear);
  #endif

  
  if(useGPS){
    while(!goodGPS){
      gpsGetTimeLatLon();
      if(!goodGPS){
        Serial.println("Unable to get GPS");
        cDisplay();
        display.println();
        display.println("Wait for GPS");
        display.println("Do not deploy");
        display.display();
        delay(2000);
      }
    }
    setTeensyTime(gpsHour, gpsMinute, gpsSecond, gpsDay, gpsMonth, gpsYear + 2000);
  }

  
  
  cDisplay();
  display.setCursor(0,30);
  display.print("Lat: ");
  display.println(latitude);
  display.print("Lon: ");
  display.print(longitude);
  display.display();

  logFileHeader();

  #ifdef IRIDIUM_MODEM
    if(sendSatellite) modem.sleep();
  #endif

  digitalWrite(hydroPowPin, HIGH);

  setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time
  
// ULONG newtime;
 
  // Power down USB if not using Serial monitor
  if (printDiags==0){
    //  usbDisable();
  }
  
  //SdFile::dateTimeCallback(file_date_time);

  setupDataStructures();

  t = getTeensy3Time();
  startTime = t;
  startTime -= startTime % moduloSeconds;  //modulo to nearest modulo seconds
  startTime += moduloSeconds; //move forward
  stopTime = startTime + rec_dur;  // this will be set on start of recording
  
  nbufs_per_file = (long) (rec_dur * audio_srate / 256.0);

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  
  AudioMemory(100);
  AudioInit(); // this calls Wire.begin() in control_sgtl5000.cpp
  mode = 0;

  // create first folder to hold data
  folderMonth = -1;  //set to -1 so when first file made will create directory
}

//
// MAIN LOOP
//

int recLoopCount;  //for debugging when does not start record

void loop() {
  t = getTeensy3Time();

  // record sensors mode
  if(mode==2){
    if(bufferImuFull){
      stopSensors();
      writeSensors();
      calcSensorStats();
      mode = 0;
    }
  }
  
  // Standby mode
  if(mode == 0)
  {
    delay(100);
    #ifdef SWARM_MODEM
      if(!goodGPS){
        pollTile(); // print tile messages 
        Serial1.println("$DT @*70");  // get dt
        delay(200);
        pollTile();
        Serial1.println("$GN @*69");
        delay(200);
        pollTile(); // print tile messages
      }
    setTeensyTime(gpsHour, gpsMinute, gpsSecond, gpsDay, gpsMonth, gpsYear);
    #endif
    if(useGPS){
      if(!goodGPS){
        gpsTimeOutThreshold = 20000;//give 20 seconds to read
        gpsGetTimeLatLon();  
      }
      else{
        digitalWrite(gpsEnable, LOW); // turn off once have good GPS
      }
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
      
      stopTime = startTime + rec_dur;
      startTime = stopTime + rec_int;

      Serial.print("Current Time: ");
      printTime(getTeensy3Time());
      Serial.print("Stop Time: ");
      printTime(stopTime);
      Serial.print("Next Start:");
      printTime(startTime);

      displayOff();

      mode = 1;
      startRecording();
      digitalWrite(gpsEnable, LOW);
    }
  }

  // Record mode
  if (mode == 1) {
    continueRecording();  // download data 

  //
  // Automated signal processing
  //
  if(fft256_1.available()){
    // calculate band level noise
    fftCount += 1;  // counter to divide meanBand by before sending
    for(int n=0; n<NBANDS; n++){
      for(int i=bandLow[n]; i<bandHigh[n]; i++){
        meanBand[n] += (fft256_1.read(i) / nBins[n]); // accumulate across band
      }
    }
  }
 
    if(buf_count >= nbufs_per_file){       // time to stop?
      total_hour_recorded += (float) rec_dur / 3600.0;
      if(total_hour_recorded > 0.1) introPeriod = 0;  //LEDS on for first file
      stopRecording();
      goodGPS = 0;
      if(introPeriod){
        displayOn();
        cDisplay();
        display.println("Booting");
        display.display();
      }

      //
      // Process audio with Pi
      //
      #ifdef PI_PROCESSING
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
        }
  
        if(introPeriod){
          displayOn();
          cDisplay();
          display.println("Processing");
          display.display();
        }
  
        // wait for Pi to finish processing or timeout
        // PI_STATUS2 around 1016-1017 during processing. 0-2 when done and Pi about to shut down.
        startPiTime = getTeensy3Time();
        t = startPiTime;
        int piStatus2 = analogRead(PI_STATUS2);
        Serial.println("PI_STATUS2 "); 
        while((t - startPiTime < piTimeout) & (piStatus2 > 200)){
          t = getTeensy3Time();
          piStatus2 = analogRead(PI_STATUS2);
          Serial.println(piStatus2);
          delay(2000);
        }
  
        if(introPeriod){
          displayOn();
          cDisplay();
          display.println("Wait on Pi");
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
        }while((piStatus<20) | (piStatus>1000) & (t - startPiTime < piTimeout));
      #endif

      digitalWrite(SD_POW, LOW); // switch off power to microSD (Pi will use SD mode, so card needs to reset)
      digitalWrite(SD_SWITCH, SD_TEENSY); // switch control to Teensy
      delay(1000);
      digitalWrite(SD_POW, HIGH); // power on microSD
      delay(100);

      int sdAttempts = 0;
      if (!(sd.begin(10)) & sdAttempts < 2000) {
        digitalWrite(SD_POW, LOW);
        delay(1000);
        digitalWrite(SD_POW, HIGH);
        delay(1000);
        Serial.println("SD restart failed");
      }

      if(sdAttempts>=10) cardFailed = 1;

      // read detections file
      if(!cardFailed) readDetections();
      
      makeDataPacket();
      if(introPeriod) {
        cDisplay();
        display.println("Payload");
        display.println();
        display.println(dataPacket);
        display.display();
      }
      
      #ifdef IRIDIUM_MODEM
        // IRIDIUM
        if(sendSatellite){
          modem.begin();  // wake Iridium
          modem.adjustSendReceiveTimeout(120);  // timeout in 120 seconds
          if(introPeriod) displayOn();
          int err = sendDataPacket();            
          modem.sleep();
        }
        //
      #endif

      digitalWrite(POW_5V, LOW); // power off Pi and Iridium

      #ifdef SWARM_MODEM
        // SWARM
        if(sendSatellite){
          Serial.println("Send SWARM Data Packet");
          if(introPeriod) displayOn();
          int err = sendDataPacket();  
          delay(1000);
          pollTile();      
          delay(1000);
          pollTile();      
        }
        //
      #endif
      
      
      resetSignals();
      goodGPS = 0;
      delay(1000); // time to read display
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
            printTime(getTeensy3Time());
            Serial.print("Snooze HH MM SS ");
            Serial.print(snooze_hour);
            Serial.print(snooze_minute);
            Serial.println(snooze_second);
          }
          delay(100);

          alarm.setRtcTimer(snooze_hour, snooze_minute, snooze_second);
          Snooze.hibernate(config_teensy32);
     
          /// ... Sleeping ....
          
          // Waking up
          
         if(printDiags>0) printTime(getTeensy3Time());
         digitalWrite(hydroPowPin, HIGH); // hydrophone on 
         AudioInit();
       }
      if(introPeriod) displayOn();
      mode = 0;
      startSensors();
      if(useGPS) digitalWrite(gpsEnable, HIGH); // wake up to get GPS in standby mode
    }
  }
  asm("wfi"); // reduce power between interrupts
}


void startRecording() {
  Serial.println("startRecording");
  FileInit();
  buf_count = 0;
  queue1.begin();
  Serial.println("Queue Begin");
}

void continueRecording() {
    byte buffer[512];
    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
    // one buffer is 512 bytes = 256 samples
    if(queue1.available() >= 2) {
      buf_count += 1;
      audioIntervalCount += 1;
      memcpy(buffer, queue1.readBuffer(), 256);
      queue1.freeBuffer();
      memcpy(buffer+256, queue1.readBuffer(), 256);
      queue1.freeBuffer();
      frec.write(buffer, 512); //audio to .wav file
    }
}

void stopRecording() {
  Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  Serial.print("Audio Memory Max");
  Serial.println(maxblocks);
  byte buffer[512];
  queue1.end();

  AudioMemoryUsageMaxReset();
  frec.close();
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
  sensor[0].cal[0] = gainDb + hydroCal; // this needs to be set based on hydrophone sensitivity + chip gain
  sensor[0].cal[1] = gainDb + hydroCal;
  sensor[0].cal[2] = gainDb + hydroCal;
  sensor[0].cal[3] = gainDb + hydroCal;

  // IMU
  strncpy(sensor[2].chipName, "ADXL343", STR_MAX);
  sensor[2].nChan = 9;
  strncpy(sensor[2].name[0], "accelX", STR_MAX);
  strncpy(sensor[2].name[1], "accelY", STR_MAX);
  strncpy(sensor[2].name[2], "accelZ", STR_MAX);
  strncpy(sensor[2].units[0], "g", STR_MAX);
  strncpy(sensor[2].units[1], "g", STR_MAX);
  strncpy(sensor[2].units[2], "g", STR_MAX);
  
  float accelFullRange = (float) accel_scale; //ACCEL_FS_SEL 2g(00), 4g(01), 8g(10), 16g(11)
  
  sensor[2].cal[0] = accelFullRange / 32768.0;
  sensor[2].cal[1] = accelFullRange / 32768.0;
  sensor[2].cal[2] = accelFullRange / 32768.0;
}

void logFileHeader(){
  if(File logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      logFile.println("filename,ID,version,gain (dB),Voltage,mBar Offset,Latitude,Longitude,GPS status");
      logFile.close();
  }

  if(File sensorFile = sd.open("IMU.csv",  O_CREAT | O_APPEND | O_WRITE)){
      sensorFile.println("date,seconds,accel_x (g),accel_y (g),accel_z (g),gyro_x (deg/s),gyro_y (deg/s),gyro_z (deg/s),mag_x (uT),mag_y (uT),mag_z (uT)");
      sensorFile.close();
  }
}

float offsetTime, samplePeriod;
void writeSensors(){
  // IMU sensors
  offsetTime = 0.0;
  samplePeriod = 1.0 / imu_srate;
  if(File sensorFile = sd.open("IMU.csv",  O_CREAT | O_APPEND | O_WRITE)){
      //sensorFile.println("date,seconds,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,gyro_x,gyro_y,gyro_z");
      for(int i=0; i<IMUBUFFERSIZE-1; i+=3){
        sensorFile.print(sensorStartTime);
        sensorFile.print(",");
        sensorFile.print(offsetTime);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i] * sensor[2].cal[0]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+1] * sensor[2].cal[1]);
        sensorFile.print(",");
        sensorFile.print((float)imuBuffer[i+2] * sensor[2].cal[2]);
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

//  // mean of Z
//  for(int i=2; i<IMUBUFFERSIZE-1; i+=9){
//    sumAccelZ += imuBuffer[i];
//    counter++;
//  }
//  float meanAccelZ = sumAccelZ / counter;
//
//  // calc RMS of Z
//  for(int i=2; i<IMUBUFFERSIZE-1; i+=9){
//    sumSquaresAccelZ += ((float) imuBuffer[i] - meanAccelZ) * ((float) imuBuffer[i] - meanAccelZ);
//  }
//  sdAccelZmg = (int) (sqrt(sumSquaresAccelZ / counter) * sensor[2].cal[2] * 1000); // mg
//  
}

void FileInit()
{
   t = getTeensy3Time();
   if (folderMonth != month(t)){
    if(printDiags > 0) Serial.println("New Folder");
    folderMonth = month(t);
    sprintf(dirname, "%04d-%02d", year(t), folderMonth);
    SdFile::dateTimeCallback(file_date_time);
    sd.mkdir(dirname);
   }

   // only audio save as wav file, otherwise save as AMX file
   
   // open file 
   sprintf(filename,"%s/%02d%02d%02d%02d.wav", dirname, day(t), hour(t), minute(t), second(t));  //filename is DDHHMM

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
      
      if(voltage < 3.0){
        logFile.println("Stopping because Voltage less than 3.0 V");
        logFile.close();  
        // low voltage hang but keep checking voltage
        while(readVoltage() < 3.0){
            delay(30000);
        }
      }
      logFile.close();
   }
   else{
    if(printDiags) Serial.print("Log open fail.");
    resetFunc();
   }
    
   frec = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);

   if(printDiags > 0){
     Serial.println(filename);
     Serial.print("Hours rec:"); Serial.println(total_hour_recorded);
     Serial.print(voltage); Serial.println("V");
   }
   
   while (!frec){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
    frec = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
   }

    //intialize .wav file header
    sprintf(wav_hdr.rId,"RIFF");
    wav_hdr.rLen=36;
    sprintf(wav_hdr.wId,"WAVE");
    sprintf(wav_hdr.fId,"fmt ");
    wav_hdr.fLen=0x10;
    wav_hdr.nFormatTag=1;
    wav_hdr.nChannels=1;
    wav_hdr.nSamplesPerSec=audio_srate;
    wav_hdr.nAvgBytesPerSec=audio_srate*2;
    wav_hdr.nBlockAlign=2;
    wav_hdr.nBitsPerSamples=16;
    sprintf(wav_hdr.dId,"data");
    wav_hdr.rLen = 36 + nbufs_per_file * 256 * 2;
    wav_hdr.dLen = nbufs_per_file * 256 * 2;
  
    frec.write((uint8_t *)&wav_hdr, 44);
    
  if(printDiags > 0){
    Serial.print("Buffers: ");
    Serial.println(nbufs_per_file);
  }
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  t = getTeensy3Time();
  *date=FAT_DATE(year(t),month(t),day(t));
  *time=FAT_TIME(hour(t),minute(t),second(t));
}


void AudioInit(){
 // Instead of using audio library enable; do custom so only power up what is needed in sgtl5000_LHI
  audio_power_up();
  delay(10); // give some time to power up circuits
  audio_enable();
 
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.0);
  sgtl5000_1.lineInLevel(systemGain);  //default = 4
  sgtl5000_1.autoVolumeDisable();
  sgtl5000_1.audioProcessorDisable();

  setGain(); 
}

void setGain(){
    sgtl5000_1.lineInLevel(systemGain);  //default = 4
  // CHIP_ANA_ADC_CTRL
  // Actual measured full-scale peak-to-peak sine wave input for max signal
  //  0: 3.12 Volts p-p
  //  1: 2.63 Volts p-p
  //  2: 2.22 Volts p-p
  //  3: 1.87 Volts p-p
  //  4: 1.58 Volts p-p (0.79 Vpeak)
  //  5: 1.33 Volts p-p
  //  6: 1.11 Volts p-p
  //  7: 0.94 Volts p-p
  //  8: 0.79 Volts p-p (+8.06 dB)
  //  9: 0.67 Volts p-p
  // 10: 0.56 Volts p-p
  // 11: 0.48 Volts p-p
  // 12: 0.40 Volts p-p
  // 13: 0.34 Volts p-p
  // 14: 0.29 Volts p-p
  // 15: 0.24 Volts p-p
  //sgtl5000_1.autoVolumeDisable();
 // sgtl5000_1.audioProcessorDisable();
  switch(systemGain){
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
  updateSensorCounter++;
  
//  if(updateSensorCounter>=(1.0 / sensor_srate) * update_rate){
//      updateSensorCounter = 0;
//      readImu();
//      calcImu();
//      if(bufferImuFull==0) incrementIMU();  
//  }
}

void incrementIMU(){
  imuBuffer[bufferposIMU] = accel_x;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = accel_y;
  bufferposIMU++;
  imuBuffer[bufferposIMU] = accel_z;
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
  while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF))
    ;
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
   float vDivider = 2.1; //when using 3.3 V ref R9 100K
   //float vDivider = 4.5;  // when using 1.2 V ref R9 301K
   float vRef = 3.3;
   pinMode(vSense, INPUT);  // get ready to read voltage
   if (vRef==1.2) analogReference(INTERNAL); //1.2V ref more stable than 3.3 according to PJRC
   int navg = 32;
   for(int n = 0; n<navg; n++){
    voltage += (float) analogRead(vSense);
   }
   voltage = vDivider * vRef * voltage / 1024.0 / navg;  
   pinMode(vSense, OUTPUT);  // done reading voltage
   return voltage;
}

void sensorInit(){
  pinMode(hydroPowPin, OUTPUT);
  digitalWrite(hydroPowPin, HIGH);
  pinMode(vSense, INPUT);
  analogReference(DEFAULT); 

  #ifdef IRIDIUM_MODEM
    pinMode(iridiumAv, INPUT);
    pinMode(iridiumRi, INPUT);
    pinMode(iridiumSleep, OUTPUT);
    digitalWrite(iridiumSleep, HIGH); // HIGH = enabled; LOW = sleeping
  #endif

  pinMode(gpsEnable, OUTPUT);
  digitalWrite(gpsEnable, HIGH);  // HIGH = enabled; LOW = Sleep
  pinMode(infraRed, INPUT);
  cDisplay();
  display.display();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void gpsGetTimeLatLon(){
    // get GPS
  int incomingByte;
  long gpsTimeOutStart = millis();

  if(introPeriod){
    cDisplay();
    display.println("GPS");
    display.println();
    display.println("Searching...");
    display.display();
  }
  goodGPS = 0;
  gpsSerial.begin(9600);
  Serial.println("GPS Serial On");
  delay(100);
  gpsSpewOn();
  Serial.println("Spew On");
  
  // can't display GPS data here, because display slows things down too much
  while((!goodGPS) & (millis()-gpsTimeOutStart<gpsTimeOutThreshold)){
    while (gpsSerial.available() > 0) {    
        incomingByte = gpsSerial.read();
        Serial.write(incomingByte);
        gps(incomingByte);  // parse incoming GPS data
    }
  }

  Serial.print("GPS search time:");
  Serial.println(millis()-gpsTimeOutStart);
  gpsSpewOff();
  delay(200);
  gpsSerial.end();
  Serial.print("Good GPS:");
  Serial.println(goodGPS);
}

void startSensors(){
  Serial.println("Start Sensors");
  bufferImuFull = 0;
  bufferposIMU = 0;
  t = getTeensy3Time();
  sensorStartTime = t;
  slaveTimer.begin(sampleSensors, 1000000 / update_rate); 
  slaveTimer.priority(255);
}

void stopSensors(){
  slaveTimer.end();
}

void pollTile(){
//  $DT 20210316170104,V*4c
//  $GN 27.2594,-82.4798,-3,0,2*1f

  while(Serial1.available()){
    byte incomingByte = Serial1.read();
    parseTile(incomingByte);
    Serial.write(incomingByte);
  }
}


#define maxChar 256
char gpsStream[maxChar];
int streamPos;

void parseTile(byte incomingByte){
  char rmcDate[25];
  // check for start of new message
  // if a $, start it at Pos 0, and continue until next G
  if(incomingByte=='$') {
    Serial.print("String position:");
    Serial.println(streamPos);
    Serial.println(gpsStream);
    //process last message
    if(streamPos > 10){
      float rmcLat; //          
      float rmcLon; //           
      float tileAlt;
      float tileCourse;
      float tileSpeed;
      uint8_t rmcChecksum; 

      // $DT 20210316170059,V*45
      if(gpsStream[1]=='D' & gpsStream[2]=='T'){
       char temp[streamPos + 1];
       const char s[2] = ",";
       char *token;
            
        memcpy(&temp, &gpsStream[4], 14);
        Serial.print("Date string extracted:"); Serial.println(temp);
        sscanf(temp, "%4i%2i%2i%2i%2i%2i", &gpsYear, &gpsMonth, &gpsDay, &gpsHour, &gpsMinute, &gpsSecond);
//        Serial.println(rmcDate);
//        Serial.print("Day-Month-Year:");
//        Serial.print(gpsDay); Serial.print("-");
//        Serial.print(gpsMonth);  Serial.print("-");
//        Serial.print(gpsYear);
//        Serial.print(" ");
//        Serial.print(gpsHour); Serial.print(":");
//        Serial.print(gpsMinute); Serial.print(":");
//        Serial.println(gpsHour);
      }

      if(gpsStream[1]=='G' & gpsStream[2]=='N'){
       char temp[streamPos + 1];
       const char s[2] = ",";
       char *token;
            
        memcpy(&temp, &gpsStream[4], streamPos - 4);
        // 27.2594,-82.4798,-3,0,2*1f
//        Serial.print("GPS String extracted:"); Serial.println(temp);
        sscanf(temp, "%f,%f,%f,%f,%f*%2hhx",&rmcLat, &rmcLon, &tileAlt, &tileCourse, &tileSpeed, &rmcChecksum);
//        Serial.print("Lat:"); Serial.println(rmcLat);
//        Serial.print("Lon:"); Serial.println(rmcLon);
//        Serial.print("Checksum:");
//        Serial.println(rmcChecksum, HEX);     

        memcpy(&temp, &gpsStream[1], streamPos - 5);
//        Serial.print("Calculated Checksum: ");
//        Serial.println(nmeaChecksum(&temp[0], streamPos-5), HEX);     

        if(nmeaChecksum(&temp[0], streamPos-5) == rmcChecksum){
           latitude = rmcLat;
           longitude = rmcLon;
           goodGPS = 1;
           Serial.println("valid GPS recvd");
        }
      }
    }
    // start new message here
    streamPos = 0;
  }
  gpsStream[streamPos] = incomingByte;
  streamPos++;
  if(streamPos >= maxChar) streamPos = 0;
}
