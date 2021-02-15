// Loggerhead Instruments
// c 2020, David Mann

// piTest will start Pi immediately and leave it running
// monitors PI_STATUS and PI_STATUS2

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
#define codeVersion 20210210
#define IRIDIUM_MODEM

boolean sendIridium = 0;
boolean useGPS = 0;
static boolean printDiags = 1;  // 1: serial print diagnostics; 0: no diagnostics 2=verbose
long rec_dur = 30; // seconds
long rec_int = 60;  // miminum is 60
long accumulationInterval = 2 * 60 * 60; //seconds to accumulate results
int moduloSeconds = 10; // round to nearest start time
float hydroCal = -170;
int systemGain = 4; // SG in script file
long piTimeout = 600 ; // timeout Pi processing in seconds
boolean cardFailed = 0; // if sd card fails, skip Pi processing

// Pin Assignments
#define hydroPowPin 8
#define vSense A14
#define iridiumAv 2 // High when Iridium network available 
#define iridiumRi 3 // Ring Indicator: active low; inactive high
#define iridiumSleep 4 // Sleep active low
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
  int sigStrength;
#endif

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
long nbufs_per_file;
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
String dataPacket; // data packed for Particle transmission after each file


void setup() {
  Serial.begin(115200);
  pinMode(POW_5V, OUTPUT);
  digitalWrite(POW_5V, LOW);
  pinMode(SD_POW, OUTPUT);
  pinMode(SD_SWITCH, OUTPUT);
  digitalWrite(SD_SWITCH, SD_PI); 
  digitalWrite(SD_POW, HIGH);
  pinMode(PI_STATUS, INPUT);
  pinMode(PI_STATUS2, INPUT);
  
  // Process audio with Pi
  digitalWrite(POW_5V, HIGH); // power on Pi


}

void loop() {
  Serial.print(analogRead(PI_STATUS));
  Serial.print(" ");
  Serial.println(analogRead(PI_STATUS2));
  delay(1000);

}
