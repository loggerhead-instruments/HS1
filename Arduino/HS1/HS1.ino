//
// HS1: High speed audio recorder
//
// Loggerhead Instruments
// c 2019
// David Mann

// Compile 180 MHz Faster
// current draw: about 80 mA at 350 kHz. 78 mA at 240 kHz

// To Do:
// - test sample rates; make sure keeps recording, and sample rate is correct

#define codeVersion 20191215
#define MQ 200 // to be used with LHI record queue (modified local version)

#include "input_i2s.h"
#include <i2c_t3.h>  //https://github.com/nox771/i2c_t3; Teensy Audio: control_sgtl5000.cpp needs to have Wire.h commented
#include <SPI.h>
#include "SdFat.h"  // https://github.com/greiman/SdFat-beta

#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <EEPROM.h>

#include "LHI_record_queue.h"
#include "control_sgtl5000.h"
#include <analyze_fft1024.h>
#include <analyze_fft256.h>
// 
// Dev settings
//

static boolean printDiags = 0;  // 1: serial print diagnostics; 0: no diagnostics 2=verbose
int moduloSeconds = 5; // round to nearest start time
int wakeahead = 5;   // want this to be about the same as moduloSeconds

float hydroCalLeft = -180.0;
float hydroCalRight = -180.0;
#define FFT1024 1024
//#define FFT256 256

#define NCHAN 1
#define ONECHAN 1
//#define TWOCHAN 2
//

//
// EEPROM SETTINGS -- THESE ONLY TAKE EFFECT FOR NEW HS1
//
int isf = 7; // index sampling frequency
long rec_dur = 30; // seconds
long rec_int = 300 - rec_dur;  // seconds
int gainSetting = 4; // SG in script file
int noDC = 0; // 0 = freezeDC offset; 1 = remove DC offset
#define SAMP_FREQS 20
// Keep 32, 44.1, and 48 kHz because of how SGTL5000 setup in audio_enable()
int32_t lhi_fsamps[SAMP_FREQS] = {32000, 44100, 48000, 96000, 192000, 240000, 256000, 260000, 270000, 275000, 280000, 288000, 290000, 300000, 310000, 320000, 330000, 340000, 350000, 384000};
float audio_srate;
float sumPeakVal; // for debugging noise levels
long sumCount;
//
// ********************************************************************************************************//
//


// Pin Assignments
const int UP = 4;
const int DOWN = 3;
const int SELECT = 8;
const int hydroPowPin = 2;
const int vSense = 16; 
#define menuStatus 31
#define menuReset 32


#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define OLED_RESET -1

#define displayLine1 0
#define displayLine2 11
#define displayLine3 22
#define displayLine4 33
#define displayLine5 44
#define displayLine6 57
Adafruit_FeatherOLED display = Adafruit_FeatherOLED();


static uint8_t myID[8];

unsigned long baud = 115200;

#ifdef TWOCHAN
  #ifdef FFT1024
    int fftPoints = 1024;
    #define FFT1 fft1024_1
    #define FFT2 fft1024_2
    
    // GUItool: begin automatically generated code
    AudioInputI2S            i2s2;           //xy=262,190
  //  AudioAnalyzeFFT1024       FFT1;       //xy=518,130
  //  AudioAnalyzeFFT1024       FFT2;       //xy=518,130
    LHIRecordQueue           queue1;         //xy=281,63
    LHIRecordQueue           queue2;         //xy=281,63
   // AudioConnection          patchCord1(i2s2, 0, FFT1, 0);
    AudioConnection          patchCord2(i2s2, 0, queue1, 0);
   // AudioConnection          patchCord3(i2s2, 1, FFT2, 0);
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
  //  AudioAnalyzeFFT256       fft256_1;       //xy=518,130
  //  AudioAnalyzeFFT256       fft256_2;       //xy=518,130
    LHIRecordQueue           queue1;         //xy=281,63
    LHIRecordQueue           queue2;         //xy=281,63
   // AudioConnection          patchCord1(i2s2, 0, FFT1, 0);
    AudioConnection          patchCord2(i2s2, 0, queue1, 0);
  //  AudioConnection          patchCord3(i2s2, 1, FFT2, 0);
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
   // AudioAnalyzeFFT1024       FFT1;       //xy=518,130
    LHIRecordQueue           queue1;         //xy=281,63
   // AudioConnection          patchCord1(i2s2, 0, FFT1, 0);
    AudioConnection          patchCord2(i2s2, 0, queue1, 0);
    AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
    // GUItool: end automatically generated code
  #endif
  
  #ifdef FFT256
    int fftPoints = 256;
    // GUItool: begin automatically generated code
    #define FFT1 fft256_1
    AudioInputI2S            i2s2;           //xy=262,190
 //   AudioAnalyzeFFT256       FFT1;       //xy=518,130
    LHIRecordQueue           queue1;         //xy=281,63
  //  AudioConnection          patchCord1(i2s2, 0, FFT1, 0);
    AudioConnection          patchCord2(i2s2, 0, queue1, 0);
    AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
    // GUItool: end automatically generated code
  #endif
#endif


const int myInput = AUDIO_INPUT_LINEIN;
float gainDb;


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
int modeHS1 = 0;  // 0=stopped, 1=recording audio, 2=recording sensors
time_t startTime;
time_t stopTime;
time_t t;
time_t autoStartTime;

boolean audioFlag = 1;
volatile boolean LEDSON = 1;
boolean introPeriod=1;  //flag for introductory period; used for keeping LED on for a little while

int snooze_hour;
int snooze_minute;
int snooze_second;
volatile long buf_count;
float total_hour_recorded = 0.0;
long nbufs_per_file;
boolean settingsChanged = 0;

byte startHour, startMinute, endHour, endMinute; //used in Diel modeHS1

#define MODE_NORMAL 0
#define MODE_DIEL 1
int recMode = MODE_NORMAL;

int nBatPacks = 8;

long file_count;
char filename[50];

// The file where data is recorded
// Use built-in SD for SPI modes on Teensy 3.5/3.6.
// Teensy 4.0 use first SPI port.
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE


// get ~18MB/sec even with short 512 byte writes
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
float voltage;

// define bands to measure acoustic signals
// these need to be recalculated if the sample rate changes from settings
float binwidth = audio_srate / fftPoints; //256 point FFT; = 172.3 Hz for 44.1kHz
float fftDurationMs = 1000.0 / binwidth;
long fftCount;


void setup() {
  read_myID(); 
  Serial.begin(baud);

  delay(500);
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire.setDefaultTimeout(10000);

  //setup display and controls
  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);
  pinMode(SELECT, INPUT_PULLUP);
  pinMode(menuStatus, INPUT);
  pinMode(menuReset, OUTPUT);
  pinMode(hydroPowPin, OUTPUT);
  digitalWrite(hydroPowPin, HIGH);
  digitalWrite(menuReset, LOW);
  
  readVoltage();
  displayOn();
  cDisplay();
  display.println("Loggerhead");
  display.display();

  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(sd.begin(SdioConfig(FIFO_SDIO)))) {
    // stop here if no SD card, but print a message
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
  // make sd the current volume.
    sd.chvol();  

  readEEPROM();
  calcGain();
  if(digitalRead(menuStatus)==1) manualSettings();  // only get manual settings after power on
  digitalWrite(menuReset, HIGH); // reset flip-flop so only runs menu once
  rec_int = 10;
  logFileHeader();

 

  setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time
  
// ULONG newtime;
 
  // Power down USB if not using Serial monitor
  if (printDiags==0){
    //  usbDisable();
  }

  t = getTeensy3Time();
  startTime = t;
   startTime -= startTime % moduloSeconds;  //modulo to nearest modulo seconds
   startTime += moduloSeconds; //move forward
  stopTime = startTime + rec_dur;  // this will be set on start of recording

  audio_srate = lhi_fsamps[isf];
  nbufs_per_file = (long) (rec_dur * audio_srate / 256.0) * NCHAN;
  binwidth = audio_srate / fftPoints; //256 point FFT; = 172.3 Hz for 44.1kHz
  fftDurationMs = 1000.0 / binwidth;

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  
  AudioMemory(MQ+10);

  AudioInit(0); // this calls Wire.begin() in control_sgtl5000.cpp
  modeHS1 = 0;

}

//
// MAIN LOOP
//

int recLoopCount;  //for debugging when does not start record

void loop() {
  t = getTeensy3Time();

  // Standby mode
  if(modeHS1 == 0)
  {   
    if(introPeriod){
      cDisplay();
      displaySettings();
      displayClock(displayLine6, t);
      display.display();
    }
      
    if(t >= startTime){      // time to start?
      if(printDiags) Serial.println("Record Start.");

      if(noDC==0) {
        audio_freeze_adc_hp(); // this will lower the DC offset voltage, and reduce noise
      }
      
      stopTime = startTime + rec_dur;
      startTime = stopTime + rec_int;  // next start time

      if(printDiags){
         Serial.print("Current Time: ");
        printTime(getTeensy3Time());
        Serial.print("Stop Time: ");
        printTime(stopTime);
        Serial.print("Next Start:");
        printTime(startTime);
        Serial.println();
      }

      displayOff();

      modeHS1 = 1;
      startRecording();
      sumPeakVal = 0;
      sumCount = 0;
    }
  }

  float value, peakVal;
  // Record mode
  if (modeHS1 == 1) {
    continueRecording();  // download data  
//    peakVal = 0.0;
//    if(FFT1.available()){ 
//        
//        for(int i=300; i<511; i++){
//          float value = FFT1.read(i);
//          if (value>peakVal) peakVal = value;
//          
//        }
//        if(peakVal<0.000001) peakVal = 0.000001;
//          peakVal = 20*log10(peakVal);
//          if(printDiags) Serial.println(peakVal); 
//      }
//      if(peakVal<-40) {  // this gets rid of zeros at start
//        sumPeakVal += peakVal;
//        sumCount ++;
//      }

    // Check if UP + DN button pressed
    if(digitalRead(UP)==0 & digitalRead(DOWN)==0){
      delay(10); // simple deBounce
      if(digitalRead(UP)==0 & digitalRead(DOWN)==0){
        if(printDiags) Serial.println("Stop");
        stopRecording();
        
        displayOn();
        delay(1000);
        cDisplay();
        display.println();
        display.println("Stopped");
        display.print("Safe to turn off");
        display.display();
        
        delay(58000); // if don't power off in 60s, restart

        cDisplay();
        display.print("Restarting");
        display.display();
        delay(2000);
        displayOff();

        FileInit();
      }
    }

    if(buf_count >= nbufs_per_file){       // time to stop?
      total_hour_recorded += (float) rec_dur / 3600.0;
      if(total_hour_recorded > 0.1) introPeriod = 0;  //LEDS on for first file
      if(rec_int == 0){
        if(printDiags > 0){
          Serial.print("Audio Memory Max");
          Serial.println(AudioMemoryUsageMax());
          Serial.print("Current Time: ");
          printTime(t);
        }
        file.close();

        FileInit();  // make a new file
        buf_count = 0;
        
      }
      else
      {
        stopRecording();
        
        long ss = startTime - getTeensy3Time() - wakeahead;
        if (ss<0) ss=0;
//        snooze_hour = floor(ss/3600);
//        ss -= snooze_hour * 3600;
//        snooze_minute = floor(ss/60);
//        ss -= snooze_minute * 60;
//        snooze_second = ss;
        if(printDiags > 0){
          Serial.print("Time: ");
          Serial.print(getTeensy3Time());
          Serial.print("  Next: ");
          Serial.println(startTime);
          printTime(getTeensy3Time());
          Serial.flush();
         }

         
//        // if have enough time, go to sleep
//        if(ss >=15){
//            audio_power_down();
//            digitalWrite(hydroPowPin, LOW); //hydrophone and audio off 
//            if(printDiags) Serial.println("Going to sleep");
//            delay(100);
//
//
//       
//            /// ... Sleeping ....
//            setWakeupCallandSleep(ss); // will reset on wakeup
//            
//            // Waking up
//
//           
//           
//            digitalWrite(hydroPowPin, HIGH); // hydrophone on 
//            AudioInit(isf);
//            if(printDiags>0){
//            printTime(getTeensy3Time());
//           }
//         }
//        if(introPeriod) displayOn();

//        Serial.print(lhi_fsamps[isf]);
//        Serial.print(": ");
//        Serial.println(sumPeakVal/sumCount);
        displayOn();
        cDisplay();
        isf++;  // change sampple rate
        if(isf==SAMP_FREQS) isf=0;
        audio_srate = lhi_fsamps[isf];
        nbufs_per_file = (long) (rec_dur * audio_srate / 256.0) * NCHAN;
        AudioInit(isf);
        modeHS1 = 0;
      }
    }
  }
  asm("wfi"); // reduce power between interrupts
}


void startRecording() {
  if(printDiags) Serial.println("startRecording");
  FileInit();
  buf_count = 0;
  queue1.begin();
  #ifdef TWOCHAN
    queue2.begin();
  #endif
  if(printDiags) Serial.println("Queue Begin");
}

void continueRecording() {
    byte buffer[512]; // data to write

    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
    // one buffer is 512 bytes = 256 samples
    if(queue1.available() >= 2) {
      #ifdef ONECHAN
        buf_count += 1;
        memcpy(buffer, queue1.readBuffer(), 256);
        queue1.freeBuffer();
        memcpy(buffer+256, queue1.readBuffer(), 256);
        queue1.freeBuffer();
      #endif
      #ifdef TWOCHAN
          buf_count += 1;
          mxLR(buffer, queue1.readBuffer(), queue2.readBuffer()); // interleave 
          queue1.freeBuffer(); 
          queue2.freeBuffer();  // free buffer
      #endif
      file.write(buffer, 512); //audio to .wav file
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
  if(printDiags) Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  if(printDiags){
    Serial.print("Audio Memory Max");
    Serial.println(maxblocks);
  }
  queue1.end();
  #ifdef TWOCHAN
    queue2.end();
  #endif

  AudioMemoryUsageMaxReset();
  file.close();
}

void logFileHeader(){
  if(file.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      file.println("filename,ID,version,gain (dB),Voltage");
      file.close();
  }
}

void FileInit()
{
   t = getTeensy3Time();
   
   // open file 
   sprintf(filename,"%04d%02d%02dT%02d%02d%02d.wav", year(t), month(t), day(t), hour(t), minute(t), second(t));  //filename is YYYYMMDDTHHMMSS

   // log file
   SdFile::dateTimeCallback(file_date_time);

   voltage = readVoltage();
   
   if(file.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      file.print(filename);
      file.print(',');
      for(int n=0; n<8; n++){
        file.print(myID[n]);
      }

      file.print(',');
      file.print(codeVersion);
      
      file.print(',');
      file.print(gainDb); 
      
      file.print(',');
      file.print(voltage); 
      
      
      file.println();
      
//      if(voltage < 3.0){
//        logFile.println("Stopping because Voltage less than 3.0 V");
//        logFile.close();  
//        // low voltage hang but keep checking voltage
//        while(readVoltage() < 3.0){
//            delay(30000);
//        }
//      }
      file.close();
   }
   else{
    if(printDiags) Serial.print("Log open fail.");
    resetFunc();
   }
    
   file.open(filename, O_WRITE | O_CREAT | O_EXCL);

   if(printDiags > 0){
     Serial.println(filename);
     Serial.print("Hours rec:"); Serial.println(total_hour_recorded);
     Serial.print(voltage); Serial.println("V");
   }

   
   while (!file){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
    file = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
   }

    //intialize  .wav file header
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
  
    file.write((uint8_t *)&wav_hdr, 44);


  if(printDiags > 0){
    Serial.print("Buffers: ");
    Serial.println(nbufs_per_file);
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
// 
//  sgtl5000_1.inputSelect(myInput);
//  sgtl5000_1.volume(0.0);
//  sgtl5000_1.lineInLevel(gainSetting);  //default = 4
//  sgtl5000_1.autoVolumeDisable();
//  sgtl5000_1.audioProcessorDisable();

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

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
