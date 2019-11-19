/***************************************
 This shows all the wakeups for sleep
 Expect IDD of  around 1.2mA (Teensy 3.x)
 and IDD of around 900uA for (Teensy LC).
 
 Sleep is the most flexable and any
 interrupt can wake the processor.
 
 Touch interface does not work in sleep
 mode.
 ****************************************/

#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <EEPROM.h>


#define OLED_RESET -1

#define displayLine1 0
#define displayLine2 11
#define displayLine3 22
#define displayLine4 33
#define displayLine5 44
#define displayLine6 57
Adafruit_FeatherOLED display = Adafruit_FeatherOLED();



int modeHS1 = 0;// 0=stopped, 1=recording audio, 2=recording sensors

const int hydroPowPin = 2;

void setup() {

  pinMode(hydroPowPin, OUTPUT);
  digitalWrite(hydroPowPin, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
    #if defined(__MK66FX1M0__)
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    #endif

    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    Wire.setDefaultTimeout(10000);
    rtcSetup();
    displayOn();
    cDisplay();
    display.println("Loggerhead");
    display.println(RTC_TSR);
    display.display();
    /********************************************************

    
    /********************************************************
     Teensy 3.x only currently.
     
     Set RTC alarm wake up in (hours, minutes, seconds).
     ********************************************************/
    //alarm.setRtcTimer(0, 0, 20);// hour, min, sec
    
    /********************************************************
     Set Low Power Timer wake up in milliseconds.
     ********************************************************/
    //timer.setTimer(5000);// milliseconds

   
    delay(5000);
    

}

void loop() {

  cDisplay();
  
  display.println("Going to sleep");
  display.println(RTC_TSR);
  display.display();
  
  setWakeupCallandSleep(10);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);

  cDisplay();
  
  display.println("Awake");
  display.display();
  delay(10000);
  
}
