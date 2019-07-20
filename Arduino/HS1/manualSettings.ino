
#define noSet 0
#define setRecDur 1
#define setRecSleep 2
#define setYear 3
#define setMonth 4
#define setDay 5
#define setHour 6
#define setMinute 7
#define setSecond 8
#define setFsamp 9
#define setBatPack 10
#define setMode 11
#define setStartHour 12
#define setStartMinute 13
#define setEndHour 14
#define setEndMinute 15

void manualSettings(){
  boolean startRec = 0, startUp, startDown;
  readEEPROM();
  calcGain();

  autoStartTime = getTeensy3Time();

  LoadScript(); // secret settings accessible from card 1
  calcGain();
  writeEEPROM(); // update EEPROM in case any settings changed from card

    // make sure settings valid (if EEPROM corrupted or not set yet)
  
  if (rec_dur < 0 | rec_dur>100000) {
    rec_dur = 60;
    writeEEPROMlong(0, rec_dur);  //long
  }
  if (rec_int<0 | rec_int>100000) {
    rec_int = 60;
    writeEEPROMlong(4, rec_int);  //long
  }
  if (startHour<0 | startHour>23) {
    startHour = 0;
    EEPROM.write(8, startHour); //byte
  }
  if (startMinute<0 | startMinute>59) {
    startMinute = 0;
    EEPROM.write(9, startMinute); //byte
  }
  if (endHour<0 | endHour>23) {
    endHour = 0;
    EEPROM.write(10, endHour); //byte
  }
  if (endMinute<0 | endMinute>59) {
    endMinute = 0;
    EEPROM.write(11, endMinute); //byte
  }
  if (recMode<0 | recMode>1) {
    recMode = 0;
    EEPROM.write(12, recMode); //byte
  }
  if (isf<0 | isf>=SAMP_FREQS) {
    isf = 5;
    EEPROM.write(13, isf); //byte
  }
  if (nBatPacks<0 | nBatPacks>8){
    nBatPacks = 8;
    EEPROM.write(14, nBatPacks); //byte
  }
  if (gainSetting<0 | gainSetting>15) {
    gainSetting = 4;
    EEPROM.write(15, gainSetting); //byte
  }

  getCardSpace();
  updatePowerDuration();
  delay(3000);  // time to read card space
  
  while(startRec==0){
    static int curSetting = noSet;
    static int newYear, newMonth, newDay, newHour, newMinute, newSecond, oldYear, oldMonth, oldDay, oldHour, oldMinute, oldSecond;
    
    // Check for mode change
    boolean selectVal = digitalRead(SELECT);
    if(selectVal==0){
      curSetting += 1;
      while(digitalRead(SELECT)==0){ // wait until let go of button
        delay(10);
      }
      if((recMode==MODE_NORMAL & curSetting>11) | (recMode==MODE_DIEL & curSetting>15)) curSetting = 0;
   }

    cDisplay();
    display.setTextSize(1);

    t = getTeensy3Time();

    if (t - autoStartTime > 600) startRec = 1; //autostart if no activity for 10 minutes
    switch (curSetting){
      case noSet:
        if (settingsChanged) {
          writeEEPROM();
          settingsChanged = 0;
          autoStartTime = getTeensy3Time();  //reset autoStartTime
        }
        display.println("UP+DN->Rec"); 
        // Check for start recording
        startUp = digitalRead(UP);
        startDown = digitalRead(DOWN);
        if(startUp==0 & startDown==0) {
          cDisplay();
          writeEEPROM(); //save settings
          display.print("Starting..");
          display.display();
          delay(1500);
          startRec = 1;  //start recording
        }
        break;
      case setRecDur:
        rec_dur = updateVal(rec_dur, 1, 21600);
        display.print("Rec:");
        display.print(rec_dur);
        display.println("s");
        break;
      case setRecSleep:
        rec_int = updateVal(rec_int, 0, 3600 * 24);
        display.print("Slp:");
        display.print(rec_int);
        display.println("s");
        break;
      case setYear:
        oldYear = year(t);
        newYear = updateVal(oldYear,2000, 2100);
        if(oldYear!=newYear) setTeensyTime(hour(t), minute(t), second(t), day(t), month(t), newYear);
        display.print("Year:");
        display.print(year(getTeensy3Time()));
        break;
      case setMonth:
        oldMonth = month(t);
        newMonth = updateVal(oldMonth, 1, 12);
        if(oldMonth != newMonth) setTeensyTime(hour(t), minute(t), second(t), day(t), newMonth, year(t));
        display.print("Month:");
        display.print(month(getTeensy3Time()));
        break;
      case setDay:
        oldDay = day(t);
        newDay = updateVal(oldDay, 1, 31);
        if(oldDay!=newDay) setTeensyTime(hour(t), minute(t), second(t), newDay, month(t), year(t));
        display.print("Day:");
        display.print(day(getTeensy3Time()));
        break;
      case setHour:
        oldHour = hour(t);
        newHour = updateVal(oldHour, 0, 23);
        if(oldHour!=newHour) setTeensyTime(newHour, minute(t), second(t), day(t), month(t), year(t));
        display.print("Hour:");
        display.print(hour(getTeensy3Time()));
        break;
      case setMinute:
        oldMinute = minute(t);
        newMinute = updateVal(oldMinute, 0, 59);
        if(oldMinute!=newMinute) setTeensyTime(hour(t), newMinute, second(t), day(t), month(t), year(t));
        display.print("Minute:");
        display.print(minute(getTeensy3Time()));
        break;
      case setSecond:
        oldSecond = second(t);
        newSecond = updateVal(oldSecond, 0, 59);
        if(oldSecond!=newSecond) setTeensyTime(hour(t), minute(t), newSecond, day(t), month(t), year(t));
        display.print("Second:");
        display.print(second(getTeensy3Time()));
        break;
      case setFsamp:
        isf = updateVal(isf, 0, SAMP_FREQS-1);
        display.printf("SF: %.1f",lhi_fsamps[isf]/1000.0f);
        break;
      case setMode:
        display.print("Mode:");
        recMode = updateVal(recMode, 0, 1);
        if (recMode==MODE_NORMAL)  display.print("Norm");
        if (recMode==MODE_DIEL) {
            display.print("Diel*");
        }
        break;
      case setBatPack:
        nBatPacks = updateVal(nBatPacks, 1, 8);
        display.print("Batt:");
        display.println(nBatPacks);
        break;
      case setStartHour:
        startHour = updateVal(startHour, 0, 23);
        display.print("Strt HH:");
        printZero(startHour);
        display.print(startHour);
        break;
      case setStartMinute:
        startMinute = updateVal(startMinute, 0, 59);
        display.print("Strt MM:");
        printZero(startMinute);
        display.print(startMinute);
        break;
      case setEndHour:
        endHour = updateVal(endHour, 0, 23);
        display.print("End HH:");
        printZero(endHour);
        display.print(endHour);
        break;
      case setEndMinute:
        endMinute = updateVal(endMinute, 0, 59);
        display.print("End MM:");
        printZero(endMinute);
        display.print(endMinute);
        break;
    }
    updatePowerDuration();
    displayClock(displayLine6, getTeensy3Time());
    display.display();
    delay(10);
  }
}

void setTeensyTime(int hr, int mn, int sc, int dy, int mh, int yr){
  tmElements_t tm;
  tm.Year = yr - 1970;
  tm.Month = mh;
  tm.Day = dy;
  tm.Hour = hr;
  tm.Minute = mn;
  tm.Second = sc;
  time_t newtime;
  newtime = makeTime(tm); 
  Teensy3Clock.set(newtime); 
  autoStartTime = getTeensy3Time();
}
  
int updateVal(long curVal, long minVal, long maxVal){
  boolean upVal = digitalRead(UP);
  boolean downVal = digitalRead(DOWN);
  static int heldDown = 0;
  static int heldUp = 0;

  if(upVal==0){
    settingsChanged = 1;
    if (heldUp < 20) delay(200);
      curVal += 1;
      heldUp += 1;
    }
    else heldUp = 0;

    if (heldUp > 100) curVal += 4; //if held up for a while skip an additional 4
    if (heldUp > 200) curVal += 55; //if held up for a while skip an additional 4
    
    if(downVal==0){
      settingsChanged = 1;
      if(heldDown < 20) delay(200);
      if(curVal < 61) { // going down to 0, go back to slow mode
        heldDown = 0;
      }
        curVal -= 1;
        heldDown += 1;
    }
    else heldDown = 0;

    if(heldDown > 100) curVal -= 4;
    if(heldDown > 200) curVal -= 55;

    if (curVal < minVal) curVal = maxVal;
    if (curVal > maxVal) curVal = minVal;
    return curVal;
}
