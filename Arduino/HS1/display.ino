
/* DISPLAY FUNCTIONS
 *  
 */

void displayOn(){
  //display.ssd1306_command(SSD1306_DISPLAYON);
  display.init();
  display.setBatteryVisible(true);
}

void displayOff(){
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void cDisplay(){
  display.clearDisplay();
  displayBattery();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(30,displayLine3);
  display.print((int) recDays);
  display.print("d");
  
  display.setCursor(0,0);
}

void displaySettings(){
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, displayLine1);
  if(mode==0) {
    display.println("SBY");
    display.print(startTime - t);
    display.print("s  ");
    display.print(lhi_fsamps[isf]);
    display.print("Hz");
    if (NCHAN==2) display.println("  Stereo");
    else
      display.println(""); // MONO
  }
  display.print("R:");
  display.print(rec_dur);
  display.print("s");
  display.print(" S:");
  display.print(rec_int);
  display.print("s");
}

void displayBattery(){
  display.setBattery(voltage);
  display.renderBattery();
}

void displayClock(int loc, time_t t){
  display.setTextSize(1);
  display.setCursor(0,loc);
  display.print(year(t));
  display.print('-');
  display.print(month(t));
  display.print('-');
  display.print(day(t));
  display.print("  ");
  printZero(hour(t));
  display.print(hour(t));
  printDigits(minute(t));
  printDigits(second(t));
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  display.print(":");
  printZero(digits);
  display.print(digits);
}

void printZero(int val){
  if(val<10) display.print("0");
}


// 
//void printDigits(int digits){
//  // utility function for digital clock display: prints preceding colon and leading 0
//  display.print(":");
//  printZero(digits);
//  display.print(digits);
//}
//
//void printZero(int val){
//  if(val<10) display.print('0');
//}

void printTime(time_t t){
  Serial.print(year(t));
  Serial.print('-');
  Serial.print(month(t));
  Serial.print('-');
  Serial.print(day(t));
  Serial.print(" ");
  Serial.print(hour(t));
  Serial.print(':');
  Serial.print(minute(t));
  Serial.print(':');
  Serial.println(second(t));
}

void readEEPROM(){
  rec_dur = readEEPROMlong(0);
  rec_int = readEEPROMlong(4);
  isf = EEPROM.read(8);
  gainSetting = EEPROM.read(9);

  if(rec_dur<1) rec_dur = 30;
  if(rec_dur>3600) rec_dur = 3600;
  if(rec_int<30) rec_int = 570;
  if(rec_int>3600*24) rec_int = 60;
  if(isf<0 | isf>SAMP_FREQS) isf=4;
  if(gainSetting<0 | gainSetting>13) gainSetting = 4;
}

union {
  byte b[4];
  long lval;
}u;

long readEEPROMlong(int address){
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address + 1);
  u.b[2] = EEPROM.read(address + 2);
  u.b[3] = EEPROM.read(address + 3);
  return u.lval;
}

void writeEEPROMlong(int address, long val){
  u.lval = val;
  EEPROM.write(address, u.b[0]);
  EEPROM.write(address + 1, u.b[1]);
  EEPROM.write(address + 2, u.b[2]);
  EEPROM.write(address + 3, u.b[3]);
}

void writeEEPROM(){
  writeEEPROMlong(0, rec_dur);  //long
  writeEEPROMlong(4, rec_int);  //long
  EEPROM.write(8, isf); //byte
  EEPROM.write(9, gainSetting); //byte
}
