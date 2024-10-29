// GPS_reference_oscylator Si5351_GPS_EP12a_kpa.ino
// Please refer GITHUB https://github.com/Nobcha/R909-VFO-GPS/
// Change to R909-VFO assignment by nobcha
//  LCD pin, EN & LCD B.L
//  RE pin, przycisk is assigned as RE-SW 
//  Changed GPS port for SoftwareSerial 
//  Replace CLK0:VFO OUT with CLK1:2.5MHz

// Original Si5351_GPS_EP12a.ino
// include the library code:
#include <TinyGPS++.h>
#include <LiquidCrystal.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>
#include <si5351.h>
#include <EEPROMex.h>

#include <SoftwareSerial.h>         // Changed GPS port for SoftwareSerial 

// The TinyGPS++ object
TinyGPSPlus gps;

Si5351 si5351;

int rx=6,tx=17;
SoftwareSerial ss(rx,tx);           // Changed GPS port for SoftwareSerial 

// Set up MCU pins
#define ppsPin                   2
#define encoderPinA              3  // <A1
#define encoderPinB              4  // <A0
#define clk1_2500                5  // 2.5MHz
#define przycisk                A0  // < RE-SW
// 4 switches service by A2 port ADC  SW1:<80, SW2:<250, SW3:<430, SW4:<600

#define RS                       8  // <7
#define E                        9  // <8
#define DB4                     10  // < 9
#define DB5                     11  // <10
#define DB6                     12  // <11
#define DB7                     13  // <12

// GPS/EN & LCD back light on
#define EN                       7  // 

LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

// configure variables
unsigned long XtalFreq = 100000000;
unsigned long XtalFreq_old = 100000000;
long stab;
long correction = 0;
byte stab_count = 44;
unsigned long mult = 0, Freq = 10000000;
int second = 0, minute = 0, hour = 0;
int zone = 1;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
int validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
byte new_freq = 1;
unsigned long freq_step = 1000;
byte encoderOLD, menu = 0, band = 1, f_step = 1;
boolean time_enable = true;
unsigned long pps_correct;
byte pps_valid = 1;
//*************************************************************************************
//                                    SETUP
//*************************************************************************************
void setup()
{
  pinMode(encoderPinA, INPUT_PULLUP);                   // Set up rotary encoder
//  digitalWrite(encoderPinA, HIGH);
  pinMode(encoderPinB, INPUT_PULLUP);
//  digitalWrite(encoderPinB, HIGH);

  pinMode(przycisk, INPUT_PULLUP);               // Set up push buttons
  
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag

//  pinMode(ppsPin, INPUT_PULLUP);                 // Inititalize GPS 1pps input
//  pinMode(clk1_2500, INPUT);

  pinMode(EN, OUTPUT);                           // Turn on GPS WE
  digitalWrite(EN, HIGH);                        // Also turn on LCD back light
  
  lcd.begin(16, 2);
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA); // add

  Serial.begin(9600);
  ss.begin(9600);                 // GPS's default communication speed  
  

  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);         // B A
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK0);
  
    // Set CLK1 to output 2,5MHz
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);        // A B
  si5351.set_freq( 2500000 * SI5351_FREQ_MULT, SI5351_CLK1);
  
  si5351.update_status();

  lcd.display();
  if (digitalRead(przycisk) == 0) {               // RE-SW key pressed
    lcd.print("Initialization");
    EEPROM.writeLong(1 * 4, 10000000);
    EEPROM.writeLong(2 * 4, 16700000);
    EEPROM.writeLong(3 * 4, 37200000);
    EEPROM.writeLong(4 * 4, 64800000);
    EEPROM.writeLong(5 * 4, 145000000);
    delay(3000);
  }
  lcd.clear();
  lcd.print(" GPS  GENERATOR");
  lcd.setCursor(0, 1);
  lcd.print("by SQ1GU&JA3KPA");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Waiting for GPS");

  GPSproces(6000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    lcd.home();
    lcd.print("No GPS connected");
    lcd.setCursor(0, 1);
    lcd.print (" check wiring!! ");
    delay(5000);
    GPSstatus = false;
  }
  lcd.clear();
  if (GPSstatus == true) {                     // GPS flag true
    lcd.print("Waiting for SAT");
    time_on_lcd();
    sat_on_lcd();
    do {
      GPSproces(1000);
    } while (gps.satellites.value() == 0);     // Wait GPS being active

    hour = gps.time.hour() + zone;             // when becoming active, display time
    minute = gps.time.minute();
    second = gps.time.second();
    time_on_lcd();
    sat_on_lcd();

    attachInterrupt(0, PPSinterrupt, RISING);  // D2 INT enable
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
    
  }
  freq_on_lcd();
  lcd.print(" kHz");
  sat_on_lcd();
  time_on_lcd();
}
//***************************************************************************************
//                                         LOOP
//***************************************************************************************
void loop()
{
  if (tcount2 != tcount) {
    tcount2 = tcount;
    pps_correct = millis();
  }
  if (tcount < 4 ) {
    GPSproces(0);
  }
  if (gps.time.isUpdated()) {
    hour = gps.time.hour() + zone;
    minute = gps.time.minute();
    second = gps.time.second();
  }
  
  if (gps.satellites.isUpdated() && menu == 0) {
    sat_on_lcd();
  }
  
  if (new_freq == 1) {
    correct_si5351a();
    new_freq = 0;
    lcd.setCursor(15, 0);
    lcd.print("*");
  }
  if (new_freq == 2) {
    update_si5351a();
    freq_on_lcd();
    new_freq = 0;
  }

// 4 switches service by RE-SW
  if (digitalRead(przycisk) == 0) {
    delay(30);
    if (digitalRead(przycisk) == 0) {
      menu++;
      if (menu > 5) menu = 0;
      lcd.setCursor(0, 1);
      switch (menu) {
        case 0:
          sat_on_lcd();
          time_on_lcd();
          time_enable = true;
          break;
        case 1:
          band_on_lcd();
          break;
        case 2:
          step_on_lcd();
          break;
        case 3:
          freq2_on_lcd();
          break;
        case 4:
          EEPROM.writeLong(band * 4, Freq);
          stab_on_lcd();
          break;
        case 5:
          timezone_on_lcd();
          break;
      }
      delay(300);
    }
  }




  if (millis() > pps_correct + 1200) {
    pps_valid = 0;
    pps_correct = millis();
    time_enable = false;
    lcd.setCursor (15, 0);
    lcd.print(" ");

  }
  ENCread();
}

//**************************************************************************************
//                       INTERRUPT  ENC
//**************************************************************************************
void ENCread()
{
  if (digitalRead(encoderPinA) != encoderOLD) {
    delay(5);
    if (digitalRead(encoderPinA) != encoderOLD) {
      encoderOLD = digitalRead(encoderPinA);
      if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {

        switch (menu) {
          case 1: {
              band++;
              if (band > 5) band = 5;
              band_on_lcd();
            }
            break;
          case 2: {
              f_step++;
              if (f_step > 8)f_step = 8;
              step_on_lcd();
            }
            break;
          case 3: {
              Freq += freq_step;
              if (Freq > 160000000) Freq -= freq_step;
              new_freq = 2;
            }
            break;
          case 5: {
              zone++;
              if (zone > 6)zone = 5;
              EEPROM.writeInt(80, zone);
              timezone_on_lcd();
            }
            break;
        }
      }
      else {
        switch (menu) {
          case 1: {
              band--;
              if (band == 0) band = 1;
              band_on_lcd();
            }
            break;
          case 2: {
              f_step--;
              if (f_step == 0 )f_step = 1;
              step_on_lcd();
            }
            break;
          case 3: {
              Freq -= freq_step;
              if (Freq > 160000000 || Freq < 1000) Freq += freq_step;
              new_freq = 2;
            }
            break;
          case 5: {
              zone--;
              if (zone < -6)zone = -5;
              EEPROM.writeInt(80, zone);
              timezone_on_lcd();
            }
            break;
        }
      }
      delay(60);
    }
  }
}
//**************************************************************************************
//                       INTERRUPT  1PPS
//**************************************************************************************
void PPSinterrupt()
{

  tcount++;
  stab_count--;
  if (tcount == 4)                               // Start counting the 2.5 MHz signal from Si5351A CLK1
  {
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
    // loop();
  }
  if (tcount == 44)                              //The 40 second gate time elapsed - stop counting
  {
    TCCR1B = 0;                                  //Turn off counter
    if (pps_valid == 1) {
      XtalFreq_old = XtalFreq;
      XtalFreq = mult * 0x10000 + TCNT1;         //Calculate correction factor
      new_freq = 1;
  }
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
    pps_valid = 1;

    stab_count = 44;
  if (menu == 4)stab_on_lcd();
}
if (validGPSflag == 1)                           //Start the UTC timekeeping process
{
    second++;
    if (second == 60)                            //Set time using GPS NMEA data
    {
      minute++ ;
      second = 0 ;
    }
    if (minute == 60)
    {
      hour++;
      minute = 0 ;
    }
    if (hour == 24) hour = 0 ;
    if (time_enable) time_on_lcd();
  }
  if (menu == 4) {
    lcd.setCursor(14, 1);
    if (stab_count < 10) lcd.print(" ");
    lcd.print(stab_count);
  }
  
}
//*******************************************************************************
//                   Timer 1 overflow intrrupt vector.
//*******************************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}
//********************************************************************************
//                                TIMEZONE on LCD <>
//********************************************************************************
void timezone_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("TIME zone ");
  if (zone > 0) lcd.print("+");
  lcd.print(zone);
  lcd.print(" < > ");
}
//********************************************************************************
//                                STAB on LCD  Frequency stability
//********************************************************************************
void stab_on_lcd() {
  float stab_float;
  long pomocna;
  time_enable = false;
  lcd.setCursor(7, 1);
  lcd.print("         ");
  lcd.setCursor(0, 1);
  lcd.print("Fstab ");

  stab = XtalFreq - 100000000;
  stab = stab * 10 ;
  if (stab > 100 || stab < -100) {
    correction = correction + stab;
  }
  else if (stab > 20 || stab < -20) {
    correction = correction + stab / 2;
  }
  else correction = correction + stab / 4;
  pomocna = (10000 / (Freq / 1000000));
  stab = stab * 100;
  stab = stab / pomocna;
  stab_float = float(stab);
  stab_float = stab_float / 10;
  lcd.print(stab_float);
  lcd.print("Hz");
}
//********************************************************************************
//                                FREQ_2 on LCD <>
//********************************************************************************
void freq2_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("FREQ Bank ");
  lcd.print(band);
  lcd.print(" < > ");
}
//********************************************************************************
//                                STEP on LCD
//********************************************************************************
void step_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("STEP ");
  switch (f_step) {
    case 1: freq_step = 1, lcd.print("   1Hz");
      break;
    case 2: freq_step = 10, lcd.print("  10Hz");
      break;
    case 3: freq_step = 100, lcd.print(" 100Hz");
      break;
    case 4: freq_step = 1000, lcd.print("  1kHz");
      break;
    case 5: freq_step = 10000, lcd.print(" 10kHz");
      break;
    case 6: freq_step = 100000, lcd.print("100kHz");
      break;
    case 7: freq_step = 1000000 , lcd.print("  1MHz");
      break;
    case 8: freq_step = 10000000, lcd.print(" 10Mhz");
      break;
  }
}
//********************************************************************************
//                                BAND on LCD
//********************************************************************************
void band_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("BANK ");
  lcd.print(band);
  lcd.print("      < > ");
  Freq = EEPROM.readLong(band * 4);
  freq_on_lcd();
  update_si5351a();
}
//********************************************************************************
//                                TIME on LCD
//********************************************************************************
void time_on_lcd()
{
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d ", hour, minute, second);
  lcd.setCursor(8, 1);
  lcd.print(sz);
}
//********************************************************************************
//                                SAT nr. on LCD
//********************************************************************************
void sat_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("SAT ");
  lcd.print(gps.satellites.value());
  lcd.print("   ");

   time_enable = true;  
}

//*********************************************************************************
//                             Freq on LCD
//*********************************************************************************
void freq_on_lcd() {
  char buf[10];

  // Print frequency to the LCD
  ltoa(Freq, buf, 10);
  time_enable = false;
  lcd.home();
  if (Freq < 1000000)
  {
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print('.');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
  }

  if (Freq >= 1000000 && Freq < 10000000)
  {
    lcd.print(" ");
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print(',');
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print('.');
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
  }

  if (Freq >= 10000000 && Freq < 100000000)
  {
    lcd.print(" ");
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(',');
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print('.');
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
  }

  if (Freq >= 100000000)
  {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(',');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print('.');
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(buf[8]);
  }
}
//********************************************************************
//             NEW frequency
//********************************************************************
void update_si5351a()
{
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK0);

}
//********************************************************************
//             NEW frequency correction
//********************************************************************
void correct_si5351a()
{
  si5351.set_correction(correction, SI5351_PLL_INPUT_XO);

  //update_si5351a();

}
//*********************************************************************
//                    Data read from GPS
//**********************************************************************
static void GPSproces(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())           // Software Serial
      gps.encode(ss.read());         // Changed GPS port for SoftwareSerial  
  } while (millis() - start < ms);
}
//*********************************************************************
