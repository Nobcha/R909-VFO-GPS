// GPS_watch_Loc_kpa1 is ombined the watch with location monitor
//  Thanks for sharing information. I refered below WEB site.
// 1.Ported from "ArduinoとGPSモジュールを使ったLCD時計の製作(３)：利用中の衛星数の表示"
//   https://yokahiyori.com/arduino_gps_lcd_clock_number-of-satellites/
// 2. GPS_TEST20241022_3.ino TinyGPS++機能を利用した。LCDに緯度経度表示
//  https://lastminuteengineers.com/neo6m-gps-arduino-tutorial/ 
//
// Change ports addresses, add D7 high, 1602 LCD @2024.10.27
// Combined the watch and location 2024.11.01
//

#include <Arduino.h>
#include <Wire.h>            // Arduino IDE I2C Library
#include <TinyGPS++.h>       // https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>
#include <TimeLib.h>         // https://github.com/PaulStoffregen/Time
#include <LiquidCrystal.h>

#define time_offset 32400    // UTC+9Hrs(60*60*9 SEC）JST!! in Japan
TinyGPSPlus gps;             // 
SoftwareSerial ss(6, 5);     // RXPin = 6, TXPin = 5

#define RS                       8  // 
#define E                        9  // 
#define DB4                     10  //
#define DB5                     11  //
#define DB6                     12  // 
#define DB7                     13  // 
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

#define EN                       7  // GPS/EN & LCD back light on 
#define RESW                    A0  // RE-SW 

boolean timer=true;                 // timer=true:clock false:location monitor 
int jst_year;
int jst_month;
int jst_day;
int jst_hour;
int jst_minute;
int jst_second;
int day_week;
char DayWeekData[7][4] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"} ;

int getDayWeek(int ,int ,int );
void lcdzeroSup(int );
void displayInfo();
void timerDisp ();

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  Wire.begin(); 
  lcd.begin(16, 2);              // 
  
  pinMode(EN, OUTPUT);           // Turn on GPS WE
  digitalWrite(EN, HIGH);        // Also turn on LCD back light
  pinMode(RESW, INPUT);          // RE-SW
  digitalWrite(RESW, HIGH);      // Pull up RE-SW port
  
  lcd.setCursor(0, 0);
  lcd.print("GPS LOC&TIME MON");  
  lcd.setCursor(0, 1);
  lcd.print("v1.0 on R909-VFO");    
  delay(3000);
  lcd.clear();
}

void loop()
{
  if ( timer == 1 )               // Clock or Location monitor
    timerDisp ();
  else
  {
  // Check the format for every gps response
    while (ss.available() > 0)
    if (gps.encode(ss.read())) displayInfo();
 
  // Wait 5000mS and no character available, print "No GPS detected"
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println("No GPS detected");
      lcd.setCursor(0, 1);
      lcd.print("No GPS detected");
      while(true);
    }
  } 

  if ( digitalRead( RESW )== 0 )      // Sence RE-SW cricked
  {
    timer = !timer;                   // Inverting mode
    lcd.clear();
    delay(100);
  }
  delay(100);  
}
//****************end of loop

// Zeller's calculation for getting week days
int getDayWeek(int year,int month,int day)
  {
    int w ;
    if(month < 3) 
    {
      year = year - 1;
      month = month + 12 ;
    }
    w = (year + (year/4) - (year/100) + (year/400) + (13*month+8)/5 + day ) % 7;
    return w;
  }

// Insert space when the number is singlecolumn
void lcdzeroSup(int digit)
  {
    if(digit < 10)
    lcd.print('0');   // space
    lcd.print(digit);
  }

// Display Latitude, Longitude
void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
    lcd.setCursor(0, 0);
    lcd.print("LAT=");
    lcd.print(gps.location.lat(), 6);
    lcd.setCursor(0, 1);
    lcd.print("LONG=");
    lcd.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.println("Location: Not Available");
    lcd.setCursor(0, 1);
    lcd.print("LOC: No Available");
  }
  
  Serial.print("Date: ");
  if ( gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }
 
  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }
  Serial.println();
  delay(1000);
}  


void timerDisp ()
  {

// Get GPS data    
    while (ss.available()>0 ) gps.encode(ss.read()) ;
// Get the number of the satelites
    lcd.setCursor(10, 1);
    lcd.print("s=");
    if (gps.satellites.isValid())
    {
      lcd.setCursor(12, 1);
      lcd.print(gps.satellites.value());  // Number of satellites in use (u32)
      lcd.print("  "); 
    } 
    else
    {
      lcd.setCursor(12, 1);
      lcd.print("--");
    }
// Update if change occured
    if(jst_minute != gps.time.minute() || 
      jst_hour   != gps.time.hour()   || 
      jst_day    != gps.date.day()    || 
      jst_month  != gps.date.month()  || 
      jst_year   != gps.date.year() )
    {
      jst_day    =gps.date.day();
      jst_month  =gps.date.month();
      jst_year   =gps.date.year();
      jst_hour   =gps.time.hour();
      jst_minute =gps.time.minute();
      jst_second =gps.time.second();
      setTime(jst_hour, jst_minute, jst_second, jst_day, jst_month, jst_year);
  // Change to JST
      adjustTime(time_offset);
    }
// JST date notation
    lcd.setCursor(0, 0);
    lcd.print(year());
    lcd.setCursor(4, 0);
    lcd.print("/");
    lcd.setCursor(5, 0);
    lcdzeroSup(month());

    lcd.setCursor(7, 0);
    lcd.print("/");
    lcd.setCursor(8, 0);
    lcdzeroSup(day());

    lcd.setCursor(11, 0);
    lcd.print("(");
    day_week = getDayWeek(year(), month(), day()); // 
    lcd.print(DayWeekData[day_week]) ;
    lcd.print(")");
    lcd.setCursor(0, 1);
    lcdzeroSup(hour());

    lcd.setCursor(2, 1);
    lcd.print(":");
    lcd.setCursor(3, 1);
    lcdzeroSup(minute());

    lcd.setCursor(5, 1);
    lcd.print(":");
    lcd.setCursor(6, 1);
    lcdzeroSup(second());

  }
