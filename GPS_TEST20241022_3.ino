//  GPS_TEST20241022_3.ino TinyGPS++機能を利用した。LCDに緯度経度表示
//  GPS LOC MONITOR 
//  Thanks for sharing information. I refered below WEB site.
//  https://lastminuteengineers.com/neo6m-gps-arduino-tutorial/
/*
 A3/D17  GPS/RX
 D6  GPS/TX 
 D7  LCD RS -> Back Light & GPS/EN
 D8  LCD enable -> LCD RS 
 D9  LCD DB4 -> LCD enable
 D10 LCD DB5 ->  LCD DB4
 D11 LCD DB6 ->  LCD DB5
 D12 LCD DB7 ->  LCD DB6
 */
 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

#define RS      8              // R909-VFO 1602 assign
#define E       9   
#define DB4    10  
#define DB5    11   
#define DB6    12   
#define DB7    13  
 
int RXPin = 6;                 //pin assign for softwqre serial
int TXPin = 17;
int GPSBaud = 9600;            // GPS default          
 
// Genarate TinyGPS++ object
TinyGPSPlus gps;
// Name gpsSerial fo GPS service
SoftwareSerial gpsSerial(RXPin, TXPin);
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);
 
void setup()
{
  Serial.begin(9600);
  lcd.begin(16,2);
  lcd.display();                // initialize LCD
  gpsSerial.begin(GPSBaud);
  pinMode(7, OUTPUT);           // GPS/EN & LCD back light on
  digitalWrite( 7, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("GPS LOC MONITOR");
}
 
void loop()
{
  // Check the format for every gps response
  while (gpsSerial.available() > 0)
  if (gps.encode(gpsSerial.read())) displayInfo();
 
  // Wait 5000nS and no character available, print "No GPS detected"
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    lcd.setCursor(0, 1);
    lcd.print("No GPS detected");
    while(true);
  }
}
 
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
