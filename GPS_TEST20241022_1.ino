// Refer https://miraluna.hatenablog.com/entry/2018/01/10/000000
// GPS_TEST20241022_1.ino
// OK! 2024.10.21 22:05  Works well rx=D6(GPS/TX), D7:HIGH(GPS/EN)
// 2024.10.22 LCD display is not enough
// 2024.10.23 A3/D17  GPS/RX
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
#include <TinyGPS++.h>         // You shall include this library
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

#define RS      8              // R909-VFO 1602 assign
#define E       9   
#define DB4    10  
#define DB5    11   
#define DB6    12   
#define DB7    13  

int rx=6,tx=17;
TinyGPSPlus gps;
SoftwareSerial ss(rx,tx);
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

void setup() {
  Serial.begin(9600);            // ArduinoIDE　
  ss.begin(9600);                // GPS's default communication speed
  Serial.print("GPS DATA MONITOR");
  pinMode( 7, OUTPUT);
  digitalWrite(7, HIGH);         // GPS/EN must be high connected to D7

// set up the LCD's number of columns and rows 
  lcd.begin(16,2);
  lcd.display();                          // initialize LCD
  lcd.setCursor(0, 0);
  lcd.print("GPS DATA MONITOR");
  delay(2000);
}

void loop() {
  while (ss.available()>0){      // Check if GPS is active
  char c = (ss.read());          // Read GPS print out
  gps.encode(c);                 // Process data
  Serial.print(c);               // Print text on ArduinoIDE serial display
  lcd.setCursor(0, 1);
  lcd.print(c);
  }

}
