// 2025.01.29 PPS ok, SW1-SW4 optioned GL Grid locator 94%/61% V1.23
//  GL calculation https://www.jarl.org/Japanese/1_Tanoshimo/1-2_Award/gl.htm
// 2024.11.16 Refered and Changed for R909-VFO-GPS 
//  added correction FREQ display
//  changed store condition correction
// GITHUB https://github.com/Nobcha/R909-VFO-GPS
// D2:PPS,D5:2.5M,D7:EN,D0/RX:GPS/TX,LCD1602
//
// GITHUB updated
//
// gps-calibration-5351
//　https://github.com/csqwdy/gps-calibration-5351/blob/main/README.md
// 2022-03-06 Modified the encoder to work in signal change interrupt mode, 
// Original was SQ1GU  http://sq1gu.tobis.com.pl/en/dds

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
#include <math.h>                               // To be asked for GPS calculation

// The TinyGPS++ object
TinyGPSPlus gps;
Si5351 si5351;

#define control	                 2              // 1=button ,2=EP11 ,3=EP12

// Set up MCU pins
#define GPS_Enable               7
#define ppsPin                   2              // 
#define RE_SW                   A0
#define encoderPinA					     4
#define encoderPinB				       3
#define RS                       8
#define E                        9
#define DB4                     10
#define DB5                     11
#define DB6                     12
#define DB7                     13
#define ledPin                  A1
#define func_sw                 A2              // SW1:<80, SW2:<250,

#define Freq2					1000000000ULL

volatile byte seqA = 0;
volatile byte seqB = 0;
volatile byte cnt1 = 0;
volatile byte cnt2 = 0;
volatile boolean right = false;
volatile boolean left = false;
volatile boolean button = false;
volatile boolean led_onoff = false;

volatile byte menu = 8;
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

// configure variables
unsigned long XtalFreq = 100000000;
unsigned long XtalFreq_old = 100000000;
long stab;
long correction = 0;
byte stab_count = 44;
unsigned long mult = 0, Freq = 10000000;
int second1 = 0, minute1 = 0, hour1 = 0;
int zone = 9;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
int validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
byte new_freq = 1;
unsigned long freq_step = 1000;
byte encoderOLD,  band = 1, f_step = 1;
boolean time_enable = true;
unsigned long pps_correct_time;
byte pps_valid = 1;
byte correct_byte = 1;
float stab_float;
unsigned long button_on_time;
boolean vfo_counter = true;       // true vfo
boolean new_c_freq = 0;
boolean pps_error = 1;
int func_sw_value;
// The string for the grid locator data
char locator[7]; // 6 characters + delimitter

// prototype
void PPSinterrupt(void);
void timezone_on_lcd(void);
void stab_on_lcd(void);
void correct_freq(void);
void corr_on_lcd(void);
void freq2_on_lcd(void);
void step_on_lcd(void);
void band_on_lcd(void);
void time_on_lcd(void);
void sat_on_lcd(void);
void freq_on_lcd(void);
void update_si5351a(void);
void correct_si5351a(void);
static void GPSproces(unsigned long);
void change_up(void);
void change_down(void);
void loglat_on_lcd(void);
void vfo_fr_lcd (void);
void date_on_lcd(void);
void calculateGridLocator(float, float);
void gl_on_lcd(void);

//*******************************************************************************
// Timer 1 overflow intrrupt vector.
//*******************************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                  //Increment multiplier
  TIFR1 = (1 << TOV1);                     //Clear overlow flag
}
//*********************************************************************
//                    Judge the rotating direction
//*********************************************************************
ISR (PCINT2_vect) {
    // Read A and B signals
    boolean A_val = digitalRead(encoderPinB);
    boolean B_val = digitalRead(encoderPinA);
    // Record the A and B signals in seperate sequences
    seqA <<= 1;
    seqA |= A_val;
    
    seqB <<= 1;
    seqB |= B_val;
    
    // Mask the MSB four bits
    seqA &= 0b00001111;
    seqB &= 0b00001111;
    
    // Compare the recorded sequence with the expected sequence
    if (seqA == 0b00001001 && seqB == 0b00000011) {
      cnt1++;
      left = true;
    }
     
    if (seqA == 0b00000011 && seqB == 0b00001001) {
      cnt2++;
      right = true;
    }
}  
//*************************************************************************************
//                                    SETUP
//*************************************************************************************
void setup()
{
  pinMode(GPS_Enable, OUTPUT);             // GPS-EN and LCD backlight on
  digitalWrite(GPS_Enable, HIGH);          //
  pinMode(encoderPinA, INPUT);             // Set up rotary encoder
  digitalWrite(encoderPinA, HIGH);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);
  pinMode(RE_SW, INPUT);                   // Set up RE push buttons
  digitalWrite(RE_SW, HIGH);
  pinMode(ledPin, OUTPUT);                 // Set up RE push buttons
  digitalWrite(ledPin, LOW);
  
  lcd.begin(16, 2); 
// (PCINT19/OC2B/INT1) PD3, (PCINT20/XCK/T0) PD4
  PCICR =  0b00000100;                      // PCIE0: Pin Change Interrupt Enable 0
  PCMSK2 = 0b00011000;                      // Enable Pin Change Interrupt for D3, D4
  
  TCCR1B = 0;                               // Disable Timer5 during setup
  TCCR1A = 0;                               // Reset
  TCNT1  = 0;                               // Reset counter to zero
  TIFR1  = 1;                               // Reset overflow
  TIMSK1 = 1;                               //  Turn on overflow flag
  pinMode(ppsPin, INPUT);                   // Inititalize GPS 1pps input

  lcd.display();
 
  if (digitalRead(RE_SW) == 0) {            // EEPROM is initilised 
	  lcd.print("Initialization");            // by RE-SW pushing on turning on
	  EEPROM.writeLong(1 * 4, 10000000);
	  EEPROM.writeLong(2 * 4, 16700000);
	  EEPROM.writeLong(3 * 4, 37200000);
	  EEPROM.writeLong(4 * 4, 64800000);
	  EEPROM.writeLong(5 * 4, 145000000);
	  EEPROM.writeByte(82, 1);
	  EEPROM.writeInt(80, 1);
	  EEPROM.writeLong(90, 0);
	  EEPROM.writeByte(99, 1);
	  delay(3000);
  }
	band = EEPROM.readByte(82);
	Freq = EEPROM.readLong(band * 4);
	zone = EEPROM.readInt(80);
	correct_byte = EEPROM.readByte(99);
	correction = EEPROM.readLong(90);
	
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);

  Serial.begin(9600);                       // To serve the GPS module
  
  // Set CLK0 to output 2,5MHz
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLA);
  si5351.set_freq(250000000ULL, SI5351_CLK1);
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLB);
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.set_freq(Freq2, SI5351_CLK2);
  correct_si5351a();
  si5351.update_status();

  lcd.clear();
  lcd.print("GPS COR VFO v123");             // Starting banner
  lcd.setCursor(0, 1);
  lcd.print("5351a @ R909-VFO");
  delay(4000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Waiting for GPS");

  GPSproces(2000);

 // if ((millis() > 5000) && (gps.charsProcessed() < 10)) {
    if ( gps.charsProcessed() < 10) {
    lcd.home();
    lcd.print("No GPS connected");
    lcd.setCursor(0, 1);
    lcd.print (" check wiring!! ");
    delay(5000);
    GPSstatus = false;      // OK
  }
  lcd.clear();
  
  // if GPS connected then...
  if (GPSstatus == true) {
    lcd.print("Waiting for SAT");
    time_on_lcd();
    sat_on_lcd();
    do {
      GPSproces(1000);
    } while (gps.satellites.value() == 0);

    hour1 = (gps.time.hour() + zone) % 24;           // JPN: zone#9
    minute1 = gps.time.minute();
    second1 = gps.time.second();
    time_on_lcd();
    sat_on_lcd();
    attachInterrupt(0, PPSinterrupt, RISING);
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
  }
  // Display contents at start timing
  freq_on_lcd();
  lcd.print("kHz  ");
  sat_on_lcd();
  time_on_lcd();
  menu = 0;
}
//***************************************************************************************
//                                         LOOP
//***************************************************************************************
void loop()
{
  if (tcount2 != tcount) {                  // PPS 1 sec check
    tcount2 = tcount;                       // L#202
    pps_correct_time = millis();            // Watching PPS period as 1 sec
  }
  GPSproces(0);                           // get GPS data

  if (gps.time.isUpdated()) {
    hour1 = ((gps.time.hour() + zone)) % 24;
	  if(hour1<0) hour1 = hour1 * -1;
    minute1 = gps.time.minute();
    second1 = gps.time.second();
    if(menu==0) time_on_lcd();
  }
  if (gps.satellites.isUpdated() && menu == 0) {
    sat_on_lcd();
  }

  if (new_freq == 1) {                      // After correcting FREQ, display * @(15,0)
    correct_si5351a();
    new_freq = 0;
    lcd.setCursor(15, 0);
    lcd.print("*");
  }
  if (new_freq != 1) {                      // No correcting FREQ, display * @(15,0)
    lcd.setCursor(15, 0);
    lcd.print(" ");
  }  
  if (new_freq == 2) {                      // After setting new FREQ by rotating
    update_si5351a();
    freq_on_lcd();
    new_freq = 0;

  }
  
  if ((digitalRead(RE_SW) == 0) & (button == false)){            // RE-SW pressed?
    button = true;                          // RE-SW press
    button_on_time = millis();              // Time to avoid chattering
  }
  if (button == true && (button_on_time + 400) < millis() ) {   // RE-SW settled
    menu++;
    if (menu > 8) menu = 0;                 // increase menu 8 of correction
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
        case 5:
          EEPROM.writeLong(band * 4, Freq);
          stab_on_lcd();
          break;
        case 4:
          timezone_on_lcd();
          break;
        case 6:                              // added new menu
          corr_on_lcd();                     // display correction value
          break;
        case 7:                              // added new menu
          loglat_on_lcd();                   // display Latitude and longtitude
          break;          
        case 8:                              // added new menu
          gl_on_lcd();                       // display grid locator
          break;
      }
    button = false ;
  }
  
  if (millis() > pps_correct_time + 1200) {  // pps timing is out of order
    pps_valid = 0;
    pps_correct_time = millis();
    time_enable = false;
    lcd.setCursor (14, 0);
    lcd.print("?");                          // Ararming PPS is out of order
  }
  else {
    lcd.setCursor (14, 0);
    lcd.print("!");    
  }
  // Encoder responce on signal changing interrupt
  if (left) {
    left = false;
    change_down();     
  }

  if (right) {
    right = false;
    change_up();     
  }

  // This set keeps 4 keys of SW1, SW2, SW3, SW4
  // SW1:<80, SW2:<250, SW3:<430, SW4:<600 
  // actual 0, 166, 356, 519

  func_sw_value = analogRead(func_sw);
  if (func_sw_value < 80 ) {                  // SW1:menu=2/step
    menu = 2;
    step_on_lcd(); 
  }
  else if (func_sw_value < 250  ) {           // SW2:menu=3/frequncy,
    menu = 3;
    vfo_fr_lcd();
    freq2_on_lcd();
  }
  else if (func_sw_value < 430  ) {           // SW3:menu=5 / f-cor
    menu = 5;
    stab_on_lcd();
  }
  else if (func_sw_value < 600  ) {           // SW4:menu=7 / grid locator
    menu = 6;
    corr_on_lcd();                            // display correction value
  }
  
  // Diag monitor for PPS & the counter
  lcd.setCursor (13, 1);
  if (menu == 5) { 
    lcd.print(mult%10);
    lcd.print(tcount%100); 
    lcd.print(" "); 
  }
}
// The end of loop
//
//**************************************************************************************
//                       INTERRUPT  1PPS
//**************************************************************************************
void PPSinterrupt()
{
  tcount++;
  stab_count--;
  if (tcount == 4)                        // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                           //Clock on rising edge of pin 5
  }

  if (tcount == 44)                       //The 40 second gate time elapsed - stop counting
  {
    TCCR1B = 0;                           //Turn off counter
    if (pps_valid == 1) {
      XtalFreq_old = XtalFreq;
      XtalFreq = mult * 0x10000 + TCNT1;  //Calculate correction factor
      new_freq = 1;
    }
    TCNT1 = 0;                            //Reset count to zero
    mult = 0;
    tcount = 0;                           //Reset the seconds counter
    pps_valid = 1;                        // Keeping 1 sec cycle time

    stab_count = 44;
	  correct_freq();
  }
  digitalWrite(ledPin, led_onoff);
  led_onoff =! led_onoff;
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
//                                STAB on LCD  stabilnośc częstotliwości
//********************************************************************************
void stab_on_lcd() {

  lcd.setCursor(7, 1);
  lcd.print("         ");
  lcd.setCursor(0, 1);
  lcd.print("Fstab ");

  lcd.print(stab_float);
  lcd.print("Hz");
}
//********************************************************************************
//   correct ferquency  Crystal is 25MHz, 
//********************************************************************************
void correct_freq()
{
  long pomocna;
  time_enable = false;

  stab = XtalFreq - 100000000;           // measured value was 100MHz
  stab = stab * 10 ;                     // 
  if (stab > 100 || stab < -100) {
	  correction = correction + stab;
  }
  else if (stab > 20 || stab < -20) {
	  correction = correction + stab / 2;
  }
  else correction = correction + stab / 4;
  
  if(stab < 20 || stab > -20) {
	  if(correct_byte != 0){
		  EEPROM.writeLong(90, correction);
		  EEPROM.writeByte(99, 0);
	  }
  }
  
  pomocna = (10000 / (Freq / 1000000));
  stab = stab * 100;
  stab = stab / pomocna;
  stab_float = float(stab);
  stab_float = stab_float / 10;
	
}
//********************************************************************************
//                                corr_on_lcd
//********************************************************************************
void corr_on_lcd()                // Display correction frequency 
{
   lcd.setCursor(0, 1);
   lcd.print("CORR=");
   lcd.print(correction/100);     // Freq is xxxxULL
   lcd.print(".");
   lcd.print(correction%100);
   lcd.print("Hz        ");   
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
    case 1: freq_step = 1, lcd.print("  1Hz ");
      break;
    case 2: freq_step = 10, lcd.print(" 10Hz ");
      break;
    case 3: freq_step = 100, lcd.print("100Hz ");
      break;
    case 4: freq_step = 1000, lcd.print("  1kHz");
      break;
    case 5: freq_step = 10000, lcd.print(" 10kHz");
      break;
    case 6: freq_step = 100000, lcd.print("100kHz");
      break;
    case 7: freq_step = 1000000 , lcd.print("  1MHz");
      break;
    case 8: freq_step = 10000000, lcd.print(" 10MHz");
      break;
  }
  int i;
  for (i=1; i<=6; i++) lcd.print(" ");
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
  sprintf(sz, "%02d:%02d:%02d ", hour1, minute1, second1);
  lcd.setCursor(7, 1);
  lcd.print(" ");
  lcd.print(sz);
}

//********************************************************************************
//                                SAT number on LCD
//********************************************************************************
void sat_on_lcd()
{
  time_enable = false;
  lcd.setCursor(0, 1);
  lcd.print("SAT ");
  lcd.print(gps.satellites.value());
  lcd.print("  ");
  time_enable = true;
}
//********************************************************************************
//                       Latitude & longtitude on lcd
//********************************************************************************
void loglat_on_lcd()
{
  lcd.setCursor(0, 1);
  lcd.print("L"+String(gps.location.lat(),3));
  lcd.print(","+String(gps.location.lng(),3));
  lcd.print("   ");
}
//********************************************************************************
//                       Grid locator on lcd
//********************************************************************************
void gl_on_lcd()
{
  lcd.setCursor(0, 1);
  // calculate the gridLocator
  calculateGridLocator (gps.location.lat(), gps.location.lng());
  lcd.print("GL ");
  lcd.print(locator);
  lcd.print("        ");
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
//             VFO frequency 
//********************************************************************
void vfo_fr_lcd ()
{
    lcd.setCursor(0, 0);
    delay(10);
    lcd.print("VFO:");
    freq_on_lcd();
}
//********************************************************************
//             NEW frequency
//********************************************************************
void update_si5351a()
{
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK0);

}
//********************************************************************
//             Frequency correction
//********************************************************************
void correct_si5351a()
{
  si5351.set_correction(correction, SI5351_PLL_INPUT_XO);

  //update_si5351a();

}
//*********************************************************************
//                    Get data from GPS
//**********************************************************************
static void GPSproces(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while ((millis() - start ) < ms);
}

//**************************************************************************************
//                       Clockwise by rotary encoder
//**************************************************************************************
void change_up(){
    switch (menu) {
        case 1: {
           band++;
           if (band > 5) band = 5;
           EEPROM.writeByte(82, band);
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
           if (Freq > 200000000) Freq -= freq_step;
           new_freq = 2;
        }
        break;
        case 5: {
           zone++;
           if (zone > 12)zone = 12;
           EEPROM.writeInt(80, zone);
           timezone_on_lcd();
        }
        break;
        case 6: {
           EEPROM.writeLong(90, correction);
           EEPROM.writeByte(99, 0);              
        }
        break;       
    }
}
//**************************************************************************************
//                       Counterclockwise by rotary encoder
//**************************************************************************************
void change_down(){
        switch (menu) {
          case 1: {
              band--;
              if (band == 0) band = 1;
              EEPROM.writeByte(82, band);
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
              if (Freq > 200000000 || Freq < 1000) Freq += freq_step;
              new_freq = 2;
            }
            break;
          case 5: {
              zone--;
              if (zone < -12)zone = -12;
              EEPROM.writeInt(80, zone);
              timezone_on_lcd();
            }
            break;
        }   
}
//**************************************************************************************
//                       GridLocator calculation
//**************************************************************************************
void calculateGridLocator(float latitude, float longitude) {
    // 1. The adjustment for data
    longitude += 180.0;
    latitude += 90.0;

    // 2. The first fields (1–2 column)
    locator[0] = 'A' + (char)(longitude / 20);
    locator[1] = 'A' + (char)(latitude / 10);

    // 3. The square (3–4 column)
    locator[2] = '0' + (char)((fmod(longitude, 20) / 2));
    locator[3] = '0' + (char)(fmod(latitude, 10));

    // 4. The sub-square (5–6 column)
    locator[4] = 'a' + (char)(fmod(longitude * 12, 24));
    locator[5] = 'a' + (char)(fmod(latitude * 24, 24));

    // 5. The delimitter
    locator[6] = '\0';
}
