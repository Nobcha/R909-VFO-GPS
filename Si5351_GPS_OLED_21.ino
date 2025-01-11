// 2025.01.07 pps OK, Sw1.2 OK 91%/62%
// 2024.12.09 tiny4kOLED
// 2024.11.16 Refered and Changed for R909-VFO-GPS 
//  added correction FREQ display
//  changed store condition correction
// GITHUB https://github.com/Nobcha/R909-VFO-GPS
// D2:PPS,D5:2.5M,D7:EN,D0/RX:GPS/TX,LCD1602
//
// gps-calibration-5351
//　https://github.com/csqwdy/gps-calibration-5351/blob/main/README.md
// 2022-03-06 Modified the encoder to work in signal change interrupt mode, 
// Original was SQ1GU  http://sq1gu.tobis.com.pl/en/dds

// include the library code:
#include <TinyGPS++.h>
// Change to OLED 
#include <Tiny4kOLED.h>

#include <ctype.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>
#include <si5351.h>
#include <EEPROMex.h>

// The TinyGPS++ object
TinyGPSPlus gps;
Si5351 si5351;

#define control	                 2              // 1=button ,2=EP11 ,3=EP12

// Set up MCU pins
#define GPS_Enable               7
#define ppsPin                   2              // GPS-PPS INT
#define re_sw                   A0
#define encoderPinA					     4              // Signal change INT
#define encoderPinB				       3              //
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

// configure variables
unsigned long XtalFreq = 100000000;
unsigned long XtalFreq_old = 100000000;
long stab;
long correction = 0;
byte stab_count = 44;
unsigned long mult = 0, Freq = 10000000;
int second = 0, minute = 0, hour = 0;
int zone = 9;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
int validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
byte new_freq = 1;
unsigned long freq_step = 1000;
byte encoderOLD, menu = 6, band = 1, f_step = 1;
boolean time_enable = true;
unsigned long pps_correct_time;
byte pps_valid = 0;
byte correct_byte = 1;
float stab_float;
unsigned long button_on_time;
boolean oled_turn = 1;
boolean vfo_counter = true;       // true vfo
boolean new_c_freq = 0;
boolean pps_error = 1;
int func_sw_value;

// prototype
void PPSinterrupt(void);
void timezone_on_oled(void);
void stab_on_oled(void);
void correct_freq(void);
void corr_on_oled(void);
void freq2_on_oled(void);
void step_on_oled(void);
void band_on_oled(void);
void time_on_oled(void);
void sat_on_oled(void);
void freq_on_oled(unsigned long);
void update_si5351a(void);
void correct_si5351a(void);
static void GPSproces(unsigned long);
void change_up(void);
void change_down(void);
void loglat_on_oled(void);
void vfo_fr_oled (void);
void date_on_oled(void);


//*******************************************************************************
// Timer 1 overflow intrrupt vector.  Count 2.5MHz
//*******************************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                  //Increment multiplier
  TIFR1 = (1 << TOV1);                     //Clear overlow flag
}
//*********************************************************************
//                    Judge the rotating direction
//                    Signal change interrupt
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
  pinMode(GPS_Enable, OUTPUT);              // GPS-EN and LCD backlight on
  digitalWrite(GPS_Enable, HIGH);           //
  pinMode(encoderPinA, INPUT);              // Set up rotary encoder
  digitalWrite(encoderPinA, HIGH);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);
  pinMode(re_sw, INPUT);                    // Set up RE push buttons
  digitalWrite(re_sw, HIGH);
  pinMode(ledPin, OUTPUT);                  // Set up RE push buttons
  digitalWrite(ledPin, LOW);
  
  oled.begin();
  
  oled.setFont(FONT6X8);
  oled.clear();
  oled.on();
  delay(50);

// (PCINT19/OC2B/INT1) PD3, (PCINT20/XCK/T0) PD4
  PCICR =  0b00000100;                      // PCIE0: Pin Change Interrupt Enable 0
  PCMSK2 = 0b00011000;                      // Enable Pin Change Interrupt for D3, D4
  
  TCCR1B = 0;                               // Disable Timer5 during setup
  TCCR1A = 0;                               // Reset
  TCNT1  = 0;                               // Reset counter to zero
  TIFR1  = 1;                               // Reset overflow
  TIMSK1 = 1;                               //  Turn on overflow flag
  pinMode(ppsPin, INPUT);                   // Inititalize GPS 1pps input

  if (digitalRead(re_sw) == 0) {            // EEPROM is initilised 
    oled.print("Initialization");           // by re_sw pushing on turning on
    digitalWrite(ledPin, HIGH);   
	  EEPROM.writeLong(1 * 4, 10000000);
	  EEPROM.writeLong(2 * 4, 16700000);
	  EEPROM.writeLong(3 * 4, 37200000);
	  EEPROM.writeLong(4 * 4, 64800000);
	  EEPROM.writeLong(5 * 4, 145000000);
	  EEPROM.writeByte(82, 1);
	  EEPROM.writeInt(80, 1);
	  EEPROM.writeLong(90, 0);
	  EEPROM.writeByte(99, 1);
	  delay(1000);
    digitalWrite(ledPin, LOW); 
  }
	band = EEPROM.readByte(82);
	Freq = EEPROM.readLong(band * 4);
	zone = EEPROM.readInt(80);
	correct_byte = EEPROM.readByte(99);
	correction = EEPROM.readLong(90);
	
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);

  Serial.begin(9600);                       // To serve for the GPS module
  
  // Set CLK0 to output 2,5MHz
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLA);
  si5351.set_freq(250000000ULL, SI5351_CLK1);
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLB);
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  si5351.set_freq(Freq * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.set_freq(Freq2, SI5351_CLK2);
  correct_si5351a();
  si5351.update_status();

  oled.clear();
  delay(100);
  
  oled.setCursor(0, 0);
  oled.print("GPS corrected VFO v1.2");
  oled.setCursor(0, 1);
  oled.print("5351a @ R909-VFO");
  delay(2000);
  
  oled.clear();
  oled.setCursor(0, 1);
  oled.print("Waiting for GPS");

  GPSproces(3000);

  if ( gps.charsProcessed() < 10) {
    oled.clear();  
    delay(100);
    oled.setCursor(0, 0);
    oled.print("No GPS connected");
    oled.setCursor(0, 1);
    oled.print (" check wiring!! ");
    
    GPSstatus = false;      // OK
    pps_error = 0;    
  }
  delay(2000);  
  
  // if GPS connected then...
  if (GPSstatus == true) { 
    oled.setCursor(0, 1);
    oled.print("Waiting for SAT");
    delay(100);

    time_on_oled();
    sat_on_oled();
  // Go out when !SAT(0) or RE-SW/on 
    do {
      GPSproces(1000);
    } while ((gps.satellites.value() == 0)&(digitalRead(re_sw) == 1 ));

    hour = (gps.time.hour() + zone) % 24;           // JPN: zone#9
    minute = gps.time.minute();
    second = gps.time.second();
    time_on_oled();
    sat_on_oled();
    attachInterrupt(0, PPSinterrupt, RISING);
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
  }
  
  oled.clear();  
  delay(50);  
  vfo_fr_oled ();

  sat_on_oled();
  time_on_oled();
  loglat_on_oled();
  delay(500);
  oled_turn = 1;
}

//***************************************************************************************
//                                         LOOP
//***************************************************************************************
void loop()
{
  if (tcount2 != tcount) {                   // PPS 1 sec check
    tcount2 = tcount;                        // L#202
    pps_correct_time = millis();             // Watching PPS period as 1 sec
  }
  if (tcount < 4 ) {                         // warm up up to 4
    GPSproces(0);                            // get GPS data
  }
  else   digitalWrite(ledPin, tcount%2 | pps_error);
  
  if (gps.time.isUpdated()) {
    hour = ((gps.time.hour() + zone)) % 24;
	  if(hour<0) hour = hour * -1;
    minute = gps.time.minute();
    second = gps.time.second();
    time_on_oled();
    loglat_on_oled();   
  }
  if (gps.satellites.isUpdated() && menu == 0) {
    sat_on_oled();
  }
  if (new_freq == 1) {                       // After correcting FREQ, display * @(15,0)
    correct_si5351a();
    new_freq = 0; 
//    oled.setCursor(120, 0);
//    oled.print("*");
  }
/*  if (new_freq != 1) {                       // No correcting FREQ, display * @(15,0)
    oled.setCursor(120, 0);
    oled.print(" ");
  }
  */  
  if (new_freq == 2) {                     // After setting new FREQ by rotating
    update_si5351a();
    vfo_fr_oled();
    new_freq = 0;
  }
  
  if (digitalRead(re_sw) == 0 ){           //re_sw pressed?
    button = true;                         // re_sw press
    button_on_time = millis();             // Time to avoid chattering
  }
  if (button == true && (button_on_time + 200) < millis() ) {   // re_sw settled
    menu++;
    if (menu > 6 ) menu = 1;                // increase menu 6 of correction
      switch (menu) {
        case 0:
          sat_on_oled();
          time_on_oled();
          time_enable = true;
          break;
        case 1:
          band_on_oled();
          break;
        case 2:
          step_on_oled();
          break;
        case 3:
          vfo_fr_oled();
          freq2_on_oled();
          break;
        case 5:
          EEPROM.writeLong(band * 4, Freq);
          stab_on_oled();
          break;
        case 4:
          timezone_on_oled();
          break;
        case 6:                              // added new menu
          corr_on_oled();                    // display correction value
          break;         
      }
    button = false ;
  }
  if(pps_error == 1){
    oled.setCursor (112, 0);
    oled.print("?");                         // Ararming PPS is out of order    
  }
  if (millis() > pps_correct_time + 1200) {  // pps timing is out of order
    pps_valid = 0;
    pps_correct_time = millis();
    time_enable = false;

    oled.setCursor (112, 0);
    oled.print("?");                         // Ararming PPS is out of order
  
  }
  else {
 
    oled.setCursor (112, 0);
    oled.print("!");   
  
  }
  // Encoder responce on signal changing interrupt
  if (left) {
    left = false;
    change_down(); 
//    oled.setCursor (112, 1);
//    oled.print(cnt1%10);        
  }

  if (right) {
    right = false;
    change_up();
//    oled.setCursor (118, 1);
//    oled.print(cnt2%10);          
  }
  // SW1:<80, SW2:<250 (actual 0, 166)
  // This set keeps 2 keys of SW1 & SW2 
  func_sw_value = analogRead(func_sw);
  if (func_sw_value < 100 ) {                 // SW2:menu=2/step
    menu = 2;
    step_on_oled(); 
  }
  else if (func_sw_value < 300  ) {           // SW1:menu=3/frequncy,
    menu = 3;
    vfo_fr_oled();
    freq2_on_oled();
  }
  oled.setCursor (108, 3);
  if (menu > 4) { 
    oled.print(mult%10);
    oled.print(tcount%100); 
    oled.print(" "); 
  }
  else oled.print("   "); 
}

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
    if((XtalFreq_old - XtalFreq)<1000000 | (XtalFreq - XtalFreq_old)<1000000 )
    {
	    correct_freq();
      pps_error = 0;
    }
    else pps_error = 1;
  }
}

//********************************************************************************
//                                TIMEZONE on OLED <>
//********************************************************************************
void timezone_on_oled()
{
  time_enable = false;
  oled.setCursor(0, 1);
  oled.print("TIME zone ");
  if (zone > 0) oled.print("+");
  oled.print(zone);
  oled.print(" < > ");  
}
//********************************************************************************
//                           STAB on OLED  stability index
//********************************************************************************
void stab_on_oled() {
  oled.setCursor(56, 1);
  oled.print("         ");
  oled.setCursor(0, 1);
  oled.print("Fstab ");

  oled.print(stab_float);
  oled.print("Hz    ");
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
//                                corr_on_OLED
//********************************************************************************
void corr_on_oled()                // Display correction frequency 
{
   oled.setCursor(0, 1);
   oled.print("CORR=");
   oled.print(correction/100);     // Freq is xxxxULL
   oled.print(".");
   oled.print(correction%100);
   oled.print("Hz      ");   
   
}
//********************************************************************************
//                                FREQ_2 on OLED <>
//********************************************************************************
void freq2_on_oled()
{
  time_enable = false;

  oled.setCursor(0, 1);
  oled.print("FREQ Bank ");
  oled.print(band);
  oled.print(" < > ");

}
//********************************************************************************
//                                STEP on OLED
//********************************************************************************
void step_on_oled()
{
  time_enable = false;

  oled.setCursor(0, 1);
  oled.print("STEP ");
  switch (f_step) {
    case 1: freq_step = 1, oled.print("   1Hz");
      break;
    case 2: freq_step = 10, oled.print("  10Hz");
      break;
    case 3: freq_step = 100, oled.print(" 100Hz");
      break;
    case 4: freq_step = 1000, oled.print("  1kHz");
      break;
    case 5: freq_step = 10000, oled.print(" 10kHz");
      break;
    case 6: freq_step = 100000, oled.print("100kHz");
      break;
    case 7: freq_step = 1000000 , oled.print("  1MHz");
      break;
    case 8: freq_step = 10000000, oled.print(" 10Mhz");
      break;
  }
  
}
//********************************************************************************
//                                BAND on OLED
//********************************************************************************
void band_on_oled()
{
  time_enable = false;
  oled.setCursor(0, 1);
  oled.print("BANK ");
  oled.print(band);
  oled.print("      < > ");
  
  Freq = EEPROM.readLong(band * 4);
  freq_on_oled(Freq);
  update_si5351a();
}
//********************************************************************************
//                                TIME on OLED
//********************************************************************************
void time_on_oled()
{
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d ", hour, minute, second);
  oled.setCursor(56, 2);
  oled.print(" ");
  oled.print(sz);

}
//********************************************************************************
//                                SAT number on OLED
//********************************************************************************
void sat_on_oled()
{
  time_enable = false;
  int sat_num = gps.satellites.value();  
  oled.setCursor(0, 2);
  oled.print("SAT ");
  oled.print(sat_num);
  oled.print("  ");

  time_enable = true;
}

//********************************************************************************
//                       Latitude & longtitude on oled
//********************************************************************************
void loglat_on_oled()
{
  oled.setCursor(0, 3);
  oled.print("LAT"+String(gps.location.lat(),2));
  oled.print(" LON"+String(gps.location.lng(),2));
}


//*********************************************************************************
//                             Freq on OLED
//*********************************************************************************
void freq_on_oled(unsigned long freq_disp) {
  char buf[10];

  // Print frequency to the LCD
  ltoa(Freq, buf, 10);
  time_enable = false;
  oled.setCursor(24, 0);
  delay(10);
  // Print frequency to the oled
  ltoa(freq_disp, buf, 10);
  time_enable = false;

  if (Freq < 1000000)
  {
    oled.print(" ");
    oled.print(" ");
    oled.print(" ");
    oled.print(" ");
    oled.print(buf[0]);
    oled.print(buf[1]);
    oled.print(buf[2]);
    oled.print('.');
    oled.print(buf[3]);
    oled.print(buf[4]);
    oled.print(buf[5]);
  }

  if (Freq >= 1000000 && Freq < 10000000)
  {
    oled.print(" ");
    oled.print(" ");
    oled.print(buf[0]);
    oled.print(',');
    oled.print(buf[1]);
    oled.print(buf[2]);
    oled.print(buf[3]);
    oled.print('.');
    oled.print(buf[4]);
    oled.print(buf[5]);
    oled.print(buf[6]);
  }

  if (Freq >= 10000000 && Freq < 100000000)
  {
    oled.print(" ");
    oled.print(buf[0]);
    oled.print(buf[1]);
    oled.print(',');
    oled.print(buf[2]);
    oled.print(buf[3]);
    oled.print(buf[4]);
    oled.print('.');
    oled.print(buf[5]);
    oled.print(buf[6]);
    oled.print(buf[7]);
  }

  if (Freq >= 100000000)
  {
    oled.print(buf[0]);
    oled.print(buf[1]);
    oled.print(buf[2]);
    oled.print(',');
    oled.print(buf[3]);
    oled.print(buf[4]);
    oled.print(buf[5]);
    oled.print('.');
    oled.print(buf[6]);
    oled.print(buf[7]);
    oled.print(buf[8]);
  }
  oled.print("kHz");
  
}

//********************************************************************
//             VFO frequency 
//********************************************************************
void vfo_fr_oled ()
{
    oled.setCursor(0, 0);
    delay(10);
    oled.print("VFO:");
    freq_on_oled(Freq);
    
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
           band_on_oled();
        }
        break;
        case 2: {
           f_step++;
           if (f_step > 8)f_step = 8;
           step_on_oled();
        }
        break;
        case 3: {
           Freq += freq_step;
           if (Freq > 200000000) Freq -= freq_step;
           new_freq = 2;
        }
        break;
        case 4: {
           zone++;
           if (zone > 12)zone = 12;
           EEPROM.writeInt(80, zone);
           timezone_on_oled();
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
        band_on_oled();
        }
            break;
          case 2: {
              f_step--;
              if (f_step == 0 )f_step = 1;
              step_on_oled();
            }
            break;
          case 3: {
              Freq -= freq_step;
              if (Freq > 200000000 || Freq < 1000) Freq += freq_step;
              new_freq = 2;
            }
            break;
          case 4: {
              zone--;
              if (zone < -12)zone = -12;
              EEPROM.writeInt(80, zone);
              timezone_on_oled();
            }
            break;
        }   
}
