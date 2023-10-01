///////////////////////////////////////////////////////////////////////////////
// T12 solder station - This is used for calibration
//	Cee'z 30 Mar 2023
//  
//	///////////////
//	// 23*C-1023 //    Mosfet temp - ADC value
//	// TD-24*C   //    Tip plugged - Docking - Tip temperature
//	///////////////
//
///////////////////////////////////////////////////////////////////////////////
// Target MCU: ATMega328 - bootloader Arduino UNO
// Clock: Ext 16MHz
//
// Character LCD 8x2
//		Fn:      5V+    GND   Enable  RS    R/W   Contrast  DB4 DB5 DB6 DB7	
//		Pin no:	 2      1     6       4     5     3         11  12  13  14
//		Arduino: 5v+    GND   D6      D5    D7    pot-10k   D10 D8  D12 D11
//
// The gain: (470K/1K)+1 = 471 v/v
// Take from the chart
// T = 0.47*ADC + 45.98
// T = 0.47*(ADC + 97.83)
// new data:
// T = 0.403*ADC + 49.60


//#define _DEBUG
#include <avr/interrupt.h>
#include <EEPROM.h>

// new configuration
#include <LiquidCrystal.h>
#define LCD_EN 6
#define LCD_RS 5
#define LCD_RW 7
#define LCD_D4 10
#define LCD_D5 8 
#define LCD_D6 12
#define LCD_D7 11

LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// custom chars
byte nextc[8] = {B00000,B01000,B01100,B01110,B01110,B01100,B01000,B00000};
byte stopc[8] = {B00000,B11011,B11011,B11011,B11011,B11011,B11011,B00000};
byte backc[8] = {B00000,B00010,B00110,B01110,B01110,B00110,B00010,B00000};

bool bSetUpdate;
int16_t tip_T;
int16_t tip_ADC;
int16_t air_T;

#define CHECK_TIP A7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //add custom char;
  lcd.begin(8, 2);
  lcd.clear();
  lcd.createChar (0, nextc);
  lcd.createChar (1, stopc);
  lcd.createChar (2, backc);

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("T12 v0.1");
  lcd.setCursor(0, 1);
  lcd.print("Calibrate");  
  delay(2000);
  lcd.setCursor(0, 0);
  lcd.print("        ");
  lcd.setCursor(0, 1);
  lcd.print("        ");
    
  //EncoderSetup();
  
  //check if tip is 
  //pinMode(CHECK_TIP, INPUT_PULLUP);
  
  initTimer();
  // init variables

  //init PWM for controlling Heat element
  //initPWM_Tip();
  initOpto();

  //bAirUpdate = true;
  //bAir = true;
  
  //target_T = eepromRead_T();

}


// do not use any blocking function such as delay
void loop() 
{
  if (bSetUpdate)
  {
    read_T12();
    lcd_ADC_T();
    lcd_T();
    air_T = read_NTC();
    lcd_A();
    bSetUpdate = false; //updated
  }
  //isDocking();
  //isTipPlug();
}



////////////////////////////////
/// LCD write


void lcd_ADC_T()
{
  lcd.setCursor(3, 0);
  lcd.print(" ");
  lcd.print(tip_ADC);
  lcd.print("    ");
}
void lcd_T()
{
  lcd.setCursor(2, 1);
  lcd.print(" ");
  lcd.print(tip_T);
  //lcd.print((char)223);
  lcd.print("C  ");
}

void lcd_A()
{
  lcd.setCursor(0, 0);
  lcd.print((int)air_T);
  lcd.print("C");
}

////////////////////////////////
// T12 tip: read temp/ PWM out

// for docking check
#define OPTO A2
#define IR_LED A3
#define DOCK_TEMPERATURE 100


#define TIP_AN A0 // tip voltage
#define TIP_PWM 9 // D9
//#define NUM_OF_SAMPLE 4
#define SLEEP_T 100 // sleep temperature 60*C

//setup timer1 for PWM at D9
void initPWM_Tip()
{
  pinMode(TIP_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = TCCR1B & B11111000 | B00000001;
  OCR1A = 0;
}

// T = 0.47*ADC + 45.98
// T = 0.403*ADC + 49.60

// Read T12 temp
void read_T12()
{
  int32_t T=0;
  for (int i=0;i<8; i++)
    T += analogRead(TIP_AN);
  //T = T>>3;
  tip_ADC = T>>3; //average by 8 reading
  T = 403*tip_ADC + 45980;
  tip_T = (int16_t)T/1000;
  //return (int16_t)T;
}

// increase target temp
void initOpto()
{
  digitalWrite(IR_LED, LOW); //turn off IR_LED
  pinMode(OPTO, INPUT);
  pinMode(IR_LED, OUTPUT);
}

bool isDocking()
{
  digitalWrite(IR_LED, HIGH); //turn on IR_LED
  bool ret = !digitalRead(OPTO); //if LOW == SLEEP
  digitalWrite(IR_LED, LOW); //turn off IR_LED
  return ret;
}


////////////////////////////////
/// READ THERMISTOR

#define SERIES_RESISTOR 10000
#define VCC 4950
#define NTC_NOMINAL 10000      
#define NTC_TEMP_NOMINAL 25   

#define B_COEFF 3950

#define NUMSAMPLES 8
#define NTC_AN A1


float read_NTC()
{
  float Vo=0;
  //read n times
  for (int i=0; i< NUMSAMPLES; i++) 
    Vo += analogRead(NTC_AN);
    
  Vo /= NUMSAMPLES; //averaging
  
  // convert to resistance R_NTC
  Vo = SERIES_RESISTOR /(1023/Vo  - 1);     // 10K * (ADC - 1) / 1023
    
  float NTC_temp;

  // T_NTC = 1/ [ ln(R_NTC/Ro) / B + 1/(To) ]  deg Kelvin
  // 1/T_NTC = ln(R_NTC /Ro) / B + 1/To, where T_NTC and To in Kelvin
 
  NTC_temp = Vo / NTC_NOMINAL;                    // (R/Ro)
  NTC_temp = log(NTC_temp);                       // ln(R/Ro)
  NTC_temp /= B_COEFF;                            // 1/B * ln(R/Ro)
  NTC_temp += 1.0 / (NTC_TEMP_NOMINAL + 273.15);  // + (1/To)
  NTC_temp = 1.0 / NTC_temp;                      // Invert
  NTC_temp -= 273.15;                             // convert to C

  return NTC_temp;
}

////////////////////////////////
/// TIMER STUFF

uint16_t HeaterTick;
uint16_t AirTick;
uint16_t OptoTick;

// UPDATE RATE
// Heater 
#define HEATER_RATE 20
#define AIR_RATE 1000
#define OPTO_CHECK_RATE 5

void resetAirTick()
{
  //AirTick =0;
  //bAirUpdate = false;
}

// Call every 50ms
ISR(TIMER2_OVF_vect)        
{
  TCNT2 = 1310;   // preload counter for 20Hz
  HeaterTick++;
  AirTick++;
  OptoTick++;
  
  if (HeaterTick > HEATER_RATE) //update every 10 ticks
  {
    HeaterTick =0;
    bSetUpdate = true;
  }
}

/// Setup timer for 20Hz (or 50ms each INT event)
void initTimer()
{
  //initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  
  TCNT2 = 1310;            // preload timer 65536-16MHz/256/20Hz
  TCCR2B = TCCR2B & B11111000 | B00000111;    // 30.6Hz
  TIMSK2 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}
