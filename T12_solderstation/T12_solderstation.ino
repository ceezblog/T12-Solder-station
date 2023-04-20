///////////////////////////////////////////////////////////////////////////////
// T12 solder station r0.3
//	Cee'z 30 Mar 2023
//
//	///////////////   ////////////////
//	//  T12 v0.1 //   //  T< 200*C  //
//	//   by Ceez //   //  H> 200*C  //
//	///////////////   ////////////////
//
///////////////////////////////////////////////////////////////////////////////
// Target MCU: ATMega328PB - MiniCore - Arduino IDE
// Clock: Ext 16MHz
//
// Character LCD 8x2 - board r0.3
//		Fn:      5V+    GND   Enable  RS    R/W   Contrast  DB4 DB5 DB6 DB7	
//		Pin no:	 2      1     6       4     5     3         11  12  13  14
//		Arduino: 5v+    GND   D6      D5    D7    pot-10k   D10 D8  D12 D11
//
// The gain: (470K/1K)+1 = 471 v/v
//
// For TIP that has 8OHM heater:
//    T = 0.403*(A0) + 49.60 
// ADC = 1023 => T = 498*C, set limit to 490
// ADC = 1023 => T = 52*C, set limit to 490
// For TIP that has 4 OHM heater: 
//    T = 0.6089*(ADC-25) degC // this 

#define MAX_TEMP 495 // At max ADC 1023

//#define _DEBUG
#include <avr/interrupt.h>
#include <EEPROM.h>

// LCD configuration
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

// Pin configuration
#define TIP_AN A0 // tip voltage
#define NTC_AN A1 // Thermistor for air temp
#define OPTO A2
#define IR_LED A3
#define CHECK_TIP A7

#define ENCA 2    // D2
#define ENCB 3    // D3
#define ENCBTN 4  // D4
#define TIP_PWM 9 // D9

#define DOCK_TEMPERATURE 100
#define NUMSAMPLES 8
#define SLEEP_T 100 // sleep temperature 100*C


// UPDATE RATE
#define HEATER_RATE 20
#define AIR_RATE 1000
#define OPTO_CHECK_RATE 5

// Thermistor reading
#define SERIES_RESISTOR 10000
#define VCC 4950
#define NTC_NOMINAL 10000      
#define NTC_TEMP_NOMINAL 25   
#define B_COEFF 3950


// Global variable
bool bSleep;
bool bHeating;
bool bHUpdate;
bool bAirUpdate;
bool bOptoCheck;
bool bEnableOpto;

bool bAir;
bool bTipFail;
bool bSetUpdate;


int16_t target_T; //target temperature
int16_t tip_T;
int16_t air_T;

// Counting time
uint16_t HeaterTick;
uint16_t AirTick;
uint16_t OptoTick;


// Encoder stuff
volatile byte A_STATE;
volatile byte B_STATE;
volatile byte RT_CCW;
volatile byte RT_CW;
volatile byte BT_PREV_STATE = HIGH;

// Atmega328, not sure for ATMEGA328PB
// PCINT0_vect for D8 to D13
// PCINT1_vect for A0 to A5
// PCINT2_vect for D0 to D7

// bouncing stuffs
unsigned long btDown_tm;
unsigned long btUp_tm;
unsigned long rot_next_tm;   
unsigned long rot_prev_tm;


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
  lcd.print(" by Ceez");  
  
  delay(2000);
  lcdSet_T();

  initTimer();
  // init variables
  bTipFail = true;

  bAirUpdate = true;
  bAir = true;
  target_T = eepromRead_T();
  bEnableOpto = eepromRead_Opto();

  //init PWM for controlling Heat element
  initPWM_Tip();
  initOpto();
  EncoderSetup();
  
  //check if tip is plugged
  pinMode(CHECK_TIP, INPUT_PULLUP);
}


bool bIsLongPress = false;

// Do not use any blocking function such as delay
void loop() 
{
  if (bSetUpdate)
  {
    bSetUpdate = 0;
    lcdSet_T();      
    resetAirTick();
    eepromWrite_T(target_T);
  }
  
  if (bIsLongPress)
  {
    bIsLongPress = false;
    bEnableOpto = !bEnableOpto;
    eepromWrite_Opto(bEnableOpto);
    lcdOpto();
    delay(1000);
    lcd_T();
    lcdSet_T();
  }

  if (BT_PREV_STATE == LOW && millis() - btDown_tm > 3000)
  {
    bIsLongPress = true;
  }

  //check if tip is plugged
  if (digitalRead(CHECK_TIP)== HIGH) bTipFail = true;
  else bTipFail = false;

  if (bOptoCheck) 
  {
    bOptoCheck = false; //done update
    
    bSleep = isDocking();
    if (!bEnableOpto) bSleep = false;
  }
  
  if (bHUpdate) 
  {
    bHUpdate = false; //done update
    bool m = tipUpdate();
    if (bTipFail) 
    {      
      lcdTipFail();
    }
    else 
    {
      if (!bSleep) lcd_T();  
      else lcdSleep();
    }
    
  }

  if (bAirUpdate)
  {
    bAirUpdate = false;
    bAir = !bAir;
    
    read_NTC();
    if (bAir) lcd_A();
    else lcdSet_T();
  }
}



////////////////////////////////
/// LCD write

void lcdTipFail()
{
  lcd.setCursor(0, 1);
  lcd.print("  FAIL  ");  
}

void lcdSleep()
{
  lcd.setCursor(0, 1);
  lcd.print(" SLEEP  ");  
}


void lcdOpto()
{
  lcd.setCursor(0, 0);
  lcd.print(" DOCKING");
  lcd.setCursor(0, 1);
  if (bEnableOpto) lcd.print("   ON   ");
  else lcd.print("   OFF  ");
}

void lcdSet_T()
{
  lcd.setCursor(0, 0);
  lcd.print("T");
  lcd.print((char)2);
  lcd.print(" ");
  lcd.print(target_T);
  lcd.print((char)223);
  lcd.print("C  ");
}

void lcd_T()
{
  lcd.setCursor(0, 1);
  lcd.print("H");
  lcd.print((char)0);
  lcd.print(" ");
  lcd.print(tip_T);
  lcd.print((char)223);
  lcd.print("C  ");
}
void lcd_A()
{
  lcd.setCursor(0, 0);
  lcd.print("Air ");
  air_T = read_NTC();
  lcd.print((int)air_T);
  lcd.print((char)223);
  lcd.print("C   ");
}

////////////////////////////////
// Read /write EEPROM
#define ADDR_H 1
#define ADDR_L 0
#define ADDR_OPTO 10

//#define ADDR_ONCE 2
#define DEFAULT_T 200
#define MAX_T MAX_TEMP
#define MIN_T 100



int16_t validate_T(int16_t T)
{
  if (T > MAX_T) return MAX_T;
  if (T < MIN_T) return MIN_T;
  return T;
}

//limit to 100degC
int16_t eepromRead_T()
{
  int16_t ret= 0;
  ret += EEPROM.read(ADDR_L);
  ret += EEPROM.read(ADDR_H)<<8;
  return validate_T(ret);
}

void eepromWrite_T(int16_t T)
{
  T = validate_T(T);  
  EEPROM.write(ADDR_L, T & 0xFF); //low byte
  EEPROM.write(ADDR_H, T >>8); //high byte
}

bool eepromRead_Opto()
{
  int ret = false;
  ret = EEPROM.read(ADDR_OPTO);
  if (ret == 0xFF) return true;
  if (ret > 0) return true;
  return false;
}

void eepromWrite_Opto(bool enable)
{
  if (enable) EEPROM.write(ADDR_OPTO, 1); 
  else EEPROM.write(ADDR_OPTO, 0);
}


////////////////////////////////
// T12 tip: read temp/ PWM out


//setup timer1 for PWM at D9
void initPWM_Tip()
{
  digitalWrite(TIP_PWM, HIGH);
  pinMode(TIP_PWM, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = TCCR1B & B11111000 | B00000001;
  OCR1A = 0;
}


// pwm = 0 // turn off
void PWM_TipOut(int pwm)
{
  pwm &= 0xFF; //get only low byte
  OCR1A = (byte)pwm;
}

// Read T12 temp
int16_t read_T12()
{
  // x8 sample
  int32_t T=0;
  for (int8_t i=0;i<8; i++) T += analogRead(TIP_AN);
 
  // For TIP that has 8 OHM heater:
  // T = 0.403*(A0) + 49.60
  // T = 403*A0 + 50
  
  T = ((int32_t)403*T/1000) >>3;
  return (int16_t)T + 50;
}

// increase target temp by 1 degree or 10 degree
bool multi = true;

void flipMulti()
{
  multi = !multi;
}

void plus_T()
{
  if (multi) target_T += 10;
  else target_T += 1;
  target_T = validate_T(target_T);
  bSetUpdate = 1;
}

void minus_T()
{
  if (multi) target_T -= 10;
  else target_T -= 1;
  target_T = validate_T(target_T);
  bSetUpdate = 1;
}

/// UPDATE T12 Temp
bool tipUpdate()
{
  int pwm_power;
  PWM_TipOut(0); // Turn off Heater
  delay(100); //let the temp settle down, stray voltage goes away

  tip_T = read_T12();  
  int16_t dT = tip_T;

  //limit temp to SLEEP_T (100degC)
  if (bSleep) dT = SLEEP_T - dT;
  else dT = target_T - dT;
  
  if (dT > 50) pwm_power = 255;
  else 
  {
    if (dT<=0) pwm_power = 0;
    else pwm_power = dT*255/50;
  }

  if (tip_T>550 || tip_T <= 0) bTipFail = true;

  // If Tip reading is normal Turn on TIP
  if (!bTipFail) PWM_TipOut(pwm_power);
  
  if (pwm_power==0) return false; //not heating
  return true;
}

void initOpto()
{
  digitalWrite(IR_LED, LOW);
  pinMode(OPTO, INPUT_PULLUP);
  pinMode(IR_LED, OUTPUT);
}

bool isDocking()
{
  digitalWrite(IR_LED, HIGH);
  bool ret = digitalRead(OPTO); //if HIGH == SLEEP
  digitalWrite(IR_LED, HIGH);
  return ret;
}


////////////////////////////////
/// READ THERMISTOR

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

void resetAirTick()
{
  AirTick =0;
  bAirUpdate = false;
}

// Call every 50ms
ISR(TIMER2_OVF_vect)        
{
  TCNT2 = 1310;   // preload counter for 40Hz
  HeaterTick++;
  AirTick++;
  OptoTick++;
  
  if (HeaterTick > HEATER_RATE) //update every 10 ticks
  {
    HeaterTick =0;
    bHUpdate = true;
  }
  
  if (AirTick > AIR_RATE)   //update every 1000 ticks
  {
    resetAirTick();
    bAirUpdate = true;
  }
  
  if (OptoTick > OPTO_CHECK_RATE)   //update every 5 ticks
  {
    OptoTick = 0;
    bOptoCheck = true;
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

////////////////////////////////
// Encoder


void EncoderSetup()
{
  pciSetup(ENCA);
  pciSetup(ENCB); 
  pciSetup(ENCBTN);
}

void pciSetup(byte pin)
{
  pinMode(pin, INPUT_PULLUP);
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


// vector PCINT2_vect for for D0 to D7
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  //push down
  if (digitalRead(ENCBTN)==LOW && BT_PREV_STATE == HIGH) 
  {
    BT_PREV_STATE = LOW;
    btDown_tm = millis();
    // run button down event    
    return;
  }
  //release
  if (digitalRead(ENCBTN)==HIGH && BT_PREV_STATE == LOW) 
  {
    BT_PREV_STATE = HIGH;
    btUp_tm = millis();
    // run button up event

    if (btUp_tm - btDown_tm > 10 || btUp_tm - btDown_tm < 200) flipMulti();
 
    return;
  }


  A_STATE = digitalRead(ENCA);
  B_STATE = digitalRead(ENCB);
  
  if (A_STATE && B_STATE) //both HIGH, reset
  {
    RT_CCW = 0;
    RT_CW = 0;
    return;
  }
  
  if (A_STATE==LOW && B_STATE==HIGH && RT_CW==LOW) RT_CW = 1; //enter Rotation CW
  if (A_STATE==HIGH && B_STATE==LOW && RT_CCW==LOW) RT_CCW = 1; //enter Rotation CW

  // do rotate CW, call function or let the main thread do the job
  if (RT_CW==HIGH && A_STATE==HIGH && B_STATE==LOW) 
  {
    //if happens within 5ms, ignore it
    if (millis() - rot_next_tm < 5) return; 
    plus_T();
    rot_next_tm = millis();
  }
  if (RT_CCW==HIGH && A_STATE==LOW && B_STATE==HIGH) 
  {
    if (millis() - rot_prev_tm < 5) return; 
    minus_T();
    rot_prev_tm = millis();
  }
}  
