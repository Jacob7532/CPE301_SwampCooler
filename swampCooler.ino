#include <DHT.h>
#include <LiquidCrystal.h>
#include <RTClib.h>

typedef enum{Off, Error, Idle, Running} states;

//memory for ADC
volatile uint8_t* const MUX = (uint8_t*) 0x7c;
volatile uint8_t* const SRA = (uint8_t*) 0x7a;
volatile uint8_t* const SRB = (uint8_t*) 0x7b;
volatile uint16_t* const Data = (uint16_t*) 0x78;

//macros
#define DHTPIN 14
#define DHTTYPE DHT11
#define SENSORPIN 0
#define WATER 100
#define TEMP 25


//class objects
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
RTC_DS1307 rtc;


//variables
static states coolerState;
float h, t;


//states
void coolerOffState();
void coolerErrorState();
void coolerIdleState();
void coolerRunningState();


//headers
void LED(int pinNumber);
void adcInitialize();
void adcDisable();


void setup(){

  //lights
  DDRB |= (1 << PB4); //yellow
  DDRH |= (1 << PH5); //blue
  DDRB |= (1 << PB5); //red
  DDRH |= (1 << PH6); //green

  //buttonOn
  DDRB &= ~(1 << PB7);

  //buttonVent
  DDRH &= ~(1 << PH4);

  //buttonReset
  DDRB &= ~(1 <<PB6);
  
  //fan
  DDRG |= (1 << PG5);
  DDRE |= (1 << PE5);
  DDRE |= (1 << PE4);

  //vent
  DDRC |= (1 << PC1);
  DDRD |= (1 << PD7);
  DDRG |= (1 << PG1);
  DDRL |= (1 << PL7);

  //clock
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //LCD
  lcd.begin(16, 2);

  //cooler
  coolerOffState();

  //Enable interrupts
  sei();

  //DHT
  dht.begin();

  //serial port
  Serial.begin(9600);
  
}


void loop(){

  statesMachine();

}


void statesMachine(){

  adcDisable();

    switch(coolerState){
      
      case(Off):
        //buttonOn pressed to turn on
        if(buttonOn() == 0){
          coolerIdleState();
        }
        break;

      case(Idle):
        //temp and water check
        temperature();
        humidity();
        displayLCD();
        if(t >= TEMP){
          printRTC();
          coolerRunningState();
        }
        if(*Data < WATER){
          coolerErrorState();
        }
        if(buttonOn() == 0){
          //buttonOn pressed to turn off
          coolerOffState();
        }
        break;

       case(Running):
       //check temp and water
        temperature();
        humidity();
        displayLCD();
        if(t < TEMP){
          printRTC();
          coolerIdleState();
        }
        if(*Data < WATER){
          printRTC();
          coolerErrorState();
        }
        if(buttonOn() == 0){
          //buttonOn pressed to print time.
          printRTC();
          //also turns off
          coolerOffState();
        }
        break;

       case(Error):
        temperature();
        humidity();
        displayLCD();
        if(*Data >= WATER){
          coolerIdleState();
        }
        if(buttonOn() == 0){
          //buttonOn press to turn off
          coolerOffState();
        }
        break;
    }
}


//FUNCTIONS


void coolerOffState(){
  
  adcDisable();
  driveLow();
  coolerState = Off;
  //yellow on
  PORTH = 0b00100000;
  //motor off
  PORTA = 0b00000010;
  
}


void coolerErrorState(){

  driveLow();
  coolerState = Error;
  //red on
  PORTB = 0b00010000;
  //motor off
  PORTA = 0b00000010;
  
}


void coolerIdleState(){

  adcInitialize();
  driveLow();
  coolerState = Idle;
  //green on
  PORTB = 0b00100000;
  //display to the lcd
  displayLCD();
  
}


void coolerRunningState(){
  
  driveLow();
  coolerState = Running;
  //blue on
  PORTH = 0b01000000;
  //motor on
  PORTA = 0b00001010;
  //display to the lcd
  displayLCD();
  
}


void temperature(){
  
  adcDisable();
  float previous = t;
  t = dht.readTemperature();
  if(isnan(t)){
    t = previous;
  }
  adcInitialize();
  
}


void humidity(){
  
  adcDisable();
  float previous = h;
  h = dht.readHumidity();
  if(isnan(h)){
    h = previous;
  }
  adcInitialize();
  
}


int buttonOn(){

  adcDisable();
  //debouncing
  if((PINB & (1 << PB6))!=0){
    _delay_ms(250);
  }
  if((PINB & (1 << PB6))!=0){
    return 0;
  }
  else{
    return 1;
  }
  adcInitialize();
  
}


void displayLCD(){
  adcDisable();
  if(coolerState == Off){
    return;
  }
  else{
    if(coolerState == Error){
      lcd.setCursor(0,0);
      lcd.print("Error");
    }
    lcd.setCursor(0,1);
    lcd.print(t);
    lcd.print("C ");
    lcd.print(h);
    lcd.print("%");
  }
  adcInitialize();
  
}


void printRTC(){

  DateTime now = rtc.now();
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.println();
  
}


void adcInitialize(){

  //bits 0,1,2,3,4 for pin selection
  //bits 6,7 for refrence selection
  *MUX = 0b01000000;
  //bit 0,1,2 prescaling
  //bit 3 interrupt
  //bit 4 signals conversion finish
  //bit 5 contols autotriggers
  //bit 6 conversion initialization
  //bit 7 enable adc
  *SRA = 0b10001000;
  //bit 0,1,2 control autotrigger
  //bit 6 comperator control
  *SRB = 0b00000000;
  //choose input pin
  *MUX |= SENSORPIN;
  //conversion initialization
  *SRA |= (1 << 6);
  
}


void adcDisable(){
  //bit 7 to 0 to disable
}


void driveLow(){
  PORTB = 0b00000000;
  PORTH = 0b00000000;
}


//INTERRUPT


ISR(ADC_vect){

  //interrupts and restarts conversion
  *SRA |= (1 << 6);
  
}
