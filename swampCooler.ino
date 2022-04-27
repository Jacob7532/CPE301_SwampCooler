#include <DHT.h>
#include <LiquidCrystal.h>

typedef enum{Off, Error, Idle, Running}

//memory for ADC
volatile uint8_t* const MUX = (uint8_t*) 0x7c;
volatile uint8_t* const SRA = (uint8_t*) 0x7a;
volatile uint8_t* const SRB = (uint8_t*) 0x7b;
volatile uint16_t* const Data = (uint16_t*) 0x78;

//macros
#define DHTPIN xx
#define DHTTYPE DHT11
#define SENSORPIN xx
#define WATER 100
#define TEMP 25


//class objects
DHT dht(DHTPIN, DHTTYPE);'LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
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
  DDRH |= (1 << PH5); //yellow
  DDRH |= (1 << PH6); //blue
  DDRB |= (1 << PB4); //red
  DDRB |= (1 << PB5); //green

  //button
  DDRB &= ~(1 << PB6);
  
  //motor
  DDRA |= (1 << PA1);
  DDRA |= (1 << PA3);
  DDRA |= (1 << PA5);

  //clock
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //LCD
  lcd.begin(16, 2)

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
        //button pressed to turn on
        if(button() == 0){
          coolerIdleState();
        }
        break;

      case(Idle):
        //temp and water check
        temperatureMonitor();
        humidity();
        displayLCD();
        if(t >= TEMP){
          RTCFcn();
          coolerRunningState();
        }
        if(*Data < WATER){
          coolerErrorState();
        }
        if(button() == 0){
          //button pressed to turn off
          coolerOffState();
        }
        break;

       case(Running):
       //check temp and water
        temperatureMonitor();
        humidity();
        displayLCD();
        if(t < TEMP){
          printRCT();
          coolerIdleState();
        }
        if(*Data < WATER){
          printRCT();
          coolerErrorState();
        }
        if(button() == 0){
          //button pressed to print time.
          RTCF0n();
          //also turns off
          coolerOffState();
        }
        break;

       case(Error):
        temperatureMonitor();
        humidity();
        displayLCD();
        if(*Data >= WATER){
          coolerIdleState();
        }
        if(button() == 0){
          //button press to turn off
          coolerDisableState();
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
  
}


void humidity(){
  
}


int button(){
  
}


void displayLCD(){
  
}


void printRCT(){
  
}


void adcInitialize(){
  
}


void adcDisable(){
  
}


void driveLow(){
  PORTB = 0b00000000
  PORTH = 0b00000000
}


//INTERRUPT


ISR(ADC_vect){
  
  *SRA |= (1 << 6);
  
}
