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
static states Cooler_State;
float h, t;


//states
void CoolerOffState();
void CoolerErrorState();
void CoolerIdleState();
void CoolerRunningState();


//headers
void LED(int pinNumber);
void adcIn();
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
  CoolerOffState();

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

    switch(Cooler_State){
      
      case(Off):
        //button pressed to turn on
        if(buttonMonitor() == 0){
          CoolerIdleState();
        }
        break;

      case(Idle):
        //temp and water check
        temperatureMonitor();
        humidityMonitor();
        displayLCD();
        if(t >= TEMP){
          RTCFcn();
          CoolerRunningState();
        }
        if(*Data < WATER){
          CoolerErrorState();
        }
        if(buttonMonitor() == 0){
          //button pressed to turn off
          CoolerOffState();
        }
        break;

       case(Running):
       //check temp and water
        temperatureMonitor();
        humidityMonitor();
        displayLCD();
        if(t < TEMP){
          RTCFon();
          CoolerIdleState();
        }
        if(*Data < WATER){
          RTCFon();
          CoolerErrorState();
        }
        if(buttonMonitor() == 0){
          //button pressed to print time.
          RTCF0n();
          //also turns off
          CoolerOffState();
        }
        break;

       case(Error):
        temperatureMonitor();
        humidityMonitor();
        displayLCD();
        if(*Data >= WATER){
          CoolerIdleState();
        }
        if(buttonMonitor() == 0){
          //button press to turn off
          CoolerDisableState();
        }
        break;
    }
}
