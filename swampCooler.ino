#include <DHT.h>
#include <LiquidCrystal.h>
#include <RTClib.h>

typedef enum{Off, Error, Idle, Running}

//memory for ADC
volatile uint8_t* const MUX = (uint8_t*) 0x7c
volatile uint8_t* const SRA = (uint8_t*) 0x7a
volatile uint8_t* const SRB = (uint8_t*) 0x7b
volatile uint16_t* const Data = (uint16_t*) 0x78
