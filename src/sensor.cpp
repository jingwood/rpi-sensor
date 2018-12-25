#include <stdio.h>
#include <math.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "sensor.h"

#define SPI_CHANNEL 0
#define DEV_ADC_CS  16
#define DEV_ENS_CS  4
#define CUR_DEV     DEV_ENS_SEL

#define ADC_CHN_TEMP_B1 0b00000110
#define ADC_CHN_TEMP_B2 0b01000000
#define ADC_CHN_V12T_B1 0b00000110
#define ADC_CHN_V12T_B2 0b10000000
#define ADC_CHN_BRIG_B1 0b00000111
#define ADC_CHN_BRIG_B2 0b00000000

static uint _samples = 5;

void setSamples(uint samples) {
  _samples = samples;
}

void SPI::init() {
  wiringPiSetupGpio();
  wiringPiSPISetup(SPI_CHANNEL, 500000);

  pinMode(DEV_ADC_CS, OUTPUT);
  deselectDevice(DEV_ADC_CS);

  pinMode(DEV_ENS_CS, OUTPUT);
  deselectDevice(DEV_ENS_CS);
}

