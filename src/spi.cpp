#include "spi.h"

#define RPI_SPI_CHANNEL 0

void SPI::init(speed) {
  
  wiringPiSetupGpio();
  wiringPiSPISetup(RPI_SPI_CHANNEL, speed);

  pinMode(DEV_ADC_CS, OUTPUT);
  deselectDevice(DEV_ADC_CS);

  pinMode(DEV_ENS_CS, OUTPUT);
  deselectDevice(DEV_ENS_CS);

}

uint SPI::registerDevice(uint gpio) {
  devices.push_back(gpio);
  return (uint)devices.size();
}

void SPI::selectDevice(int gpio) {
  digitalWrite(gpio, LOW);
  delay(10);
}

void SPI::deselectDevice(int gpio) {
  digitalWrite(gpio, HIGH);
  delay(10);
}
