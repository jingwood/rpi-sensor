#include <stdio.h>

class ADConvertor {
public:
  // analog read from specified channel and convert it to voltage
  virtual double sample() = 0;
};

void initSensor();

double get_battery_voltage();
uint get_brightness();
double get_temperature(int sensor);
double get_humidity();
double get_pressure();

void setSamples(uint samples);