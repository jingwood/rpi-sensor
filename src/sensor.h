#include <stdio.h>

typedef unsigned int uint;
typedef unsigned char byte;

void init();

double get_battery_voltage();
uint get_brightness();
double get_temperature(int sensor);
double get_humidity();
double get_pressure();

void setSamples(uint samples);