#include <stdio.h>
#include <string.h>
#include <math.h>

#include "sensor.h"

#define ISARG(arg) strcmp(argv[1], arg) == 0

void printHelp() {
    printf("available argument:\n  temperature battery_voltage brightness humidity pressure\n");
}

int main(int argc, char* argv[]) {
  init();

  if (argc<2){
    printHelp();
    return 1;
  }

  if (ISARG("temperature")) {
    printf("%lf\n", get_temperature(0));
  } else if (ISARG("battery_voltage")) {
    printf("%lf\n", get_battery_voltage());
  } else if (ISARG("brightness")) {
    printf("%d\n", get_brightness());
  } else if (ISARG("humidity")) {
    printf("%lf\n", get_humidity());
  } else if (ISARG("pressure")) {
    printf("%lf\n", get_pressure());
  } else if (ISARG("csv")) {
    double temp = get_temperature(0);
    double v12 = get_battery_voltage();
    double hum = get_humidity();
    int press = (int)round(get_pressure());
    uint brigh = get_brightness();
    printf("%0.1f,%0.1f,%d,%d,%0.2f\n", temp, hum, press, brigh, v12); 
  } else {
    printHelp();
  }

}

