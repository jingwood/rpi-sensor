
/*
 * BME280 Combined humidity and pressure sensor wrapper class
 *
 * Datasheet: 
 *   http://akizukidenshi.com/download/ds/bosch/BST-BME280_DS001-10.pdf
 *
 * References:
 *   http://trac.switch-science.com/wiki/BME280
 *
 * No warranty! Use at your own risk!
 *
 * This library is release under MIT License.
 * Copyright © 2018 Jingwood, All rights reserved.
 */

#include "sensor.h"

class BME280 {
private:
  signed long int t_fine;

  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t  dig_H6;

  readTrim();

public:
  void init();

  double getTemperature();
  double getHumidity();
  double getPressure();
};
