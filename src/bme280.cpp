
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

#include <stdio.h>
#include <math.h>

#include "bme280.h"

signed long int BME280::calibration_T(signed long int adc_T) {
  signed long int var1, var2, T;

  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = ((t_fine * 5 + 128) >> 8);
  return T;
}

unsigned long int BME280::calibration_P(signed long int adc_P) {
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1))*((signed long int)dig_P1)) >> 15);

  if (var1 == 0) {
    return 0;
  }

  P = (((unsigned long int)(((signed long int)1048576) - adc_P)-(var2 >> 12))) * 3125;
  if (P < 0x80000000) {
    P = (P << 1) / ((unsigned long int) var1);
  } else {
    P = (P / (unsigned long int)var1) * 2;
  }

  var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8)) >> 13;

  P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

unsigned long int BME280::calibration_H(signed long int adc_H) {
  signed long int v_x1;

  v_x1 = (t_fine - ((signed long int)76800));
  v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
           ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
           (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
           ((signed long int) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);

  return (unsigned long int)(v_x1 >> 12);
}

void BME280::writeReg(uint8_t reg_address, uint8_t data) {
  byte b1 = reg_address & 0b01111111, b2 = data;
  selectDevice(DEV_ENS_CS);
  wiringPiSPIDataRW(SPI_CHANNEL, &b1, 1);
  wiringPiSPIDataRW(SPI_CHANNEL, &b2, 1);
  deselectDevice(DEV_ENS_CS);
}

void BME280::readReg(uint8_t reg_address, uint8_t* data, int numBytes) {
  uint8_t addr = reg_address | 0b10000000;
  selectDevice(DEV_ENS_CS);

  wiringPiSPIDataRW(SPI_CHANNEL, &addr, 1);
  wiringPiSPIDataRW(SPI_CHANNEL, data, numBytes);

  deselectDevice(DEV_ENS_CS);
}

void BME280::init() {

  uint8_t osrs_t = 1;             // Temperature oversampling x 1
  uint8_t osrs_p = 1;             // Pressure oversampling x 1
  uint8_t osrs_h = 1;             // Humidity oversampling x 1
  uint8_t mode = 3;               // Normal mode
  uint8_t t_sb = 5;               // Tstandby 1000ms
  uint8_t filter = 0;             // Filter off
  uint8_t spi3w_en = 0;           // 3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;

  writeReg(0xF2, ctrl_hum_reg);
  writeReg(0xF4, ctrl_meas_reg);
  writeReg(0xF5, config_reg);

  readTrim();
}

void BME280::readTrim() {
  byte data[32];

  readReg(0x88, data, 24);
  readReg(0xA1, data + 24, 1);
  readReg(0xE1, data + 25, 7);

  dig_T1 = (data[1]  << 8) | data[0];
  dig_T2 = (data[3]  << 8) | data[2];
  dig_T3 = (data[5]  << 8) | data[4];
  dig_P1 = (data[7]  << 8) | data[6];
  dig_P2 = (data[9]  << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  dig_H1 = data[24];
  dig_H2 = (data[26] << 8) | data[25];
  dig_H3 = data[27];
  dig_H4 = (data[28] << 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
  dig_H6 = data[31];
}

void ens_sample() {
  byte data[8];
  readReg(0xF7, data, 8);

  int pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  int temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  int hum_raw  = (data[6] << 8) | data[7];

  int temp_cal  = calibration_T(temp_raw);
  int press_cal = calibration_P(pres_raw);
  int hum_cal   = calibration_H(hum_raw);

  double temp_act  = (double)temp_cal / 100.0;
  double press_act = (double)press_cal / 100.0;
  double hum_act   = (double)hum_cal / 1024.0;

  printf("    = %f (0x%x), %f (0x%x), %f (0x%x)\n", temp_act, temp_cal,
    hum_act, hum_cal, press_act, press_cal);
}

double BME280::getPressure() {
  byte data[3];
  readReg(0xF7, data, 3);

  int pres_raw     = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  int press_cal    = calibration_P(pres_raw);
  double press_act = (double)press_cal / 100.0;

  return press_act;
}

double BME280::getTemperature(int sensor) {
  byte data[3];
  readReg(0xFA, data, 3);

  int temp_raw    = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  int temp_cal    = calibration_T(temp_raw);
  double temp_act = (double)temp_cal / 100.0;

  return temp_act;
}

double BME280::getHumidity() {
  byte data[2];
  readReg(0xFD, data, 2);

  int hum_raw    = (data[0] << 8) | data[1];
  int hum_cal    = calibration_H(hum_raw);
  double hum_act = (double)hum_cal / 1024.0;

  return hum_act;
}