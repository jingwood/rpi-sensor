
/*
 * LM60/LM61 Wrapper Class
 *
 * Datasheet: 
 *   LM60 http://akizukidenshi.com/download/LM60.pdf
 *   LM61 http://akizukidenshi.com/download/ds/ti/LM61CIZ.pdf
 *
 * No warranty! Use at your own risk!
 *
 * This library is release under MIT License.
 * Copyright © 2018 Jingwood, All rights reserved.
 */

#include "lm60.h"

double LM6x::getTemperature() {
  if (!this->adConvertor) return 0;

  const double v = this->adConvertor->sample();
  return (v - this->vOffset) / 0.00625;
}