
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

class LM6x {
private:
  double vOffset = 0;
  const ADConvertor* adConvertor = NULL;
  const uint adChannel = 0;

protected:
  LM6x(double vOffset, const ADConvertor* adConvertor) 
    : vOffset(vOffset), adConvertor(adConvertor) {
  }

public:
  double getTemperature() const;
};

class LM60 : public LM6x {
public:
  LM60(const ADConvertor* adConvertor, uint adChannel = 0) 
    : LM6x(0.424, adConvertor), adChannel(adChannel) {
  }
};