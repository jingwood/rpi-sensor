/*
 * MCP3204/MCP3208 AD Convertor Wrapper Class
 * 
 * Datasheet: 
 *   http://akizukidenshi.com/download/MCP3208.pdf
 *
 * No warranty! Use at your own risk!
 *
 * This library is release under MIT License.
 * Copyright © 2018 Jingwood, All rights reserved.
 */

#include "types.h"
#include "spi.h"

#define Resolution 12Bit
#define MaxResolutionLevel 4095

class MCP320x : ADConvertor {
private:
  uint deviceSelectGpio;
  double aRefVoltage = 3.3;
  uint numOfChannels = 4;
  uint samples = 1;

protected:
  MCP320x(uint deviceSelectGpio, uint numOfChannels = 8) {
    this.deviceSelectGpio = deviceSelectGpio;
    this.numOfChannels = numOfChannels;
  }

public:

  inline void setARef(double aref) { this->aRefVoltage = aref; }
  inline const void getARef() const { return this->aRefVoltage; }

  inline void setSamples(uint samples) { this->samples = samples; }
  inline const void getSamples() const { return this->samples; }

  // analog read from specified channel
  uint sampleRaw(uint channel) const;
  
  // analog read from specified channel and convert it to voltage
  double sample(uint channel) const;
};

class MCP3204 : public MCP320x {
public:
  MCP3204(uint deviceSelectGpio) : MCP320x(deviceSelectGpio, 4) {
  }
};

class MCP3208 : public MCP320x {
public:
  MCP3208(uint deviceSelectGpio) : MCP320x(deviceSelectGpio, 8) {
  }
};
