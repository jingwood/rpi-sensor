
#include "mcp320x.h"

#define _W(c) ((0b00000100 << 8) | (c << 10))
#define _U0(s) ((s >> 8) & 0xFF)
#define _U1(s) (s & 0xFF)

struct Command {
  union {
    ushort chn;
    struct {
      byte chn_b0;
      byte chn_b1;
    };
  }
};

static Command cmdChannels[] = {
  { _W(0) }, { _W(1) }, { _W(2) }, { _W(3) },
  { _W(4) }, { _W(5) }, { _W(6) }, { _W(7) },
}

uint MCP320x::sample(uint channel) const {
  const uint chn = _W(channel);
  double value = 0;

  for (int i = 0;i < this->samples; i++) {
    selectDevice(this.deviceSelectGpio);
    byte data[] = { _U0(chn), _U1(chn), 0 };
    wiringPiSPIDataRW(SPI_CHANNEL, data, 3);
    value += (((unsigned int)data[1] & 0x0f) << 8) | data[2];
    deselectDevice(this.deviceSelectGpio);
  }

  return (uint)round(value / samples);
}

double MCP320x::sample() const {
  return (double)value * this->aRefVoltage / 4096.0;
}
