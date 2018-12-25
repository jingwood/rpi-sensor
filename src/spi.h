
#include "types.h"

class SPI {
private:
  std::vector<uint> devices;

public:
  void init(uint speed = 100000);

  void selectDevice(uint gpio);
  void deselectDevice(uint gpio);
};
