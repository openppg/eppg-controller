#pragma once
#include <cstdint>
namespace platform {
class St7735 {
 public:
  bool initialize(int,int,int,uint8_t) { return true; }
  bool writeRegion(int,int,int,int,const uint16_t*) { return true; }
};
}
