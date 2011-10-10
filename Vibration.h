#include "openmath/SO3.hh"
#include "openwam/Plugin.hh"
#include "math.h"

class Vibration {
 public:
  Vibration(R3 direction, double amplitude, double frequency);

  R3 eval(double dt);

 private:
  double time;
  double amplitude, frequency;
  R3 direction;
  static const double twopi = 2.0*3.141592654;
  
};
