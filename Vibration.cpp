#include "Vibration.h"

Vibration::Vibration(R3 dir, double amp, double freq)
: time(0),
  amplitude(amp),
  frequency(freq),
  direction(dir)
{
}

R3 Vibration::eval(double dt) {
  time += dt;
  double magnitude = 0.5 * amplitude * sin(time * frequency * twopi);
  R3 world_direction = (SO3)OWD::Plugin::endpoint * direction;
  return magnitude*world_direction;
}
