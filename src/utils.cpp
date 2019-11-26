#include "utils.h"

#include <math.h>

double normalize_angle(double angle) {
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0)
    angle += 360;
  return angle - M_PI;
}