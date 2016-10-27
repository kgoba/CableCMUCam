#include "util.h"

float clip(float value, float minimum, float maximum) 
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}
