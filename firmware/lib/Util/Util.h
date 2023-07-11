#include <Arduino.h>

class Util {
  public:
  void upper(char* s);
  double angleConversion(int leg, int joint, double angle);
  int inverse_angleConversion(int leg, int joint, double angle);
  double toDegrees(double radianVal);
  double maximum(double a0, double a1, double a2);
};
