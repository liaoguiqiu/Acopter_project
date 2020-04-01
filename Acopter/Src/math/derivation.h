#pragma once

class Derivation
{
public:
  double derivate_filter(double input,float T)
  {
        histd[1] = input ;
        double ret = (histd[1] - histd[0])/T;
        histd[0] = histd[1];
        return ret;
  }
private:
    float hist[2];
    double histd[2];
};

Derivation derivate;