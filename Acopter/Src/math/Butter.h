#ifndef __BUTTER_H
#define __BUTTER_H

class Butter2
{
public:
  float filter_100_05(float input)
  {
    static  float A1 = 1.9555782403f;
    static  float A2 =  -0.9565436765f;
    static  float GAIN = 4.143204922e+03f;
    float newhist = input + A1*hist[1] +A2*hist[0];
    float ret = (newhist + 2*hist[1] + hist[0])/GAIN;
    hist[0] = hist[1]; hist[1] = newhist;
    return ret;
  }
  double filter_50_8(double input)
  {
    static  float A1 = 0.6710290908f;
    static  float A2 =  -0.2523246263f;
    static  float GAIN = 6.881181354e+00f;
    double newhist = input + A1*histd[1] +A2*histd[0];
    double ret = (newhist + 2*histd[1] + histd[0])/GAIN;
    histd[0] = histd[1]; histd[1] = newhist;
    return ret;
  }
  float filter_100_025(float input)
  {
    static float A1 = 1.9777864838f;
    static float A2 = -0.9780305085f;
    static float GAIN = 1.639178228e+04f;
    float newhist = input + A1*hist[1] +A2*hist[0];
    float ret = (newhist + 2*hist[1] + hist[0])/GAIN;
    hist[0] = hist[1]; hist[1] = newhist;
    return ret;
  }
private:
    float hist[2];
    double histd[2];
};

#endif

    //100 3
    /*
        static  float A1 = 1.7347257688f;
        static  float A2 = -0.7660066009f;
        static  float GAIN = 1.278738361e+02f;
    */
    //100 1.5
    /*
        static float A1 = 1.8668922797f;
        static float A2 = -0.8752145483f;
        static float GAIN = 4.806381793e+02f;
    */
    //100 1
    /*
        static float A1 = 1.9111970674f;
        static float A2 = -0.9149758348f;
        static float GAIN = 1.058546241e+03f;
    */