/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Low Pass
Filter model: Rectangular Window
Sampling Frequency: 20 Hz
Cut Frequency: 2.000000 Hz
Coefficents Quantization: float
***************************************************************/
#define Ntap 8

float fir(float NewSample) {
    float FIRCoef[Ntap] = { 
        0.08997650465060308400,
        0.13443834128434992000,
        0.16588587705222707000,
        0.17723405264792491000,
        0.16588587705222707000,
        0.13443834128434992000,
        0.08997650465060308400,
        0.04216450137771470000
    };

    static float x[Ntap]; //input samples
    float y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];
    
    return y;
}
