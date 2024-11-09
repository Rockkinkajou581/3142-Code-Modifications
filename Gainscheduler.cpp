struct Gainvalues {
    std::vector<float> xvalues {};
    std::vector<float> yvalues {};
    std::vector<float> timeouts {};
    float lerp(float initialerror, int x1, int x2);
    float findgain(float initialerror);
};

#include "lemlib\chassis\gainscheduler.hpp"
float Gainvalues::findgain(float initialerror)  {
    if(fabs(initialerror) <= xvalues[0]) {
        return yvalues[0];
    }
    for (int i = 1; i <= xvalues.size() - 1; ++i) {
        if(fabs(initialerror) <= xvalues[i]) {
            return lerp(initialerror, i - 1, i);
        }
    }
    return yvalues[yvalues.size() - 1];
}
float Gainvalues::lerp(float initialerror, int x1, int x2) {
    float m = (yvalues[x2] - yvalues[x1])/(xvalues[x2] - xvalues[x1]);
    return (fabs(initialerror) - xvalues[x1])*m + yvalues[x1];
}
