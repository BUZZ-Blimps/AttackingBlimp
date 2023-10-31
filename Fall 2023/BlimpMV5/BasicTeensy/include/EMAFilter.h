#pragma once

class EMAFilter {
    private:
    double alpha;
    
    public:
    void Init(double alpha);
    void Init();
    void setAlpha(double alpha);
    void setInitial(double initial);
    double filter(double current);
    double last;
};
