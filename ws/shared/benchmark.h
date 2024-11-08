#pragma once
#include "AMPCore.h"
#include "hw9/MyKinoRRT.h"


struct benchmarkResult {
    std::vector<double> cputime;
    std::vector<double> pathlength;
    double validSolns; 
}

class benchmark {
    public: 
        benchmarkKinoRRT(const amp::KinodynamicProblem2D& problem);
    private:


}