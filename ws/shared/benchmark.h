#pragma once
#include "AMPCore.h"
#include "../hw9/MyKinoRRT.h"
#include "hw/HW9.h"


struct benchmarkResult {
    std::vector<std::vector<double>> cputime;
    std::vector<std::vector<double>> pathlength;
    std::vector<double> validSolns; 
};

class benchmark {
    public: 
        static benchmarkResult benchmarkKinoRRT(const amp::KinodynamicProblem2D &problem, const std::tuple<int, std::vector<int>>& set);    // set = {n, u_samples}
    private:
};