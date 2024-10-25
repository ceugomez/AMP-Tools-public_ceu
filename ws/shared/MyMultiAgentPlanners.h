#pragma once
#include "AMPCore.h"
#include "hw/HW8.h"
#include "SamplingBasedPlanners.h" 


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
};