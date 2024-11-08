#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"
#include "../shared/collisionCheckers.h"


class MyKinoRRT : public amp::KinodynamicRRT {
public:
    // default constructor 
    MyKinoRRT() : biasProb(0.05), controlIter(100) {}

    // parameterized constructor
    MyKinoRRT(double biasProb, int controlIter) 
        : biasProb(biasProb), controlIter(controlIter) {}
    virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

private:
    double biasProb;
    int controlIter;
    const int max_iter = 50000;
};


class MySingleIntegrator : public amp::DynamicAgent
{
public:
    virtual void propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt) override;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent
{
public:
    virtual void propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt) override;
};

class MySecondOrderUnicycle : public amp::DynamicAgent
{
public:
    virtual void propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt) override;
};

class MySimpleCar : public amp::DynamicAgent
{
public:
    virtual void propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt) override{};
};