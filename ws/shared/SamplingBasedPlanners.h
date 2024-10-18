#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class PRM : public amp::PRM2D {    
    public:
        // Methods
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        Eigen::Vector2d sampleRandomPoint(const amp::Problem2D& problem);
        bool isPathFree(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem);
        void connectNeighbors(const amp::Node& newNode, std::map<amp::Node, Eigen::Vector2d>& nodes, const double& connectionradius, std::shared_ptr<amp::Graph<double>>& graphPtr, const amp::Problem2D& problem);
    private: 
        std::mt19937 rng; // Mersenne Twister random number generator
        std::uniform_real_distribution<double> dist_x; // Distribution for x-coordinate
        std::uniform_real_distribution<double> dist_y; // Distribution for y-coordinate
};

class RRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        bool isPathFree(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem);
        Eigen::Vector2d sampleRandomPoint(const amp::Problem2D& problem, const double& goal_bias);
        int nearestNode(const amp::Problem2D& problem, const std::map<amp::Node, Eigen::Vector2d>& nodes, const Eigen::Vector2d& sample);
        Eigen::Vector2d extend(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
    private:
        std::mt19937 rng; // Mersenne Twister random number generator
        std::uniform_real_distribution<double> dist_x; // Distribution for x-coordinate
        std::uniform_real_distribution<double> dist_y; // Distribution for y-coordinate
        int max_iterations = 1000;
        double step_size = 0.25;
        double goal_bias = 0.1;
        double goal_threshold = 0.15;
};
