#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        double sniff(const Eigen::Vector2d& pos, const Eigen::Vector2d& goal);
        double sonar(const Eigen::Vector2d& pos, const Eigen::Vector2d& goal);
        double crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b); 
        bool isPointInPolygon(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& vertices);
        Eigen::Vector2d planStep(const Eigen::Vector2d& pos, const Eigen::Vector2d& goal, const double& angle2goal, const amp::Problem2D& problem);
        int getClosestVertex(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& vertices);
        std::vector<Eigen::Vector2d> circumnavigateObstacle(const Eigen::Vector2d& pos, const amp::Polygon& obstacle);
    private:
        // Add any member variables here...
};