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
        bool boolCollision(const Eigen::Vector2d& pos, const amp::Problem2D problem);
        bool isPointInPolygon(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& vertices);
        bool boolCollision(Eigen::Vector2d& pos, const amp::Problem2D problem);
        double crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b); 
        std::vector<Eigen::Vector2d> getCollisionObstacle(Eigen::Vector2d& pos, const amp::Problem2D problem);
        Eigen::Vector2d parallelVec(const std::vector<Eigen::Vector2d>& vertices, const Eigen::Vector2d& point);
    private:
        // Add any member variables here...
};