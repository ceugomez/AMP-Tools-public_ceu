#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"
struct BenchmarkResult {
    int valid_solutions = 0;
    std::vector<double> path_lengths;
    std::vector<double> computation_times;
};
// 2-dimensional PRM planner
class PRM : public amp::PRM2D {    
    public:
        // default constructor 
        PRM()
            : max_iterations(7500),connectionradius(1.5), rng(std::random_device{}()) {}
        // Constructor with parameters
        PRM(int max_iterations, double connectionradius)
            : max_iterations(max_iterations), connectionradius(connectionradius), rng(std::random_device{}()) {}
         // Methods
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        Eigen::Vector2d sampleRandomPoint(const amp::Problem2D& problem);
        bool isPathFree(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem);
        void connectNeighbors(const amp::Node& newNode, std::map<amp::Node, Eigen::Vector2d>& nodes, const double& connectionradius, std::shared_ptr<amp::Graph<double>>& graphPtr, const amp::Problem2D& problem);
        BenchmarkResult benchmark_prm(int n, double r, const amp::Problem2D& problem);
    private: 
        std::mt19937 rng; 
        std::uniform_real_distribution<double> dist_x; 
        std::uniform_real_distribution<double> dist_y; 
        int max_iterations;
        double connectionradius;
};
// 2-dimensional RRT implementation
class RRT : public amp::GoalBiasRRT2D {
public:
    // default constructor
    RRT()
        : max_iterations(100000), step_size(0.25), goal_bias(0.3), goal_threshold(0.1), rng(std::random_device{}()) {}
    // Constructor with parameters
    RRT(int max_iterations, double step_size, double goal_bias, double goal_threshold)
        : max_iterations(max_iterations), step_size(step_size), goal_bias(goal_bias), goal_threshold(goal_threshold), rng(std::random_device{}()) {}
    // methods
    virtual amp::Path2D plan(const amp::Problem2D& problem) override;
    bool isPathFree(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem);
    Eigen::Vector2d sampleRandomPoint(const amp::Problem2D& problem, const double& goal_bias);
    int nearestNode(const amp::Problem2D& problem, const std::map<amp::Node, Eigen::Vector2d>& nodes, const Eigen::Vector2d& sample);
    Eigen::Vector2d extend(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
    BenchmarkResult benchmark_rrt(int n, double delta, double eta, double gt,const amp::Problem2D& problem);
private:
    std::mt19937 rng; // mersenne twister random number generator
    std::uniform_real_distribution<double> dist_x; 
    std::uniform_real_distribution<double> dist_y; 
    int max_iterations; 
    double step_size;
    double goal_bias;
    double goal_threshold;
};

class maRRT {
    public:
        amp::Path plan(const Eigen::VectorXd& init, const Eigen::VectorXd& goal, const amp::configurationSpace& collcheck);
    
}