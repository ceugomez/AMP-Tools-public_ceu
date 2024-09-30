#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    // initialize path
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    // initialize potential function
    MyPotentialFunction f;

    // runtime loop, max iter 1000
    for (int n=0; n<1000; n++) { 
        Eigen::Vector2d pos = path.waypoints.back();
        // evaluate the gradient of the potential function at previous waypoint;
        Eigen::Vector2d del = f.gradient(pos);
        LOG(del);
        // take step in gradient direction
        path.waypoints.push_back((pos-del));
        // check for convergence to goal state
        if (checkEnd(pos, problem.q_goal)){
            break;
        }
        if (n>998){
            LOG("reached iterator limit - possible local minima");
        }
    }
    // send robot to goal
    path.waypoints.push_back(problem.q_goal);
    return path;
}
// end condition
bool MyGDAlgorithm::checkEnd(const Eigen::Vector2d& pos, const Eigen::Vector2d& goal){
    Eigen::Vector2d dist = (pos-goal);
    return((dist.norm()<0.25));
}
