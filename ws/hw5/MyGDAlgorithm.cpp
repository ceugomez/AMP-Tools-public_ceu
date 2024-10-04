#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    std::default_random_engine rgen;
    std::uniform_real_distribution<double> perturb(-1,1);    
    double max_n = 100000;
    double alpha = 0.0; // Momentum factor
    double stepsize = 0.5e-2;
    Eigen::Vector2d prev_step = Eigen::Vector2d::Zero();
    
    // initialize path
    amp::Path2D path;
    // initialize potential function
    path.waypoints.push_back(problem.q_init);
    MyPotentialFunction f(problem);
    // runtime loop, max iter max_n
    for (int n = 0; n < max_n; n++) { 
        Eigen::Vector2d pos = path.waypoints.back();
        // evaluate the gradient of the potential function at previous waypoint
        Eigen::Vector2d del = f.getGradient(pos)*stepsize;
        if (del.norm()<1e-5){   // perturb if stuck
            del += Eigen::Vector2d(perturb(rgen), perturb(rgen))*55*stepsize;
        }
        // sum momentum term
        Eigen::Vector2d step = del + alpha * prev_step;
        prev_step = step;
        // take step in gradient direction
        path.waypoints.push_back(pos - step);
        // check for convergence to goal state
        if (checkEnd(pos, problem.q_goal)) {
            break;  // send to goal if end condition reached
        }
        // flag infinite loop
        if (n > max_n) {
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
