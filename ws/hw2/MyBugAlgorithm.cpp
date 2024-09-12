#include "MyBugAlgorithm.h"
// simple 2d cross product of vec a & b
double MyBugAlgorithm::crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}
// angle bug to target
double MyBugAlgorithm::sniff(const Eigen::Vector2d& pos, const Eigen::Vector2d& goal) {
    return atan2(goal[1] - pos[1], goal[0] - pos[0]); // ray to goal
}
// range bug to target
double MyBugAlgorithm::sonar(const Eigen::Vector2d& pos, const Eigen::Vector2d& goal) {
    Eigen::Vector2d goalvec = pos - goal;
    double dist = sqrt(abs(goalvec[0] * goalvec[0] + goalvec[1] * goalvec[1]));
    return dist;
}
// check if point lies within set of ordered vertices
bool MyBugAlgorithm::isPointInPolygon(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& vertices) {
    int windingNumber = 0;

    for (size_t i = 0; i < vertices.size(); ++i) {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];

        if (v1.y() <= pos.y()) {
            if (v2.y() > pos.y() && crossProduct(v2 - v1, pos - v1) > 0) {
                ++windingNumber;
            }
        } else {
            if (v2.y() <= pos.y() && crossProduct(v2 - v1, pos - v1) < 0) {
                --windingNumber;
            }
        }
    }

    return windingNumber != 0;
}
int MyBugAlgorithm::getClosestVertex(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& vertices) {
    int vertexIdx = -1;
    double minDistance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < vertices.size(); ++i) {
        double distance = (vertices[i] - pos).norm();
        if (distance < minDistance) {
            minDistance = distance;
            vertexIdx = i;
        }
    }
    return vertexIdx;
}
// plan next step in path
std::vector<Eigen::Vector2d> MyBugAlgorithm::circumnavigateObstacle(const Eigen::Vector2d& pos, const amp::Polygon& obstacle) {
    std::vector<Eigen::Vector2d> steps;
    Eigen::Vector2d nextStep = pos;
    Eigen::Vector2d startPoint = pos;
    bool circumnavigated = false;

    while (!circumnavigated) {
        const std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
        double minDist = std::numeric_limits<double>::max();
        Eigen::Vector2d nearestEdgeDir;

        for (size_t i = 0; i < vertices.size(); ++i) {
            Eigen::Vector2d v1 = vertices[i];
            Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()]; // Wrap around to the first vertex
            Eigen::Vector2d edgeDir = (v2 - v1).normalized();

            // Project the position onto the edge to find the closest point on the edge
            Eigen::Vector2d proj = v1 + edgeDir * ((nextStep - v1).dot(edgeDir));
            double dist = (proj - nextStep).norm();

            // Update the nearest edge direction if this edge is closer
            if (dist < minDist) {
                minDist = dist;
                nearestEdgeDir = edgeDir;
            }
        }

        // Move along the nearest edge direction
        nextStep = nextStep + nearestEdgeDir / 25;
        steps.push_back(nextStep);

        // Check if we have circumnavigated the obstacle
        if ((nextStep - startPoint).norm() < 0.1) {
            circumnavigated = true;
        }
    }

    return steps;
}

Eigen::Vector2d MyBugAlgorithm::planStep(const Eigen::Vector2d& pos, const Eigen::Vector2d& goal, const double& angle2goal, const amp::Problem2D& problem) {
    static std::vector<Eigen::Vector2d> circumnavigationSteps;
    static size_t stepIndex = 0;

    // If there are remaining circumnavigation steps, return the next one
    if (stepIndex < circumnavigationSteps.size()) {
        return circumnavigationSteps[stepIndex++];
    }

    Eigen::Vector2d nextStep; 
    bool validStep = false;
    Eigen::Vector2d closestPointToGoal = pos;
    double minDistToGoal = (goal - pos).norm();

    // Propose some next step along M-line
    nextStep[0] = pos[0] + cos(angle2goal) / 25;
    nextStep[1] = pos[1] + sin(angle2goal) / 25;

    // Check if the proposed next step collides with any obstacle
    for (const amp::Polygon& obstacle : problem.obstacles) {
        if (isPointInPolygon(nextStep, obstacle.verticesCCW())) {
            // Circumnavigate the obstacle
            circumnavigationSteps = circumnavigateObstacle(pos, obstacle);
            stepIndex = 0;

            // Update the closest point to the goal
            for (const Eigen::Vector2d& step : circumnavigationSteps) {
                double distToGoal = (goal - step).norm();
                if (distToGoal < minDistToGoal) {
                    minDistToGoal = distToGoal;
                    closestPointToGoal = step;
                }
            }

            // Return the first step of the circumnavigation
            return circumnavigationSteps[stepIndex++];
        }
    }

    // Proceed towards the goal from the closest point found
    if (validStep) {
        nextStep = closestPointToGoal;
    }

    return nextStep;
}



amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d pos;        // position coordinates
    Eigen::Vector2d goal;       // goal coordinates
    Eigen::Vector2d vec2goal;   // vector from current position to goal
    bool completepath = false;  // flag for completion
    double range2goal;
    double angle2goal;
    int i = 0;                  // iterator, catchall
    // unpack initial position vector
    pos[0] = problem.q_init[0]; pos[1] = problem.q_init[1];
    goal[0] = problem.q_goal[0]; goal[1] = problem.q_goal[1];
    path.waypoints.push_back(pos);  // push initial waypoint
    // main planning loop
    while (!completepath && i < 10000) {
        // get direction and range to goal
        range2goal = sonar(pos, goal);
        angle2goal = sniff(pos, goal);
        // main state propagation loop
        if (range2goal > 0.5) {
            // if bug IS NOT close to the goal
            pos = planStep(pos, goal, angle2goal, problem);   // plan forward
            path.waypoints.push_back(pos);                                      // move forward
        } else {
            // if bug IS close to the goal
            path.waypoints.push_back(goal);                                     // move to goal
            completepath = true;                                                // flag path as complete
        }
        i = i + 1;
    }
    return path;
}
