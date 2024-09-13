#include "MyBugAlgorithm.h"

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d pos;        // position coordinates
    Eigen::Vector2d goal;       // goal coordinates
    Eigen::Vector2d proposedStep;// hypothetical next step
    bool completepath = false;  // flag for completion
    double range2goal;          // obvi
    double angle2goal;          // obvi
    int bugMode = 1;            // 0: pure pursuit; 1: follow obstacle; 2: straight to goal
    int i = 1;                  // iterator, catchall
    double stepsize = 4;        // how big step size etc 
    Eigen::Vector2d closestPoint; // closest point to goal on obstacle

    // unpack initial position vector
    pos[0] = problem.q_init[0]; pos[1] = problem.q_init[1];
    goal[0] = problem.q_goal[0]; goal[1] = problem.q_goal[1];
    path.waypoints.push_back(pos);  // push initial waypoint

    // main planning loop
    while (!completepath && i < 10000) {
        range2goal = sonar(pos, goal);
        angle2goal = sniff(pos, goal);
        switch (bugMode) {
            case 1: { // pure pursuit 
                if (range2goal < 0.5) {                             // if close to goal
                    bugMode = 3;                                    // send to goal mode
                    break;
                } else {                                            // test step along M-line
                    proposedStep[0] = pos[0] + cos(angle2goal) / 2;
                    proposedStep[1] = pos[1] + sin(angle2goal) / 2;
                    if (!boolCollision(proposedStep, problem)) {     // if M-line step does not collide with obstacle                   
                        path.waypoints.push_back(proposedStep);     // push m-line step
                        pos = proposedStep;                         // set current position to m-line step
                    } else {                                         // else 
                        bugMode = 2;                                // set to obstacle following mode              
                    }                           
                }
                break; 
            }
            case 2: { // follow object 
                // identify object to be stepped over
                proposedStep[0] = pos[0] + cos(angle2goal) / 4;
                proposedStep[1] = pos[1] + sin(angle2goal) / 4;
                std::vector<Eigen::Vector2d> obs = getCollisionObstacle(proposedStep, problem);

                // get parallel to obstacle nearest edge
                bool foundBestPoint = false;
                double minDist2Goal = std::numeric_limits<double>::max();
                int j = 0;
                while (!foundBestPoint&&(j<100)) {
                    double dist2goal = sonar(pos, goal);
                    // follow the edge of the obstacle with parallelVector
                    Eigen::Vector2d parallelVector = parallelVec(obs, pos);
                    // step forward by adding parallel vector to current position
                    proposedStep = pos + parallelVector / 4;
                    // push waypoint to path
                    path.waypoints.push_back(proposedStep);
                    // update current position
                    pos = proposedStep;
                    j = j+1;
                }
                // head back to the closest point to the goal
                pos = closestPoint;
                path.waypoints.push_back(pos);
                bugMode = 1; // switch back to pure pursuit mode
                break; 
            }
            case 3: { // direct to goal
                path.waypoints.push_back(goal);
                completepath = true;
                break;
            }
        }
        i = i + 1;
    }
    return path;
}

Eigen::Vector2d MyBugAlgorithm::parallelVec(const std::vector<Eigen::Vector2d>& vertices, const Eigen::Vector2d& point) {
    double minDistance = std::numeric_limits<double>::max();
    Eigen::Vector2d parallelVec;

    for (size_t i = 0; i < vertices.size(); ++i) {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];
        Eigen::Vector2d edgeVector = v2 - v1;

        // Calculate the projection of the point onto the edge
        Eigen::Vector2d pointVector = point - v1;
        double projection = pointVector.dot(edgeVector) / edgeVector.squaredNorm();
        projection = std::max(0.0, std::min(1.0, projection)); // Clamp projection to the edge segment

        Eigen::Vector2d closestPointOnEdge = v1 + projection * edgeVector;
        double distance = (point - closestPointOnEdge).norm();

        if (distance < minDistance) {
            minDistance = distance;
            parallelVec = edgeVector;
        }
    }

    // Normalize the closest edge vector to get the unit vector
    return parallelVec.normalized();
}

// get obstacle that point is colliding with
std::vector<Eigen::Vector2d> MyBugAlgorithm::getCollisionObstacle(Eigen::Vector2d& pos, const amp::Problem2D problem){
    std::vector<Eigen::Vector2d> OOI = problem.obstacles[1].verticesCW();
    for (const amp::Polygon& obstacle : problem.obstacles) {
        if (isPointInPolygon(pos, obstacle.verticesCW())) {
            OOI =  obstacle.verticesCW();
        } 
    }
    return OOI;
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
// returns true if point pos is within ANY obstacle
bool MyBugAlgorithm::boolCollision(Eigen::Vector2d& pos, const amp::Problem2D problem){
    bool foundcollision = false;
    for (const amp::Polygon& obstacle : problem.obstacles) {
        if (isPointInPolygon(pos, obstacle.verticesCW())) {
                foundcollision=true;
        } 
    }
return foundcollision;
} 
// returns true if point position is within SPECIFIED obstacle
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
// simple cross-product axb
double MyBugAlgorithm::crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}
