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
    double stepsize = 0.1;        // how big step size etc 
    Eigen::Vector2d closestPoint; // closest point to goal on obstacle

    // unpack initial position vector
    pos[0] = problem.q_init[0]; pos[1] = problem.q_init[1];
    goal[0] = problem.q_goal[0]; goal[1] = problem.q_goal[1];
    path.waypoints.push_back(pos);              // push initial waypoint
    double mlineangle = sniff(pos,goal);        // m-line
    // main planning loop
    while (!completepath && i <10000) {
        range2goal = sonar(pos, goal);
        angle2goal = sniff(pos, goal);
        switch (bugMode) {
            case 1: { // pure pursuit 
                if (range2goal < 0.5) {                             // if close to goal
                    bugMode = 3;                                    // send to goal mode
                    break;
                } else {                                            // test step along M-line
                    proposedStep[0] = pos[0] + cos(angle2goal)*stepsize;
                    proposedStep[1] = pos[1] + sin(angle2goal) *stepsize;
                    if (!boolCollision(proposedStep, problem)) {     // if M-line step does not collide with obstacle                   
                        path.waypoints.push_back(proposedStep);     // push m-line step
                        pos = proposedStep;                         // set current position to m-line step
                    } else {                                         // else 
                        bugMode = 2;                                // set to obstacle following mode   
                        LOG("Switching bug mode to 2 - detected collision in path ");            
                    }                           
                }
                break; 
            }
            case 2: { // follow object - BUG 2
                // identify object to be stepped over
                proposedStep[0] = pos[0] + cos(angle2goal)*stepsize;
                proposedStep[1] = pos[1] + sin(angle2goal)*stepsize;
                // get collision obstacle 
                std::vector<Eigen::Vector2d> obs = getCollisionObstacle(proposedStep, problem);
                    // move around obstacle 
                        int verticesTransited = 0;
                        int currentVertexIdx = getClosestVertex(pos, obs);
                        int nextCWVertexIdx;
                        Eigen::Vector2d currentVertexPoint;
                        Eigen::Vector2d nextCWVertexPoint;
                        double distMovedAlongEdge = 0;
                        double angle2goalfrompos;
                        double angle2goalfromstart;
                        double edgeLength;                                 
                        Eigen::Vector2d parallelVector;                    // vector parallel to edge to transit
                        bool breakCircumnavigateLoop = false;              // flag if the bug re-encounters m-line
                        bool breakEdgeLoop = false;                        // flag if the bug encounters another obstacle in transit around first obstacle 
                        std::vector<double> distToGoalInLoop;
                        std::vector<Eigen::Vector2d> posInLoop;
                        int circumnavigateVectorIDX = 0 ;
                        // circumnavigate obstacle
                        while (verticesTransited<obs.size()&&!breakCircumnavigateLoop){                 // while we have yet to circumnavigate the obstacle and haven't run into another obstacle
                            if (breakEdgeLoop){     
                                breakEdgeLoop=false;
                                currentVertexIdx = getClosestVertex(pos, obs);
                            }
                            nextCWVertexIdx = (currentVertexIdx + 1)%obs.size();                        // vertex index to move to next, clockwise 
                            currentVertexPoint = obs[currentVertexIdx];                                 // current vertex point
                            nextCWVertexPoint = obs[nextCWVertexIdx];                                   // next vertex point, clockwise
                            distMovedAlongEdge = 0;                                                     // distance moved along this edge 
                            edgeLength = (pos-nextCWVertexPoint).norm();                                // distance we need to move along the edge
                            parallelVector = parallelVecPoints(currentVertexPoint, nextCWVertexPoint);  // vector parallel to this edge
                            // move along edge
                            //LOG("Running edge of obstacle");
                            while (distMovedAlongEdge<(edgeLength+0.1) && !breakEdgeLoop){                                // while distance we need to move < distance we have moved 
                                distMovedAlongEdge = distMovedAlongEdge + (parallelVector*stepsize).norm();    // iterate distance moved
                                Eigen::Vector2d pos_step = pos + parallelVector*stepsize;                                           // shift position
                                if (!boolCollision(pos_step, problem)){                                      // if edge-parallel step doesn't intersect obstacle 
                                    path.waypoints.push_back(pos_step);                                      // push waypoints
                                    i++;
                                    angle2goalfromstart = sniff(problem.q_init, goal);
                                    angle2goalfrompos = sniff(pos_step, goal);
                                    pos = pos_step;
                                    if ((abs(abs(angle2goalfromstart)-abs(angle2goalfrompos)))<0.005){
                                        breakCircumnavigateLoop=true;
                                        //LOG("Switching to bug 1 mode");
                                        LOG("Switching bug mode to 1");
                                        bugMode=1;
                                    }
                                }else{                                                                      // else if our circumnavigation runs into an obstacle
                                    //LOG("Ran into obstacle while circumnavigating");          
                                    obs = getCollisionObstacle(pos, problem);                                      
                                    breakEdgeLoop = true;                                                   // break the loop and try again with next obstacle 
                                }
                            }
                            verticesTransited= verticesTransited+1;
                            currentVertexIdx = (currentVertexIdx+1)%obs.size();
                        }
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
    if (!(path.waypoints[i] == goal)){
        path.waypoints.push_back(goal);
    }
    return path;
}

Eigen::Vector2d MyBugAlgorithm::parallelVecPoints(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2){
    Eigen::Vector2d edgeVector = v2 - v1;
    return edgeVector.normalized();
}
Eigen::Vector2d MyBugAlgorithm::parallelVec(const std::vector<Eigen::Vector2d>& vertices, const Eigen::Vector2d& point) {   
    int startingVertex = getClosestVertex(point, vertices);
    // Ensure the next vertex index wraps around if it exceeds the number of vertices
    int nextVertex = (startingVertex + 1) % vertices.size();
    Eigen::Vector2d v1 = vertices[startingVertex];
    Eigen::Vector2d v2 = vertices[nextVertex];
    // Calculate the edge vector
    Eigen::Vector2d edgeVector = v2 - v1;
    return edgeVector.normalized();
}
// 
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
    