#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Override this method for returning whether or not a point is in collision
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implement your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    // simply returns the cell indices of my discrete grid when given an (x0, x1) state in continuous space. 
    // 100x100 cell grid
    double m = 2*std::numbers::pi/100;
    // get what cell it's in
    std::size_t cell_x = floor(x0/m); // x index of cell
    std::size_t cell_y = floor(x1/m); // x index of cell
    return {cell_x, cell_y};
}


std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()

    double pi = M_PI;
    double cspace_xmin = 0.0;
    double cspace_xmax = 2*pi;
    double cspace_ymin = 0.0;
    double cspace_ymax = 2*pi;

    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, cspace_xmin, cspace_xmax, cspace_ymin, cspace_ymax);
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Iterate through every discretization of t1 and t2
    for (int i = 0; i < m_cells_per_dim; ++i) {              // for each element in dimension 1
        for (int j = 0; j < m_cells_per_dim; ++j) {          // for each element in dimension 2
            double n = manipulator.nLinks();
            amp::ManipulatorState cspace_state(2);                          // initialize cspace-state
            cspace_state[0] = i * (2 * std::numbers::pi / m_cells_per_dim); // set joint angle, evenly spaced between 0 and 2pi, for x1
            cspace_state[1] = j * (2 * std::numbers::pi / m_cells_per_dim); // set joint angle, evenly spaced between 0 and 2pi, for x2

            // Ensure indices are within bounds
            if (i >= m_cells_per_dim || j >= m_cells_per_dim) {
                LOG("Index out of bounds: i = " + std::to_string(i) + ", j = " + std::to_string(j));
                continue;
            }

            cspace(i,j) = false;    // initialize to false
            Eigen::Vector2d endpos_rspace = manipulator.getJointLocation(cspace_state, n);

            for (const amp::Polygon& obstacle : env.obstacles) {
                if (cspace.isPointInPolygon(endpos_rspace, obstacle.verticesCW())) {    // if end effector position is within an obstacle
                    //LOG(endpos_rspace);
                    cspace(i,j) = true;
                    break;
                }
            }
        }
    }
    return cspace_ptr;
}

// Helper functions
std::vector<Eigen::Vector2d> MyGridCSpace2D::getMinkowskiSumRobotObstacle(const amp::Obstacle2D& obstacle, const amp::Obstacle2D& robot) {
    std::vector<Eigen::Vector2d> CSpaceObstacle;
    
    // Get the vertices of the obstacle and the robot
    const std::vector<Eigen::Vector2d>& obstacleVertices = obstacle.verticesCCW();
    const std::vector<Eigen::Vector2d>& robotVertices = robot.verticesCCW();
    Eigen::Vector2d obstacleCentroid;
    int n = obstacleVertices.size();
    double sumX; double sumY;
    for (const auto& vertex : obstacleVertices) {
        sumX += vertex.x();
        sumY += vertex.y();
    }


    obstacleCentroid.x() = (sumX/n);
    obstacleCentroid.y() = (sumY/n);

    for (int i=0; i<=obstacleVertices.size(); ++i){
        // for every vertex in the obstacle 

        for (int j=0; j<=robotVertices.size(); ++j){
            // for every vertex in the robot
            // sum vertex to point 
            Eigen::Vector2d point = robotVertices[j]+obstacleVertices[i];
            Eigen::Vector2d diff = obstacleCentroid - point;
            CSpaceObstacle.push_back(point);
        }
    }
    
    return CSpaceObstacle;
}
bool MyGridCSpace2D::isPointInPolygon(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& vertices) {
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
double MyGridCSpace2D::crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b){
    return a.x() * b.y() - a.y() * b.x();
}
std::vector<double> MyGridCSpace2D::linspace(double start, double end, int num) {
    std::vector<double> result;
    if (num <= 0) return result;
    if (num == 1) {
        result.push_back(start);
        return result;
    }
    double step = (end - start) / (num - 1);
    for (int i = 0; i < num; ++i) {
        result.push_back(start + i * step);
    }
    return result;
}