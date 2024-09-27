#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implement your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    // simply returns the cell indices of my discrete grid when given an (x0, x1) state in continuous space. 
    // 100x100 cell grid
    double gridcell_size = 2*std::numbers::pi/100;
    // get what cell it's in
    std::size_t cell_x = floor(x0/gridcell_size); // x index of cell
    std::size_t cell_y = floor(x1/gridcell_size); // x index of cell
    return {cell_x, cell_y};
}
// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    cspace(1, 3) = true;
    cspace(3, 3) = true;
    cspace(0, 1) = true;
    cspace(1, 0) = true;
    cspace(2, 0) = true;
    cspace(3, 0) = true;
    cspace(4, 1) = true;

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
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