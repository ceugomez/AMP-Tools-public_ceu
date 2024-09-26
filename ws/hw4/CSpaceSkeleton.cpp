#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    std::size_t cell_x = 0; // x index of cell
    std::size_t cell_y = 0; // x index of cell
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
