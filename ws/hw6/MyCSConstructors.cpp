#include "MyCSConstructors.h"


std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    double x0min, x0max, x1min, x1max, delx0, delx1, x0dist, x1dist;
    std::size_t x0cell, x1cell;
    std::tie(x0min, x0max) = x0Bounds();  // bound space
    std::tie(x1min, x1max) = x1Bounds();
    std::tie(x0cell, x1cell) = size();
    delx0 = (x0max-x0min)/x0cell;  // get delta for each step 
    delx1 = (x1max-x1min)/x1cell;
    x0dist = x0 - x0min;               // dist to wkspc end
    x1dist = x1 - x1min;
    // get cell
    std::size_t i = x0dist/delx0; // x index of cell
    std::size_t j = x1dist/delx1; // x index of cell
    return {i,j};
}


// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for manipulator" << std::endl;
    
    
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

// build bool cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    // For every cell: check collision
    double stepx, stepy; // get cell dimensions for each axis
    stepx = (env.x_max-env.x_min)/m_cells_per_dim;
    stepy = (env.y_max-env.y_min)/m_cells_per_dim;

    //check every cell for obstacle:
    for (int i = 0; i < m_cells_per_dim; i++) {
        // iter through x dim
        for (int j=0; j<m_cells_per_dim; j++) {
            cspace(i,j) = false;
            double x = stepx*i; double y = stepy*j;
            for (amp::Obstacle2D obstacle : env.obstacles){
                // for every obstacle 
                if (isPointInPolygon(obstacle,Eigen::Vector2d(x,y))){
                    cspace(i,j=true);
                    break;
                }
            }

        }
    }
    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}
// helper functions
bool MyPointAgentCSConstructor::isPointInPolygon(const amp::Obstacle2D& obs, const Eigen::Vector2d& pos) {
    int windingNumber = 0;
    std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
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
double MyPointAgentCSConstructor::crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b){
    return a.x() * b.y() - a.y() * b.x();
}


amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    path.waypoints.push_back(q_goal);
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}
