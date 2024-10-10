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

std::pair<double, double> MyGridCSpace2D::getPointFromCell(int i, int j) const {
    double x, y, xmin, xmax, ymin, ymax, stepx, stepy, offsetx, offsety;
    int x_cells, y_cells;
    std::size_t x0cell, x1cell;
    std::tie(xmin, xmax) = x0Bounds();  // bound space
    std::tie(ymin, ymax) = x1Bounds();
    std::tie(x_cells, y_cells) = size();    // get number of cells in each dim

    stepx = (xmax-xmin)/x_cells;
    stepy = (ymax-ymin)/y_cells;
    offsetx=xmin; offsety =ymin;
    x = i*stepx+offsetx; y=j*stepy + offsety;
    return {x,y};
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

// build bool cspace for point 2d 
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    // For every cell: check collision
    double stepx, stepy, offsetx, offsety;  // get cell dimensions for each axis
    stepx = (env.x_max-env.x_min)/m_cells_per_dim;
    stepy = (env.y_max-env.y_min)/m_cells_per_dim;
    offsetx=env.x_min; offsety = env.y_min;

    //check every cell for obstacle:
    for (int i = 0; i < m_cells_per_dim; i++) {
        // iter through x dim
        for (int j=0; j<m_cells_per_dim; j++) {
            cspace(i,j) = false;
            double x = stepx*i+offsetx; double y = stepy*j+offsety;   //centerpoint of each cell
            for (amp::Obstacle2D obstacle : env.obstacles){
                // for every obstacle 
                if (isPointInPolygon(obstacle,Eigen::Vector2d(x,y))){
                    cspace(i,j) = true;
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
// collision checker
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
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    
    std::pair<int, int> start_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    std::pair<int, int> goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

    // build wavefront grid of values 
    std::vector<std::vector<int>> wavefront(grid_cspace.size().first, std::vector<int>(grid_cspace.size().first, -1));
    wavefront[goal_cell.first][goal_cell.second] = 0;
    // move along wavefront
    std::queue<std::pair<int, int>> q;
    q.push(goal_cell);
    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    while (!q.empty()) {
        auto cell = q.front();
        q.pop();
        int dist = wavefront[cell.first][cell.second];

        for (const auto& dir : directions) {
            int nx = cell.first + dir.first;
            int ny = cell.second + dir.second;
            if (nx >= 0 && ny >= 0 && nx < grid_cspace.size().first && ny < grid_cspace.size().first && wavefront[nx][ny] == -1 && !grid_cspace(nx, ny)) {
                wavefront[nx][ny] = dist + 1;
                q.push({nx, ny});
            }
        }
    }
    //retrace path
    std::pair<int, int> current_cell = start_cell;
    while (current_cell != goal_cell) {
        int min_dist = wavefront[current_cell.first][current_cell.second];
        std::pair<int, int> next_cell = current_cell;
        for (const auto& dir : directions) {
            int nx = current_cell.first + dir.first;
            int ny = current_cell.second + dir.second;
            if (nx >= 0 && ny >= 0 && nx < grid_cspace.size().first && ny < grid_cspace.size().first && wavefront[nx][ny] != -1 && wavefront[nx][ny] < min_dist) {
                min_dist = wavefront[nx][ny];
                next_cell = {nx, ny};
            }
        }
        if (next_cell == current_cell) {
            break;  // No path found
        }
        current_cell = next_cell;
        std::pair<double, double> waypoint = grid_cspace.getPointFromCell(current_cell.first, current_cell.second);
        path.waypoints.push_back(Eigen::Vector2d(waypoint.first, waypoint.second));
    }

    path.waypoints.push_back(q_goal);

    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2 * M_PI, 2 * M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }

    return path;
}
