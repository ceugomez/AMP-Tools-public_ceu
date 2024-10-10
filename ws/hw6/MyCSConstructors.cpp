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
    double pi = M_PI;
    auto cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0.0, 2 * pi, 0.0, 2 * pi);
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for manipulator" << std::endl;

    for (int i = 0; i < m_cells_per_dim; ++i) {
        for (int j = 0; j < m_cells_per_dim; ++j) {
            cspace(i, j) = false;
            double x, y;
            std::tie(x, y) = cspace.getPointFromCell(i, j);

            for (const auto& obs : env.obstacles) {
                if (collisionCheckers::isArmInCollision(obs, Eigen::Vector2d(x, y), manipulator)) {
                    cspace(i, j) = true;
                    break;
                }
            }
        }
    }
    return cspace_ptr;
}


// build bool cspace for point 2d 
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    auto cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;

    double stepx = (env.x_max - env.x_min) / m_cells_per_dim;
    double stepy = (env.y_max - env.y_min) / m_cells_per_dim;
    double offsetx = env.x_min;
    double offsety = env.y_min;

    for (int i = 0; i < m_cells_per_dim; ++i) {
        for (int j = 0; j < m_cells_per_dim; ++j) {
            cspace(i, j) = false;
            double x, y;
            std::tie(x, y) = cspace.getPointFromCell(i, j);

            for (const auto& obstacle : env.obstacles) {
                if (collisionCheckers::isPointInPolygon(obstacle, Eigen::Vector2d(x, y))) {
                    cspace(i, j) = true;
                    break;
                }
            }
        }
    }
    return cspace_ptr;
}

void MyWaveFrontAlgorithm::addObstacleBuffer(amp::GridCSpace2D& grid_cspace, int buffer_size) {
    auto size = grid_cspace.size();
    std::vector<std::vector<int>> buffer_map(size.first, std::vector<int>(size.second, 0));

    for (std::size_t i = 0; i < size.first; ++i) {
        for (std::size_t j = 0; j < size.second; ++j) {
            if (grid_cspace(i, j)) { // Check if cell is an obstacle
                for (int dx = -buffer_size; dx <= buffer_size; ++dx) {
                    for (int dy = -buffer_size; dy <= buffer_size; ++dy) {
                        int nx = i + dx;
                        int ny = j + dy;
                        if (nx >= 0 && ny >= 0 && nx < size.first && ny < size.second) {
                            buffer_map[nx][ny] = 1;
                        }
                    }
                }
            }
        }
    }

    for (std::size_t i = 0; i < size.first; ++i) {
        for (std::size_t j = 0; j < size.second; ++j) {
            if (buffer_map[i][j] == 1) {
                grid_cspace(i, j) = 1;
            }
        }
    }
}

amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    amp::Path2D path;
    path.waypoints.push_back(q_init);

    std::pair<int, int> start_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    std::pair<int, int> goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

    // Cast grid_cspace to MyGridCSpace2D
    auto* buffered_grid_ptr = dynamic_cast<const MyGridCSpace2D*>(&grid_cspace);
    if (!buffered_grid_ptr) {
        std::cerr << "Error: grid_cspace is not of type MyGridCSpace2D\n";
        return path;
    }

    MyGridCSpace2D buffered_grid = *buffered_grid_ptr;

    addObstacleBuffer(buffered_grid, 3); // Add 3-cell buffer

    std::vector<std::vector<int>> wavefront(buffered_grid.size().first, std::vector<int>(buffered_grid.size().second, -1));
    wavefront[goal_cell.first][goal_cell.second] = 0;

    // Move along wavefront
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
            if (nx >= 0 && ny >= 0 && nx < buffered_grid.size().first && ny < buffered_grid.size().second && wavefront[nx][ny] == -1 && !buffered_grid(nx, ny)) {
                wavefront[nx][ny] = dist + 1;
                q.push({nx, ny});
            }
        }
    }

    // Retrace path
    std::pair<int, int> current_cell = start_cell;
    while (current_cell != goal_cell) {
        int min_dist = wavefront[current_cell.first][current_cell.second];
        std::pair<int, int> next_cell = current_cell;

        for (const auto& dir : directions) {
            int nx = current_cell.first + dir.first;
            int ny = current_cell.second + dir.second;
            if (nx >= 0 && ny >= 0 && nx < buffered_grid.size().first && ny < buffered_grid.size().second && wavefront[nx][ny] != -1 && wavefront[nx][ny] < min_dist) {
                min_dist = wavefront[nx][ny];
                next_cell = {nx, ny};
            }
        }

        if (next_cell == current_cell) {
            break;  // No path found
        }

        current_cell = next_cell;
        std::pair<double, double> waypoint = buffered_grid.getPointFromCell(current_cell.first, current_cell.second);
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