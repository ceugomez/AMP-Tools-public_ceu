#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Override this method for returning whether or not a point is in collision
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implement your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    // simply returns the cell indices of my discrete grid when given an (x0, x1) state in continuous space. 
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

            Eigen::Vector2d endpos_rspace = manipulator.getJointLocation(cspace_state, n);

            for (const amp::Polygon& obstacle : env.obstacles) {
                if (cspace.isPointInPolygon(endpos_rspace, obstacle.verticesCCW())) {    // if end effector position is within an obstacle
                    //LOG(endpos_rspace);
                    cspace(i,j) = true;
                    break;
                }else {                                                                 
                    // check if line segments are within obstacle 
                    for (int k = 0; k < n; ++k) {
                        Eigen::Vector2d link_start = manipulator.getJointLocation(cspace_state, k);
                        Eigen::Vector2d link_end = manipulator.getJointLocation(cspace_state, k + 1);
                        if (cspace.isSegmentInCollision(link_start, link_end, obstacle.verticesCCW())) {
                            cspace(i,j) = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    return cspace_ptr;
}

bool MyGridCSpace2D::isSegmentInCollision(const Eigen::Vector2d& linkStart,const Eigen::Vector2d& linkEnd, const std::vector<Eigen::Vector2d>& vertices){
    int n = vertices.size();
    for (int i = 0; i < n; ++i) {
        Eigen::Vector2d p3 = vertices[i];
        Eigen::Vector2d p4 = vertices[(i + 1) % n];
        if (doIntersect(linkStart, linkEnd, p3, p4)) {
            return true;
        }
    }
    return false;
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
bool MyGridCSpace2D::onSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
    return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
           q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
}
int MyGridCSpace2D::orientation(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) {
    double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
    if (val == 0) return 0;  // collinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}
bool MyGridCSpace2D::doIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4) return true;

    // Special cases
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false;
}
