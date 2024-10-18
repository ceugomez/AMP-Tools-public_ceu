#include "../shared/collisionCheckers.h"
#include "../shared/HelperFns.h"

bool collisionCheckers::isPointInPolygon(const amp::Obstacle2D& obs, const Eigen::Vector2d& pos){
    int windingNumber = 0;
    std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
    for (size_t i = 0; i < vertices.size(); ++i) {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];
        if (v1.y() <= pos.y()) {
            if (v2.y() > pos.y() && HelperFns::crossProduct(v2 - v1, pos - v1) > 0) {
                ++windingNumber;
            }
        } else {
            if (v2.y() <= pos.y() && HelperFns::crossProduct(v2 - v1, pos - v1) < 0) {
                --windingNumber;
            }
        }
    }
    return windingNumber != 0;
}
bool collisionCheckers::isPointInAnyPolygon(const std::vector<amp::Obstacle2D>& obstacles, const Eigen::Vector2d& pos){
    for (const amp::Obstacle2D obs : obstacles){
        int windingNumber = 0;
        std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
        for (size_t i = 0; i < vertices.size(); ++i) {
            Eigen::Vector2d v1 = vertices[i];
            Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];
            if (v1.y() <= pos.y()) {
                if (v2.y() > pos.y() && HelperFns::crossProduct(v2 - v1, pos - v1) > 0) {
                    ++windingNumber;
                }
            } else {
                if (v2.y() <= pos.y() && HelperFns::crossProduct(v2 - v1, pos - v1) < 0) {
                    --windingNumber;
                }
            }
        }
        if (windingNumber!=0){
            // point in obstacle
            return windingNumber!=0;
        }
    }
    return false;
}
bool collisionCheckers::isArmInCollision(const amp::Obstacle2D& obs, const Eigen::Vector2d& pos, const amp::LinkManipulator2D& manipulator) {
    auto n_links = manipulator.nLinks();
    Eigen::Vector2d prev_joint = manipulator.getBaseLocation();
    Eigen::Vector2d current_joint;
    std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
    
    for (std::size_t i = 0; i < n_links; ++i) {
        current_joint = manipulator.getJointLocation(pos, i + 1);
        
        for (std::size_t j = 0; j < vertices.size(); ++j) {
            Eigen::Vector2d p1 = vertices[j];
            Eigen::Vector2d p2 = vertices[(j + 1) % vertices.size()]; // Ensure the polygon wraps around
            if (segmentsIntersect(prev_joint, current_joint, p1, p2)) {
                return true;
            }
        }
        prev_joint = current_joint;
    }
    
    return false;
}
bool collisionCheckers::segmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2) {
    auto orientation = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c) {
        double val = (b.y() - a.y()) * (c.x() - b.x()) - (b.x() - a.x()) * (c.y() - b.y());
        if (val == 0) return 0;  // collinear
        return (val > 0) ? 1 : 2; // direction, ccw or cw 
    };

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;

    return false;
}
bool collisionCheckers::isLineInCollision(const std::vector<amp::Obstacle2D>& obstacles, const Eigen::Vector2d& q1, const Eigen::Vector2d q2) {
    for (const amp::Obstacle2D& obs : obstacles){
        const std::vector<Eigen::Vector2d>& vertices = obs.verticesCCW();
        size_t num_vertices = vertices.size();
        for (size_t i = 0; i < num_vertices; ++i) {
            Eigen::Vector2d a = vertices[i];
            Eigen::Vector2d b = vertices[(i + 1) % num_vertices]; // (wrap around)
            if (collisionCheckers::segmentsIntersect(q1, q2, a, b)) {
                    return true; 
                }
            }
        }
    return false; 
}