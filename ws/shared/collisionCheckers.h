#include "AMPCore.h"

class collisionCheckers {
    public:
        static bool isPointInPolygon(const amp::Obstacle2D& obs, const Eigen::Vector2d& pos);
        static bool isArmInCollision(const amp::Obstacle2D& obs, const Eigen::Vector2d& pos, const amp::LinkManipulator2D& manipulator);
        static bool segmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2);
        static bool isLineInCollision(const std::vector<amp::Obstacle2D>& obstacles, const Eigen::Vector2d& q1, const Eigen::Vector2d q2);
        static bool isPointInAnyPolygon(const std::vector<amp::Obstacle2D>& obstacles, const Eigen::Vector2d& pos);
        static bool isPathInAnyPolygon(const std::vector<amp::Obstacle2D>& obstacles, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
        static bool isPolygonPathFree(const std::vector<amp::Obstacle2D>& obstacles, const Eigen::VectorXd& q1, const Eigen::Vector2d& q2, const amp::AgentDimensions& dim); 
        static bool checkPolygonIntersection(const std::vector<Eigen::Vector2d> &poly1, const std::vector<Eigen::Vector2d> &poly2);
    private:
};