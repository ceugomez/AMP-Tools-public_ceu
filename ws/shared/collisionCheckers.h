#include "AMPCore.h"

class collisionCheckers {
    public:
        static bool isPointInPolygon(const amp::Obstacle2D& obs, const Eigen::Vector2d& pos);
        static bool isArmInCollision(const amp::Obstacle2D& obs, const Eigen::Vector2d& pos, const amp::LinkManipulator2D& manipulator);
        static bool segmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2);

};