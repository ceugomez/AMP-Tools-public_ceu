#pragma once
#include "AMPCore.h"
#include "hw/HW4.h"

// Derive the amp::GridCSpace2D class and override the missing method
class MyGridCSpace2D : public amp::GridCSpace2D {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) // Call base class constructor
        {}

        // Override this method for determining which cell a continuous point belongs to
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;        
        static std::vector<Eigen::Vector2d> getMinkowskiSumRobotObstacle(const amp::Obstacle2D& obstacle, const amp::Obstacle2D& robot);
        bool isPointInPolygon(const Eigen::Vector2d& pos, const std::vector<Eigen::Vector2d>& vertices);
        double crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b);
        std::vector<double> linspace(double start, double end, int num);
        bool isSegmentInCollision(const Eigen::Vector2d& linkStart,const Eigen::Vector2d& linkEnd, const std::vector<Eigen::Vector2d>& vertices);
        bool onSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r);
        int orientation(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r);
        bool doIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2);

};

// Derive the HW4 ManipulatorCSConstructor class and override the missing method
class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

    private:
        std::size_t m_cells_per_dim;
};