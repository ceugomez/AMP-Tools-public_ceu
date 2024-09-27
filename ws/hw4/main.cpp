// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW4.h"
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"
#include "HelpfulClass.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    const amp::Obstacle2D obstacleRspace = HW4::getEx1TriangleObstacle();

    // Compute the Minkowski sum
    std::vector<Eigen::Vector2d> cspaceVertices = MyGridCSpace2D::getMinkowskiSumRobotObstacle(obstacleRspace, obstacleRspace);
    amp::Obstacle2D obstacleCspace = amp::Obstacle2D(cspaceVertices);

    Visualizer::makeFigure({obstacleRspace});
    Visualizer::makeFigure({obstacleCspace});

    MyManipulator2D manipulator;
    double pi = std::numbers::pi;
    // You can visualize your manipulator given an angle state like so:
    amp::ManipulatorState test_state(3);
    test_state << pi/6, -pi/3, 7*pi/4;
    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator, test_state); 

    // Create the collision space constructor
    std::size_t n_cells = 5;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());

    // You can visualize your cspace 
    //Visualizer::makeFigure(*cspace);

    Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "cego6160@colorado.edu", argc, argv);
    return 0;
}