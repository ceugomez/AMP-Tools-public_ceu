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
    amp::RNG::seed(amp::RNG::randiUnbounded());
    /* Problem 1 -------------------------------------------------------------------------*/
    const amp::Obstacle2D obstacleRspace = HW4::getEx1TriangleObstacle();
    std::vector<Eigen::Vector2d> cspaceVertices = MyGridCSpace2D::getMinkowskiSumRobotObstacle(obstacleRspace, obstacleRspace);
    amp::Obstacle2D obstacleCspace = amp::Obstacle2D(cspaceVertices);
    /* Problem 2 -------------------------------------------------------------------------*/
    MyManipulator2D manipulator;
    double pi = std::numbers::pi;
    amp::ManipulatorState test_state_1(3);
    test_state_1 << pi / 6, -pi / 3, 7 * pi / 4;
    Eigen::Vector2d end_state_1 = manipulator.getJointLocation(test_state_1, 3);
    Eigen::Vector2d end_state_autodiff = manipulator.forwardKinematics<double>(test_state_1);
    amp::ManipulatorState IK_state_1 = manipulator.getConfigurationFromIK(end_state_1);
    /* Problem 3 --------------------------------------------------------------------------*/
    std::size_t n_cells = 500;
    MyManipulatorCSConstructor cspace_constructor(n_cells);
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());
    Visualizer::makeFigure(*cspace);
    Visualizer::makeFigure(HW4::getEx3Workspace1());
    Visualizer::showFigures();

    MyManipulatorCSConstructor cspace_constructor2(n_cells);
    std::unique_ptr<amp::GridCSpace2D> cspace2 = cspace_constructor2.construct(manipulator, HW4::getEx3Workspace2());
    Visualizer::makeFigure(*cspace2);
    Visualizer::makeFigure(HW4::getEx3Workspace2());
    Visualizer::showFigures();

    MyManipulatorCSConstructor cspace_constructor3(n_cells);
    std::unique_ptr<amp::GridCSpace2D> cspace3 = cspace_constructor3.construct(manipulator, HW4::getEx3Workspace3());
    Visualizer::makeFigure(*cspace3);
    Visualizer::makeFigure(HW4::getEx3Workspace3());
    Visualizer::showFigures();

    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "cego6160@colorado.edu", argc, argv);
    return 0;
}