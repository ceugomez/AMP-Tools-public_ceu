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
    /* Problem 1 -----------------------------------------------------*/
    const amp::Obstacle2D obstacleRspace = HW4::getEx1TriangleObstacle();
    std::vector<Eigen::Vector2d> cspaceVertices = MyGridCSpace2D::getMinkowskiSumRobotObstacle(obstacleRspace, obstacleRspace);
    amp::Obstacle2D obstacleCspace = amp::Obstacle2D(cspaceVertices);
    Visualizer::makeFigure({obstacleRspace});
    Visualizer::makeFigure({obstacleCspace});


    /* Problem 2 -----------------------------------------------------*/
    MyManipulator2D manipulator;        //init manipulator
    double pi = std::numbers::pi;       // whatever (yawn)
    amp::ManipulatorState test_state_1(3);// initialize to test state 1
    test_state_1 << pi/6, -pi/3, 7*pi/4;    // from hw
    Eigen::Vector2d end_state_1 = manipulator.getJointLocation(test_state_1,3);
    Eigen::Vector2d end_state_autodiff = manipulator.forwardKinematics<double>(test_state_1);
    LOG(end_state_1);
    LOG(end_state_autodiff);
    amp::ManipulatorState IK_state_1 = manipulator.getConfigurationFromIK(end_state_1);
    LOG(test_state_1);
    LOG(IK_state_1);


    // test inverse kinematics, 3-link



    //Visualizer::makeFigure(manipulator, test_state_1); 
    amp::ManipulatorState test_state_2(3);
    test_state_2 << pi/6, -pi/3, 7*pi/4;


    //Visualizer::makeFigure(manipulator, test_state_2)


    /* Problem 3 -----------------------------------------------------*/
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