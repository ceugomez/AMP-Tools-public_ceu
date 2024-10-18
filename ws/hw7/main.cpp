// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "../shared/SamplingBasedPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
    //HW7::hint(); // Consider implementing an N-dimensional planner 
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;


    // ------------- PRM ----------------
    // Test PRM on Workspace1 of HW2
    Problem2D problem = HW2::getWorkspace1();
    PRM prm;
    Visualizer::makeFigure(problem, prm.plan(problem), *graphPtr, nodes);

    // ------------- RRT ----------------
    RRT rrt;
    Path2D path;
    // Test RRRT on HW5 WS1
    problem = HW5::getWorkspace1();
    Visualizer::makeFigure(problem, rrt.plan(problem), *graphPtr, nodes);
    // Test RRT on HW2 WS1
    problem = HW2::getWorkspace1();
    Visualizer::makeFigure(problem, rrt.plan(problem), *graphPtr, nodes);
    // Test RRT on HW2 WS2
    problem = HW2::getWorkspace2();
    Visualizer::makeFigure(problem, rrt.plan(problem), *graphPtr, nodes);

    // Generate a random problem and test RRT
    HW7::generateAndCheck(rrt, path, problem);
    Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    
    
    Visualizer::showFigures();

    // Grade method
    HW7::grade<PRM, RRT>("ceu.gomez-faulk.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}