// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(1.0, 1.0, 1.0, 1.0);
    amp::Path2D path;
    amp::Problem2D prob;
    bool success = HW5::generateAndCheck(algo, path, prob);
    amp::Visualizer::makeFigure(MyPotentialFunction(prob), prob,20);
    Visualizer::makeFigure(prob, path); 
    Visualizer::showFigures();
    // Part a --------------------------------------------------------------------------------
        //visualize 2-d environment 
        prob =  HW5::getWorkspace1();
        path = algo.plan(prob);
        Visualizer::makeFigure(prob, path); 
        // Visualize your potential function
        amp::Visualizer::makeFigure(MyPotentialFunction(prob), prob,20);
        LOG("Path Length - Part A");
        LOG(path.length());
        //Visualizer::showFigures();
    // Part b --------------------------------------------------------------------------------
        prob = HW2::getWorkspace1();
        path = algo.plan(prob);
        Visualizer::makeFigure(prob, path);
        //amp::Visualizer::makeFigure(MyPotentialFunction(prob), prob, 20);
        Visualizer::showFigures();



    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    //HW5::grade<MyGDAlgorithm>("ceu.gomez-faulk@colorado.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}