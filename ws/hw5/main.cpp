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
    Path2D path;
    Problem2D prob;
    bool success = HW5::generateAndCheck(algo, path, prob);
    //amp::Visualizer::makeFigure(MyPotentialFunction(prob), prob,50, false);
    //Visualizer::makeFigure(prob, path); 
    //Visualizer::showFigures();
    // Part a --------------------------------------------------------------------------------
        //visualize 2-d environment 
        prob =  HW5::getWorkspace1();
        path = algo.plan(prob);
        Visualizer::makeFigure(prob, path); 
        // Visualize your potential function
        amp::Visualizer::makeFigure(MyPotentialFunction(prob), prob,50, false);
        LOG("Path Length - Part A");
        LOG(path.length());
    // Part b --------------------------------------------------------------------------------
        prob = HW2::getWorkspace1();
        path = algo.plan(prob);
        LOG("Path Length - Part b-i");
        LOG(path.length());
        Visualizer::makeFigure(prob, path);
        amp::Visualizer::makeFigure(MyPotentialFunction(prob), prob, 50, false);
        prob = HW2::getWorkspace2();
        path = algo.plan(prob);
        LOG("Path Length - Part b-ii");
        LOG(path.length());
        Visualizer::makeFigure(prob, path);
        amp::Visualizer::makeFigure(MyPotentialFunction(prob), prob, 50, false);
        Visualizer::showFigures();




<<<<<<< HEAD
=======
    // Visualize your potential function
    Visualizer::makeFigure(MyPotentialFunction{}, prob, 30);
    Visualizer::showFigures();
    
>>>>>>> upstream/main
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    //HW5::grade<MyGDAlgorithm>("ceu.gomez-faulk@colorado.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}