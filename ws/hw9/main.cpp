// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"
#include "benchmark.h"

using namespace amp;

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []()
     { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []()
     { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []()
     { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []()
     { return std::make_shared<MySimpleCar>(); }}};

int main(int argc, char **argv)
{
    // Select problem, plan, check, and visualize
    auto benchmarkTheSet = [&](const amp::KinodynamicProblem2D &problem, const auto &set)
    {
        benchmarkResult result = benchmark::benchmarkKinoRRT(problem, set);
        return result;
    };

    auto makeListFromVec = [&](const benchmarkResult &result)
    {
        std::list<std::vector<double>> data_sets;
        data_sets.insert(data_sets.end(), result.cputime.begin(), result.cputime.end());
        data_sets.insert(data_sets.end(), result.pathlength.begin(), result.pathlength.end());
        data_sets.push_back(result.validSolns);

        std::vector<std::string> labels;
        for (size_t i = 0; i < result.cputime.size(); ++i)
        {
            labels.push_back("CPU Time " + std::to_string(i + 1));
        }
        for (size_t i = 0; i < result.pathlength.size(); ++i)
        {
            labels.push_back("Path Length " + std::to_string(i + 1));
        }
        labels.push_back("Valid Solutions");

        return std::make_tuple(data_sets, labels);
    };

    const std::tuple<int, std::vector<int>> set = std::make_tuple(50000, std::vector<int>{1, 5, 10, 15}); // max_iter, u_samples
    amp::KinodynamicProblem2D problem = problems[0];
    benchmarkResult result = benchmarkTheSet(problem, set);
    auto [data_sets, labels] = makeListFromVec(result);

    Visualizer::makeBoxPlot(data_sets, labels, "Benchmark Results", "Metrics", "Values");
    Visualizer::showFigures();
    /*
    int select = 5;
    KinodynamicProblem2D prob = problems[select];
    MyKinoRRT kino_planner(0.1,250,50000);
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    HW9::check(path, prob);
    if (path.valid)
        Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
    Visualizer::showFigures();
    */
    // HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("ceu.gomez-faulk@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}