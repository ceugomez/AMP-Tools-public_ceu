#include "benchmark.h"

benchmarkResult benchmark::benchmarkKinoRRT(const amp::KinodynamicProblem2D &problem, const std::tuple<int, std::vector<int>>& set)
{
    // agentFactory
    std::unordered_map<amp::AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {amp::AgentType::SingleIntegrator, []()
     { return std::make_shared<MySingleIntegrator>(); }},
    {amp::AgentType::FirstOrderUnicycle, []()
     { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {amp::AgentType::SecondOrderUnicycle, []()
     { return std::make_shared<MySecondOrderUnicycle>(); }},
    {amp::AgentType::SimpleCar, []()
     { return std::make_shared<MySimpleCar>(); }}};

    // Initialize the result structure to store benchmark data
    benchmarkResult result;

    int max_iter = std::get<0>(set);          // Number of max iterations
    const auto &u_samples = std::get<1>(set); // Vector of control samples (e.g., {1, 5, 10, 15})
    // lambda function to get path length heuristic
    auto getPathLength = [&](const amp::KinoPath &path)
    {

        if (path.waypoints.size() < 2)
            return 0.0;

        double total_length = 0.0;
        for (size_t i = 1; i < path.waypoints.size(); ++i)
        {
            total_length += (path.waypoints[i] - path.waypoints[i - 1]).norm();
        }
        return total_length;
    };

    for (int i = 0; i < u_samples.size(); ++i)
    {
        // Vectors to store data for each setting of u_samples
        std::vector<double> cputime_samples;
        std::vector<double> pathlength_samples;
        int validPaths = 0; // Counter for valid paths in each sample set

        // Configure the KinoRRT planner with the given parameters
        MyKinoRRT kino_planner(0.1, u_samples[i], max_iter); // Bias probability = 0.1

        // Repeat benchmarking 10 times for statistical reliability
        for (int j = 0; j < 10; ++j)
        {
            // Timer start
            auto start = std::chrono::high_resolution_clock::now();
            
            amp::KinoPath path = kino_planner.plan(problem, *agentFactory[problem.agent_type]());
             

            // Timer end
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double cpu_time = elapsed.count();

            // Collect data if a valid path is found
            if (path.valid)
            {
                ++validPaths;
                pathlength_samples.push_back(getPathLength(path)); // record path length
            }
            else
            {
                pathlength_samples.push_back(0.0); // invalid path
            }
            cputime_samples.push_back(cpu_time); // wall time
        }
        // percentage
        result.validSolns.push_back(static_cast<double>(validPaths) / 10.0 * 100.0);

        // Store the results for each `u_samples` setting in benchmarkResult
        result.cputime.push_back(cputime_samples);
        result.pathlength.push_back(pathlength_samples);
    }

    return result;
}
