#include "MyAStar.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
        // build struct for node parameters
    struct NodeInfo {
        double g_cost; // Cost from start to current node
        double f_cost; // Estimated total cost (g_cost + heuristic)
        int parent; // Previous node in the path
    };
    GraphSearchResult result = {false, {}, 0.0};    // init value result
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> open_set;  // no need to reinvent the wheel
    std::unordered_map<int, NodeInfo> node_info;    // same deal
    std::unordered_set<int> closed_set;             // nodes already checked
    // init start node
    open_set.push({0.0, problem.init_node});        // push init node to 
    node_info[problem.init_node] = {0.0, heuristic(problem.init_node), -1};
    //
    while (!open_set.empty()) {                     // loop until we've evaluated all nodes
        auto [current_f_cost, current_node] = open_set.top();
        open_set.pop();
        // if we made it to goal, reconstruct path
        if (current_node == problem.goal_node) {
            result.path_cost = node_info[current_node].g_cost;
            while (current_node != -1) {
                result.node_path.push_back(current_node);
                current_node = node_info[current_node].parent;
            }
            std::reverse(result.node_path.begin(), result.node_path.end());
            result.success = true;
            result.print();
            return result;
        }

        closed_set.insert(current_node);

        // check each neighbor edge
        const auto& neighbors = problem.graph->children(current_node);
        const auto& edges = problem.graph->outgoingEdges(current_node);
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
            int neighbor = neighbors[i];
            double cost = edges[i];

            if (closed_set.count(neighbor)) continue; // Skip if already evaluated

            double tentative_g_cost = node_info[current_node].g_cost + cost;
            if (!node_info.count(neighbor) || tentative_g_cost < node_info[neighbor].g_cost) {
                node_info[neighbor] = {tentative_g_cost, tentative_g_cost + heuristic(neighbor), current_node};
                open_set.push({node_info[neighbor].f_cost, neighbor});
            }
        }
    }

    result.print();
    return result; // if goal unreachable, catch it
}
