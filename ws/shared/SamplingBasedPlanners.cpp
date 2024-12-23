# include "SamplingBasedPlanners.h"
# include "collisionCheckers.h"
# include "AStar.h"

// PRM implementation
amp::Path2D PRM::plan(const amp::Problem2D& problem) {
    // construct variables
    amp::Path2D path;
    int n = max_iterations;
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;
    // add initial point as node
    nodes[0] = problem.q_init;
    // sampling loop
    for (int i = 1; i < n; ++i) {                           // sample n random points within environment and construct graph
        Eigen::Vector2d xy = sampleRandomPoint(problem);    // get random point
        if (!collisionCheckers::isPointInAnyPolygon(problem.obstacles, xy)) {   // if point is in free space
            // add node 
            nodes[i] = xy;
            // evaluate points within connection radius
            connectNeighbors(i, nodes, connectionradius, graphPtr, problem);
        } 
    }
    // add goal as nodes
    nodes[n] = problem.q_goal;
    connectNeighbors(n,nodes,connectionradius,graphPtr,problem);

    // build problem for graph search
    amp::ShortestPathProblem shortestPathProblem;
    shortestPathProblem.graph = graphPtr;
    shortestPathProblem.init_node = 0;  
    shortestPathProblem.goal_node = n;  

    // build distance-based heuristic 
    struct EuclideanHeuristic : public amp::SearchHeuristic {
        const std::map<amp::Node, Eigen::Vector2d>& nodes;
        amp::Node goal_node;
        EuclideanHeuristic(const std::map<amp::Node, Eigen::Vector2d>& nodes, amp::Node goal_node) 
            : nodes(nodes), goal_node(goal_node) {}
        double operator()(amp::Node node) const override {
            // Calculate Euclidean distance to goal node
            return (nodes.at(node) - nodes.at(goal_node)).norm();
        }
    };
    EuclideanHeuristic heuristic(nodes, shortestPathProblem.goal_node);
    
    // Initialize astar  
    AStarAlgo astar;
    // search graph
    AStarAlgo::GraphSearchResult result = astar.search(shortestPathProblem, heuristic);
    // reconstruct cspace path from shortest graph path
    for (amp::Node node : result.node_path) {
        path.waypoints.push_back(nodes[node]);
        //LOG("Pushed new wp");
    }
    if (result.success){
        path.valid = true;
    }else {
        path.valid = false;
    }
    return path;
}
void PRM::connectNeighbors(const amp::Node& newNode, std::map<amp::Node, Eigen::Vector2d>& nodes, const double& connectionradius, std::shared_ptr<amp::Graph<double>>& graphPtr, const amp::Problem2D& problem) {
    for (const auto& node : nodes) {
        if (node.first != newNode) {
            double distance = (nodes[newNode] - node.second).norm();
            if (distance <= connectionradius && PRM::isPathFree(nodes[newNode], node.second, problem)) {
                graphPtr->connect(newNode, node.first, distance);
                graphPtr->connect(node.first, newNode, distance); //bidirectional
            }
        }
    }
}
Eigen::Vector2d PRM::sampleRandomPoint(const amp::Problem2D& problem){
        std::uniform_real_distribution<double> dist_x(problem.x_min, problem.x_max);
        std::uniform_real_distribution<double> dist_y(problem.y_min, problem.y_max);
        double x = dist_x(rng);
        double y = dist_y(rng);
        return Eigen::Vector2d(x,y);
}
bool PRM::isPathFree(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem) {
    return !collisionCheckers::isLineInCollision(problem.obstacles, start, end);
}
BenchmarkResult PRM::benchmark_prm(int n, double r, const amp::Problem2D& problem) {
    BenchmarkResult result;
    PRM prm(n, r);

    for (int i = 0; i < 100; ++i) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        amp::Path2D path = prm.plan(problem);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> computation_time = end_time - start_time;
        
        result.computation_times.push_back(computation_time.count());

        if (path.valid) {
            result.valid_solutions++;
            result.path_lengths.push_back(path.length());
        } else {
            result.path_lengths.push_back(0.0);
        }
    }

    return result;
}

// RRT implementation
amp::Path2D RRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;

    nodes[0] = problem.q_init;  // Root tree at initial point
    int node_count = 1;

    for (int i = 0; i < max_iterations; ++i) {
        Eigen::Vector2d random_point = sampleRandomPoint(problem, goal_bias);
        amp::Node nearest = nearestNode(problem,nodes,random_point);
        Eigen::Vector2d new_point = extend(nodes[nearest], random_point);

        if (!collisionCheckers::isPointInAnyPolygon(problem.obstacles, new_point) && isPathFree(nodes[nearest], new_point, problem)) {
            nodes[node_count] = new_point;
            graphPtr->connect(nearest, node_count, (new_point - nodes[nearest]).norm());

            if ((new_point - problem.q_goal).norm() < goal_threshold) {
                nodes[node_count + 1] = problem.q_goal;
                graphPtr->connect(node_count, node_count + 1, (new_point - problem.q_goal).norm());
                break;
            }
            ++node_count;
        }
    }

    // Check if goal node was reached
    if (nodes.count(node_count + 1) == 0) {
        LOG("ERR: Goal node not reached");
        path.waypoints.push_back(problem.q_init);
        path.waypoints.push_back(problem.q_goal);
        return path;
    }

    // set
    amp::ShortestPathProblem shortestPathProblem;
    shortestPathProblem.graph = graphPtr;
    shortestPathProblem.init_node = 0;
    shortestPathProblem.goal_node = node_count + 1;

    // Implement the heuristic (Euclidean distance)
    struct EuclideanHeuristic : public amp::SearchHeuristic {
        const std::map<amp::Node, Eigen::Vector2d>& nodes;
        amp::Node goal_node;
        EuclideanHeuristic(const std::map<amp::Node, Eigen::Vector2d>& nodes, amp::Node goal_node)
            : nodes(nodes), goal_node(goal_node) {}
        double operator()(amp::Node node) const override {
            return (nodes.at(node) - nodes.at(goal_node)).norm();
        }
    };

    EuclideanHeuristic heuristic(nodes, shortestPathProblem.goal_node);

    // Perform A* search
    AStarAlgo astar;
    AStarAlgo::GraphSearchResult result = astar.search(shortestPathProblem, heuristic);
    if (result.success) {
        for (amp::Node node : result.node_path) {
            path.waypoints.push_back(nodes[node]);
        }
    } else {
        LOG("ERR: Path not found");
        path.waypoints.push_back(problem.q_init);
        path.waypoints.push_back(problem.q_goal);
    }
    return path;
}
bool RRT::isPathFree(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::Problem2D& problem) {
    return !collisionCheckers::isLineInCollision(problem.obstacles, start, end);
}
Eigen::Vector2d RRT::sampleRandomPoint(const amp::Problem2D& problem, const double& goal_bias) {
    // consider sending direct to goal
    if(rng() < goal_bias) {
        return problem.q_goal;
    }
    // otherwise go to random point
    std::uniform_real_distribution<double> dist_x(problem.x_min, problem.x_max);
    std::uniform_real_distribution<double> dist_y(problem.y_min, problem.y_max);
    return Eigen::Vector2d(dist_x(rng), dist_y(rng)); // return random point
}
int RRT::nearestNode(const amp::Problem2D& problem, const std::map<amp::Node,Eigen::Vector2d>& nodes, const Eigen::Vector2d& sample){
    amp::Node nearest = 0;
    double min_dist = (problem.q_init - sample).norm();
    for (const auto& node : nodes) {
        double dist = (node.second - sample).norm();
        if (dist < min_dist) {
            nearest = node.first;
            min_dist = dist;
        }
    }
    return nearest;
}
Eigen::Vector2d RRT::extend(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2){
    Eigen::Vector2d direction = (q2 - q1).normalized();
    return q1 + direction * step_size;
};
BenchmarkResult RRT::benchmark_rrt(int n, double delta, double eta, double gt, const amp::Problem2D& problem) {
    BenchmarkResult result;
    RRT rrt(n, delta, eta, gt);

    for (int i = 0; i < 100; ++i) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        amp::Path2D path = rrt.plan(problem);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> computation_time = end_time - start_time;
        
        result.computation_times.push_back(computation_time.count());

        if (path.valid) {
            result.valid_solutions++;
            result.path_lengths.push_back(path.length());
        } else {
            result.path_lengths.push_back(0.0);
        }
    }
    return result;
}
// generalized RRT
amp::Path maRRT::plan(const Eigen::VectorXd& init, const Eigen::VectorXd& goal, const amp::configurationSpace& collcheck){
    
}

