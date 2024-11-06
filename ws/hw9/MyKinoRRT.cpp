#include "MyKinoRRT.h"

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};
/*
amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    // start!
    amp::KinoPath path; 
    amp::RNG rng;
    double dt = 1.0;
    
    // Initial state setup
    Eigen::VectorXd state = problem.q_init;
    Eigen::VectorXd goal(problem.q_goal.size());
    for (size_t i = 0; i < problem.q_goal.size(); ++i) {
        goal[i] = (problem.q_goal[i].first + problem.q_goal[i].second) / 2; // or another appropriate calculation
    }
    path.waypoints.push_back(state);

    
    // Construct graph and node mapping
    std::shared_ptr<amp::Graph<std::tuple<double, Eigen::VectorXd>>> graphPtr = std::make_shared<amp::Graph<std::tuple<double, Eigen::VectorXd>>>();
    std::map<amp::Node, Eigen::VectorXd> nodes;
    nodes[0] = problem.q_init; // Root tree at initial point
    int node_count = 1;

    // Lambda to sample random control
    auto randcontrol = [&]() {
        Eigen::VectorXd u(problem.u_bounds.size());
        for (int i = 0; i < problem.u_bounds.size(); ++i) {
            u[i] = rng.randd(problem.u_bounds[i].first, problem.u_bounds[i].second);
        }
        return u;
    };
    // Lambda to score random control 
    auto scoreControl = [&](Eigen::VectorXd nearest, Eigen::VectorXd control ) {
        Eigen::VectorXd next = nearest;
        agent.propagate(next,control, dt);
        return (nearest-next).norm();
    };
    // Lambda to sample random state
    auto randstate = [&]() {
        Eigen::VectorXd sample;
        double bias_prob = 0.1;  
        if (rng.randd(0.0, 1.0) < bias_prob) {
            sample = goal;  // Directly use the goal state
        } else {
            sample = Eigen::VectorXd(problem.q_bounds.size());
            for (int i = 0; i < problem.q_bounds.size(); ++i) {
                sample[i] = rng.randd(problem.q_bounds[i].first, problem.q_bounds[i].second);
            }
        }
        
        return sample;
    };
    // Lambda to find best-of-many random controls 
    auto findbestControl = [&](const Eigen::VectorXd& nearest, const Eigen::VectorXd& sample){
        int iters = 25;
        double bestScore = 1e12;
        Eigen::VectorXd bestControl;
        for (int i = 0; i<iters; ++i){
            // generate random control
            Eigen::VectorXd control = randcontrol();
            // score hypothetical control by proximity to state
            double score = scoreControl(nearest, control);
            // if score is better, 
            if (score<bestScore){
                bestScore = score;
                bestControl = control;
            }
        }
        return bestControl;
    };
    // Lambda to find nearest node
    auto nearestNode = [&](const Eigen::VectorXd& sample) {
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
    };

    // Runtime loop
    int iter = 0;
    const int max_iter = 1500;
    bool goal_reached = false;

    while (iter < max_iter && !goal_reached) {
        // sample a random state
        Eigen::VectorXd sample = randstate();
        // get node nearest to sample
        amp::Node nearest_id = nearestNode(sample);
        Eigen::VectorXd nearest_state = nodes[nearest_id];
        
        // sample many random controls and propagate the state
        Eigen::VectorXd control = findbestControl(nearest_state, sample);
        agent.propagate(state, control, dt); // Timestep of 1.0
        

        // Check for collision (if a collision-checking function is available)
        // if (!problem.is_collision_free(new_state)) continue;

        // Add new state to graph and path
        double cost = (state - nearest_state).norm();
        nodes[node_count] = state;
        graphPtr->connect(nearest_id, node_count, std::make_tuple(cost, control));

        // Check if goal is reached
        if ((state - goal).norm() < 0.25) {
            goal_reached = true;
            // Backtrack to build path
            amp::Node current = node_count;
            while (current != 0) { 
                path.waypoints.push_back(nodes[current]);
                auto outgoing_edges = graphPtr->outgoingEdges(current);
                auto parent_edge = outgoing_edges.front(); // assuming only one parent, adjust if necessary
                Eigen::VectorXd control = std::get<1>(parent_edge); // control is the second element of the tuple
                path.controls.push_back(control);
                path.durations.push_back(dt);
                current = graphPtr->parents(current).front(); 
            }
            // Reverse the path waypoints, controls, and durations to get the correct order
            std::reverse(path.waypoints.begin(), path.waypoints.end());
            std::reverse(path.controls.begin(), path.controls.end());
            std::reverse(path.durations.begin(), path.durations.end());
        } 
        node_count++;
        iter++;
    }
    if (!goal_reached){
        // push back random path through graph with goal tacked onto the end
            amp::Node current = node_count;
            for (int i=0; i < 10; ++i) { 
                path.waypoints.push_back(nodes[current]);
                auto outgoing_edges = graphPtr->outgoingEdges(current);
                auto parent_edge = outgoing_edges.front(); // assuming only one parent, adjust if necessary
                Eigen::VectorXd control = std::get<1>(parent_edge); // control is the second element of the tuple
                path.controls.push_back(control);
                path.durations.push_back(dt);
                current = graphPtr->parents(current).front(); 
            }
            // Reverse the path waypoints, controls, and durations to get the correct order
            std::reverse(path.waypoints.begin(), path.waypoints.end());
            std::reverse(path.controls.begin(), path.controls.end());
            std::reverse(path.durations.begin(), path.durations.end());
        }
    

    path.valid = goal_reached;
    LOG("goal reached? ");
    LOG(goal_reached);
    path.print();
    return path;
}
*/

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    // start!
    amp::KinoPath path; 
    amp::RNG rng;
    double dt = 1.0;
    
    // Initial state setup
    Eigen::VectorXd state = problem.q_init;
    Eigen::VectorXd goal(problem.q_goal.size());
    for (size_t i = 0; i < problem.q_goal.size(); ++i) {
        goal[i] = (problem.q_goal[i].first + problem.q_goal[i].second) / 2; // or another appropriate calculation
    }
    // Construct graph and node mapping
    std::shared_ptr<amp::Graph<std::tuple<double, Eigen::VectorXd>>> graphPtr = std::make_shared<amp::Graph<std::tuple<double, Eigen::VectorXd>>>();
    std::map<amp::Node, Eigen::VectorXd> nodes;
    nodes[0] = problem.q_init; // Root tree at initial point
    int node_count = 1;
    auto pushWp = [&](const Eigen::VectorXd& state,const Eigen::VectorXd& control, double t) {
        path.waypoints.push_back(state);
        path.controls.push_back(control);
        path.durations.push_back(t);
        return 0;
    };
    // Lambda to sample random control
    auto randcontrol = [&]() {
        Eigen::VectorXd u(problem.u_bounds.size());
        for (int i = 0; i < problem.u_bounds.size(); ++i) {
            u[i] = rng.randd(problem.u_bounds[i].first, problem.u_bounds[i].second);
        }
        return u;
    };
    // Lambda to score random control 
    auto scoreControl = [&](Eigen::VectorXd nearest, Eigen::VectorXd control ) {
        Eigen::VectorXd next = nearest;
        agent.propagate(next,control, dt);
        return (nearest-next).norm();
    };
    // Lambda to sample random state
    auto randstate = [&]() {
        Eigen::VectorXd sample;
        double bias_prob = 0.1;  
        if (rng.randd(0.0, 1.0) < bias_prob) {
            sample = goal;  // Directly use the goal state
        } else {
            sample = Eigen::VectorXd(problem.q_bounds.size());
            for (int i = 0; i < problem.q_bounds.size(); ++i) {
                sample[i] = rng.randd(problem.q_bounds[i].first, problem.q_bounds[i].second);
            }
        }
        
        return sample;
    };
    // Lambda to find best-of-many random controls 
    auto findbestControl = [&](const Eigen::VectorXd& nearest, const Eigen::VectorXd& sample){
        int iters = 25;
        double bestScore = 1e12;
        Eigen::VectorXd bestControl;
        for (int i = 0; i<iters; ++i){
            // generate random control
            Eigen::VectorXd control = randcontrol();
            // score hypothetical control by proximity to state
            double score = scoreControl(nearest, control);
            // if score is better, 
            if (score<bestScore){
                bestScore = score;
                bestControl = control;
            }
        }
        return bestControl;
    };
    // Lambda to find nearest node
    auto nearestNode = [&](const Eigen::VectorXd& sample) {
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
    };
    // Lambda to print out std::vector<std::pair<double, double>>
    auto printStuff = [&](const std::vector<std::pair<double, double>>& obj){
        for (const auto& p : obj) {
            std::cout << "(" << p.first << ", " << p.second << ")" << std::endl;
        }
        return;
    };
    auto printStuff2 = [&](const std::vector<Eigen::VectorXd>& obj){
        for (const auto& p : obj) {
            std::cout << "(" << p(0) << ", " << p(1) << ")" << std::endl;
        }
        return;
    };



    // runtime loop
    for (int i = 1; i < 3; ++i){
        Eigen::VectorXd sample = randstate();
        Eigen::VectorXd control = findbestControl(state, sample);
        pushWp(state,control, dt);
        agent.propagate(state,control, dt);
    }

    
    path.valid = false;
    path.print();
    LOG("Control Bounds:");
    printStuff(problem.u_bounds);
    LOG("State Bounds");
    printStuff(problem.q_bounds);
    printStuff2(path.controls);
    return path;
}
