#include "MyKinoRRT.h"

void MySingleIntegrator::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    state += dt * control;
};
void MyFirstOrderUnicycle::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    // unpack state
    double r = 0.25; double x = state[0]; double y = state[1]; double theta = state[2];
    // unpack ctrl
    double v = control[0];  // linear velocity 
    double omega = control[1];  // angular velocity

    // dynamics
    theta += omega * dt;       // Update orientation
    x += v * dt *  r * cos(theta);  // Update x position
    y += v * dt *  r * sin(theta);  // Update y position

    // wrap theta [-pi,pi]
    while (theta > M_PI) {
        theta -= 2 * M_PI;
    }
    while (theta < -M_PI) {
        theta += 2 * M_PI;
    }
    // updatestate
    state[0] = x;
    state[1] = y;
    state[2] = theta;
};

void MySecondOrderUnicycle::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double  dt)
{
    // unpack state
   double x = state[0]; double y = state[1]; double theta = state[2]; double sigma = state[3]; double omega  = state[4];
   // unpack ctrl
};

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D &problem, amp::DynamicAgent &agent)
{
    // set up variables
    amp::KinoPath path;
    amp::RNG rng;
    double dt = 0.1; // worth varying out of curiosity    
    int iter = 0;
    const int max_iter = 7500;
    bool goal_reached = false;
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(problem.q_goal.size());
    // pick a goal in the middle of goal region
    for (int i = 0; i < problem.q_goal.size(); ++i)
    {
        goal[i] = (problem.q_goal[i].first + problem.q_goal[i].second) / 2;
    }
    Eigen::VectorXd state;
    std::shared_ptr<amp::Graph<std::tuple<double, Eigen::VectorXd>>> graphPtr = std::make_shared<amp::Graph<std::tuple<double, Eigen::VectorXd>>>();
    // create empty graph
    std::map<amp::Node, Eigen::VectorXd> nodes;
    // initialize node at start point
    nodes[0] = problem.q_init;
    int node_count = 1;
    // lambda to push a waypoint (in all its glory)
    auto pushWp = [&](const Eigen::VectorXd &state, const Eigen::VectorXd &control, double t)
    {
        path.waypoints.push_back(state);
        path.controls.push_back(control);
        path.durations.push_back(t);
    };
    // lambda to generate a random control within control bounds
    auto randcontrol = [&]()
    {
        Eigen::VectorXd u(problem.u_bounds.size());
        for (int i = 0; i < 2; ++i)
        {
            u[i] = rng.randd(problem.u_bounds[i].first, problem.u_bounds[i].second);
        }
        return u;
    };
    // lambda to score some control based on how close it brings the agent to some provided goal (NOT the general goal)
    auto scoreControl = [&](Eigen::VectorXd nearest, Eigen::VectorXd control, Eigen::VectorXd goal)
    {
        Eigen::VectorXd next = nearest;
        agent.propagate(next, control, dt);
        if (nearest.size() == 2){   // simple integrator - weight xy positioning
            return (next - goal).norm();
        }
        if (nearest.size() == 3){   //  1st order unicycle - only weight xy positioning
            double cost = abs((next[0]-goal[0])) + abs((next[1]-goal[1]));
            return cost;
        }
        return 1e12;
    };
    // lambda to get random state from within problem bounds
    auto randstate = [&](double bias_prob)
    {
        Eigen::VectorXd sample;
        if (rng.randd(0.0, 1.0) < bias_prob)
        {
            sample = goal;
        }
        else
        {
            sample = Eigen::VectorXd(problem.q_bounds.size());
            for (int i = 0; i < problem.q_bounds.size(); ++i)
            {
                sample[i] = rng.randd(problem.q_bounds[i].first, problem.q_bounds[i].second);
            }
        }
        return sample;
    };
    // lambda  to find best-of-many controls to propagate twds sample from nearest
    auto findbestControl = [&](const Eigen::VectorXd &nearest, const Eigen::VectorXd &sample)
    {
        int iters = 50;
        double bestScore = 1e12;
        Eigen::VectorXd bestControl;
        for (int i = 0; i < iters; ++i)
        {
            Eigen::VectorXd control = randcontrol();
            double score = scoreControl(nearest, control, sample);
            if (score < bestScore)
            {
                bestScore = score;
                bestControl = control;
            }
        }
        return bestControl;
    };
    // lambda to find the nearest node to a sample
    auto nearestNode = [&](const Eigen::VectorXd &sample)
    {
        amp::Node nearest = 0;
        double min_dist = std::numeric_limits<double>::max(); // Start with a very large minimum distance

        for (const auto &node : nodes)
        {
            double dist = (node.second - sample).norm();
            if (dist < min_dist)
            {
                nearest = node.first;
                min_dist = dist;
            }
        }
        // Debugging output
        // std::cout << "Nearest node to sample found: " << nearest << " with distance: " << min_dist << "\n";
        return nearest;
    };
    // lambda to check if the point is within the problem bounds
    auto pointInBounds = [&](const Eigen::VectorXd &point)
    {
        if (point.size() != problem.q_bounds.size())
        {
            return false;
        }
        for (int i = 0; i < point.size(); ++i)
        {
            double min_bound = problem.q_bounds[i].first;
            double max_bound = problem.q_bounds[i].second;
            if (point[i] < min_bound || point[i] > max_bound)
            {
                return false;
            }
        }
        return true;
    };
    // lambda to check if goal is reached based on agent type : !!!!!!!!FINISH!!!!!!!
    auto goalReached = [&](const Eigen::VectorXd &point)
    {
        // if state is 2-dimensional: simple integrator conditions
        if (problem.q_bounds.size()==2){
            double dx = abs(point[0]-goal[0]);
            double dy = abs(point[1]-goal[1]);
            if (dy<0.25 && dx < 0.25){
                return true;
            }
        }
        // if state is 3-dimensional: 1st order unicycle conditions
        if (problem.q_bounds.size() == 3){
            double dx = abs(point[0]-goal[0]);
            double dy = abs(point[1]-goal[1]);
            if (dy<0.5 && dx < 0.5){
                return true;
            }

        }
        // if state is 5-dimensional 

        return false;
    };
    // lambda to print out std::vector<std::pair<double, double>>
    auto printStuff = [&](const std::vector<std::pair<double, double>>& obj){
        for (const auto& p : obj) {
            std::cout << "(" << p.first << ", " << p.second << ")" << std::endl;
        }
        return;
    };
    // lambda to print out std::vector<Eigen::VectorXd>
   auto printStuff2 = [&](const std::vector<Eigen::VectorXd>& obj){
        for (const auto& p : obj) {
            std::cout << "(" << p(0) << ", " << p(1) << ")" << std::endl;
        }
        return;
    };
    
    // Runtime loop
    while (iter < max_iter && !goal_reached)
    {
        // sample random state from environment with goal bias 0.1
        Eigen::VectorXd sample = randstate(0.1);
        // get the nearest node id and state
        amp::Node nearest_id = nearestNode(sample);
        Eigen::VectorXd nearest_state = nodes[nearest_id];
        // find the best control to propagate that node state towards the sample
        Eigen::VectorXd control = findbestControl(nearest_state, sample);
        // propagate using that control
        Eigen::VectorXd next_state = nearest_state;
        agent.propagate(next_state, control, dt);
        // check collision in next_state: !! needs generalized to other problem statements
        if (!collisionCheckers::isLineInCollision(problem.obstacles, nearest_state, next_state) && pointInBounds(next_state))
        {
            // determine the cost
            double cost = (next_state - nearest_state).norm();
            nodes[node_count] = next_state;
            // add node to map
            graphPtr->connect(nearest_id, node_count, std::make_tuple(cost, control));
            graphPtr->connect(node_count, nearest_id, std::make_tuple(cost, control)); // bidirectional
            // check if we've hit goal and backtrack
            if (goalReached(next_state))
            {
                amp::Node current = node_count;
                // backtrack through graph
                while (current != 0) // while we have not reached root node
                {
                    auto outgoing_edges = graphPtr->outgoingEdges(current);
                    if (outgoing_edges.empty())
                    {
                        std::cerr << "No outgoing edges for node " << current << "\n";
                        path.valid = false;
                        break;
                    }
                    // get control from parent edge
                    auto parent_edge = outgoing_edges.front();
                    Eigen::VectorXd control = std::get<1>(parent_edge);
                    // push back point, control and duration
                    pushWp(nodes[current], control, dt);
                    // get parents (with error checking)
                    auto parents = graphPtr->parents(current);
                    if (parents.empty())
                    {
                        std::cerr << "No parent node for node " << current << "\n";
                        path.valid = false;
                        return path;
                    }
                    current = parents.front();
                }
                path.waypoints.push_back(problem.q_init);
                // reverse path
                std::reverse(path.waypoints.begin(), path.waypoints.end());
                std::reverse(path.controls.begin(), path.controls.end());
                std::reverse(path.durations.begin(), path.durations.end());
                path.valid = true;
                break;
            }
            node_count++;
            iter++;
        }
    }
    // catch RRT failures and return random path
    if (!path.valid)
    {
        LOG("RRT failed - pushing random path");
        state = problem.q_init;
        path.waypoints.push_back(state);
        for (int iter = 0; iter < 100; ++iter)
        {
            Eigen::VectorXd sample = randstate(0.9);
            Eigen::VectorXd control = findbestControl(state, sample);
            agent.propagate(state, control, dt);
            pushWp(state, control, dt);
            if ((state - goal).norm() < 0.05)
                break;
        }
        path.valid = true;
    }
    path.print();
    LOG("state bounds");
    printStuff(problem.q_bounds);
    LOG("control bounds");
    printStuff(problem.u_bounds);
    LOG("Controls:");
    printStuff2(path.controls);
    return path;
};
