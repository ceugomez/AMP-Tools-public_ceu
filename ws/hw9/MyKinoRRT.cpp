#include "MyKinoRRT.h"

void MySingleIntegrator::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    state += dt * control;
};
amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D &problem, amp::DynamicAgent &agent)
{
    amp::KinoPath path;
    amp::RNG rng;
    double dt = 0.5;
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(problem.q_goal.size());
    goal[0] = (problem.q_goal[0].first + problem.q_goal[0].second) / 2;
    goal[1] = (problem.q_goal[1].first + problem.q_goal[1].second) / 2;
    Eigen::VectorXd state;
    std::shared_ptr<amp::Graph<std::tuple<double, Eigen::VectorXd>>> graphPtr = std::make_shared<amp::Graph<std::tuple<double, Eigen::VectorXd>>>();
    std::map<amp::Node, Eigen::VectorXd> nodes;
    nodes[0] = problem.q_init;
    int node_count = 1;

    auto pushWp = [&](const Eigen::VectorXd &state, const Eigen::VectorXd &control, double t)
    {
        path.waypoints.push_back(state);
        path.controls.push_back(control);
        path.durations.push_back(t);
    };

    auto randcontrol = [&]()
    {
        Eigen::VectorXd u(problem.u_bounds.size());
        for (int i = 0; i < 2; ++i)
        {
            u[i] = rng.randd(problem.u_bounds[i].first, problem.u_bounds[i].second);
        }
        return u;
    };

    auto scoreControl = [&](Eigen::VectorXd nearest, Eigen::VectorXd control, Eigen::VectorXd goal)
    {
        Eigen::VectorXd next = nearest;
        agent.propagate(next, control, dt);
        return (next - goal).norm();
    };

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

    int iter = 0;
    const int max_iter = 1500;
    bool goal_reached = false;
    while (iter < max_iter && !goal_reached)
    {
        // sample random state from environment with goal bias 0.1
        Eigen::VectorXd sample = randstate(0.9);
        // get the nearest node id and state
        amp::Node nearest_id = nearestNode(sample);
        Eigen::VectorXd nearest_state = nodes[nearest_id];
        // find the best control to propagate that node state towards the sample
        Eigen::VectorXd control = findbestControl(nearest_state, sample);
        // propagate using that control
        Eigen::VectorXd next_state = nearest_state;
        agent.propagate(next_state, control, dt);
        // check collision in next_state:
        if (!collisionCheckers::isLineInCollision(problem.obstacles, nearest_state, next_state))
        {
            // determine the cost
            double cost = (next_state - nearest_state).norm();
            nodes[node_count] = next_state;
            // add node to map
            graphPtr->connect(nearest_id, node_count, std::make_tuple(cost, control));
            graphPtr->connect(node_count, nearest_id, std::make_tuple(cost, control)); // bidirectional
            // check if we've hit goal and backtrack
            if ((next_state - goal).norm() < 0.1)
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
    //path.print();
    return path;
}
