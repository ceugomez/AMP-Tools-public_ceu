#include "MyKinoRRT.h"

void MySingleIntegrator::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    state += dt * control;
};
void MyFirstOrderUnicycle::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    // constants
    double r = 0.25;
    // unpack state
    double x = state[0];
    double y = state[1];
    double theta = state[2];
    // unpack control
    double v = control[0];     // linear velocity
    double omega = control[1]; // angular velocity
    // lambda function to compute xdot
    auto dynamics = [&](double x, double y, double theta)
    {
        Eigen::VectorXd dxdt(3);
        dxdt[0] = v * r * cos(theta); // dx/dt
        dxdt[1] = v * r * sin(theta); // dy/dt
        dxdt[2] = omega;              // dtheta/dt
        return dxdt;
    };
    // RK4 intermediate steps
    Eigen::VectorXd k1 = dynamics(x, y, theta);
    Eigen::VectorXd k2 = dynamics(x + 0.5 * dt * k1[0], y + 0.5 * dt * k1[1], theta + 0.5 * dt * k1[2]);
    Eigen::VectorXd k3 = dynamics(x + 0.5 * dt * k2[0], y + 0.5 * dt * k2[1], theta + 0.5 * dt * k2[2]);
    Eigen::VectorXd k4 = dynamics(x + dt * k3[0], y + dt * k3[1], theta + dt * k3[2]);
    // update state using RK4
    x += (dt / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    y += (dt / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    theta += (dt / 6.0) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    // wrap theta to [-pi, pi]
    while (theta > M_PI)
    {
        theta -= 2 * M_PI;
    }
    while (theta < -M_PI)
    {
        theta += 2 * M_PI;
    }
    // update state
    state[0] = x;
    state[1] = y;
    state[2] = theta;
};
// simplecar propagate
void MySecondOrderUnicycle::propagate(Eigen::VectorXd &state, Eigen::VectorXd &control, double dt)
{
    if (control.size() != 2)
    {
        LOG("Error: Control input size is invalid.");
        control = Eigen::VectorXd::Zero(2);
    }

    // consts
    double r = 0.25;
    // unpack state
    double x = state[0];
    double y = state[1];
    double theta = state[2];
    double sigma = state[3];
    double omega = state[4];
    // unpack control
    double l = control[0]; // linear acceleration
    double a = control[1]; // angular acceleration
    // lambda function as xdot = f(x,u);
    auto dynamics = [&](double x, double y, double theta, double sigma, double omega)
    {
        Eigen::VectorXd dxdt(5);
        dxdt[0] = sigma * r * cos(theta); // dx/dt
        dxdt[1] = sigma * r * sin(theta); // dy/dt
        dxdt[2] = omega;                  // dtheta/dt
        dxdt[3] = l;                      // dsigma/dt (linear acceleration)
        dxdt[4] = a;                      // domega/dt (angular acceleration)
        return dxdt;
    };
    // define intermediate derivatives (k1, k2, k3, k4)
    // compute k1
    Eigen::VectorXd k1 = dynamics(x, y, theta, sigma, omega);
    // compute k2
    Eigen::VectorXd k2 = dynamics(
        x + 0.5 * dt * k1[0],
        y + 0.5 * dt * k1[1],
        theta + 0.5 * dt * k1[2],
        sigma + 0.5 * dt * k1[3],
        omega + 0.5 * dt * k1[4]);

    // compute k3
    Eigen::VectorXd k3 = dynamics(
        x + 0.5 * dt * k2[0],
        y + 0.5 * dt * k2[1],
        theta + 0.5 * dt * k2[2],
        sigma + 0.5 * dt * k2[3],
        omega + 0.5 * dt * k2[4]);

    // compute k4
    Eigen::VectorXd k4 = dynamics(
        x + dt * k3[0],
        y + dt * k3[1],
        theta + dt * k3[2],
        sigma + dt * k3[3],
        omega + dt * k3[4]);
    // RK4 integration - assign states
    state[0] += (dt / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    state[1] += (dt / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    state[2] += (dt / 6.0) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    state[3] += (dt / 6.0) * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
    state[4] += (dt / 6.0) * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]);
}
amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D &problem, amp::DynamicAgent &agent)
{
    // set up variables
    amp::KinoPath path;
    amp::RNG rng;

    // initialize dynamic step size counter
    double dt = 0.3;      // default value for dt
    const int adjust_threshold = 5;

    // initial delT based on which problem it is
    if (problem.q_goal.size() == 2)
        dt = 0.3;
    else if (problem.q_goal.size() == 3)
        dt = 0.5;
    else if (problem.q_goal.size() == 5)
        dt = 0.5;

    int iter = 0;
    bool goal_reached = false;
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(problem.q_goal.size());
    // pick a goal in the middle of goal region
    for (int i = 0; i < problem.q_goal.size(); ++i)
    {
        goal[i] = (problem.q_goal[i].first + problem.q_goal[i].second) / 2;
    }
    Eigen::VectorXd state;
    amp::Graph<std::tuple<double, Eigen::VectorXd, double>> graph; // cost, control, dt
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
        if (nearest.size() == 2)
        { // simple integrator - weight xy positioning
            return (next - goal).norm();
        }
        if (nearest.size() == 3)
        { //  1st order unicycle - only weight xy positioning
            double cost = abs((next[0] - goal[0])) + abs((next[1] - goal[1]));
            return cost;
        }
        if (nearest.size() == 5)
        { //  2nd order unicycle - only weight xy positioning
            double cost = abs((next[0] - goal[0])) + abs((next[1] - goal[1]));
            if ((next - goal).head<2>().norm() < 3)
            {                                        // if close to goal
                cost += abs(next[4]) + abs(next[5]); // also weight the magnitude of the control vector
            }
            return cost;
        }
        else
        {
            LOG("scoreControl Error: no recognized state format");
            return 1e10;
        }
    };
    // lambda to get random state from within problem bounds
    auto randstate = [&]()
    {
        Eigen::VectorXd sample;
        if (rng.randd(0.0, 1.0) < biasProb)
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
    // lambda to find best-of-many controls to propagate twds sample from nearest
    auto findbestControl = [&](const Eigen::VectorXd &nearest, const Eigen::VectorXd &sample)
    {
        int iters = controlIter;
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
    // lambda to check if goal is reached based on agent type :
    auto goalReached = [&](const Eigen::VectorXd &point)
    {
        // if state is 2-dimensional: simple integrator conditions
        if (problem.q_goal.size() == 2)
        {
            double dx = abs(point[0] - goal[0]);
            double dy = abs(point[1] - goal[1]);
            if (dy < 0.2 && dx < 0.2)
            {
                return true;
            }
        }
        // if state is 3-dimensional: 1st order unicycle conditions
        if (problem.q_goal.size() == 3)
        {
            double dx = abs(point[0] - goal[0]);
            double goalXwidth = abs(problem.q_goal[0].first-problem.q_goal[0].second)/2;
            double dy = abs(point[1] - goal[1]);
            double goalYwidth = abs(problem.q_goal[1].first-problem.q_goal[1].second)/2;
            if (dy < goalXwidth && dx < goalXwidth)
            {
                return true;
            }
        }
        // if state is 5-dimensional
        if (problem.q_goal.size() == 5)
        {
            double dx = abs(point[0] - goal[0]);
            double dy = abs(point[1] - goal[1]);
            double ctrlmag1 = abs(point[3]);
            double ctrlmag2 = abs(point[4]);
            if (dy < 0.4 && dx < 0.4 && ctrlmag1 < 2 && ctrlmag2 < 1)
            {
                return true;
            }
        }
        if (problem.q_goal.size() != 3 && problem.q_goal.size() != 5 && problem.q_goal.size() != 2)
        {
            LOG("State mismatch - unrecognized agent");
            LOG("State Size:");
            LOG(problem.q_goal.size());
            return false;
        }
        return false;
    };
    // lambda to collision check path based on agent type
    auto isPathFree = [&](const Eigen::VectorXd &nearest, const Eigen::VectorXd &next)
    {
        // 2 states means single integrator
        if (problem.q_init.size() == 2)
        {
            return !collisionCheckers::isLineInCollision(problem.obstacles, nearest, next) && pointInBounds(next);
        }
        // 3 states means 1st order unicycle
        if (problem.q_init.size() == 3)
        {
            return collisionCheckers::isPolygonPathFree(problem.obstacles, nearest, next, agent.agent_dim) && pointInBounds(next);
        }
        // 5 states means 2nd order unicycle
        if (problem.q_init.size() == 5)
        {
            return collisionCheckers::isPolygonPathFree(problem.obstacles, nearest, next, agent.agent_dim) && pointInBounds(next);
        }
        return false;
    };
    // lambda to dynamically adjust step size
    // Runtime loop
    path.valid = false;
    while (iter < max_iter && !goal_reached)
    {
        // sample random state from environment with goal bias
        Eigen::VectorXd sample = randstate();
        // get the nearest node id and state
        amp::Node nearest_id = nearestNode(sample);
        Eigen::VectorXd nearest_state = nodes[nearest_id];
        // find the best control to propagate that node state towards the sample
        Eigen::VectorXd control = findbestControl(nearest_state, sample);
        // propagate using that control
        Eigen::VectorXd next_state = nearest_state;
        agent.propagate(next_state, control, dt);
        // check collision in next_state and along path
        if (isPathFree(nearest_state, next_state))
        {
            // determine the cost
            double cost = (next_state - nearest_state).norm();
            nodes[node_count] = next_state;
            // add node to map
            graph.connect(nearest_id, node_count, std::make_tuple(cost, control, dt));
            graph.connect(node_count, nearest_id, std::make_tuple(cost, control, dt)); // bidirectional
            // check if we've hit goal and backtrack
            if (goalReached(next_state))
            {

                amp::Node current = node_count;
                // backtrack through graph
                while (current != 0) // while we have not reached root node
                {
                    auto outgoing_edges = graph.outgoingEdges(current);
                    if (outgoing_edges.empty())
                    {
                        std::cerr << "No outgoing edges for node " << current << "\n";
                        path.valid = false;
                        break;
                    }
                    // get control from parent edge
                    auto parent_edge = outgoing_edges.front();
                    Eigen::VectorXd control = std::get<1>(parent_edge);
                    // double dt = std::get<2>(parent_edge);
                    // push back point, control and duration
                    pushWp(nodes[current], control, dt);
                    // get parents (with error checking)
                    auto parents = graph.parents(current);
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
            Eigen::VectorXd sample = randstate();
            Eigen::VectorXd control = findbestControl(state, sample);
            agent.propagate(state, control, dt);
            pushWp(state, control, dt);
            if ((state - goal).norm() < 0.05)
                break;
        }
    }
    return path;
};
