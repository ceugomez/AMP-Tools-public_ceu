#include "../shared/collisionCheckers.h"
#include "../shared/HelperFns.h"

bool collisionCheckers::isPointInPolygon(const amp::Obstacle2D &obs, const Eigen::Vector2d &pos)
{
    int windingNumber = 0;
    std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];
        if (v1.y() <= pos.y())
        {
            if (v2.y() > pos.y() && HelperFns::crossProduct(v2 - v1, pos - v1) > 0)
            {
                ++windingNumber;
            }
        }
        else
        {
            if (v2.y() <= pos.y() && HelperFns::crossProduct(v2 - v1, pos - v1) < 0)
            {
                --windingNumber;
            }
        }
    }
    return windingNumber != 0;
}
bool collisionCheckers::isPointInAnyPolygon(const std::vector<amp::Obstacle2D> &obstacles, const Eigen::Vector2d &pos)
{
    for (const amp::Obstacle2D obs : obstacles)
    {
        int windingNumber = 0;
        std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            Eigen::Vector2d v1 = vertices[i];
            Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];
            if (v1.y() <= pos.y())
            {
                if (v2.y() > pos.y() && HelperFns::crossProduct(v2 - v1, pos - v1) > 0)
                {
                    ++windingNumber;
                }
            }
            else
            {
                if (v2.y() <= pos.y() && HelperFns::crossProduct(v2 - v1, pos - v1) < 0)
                {
                    --windingNumber;
                }
            }
        }
        if (windingNumber != 0)
        {
            // point in obstacle
            return windingNumber != 0;
        }
    }
    return false;
}
bool collisionCheckers::isArmInCollision(const amp::Obstacle2D &obs, const Eigen::Vector2d &pos, const amp::LinkManipulator2D &manipulator)
{
    auto n_links = manipulator.nLinks();
    Eigen::Vector2d prev_joint = manipulator.getBaseLocation();
    Eigen::Vector2d current_joint;
    std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();

    for (std::size_t i = 0; i < n_links; ++i)
    {
        current_joint = manipulator.getJointLocation(pos, i + 1);

        for (std::size_t j = 0; j < vertices.size(); ++j)
        {
            Eigen::Vector2d p1 = vertices[j];
            Eigen::Vector2d p2 = vertices[(j + 1) % vertices.size()]; // Ensure the polygon wraps around
            if (segmentsIntersect(prev_joint, current_joint, p1, p2))
            {
                return true;
            }
        }
        prev_joint = current_joint;
    }

    return false;
}
bool collisionCheckers::segmentsIntersect(const Eigen::Vector2d &p1, const Eigen::Vector2d &q1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q2)
{
    auto orientation = [](const Eigen::Vector2d &a, const Eigen::Vector2d &b, const Eigen::Vector2d &c)
    {
        double val = (b.y() - a.y()) * (c.x() - b.x()) - (b.x() - a.x()) * (c.y() - b.y());
        if (val == 0)
            return 0;             // collinear
        return (val > 0) ? 1 : 2; // direction, ccw or cw
    };

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;

    return false;
}
bool collisionCheckers::isLineInCollision(const std::vector<amp::Obstacle2D> &obstacles, const Eigen::Vector2d &q1, const Eigen::Vector2d q2)
{
    for (const amp::Obstacle2D &obs : obstacles)
    {
        const std::vector<Eigen::Vector2d> &vertices = obs.verticesCCW();
        size_t num_vertices = vertices.size();
        for (size_t i = 0; i < num_vertices; ++i)
        {
            Eigen::Vector2d a = vertices[i];
            Eigen::Vector2d b = vertices[(i + 1) % num_vertices]; // (wrap around)
            if (collisionCheckers::segmentsIntersect(q1, q2, a, b))
            {
                return true;
            }
        }
    }
    return false;
}
bool collisionCheckers::isPathInAnyPolygon(const std::vector<amp::Obstacle2D> &obstacles, const Eigen::Vector2d &q1, const Eigen::Vector2d &q2)
{
    // unnecessary but I don't feel like deleting it
    return true;
}
bool collisionCheckers::isPolygonPathFree(const std::vector<amp::Obstacle2D> &obstacles, const Eigen::VectorXd &q1, const Eigen::Vector2d &q2, const amp::AgentDimensions &dim)
{
    double stepSize = 0.01;
    double agentLength = dim.length+0.4; // margin
    double agentWidth = dim.width+0.4;   // margin 
    // compute the direction from q1 to q2 and the path length
    Eigen::Vector2d direction = (q2 - q1.head<2>()).normalized();
    double pathLength = (q2 - q1.head<2>()).norm();
    // helper function to calculate agent corners at a given position along the path
    auto getAgentCorners = [&](const Eigen::Vector2d &pos)
    {
        Eigen::Vector2d perpendicular(-direction.y(), direction.x()); // Perpendicular direction for width
        std::vector<Eigen::Vector2d> corners;
        corners.push_back(pos + 0.5 * agentLength * direction + 0.5 * agentWidth * perpendicular);
        corners.push_back(pos + 0.5 * agentLength * direction - 0.5 * agentWidth * perpendicular);
        corners.push_back(pos - 0.5 * agentLength * direction + 0.5 * agentWidth * perpendicular);
        corners.push_back(pos - 0.5 * agentLength * direction - 0.5 * agentWidth * perpendicular);
        return corners;
    };
    // iterate from q1 to q2 in stepsize
    for (double t = 0; t <= pathLength; t += stepSize)
    {
        Eigen::Vector2d currentPos = q1.head<2>() + t * direction;
        std::vector<Eigen::Vector2d> agentCorners = getAgentCorners(currentPos);

        // Check if the agent at this position collides with any obstacle
        for (const amp::Obstacle2D &obs : obstacles)
        {
            const std::vector<Eigen::Vector2d> &vertices = obs.verticesCCW();
            if (checkPolygonIntersection(agentCorners, vertices))
            {
                return false; 
            }
        }
    }
    // check end position
    std::vector<Eigen::Vector2d> agentCornersEnd = getAgentCorners(q2);
    for (const amp::Obstacle2D &obs : obstacles)
    {
        const std::vector<Eigen::Vector2d> &vertices = obs.verticesCCW();
        if (checkPolygonIntersection(agentCornersEnd, vertices))
        {
            return false; // Collision detected at endpoint
        }
    }
    return true; // no collision
}
bool collisionCheckers::checkPolygonIntersection(const std::vector<Eigen::Vector2d> &poly1,const std::vector<Eigen::Vector2d> &poly2)
{
    // separating axis theory
    auto getAxes = [](const std::vector<Eigen::Vector2d> &poly)
    {
        std::vector<Eigen::Vector2d> axes;
        for (size_t i = 0; i < poly.size(); ++i)
        {
            Eigen::Vector2d edge = poly[(i + 1) % poly.size()] - poly[i];
            axes.push_back(Eigen::Vector2d(-edge.y(), edge.x()).normalized()); // Perpendicular axis
        }
        return axes;
    };

    auto projectPolygon = [](const std::vector<Eigen::Vector2d> &poly, const Eigen::Vector2d &axis)
    {
        double minProj = axis.dot(poly[0]);
        double maxProj = minProj;
        for (const auto &vertex : poly)
        {
            double projection = axis.dot(vertex);
            if (projection < minProj)
                minProj = projection;
            if (projection > maxProj)
                maxProj = projection;
        }
        return std::make_pair(minProj, maxProj);
    };

    std::vector<Eigen::Vector2d> axes = getAxes(poly1);
    std::vector<Eigen::Vector2d> poly2Axes = getAxes(poly2);
    axes.insert(axes.end(), poly2Axes.begin(), poly2Axes.end());

    for (const auto &axis : axes)
    {
        auto [min1, max1] = projectPolygon(poly1, axis);
        auto [min2, max2] = projectPolygon(poly2, axis);
        if (max1 < min2 || max2 < min1)
        {
            return false; // No overlap on this axis
        }
    }
    return true; 
}