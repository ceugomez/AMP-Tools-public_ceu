#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}
		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
		bool checkEnd(const Eigen::Vector2d& pos, const Eigen::Vector2d& goal);
	private:
		double d_star, zetta, Q_star, eta;
};

class MyPotentialFunction : public amp::PotentialFunction2D {
public:
    // Constructor to initialize with problem
    MyPotentialFunction(const amp::Problem2D& problem) : problem(problem) {
		for (const amp::Obstacle2D& obs : problem.obstacles){
			obs_centroid.push_back(obstacleCentroid(obs));
		}
    }

    // Returns the potential function value (height) for a given 2D point. 
    virtual double operator()(const Eigen::Vector2d& q) const override {
        Eigen::Vector2d diff = q - problem.q_goal;
        double fnval = 0.0;
        double U_rep = 0.0;
        // Attractive parabola centered on goal point
        double U_attr = diff.squaredNorm(); // (x - goal_x)^2 + (y - goal_y)^2
        for (const amp::Obstacle2D& obs : problem.obstacles) {
            // for every obstacleF
            Eigen::Vector2d closePoint = closestPointOnEdge(obs, q); // get closest point
            double dist = (closePoint - q).norm();
            if (dist < Q_star) {
                U_rep += eta * (1.0 / dist - 1.0 / Q_star) * (1.0 / dist - 1.0 / Q_star); // repulsive potential
            }
			
        }
        fnval = U_attr + U_rep;
        return fnval;
    }
    // Returns potential function gradient for a given point
    Eigen::Vector2d gradient(const Eigen::Vector2d& q) const {
        Eigen::Vector2d diff = q - problem.q_goal; 
        // attractive gradient 
        Eigen::Vector2d U_attr = Eigen::Vector2d(zeta*diff[0], zeta*diff[1]);
        // repulsive gradient 
        Eigen::Vector2d U_rep = Eigen::Vector2d::Zero();
        for (const amp::Obstacle2D& obs : problem.obstacles) {
            // for every obstacle
            Eigen::Vector2d closePoint = closestPointOnEdge(obs, q); // get closest point
            double dist = (closePoint - q).norm();
            if (dist < Q_star) {
                Eigen::Vector2d distVec = closePoint - q;
				Eigen::Vector2d closestVtx = obstacleCentroid(obs);
				U_rep += eta * (1.0 / dist - 1.0 / Q_star) * distVec / (dist * dist * dist); // repulsive gradient
				U_rep += eta * (1.0/(q-closestVtx).norm() - 1.0/Q_star)*(q-closestVtx)/(q-closestVtx).squaredNorm();
            }
        }
        return U_attr + U_rep;
    }
    virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override {
        return gradient(q);
    }

	Eigen::Vector2d getClosestVertex(const amp::Obstacle2D& obs, const Eigen::Vector2d pos) const{
		const std::vector<Eigen::Vector2d>& vertices = obs.verticesCCW();
		Eigen::Vector2d closestVertex;
		double minDistance = std::numeric_limits<double>::max();
		for (const Eigen::Vector2d& vertex : vertices) {
			double distance = (vertex - pos).norm();
			if (distance < minDistance) {
				minDistance = distance;
				closestVertex = vertex;
			}
		}

		return closestVertex;
	}
	Eigen::Vector2d obstacleCentroid(const amp::Obstacle2D& obs) const {
				double cumsumx = 0;
				double cumsumy = 0;
				int n = 0;
				for (Eigen::Vector2d vtx : obs.verticesCCW()){
					cumsumx+=vtx[0];
					cumsumy+=vtx[1];
					n++;
				}
				return Eigen::Vector2d(cumsumx/n, cumsumy/n);
			}
    Eigen::Vector2d closestPointOnEdge(const amp::Obstacle2D& obs, const Eigen::Vector2d& q) const {
        const std::vector<Eigen::Vector2d>& vertices = obs.verticesCCW();
        Eigen::Vector2d closestPoint;
        double minDistance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < vertices.size(); ++i) {
            Eigen::Vector2d p1 = vertices[i];
            Eigen::Vector2d p2 = vertices[(i + 1) % vertices.size()]; // Wrap around to the first vertex

            // Calculate the closest point on the line segment p1-p2 to q
            Eigen::Vector2d edge = p2 - p1;
            double t = (q - p1).dot(edge) / edge.squaredNorm();
            t = std::max(0.0, std::min(1.0, t)); // Clamp t to the range [0, 1]
            Eigen::Vector2d projection = p1 + t * edge;

            // Calculate the distance from q to the projection
            double distance = (q - projection).norm();
            if (distance < minDistance) {
                minDistance = distance;
                closestPoint = projection;
            }
        }

        return closestPoint;
    }

private:
    const amp::Problem2D& problem; // store the problem structure
    std::vector<Eigen::Vector2d> obs_centroid; // store center of obstacle avoidance circles
    // repulsive tuning parameters 
    double eta = 3.5; // 
    double Q_star = 0.45; // consider obstacles with radius closer than this point
    // attractive tuning parameters
    double zeta = 0.5;
};
