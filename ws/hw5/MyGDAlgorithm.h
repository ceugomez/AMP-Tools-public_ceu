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
			// get centroid and radii of each obstacle
			for (const amp::Obstacle2D& obs : problem.obstacles) {
				obs_centroid.push_back(MyPotentialFunction::obstacleCentroid(obs));
				obs_radii.push_back(MyPotentialFunction::obstacleRadii(obs, obs_centroid.back()));
			}
		}

		// Returns the potential function value (height) for a given 2D point. 
		virtual double operator()(const Eigen::Vector2d& q) const override {
			Eigen::Vector2d diff = q - problem.q_goal;
			double fnval = 0.0;
			double U_rep = 0.0;
			// Attractive parabola centered on goal point
			
			double U_attr = diff.squaredNorm(); // (x - goal_x)^2 + (y - goal_y)^2
			// build repulsive parabola centered on obstacle circles
			// Evaluate the 3 nearest obstacles
			std::vector<int> nearest_indices = getNearestObstacleIndices(q, obs_centroid, considerobs);
			for (int idx : nearest_indices) {
				double distance_to_obstacle = (obs_centroid[idx] - q).norm();
				double outer_radius = obs_radii[idx] + Q_star; // Narrower radius

				// Build repulsive force function
				if (distance_to_obstacle < outer_radius) {
					double repulsive_distance = outer_radius - distance_to_obstacle;
					U_rep += zeta * repulsive_distance * repulsive_distance; // Stronger quadratic repulsive term
				}
			}

			fnval = U_attr + U_rep;
			return fnval;
		}


		// Returns potential function gradient for a given point
		Eigen::Vector2d gradient(const Eigen::Vector2d& q) {
			Eigen::Vector2d diff = q-problem.q_goal; 
			// attractive gradient 
				Eigen::Vector2d U_attr = zeta*diff;
			// repulsive gradient 
				Eigen::Vector2d U_rep = Eigen::Vector2d::Zero();
				std::vector<int> nearest_indices = getNearestObstacleIndices(q, obs_centroid, considerobs);	// evaluate 3 nearest obstacles
				for (int idx : nearest_indices) {
					double distance_to_obstacle = (obs_centroid[idx] - q).norm();
					double outer_radius = obs_radii[idx] + Q_star; // Narrower radius

					if (distance_to_obstacle < outer_radius) {
						double repulsive_distance = outer_radius - distance_to_obstacle;
						U_rep += eta * repulsive_distance * (q - obs_centroid[idx]) / distance_to_obstacle; 
					}
				}

			return U_attr-U_rep;
		}
		// Helper function to return the indices of the nearest obstacles within the arrays of radii and centroid
		std::vector<int> getNearestObstacleIndices(Eigen::Vector2d pos, std::vector<Eigen::Vector2d> centroid_list, int k) const {
			std::vector<std::pair<double, int>> distances;
			for (size_t i = 0; i < centroid_list.size(); ++i) {
				double distance = (centroid_list[i] - pos).norm();
				distances.push_back(std::make_pair(distance, i));
			}
			std::sort(distances.begin(), distances.end());

			std::vector<int> nearest_indices;
			for (int i = 0; i < std::min(k, static_cast<int>(distances.size())); ++i) {
				nearest_indices.push_back(distances[i].second);
			}
			return nearest_indices;
		}
		// Helper function to get centroid of an obstacle
		Eigen::Vector2d obstacleCentroid(const amp::Obstacle2D& obs) {
			double cumsumx = 0;
			double cumsumy = 0;
			int n = 0;
			for (Eigen::Vector2d vtx : obs.verticesCCW()) {
				cumsumx += vtx[0];
				cumsumy += vtx[1];
				n++;
			}
			return Eigen::Vector2d(cumsumx / n, cumsumy / n);
		}
		// Helper function to get max radii of an obstacle
		double obstacleRadii(const amp::Obstacle2D& obs, const Eigen::Vector2d& centroid) {
			double maxDist = 0; // initialize distance counter
			for (Eigen::Vector2d vtx : obs.verticesCCW()) {
				if (maxDist < ((vtx - centroid).norm())) {
					maxDist = (vtx - centroid).norm(); // set circle radius to maximum vertex distance
				}
			}
			return maxDist;
		}
	private:
		const amp::Problem2D& problem; // store the problem structure
		std::vector<Eigen::Vector2d> obs_centroid; // store center of obstacle avoidance circles
		std::vector<double> obs_radii; // store obstacle radii 
		// repulsive tuning parameters 
		int considerobs = 5;
		double eta = 45.0;		// 
		double Q_star = 0.33;	// consider obstacles with radius closer than this point
		// attractive tuning parameters
		double zeta = 3.0;

};
