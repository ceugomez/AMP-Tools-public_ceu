#include "ManipulatorSkeleton.h"


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0, 1.5}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // forward kinematics to calculate the joint position given the manipulator state (angles)
    std::vector<double> links = m_link_lengths;
    Eigen::Vector2d ip = m_base_location;
    std::vector<Eigen::Vector2d> joint_positions;
    double theta_accum = 0.0;
    joint_positions.push_back(m_base_location);
    for (int i = 0; i <= joint_index; ++i) {
        theta_accum += state[i];
        Eigen::Vector2d joint_position;
        joint_position.x() = links[i] * cos(theta_accum)+joint_positions[i].x();
        joint_position.y() = links[i] * sin(theta_accum)+joint_positions[i].y();
        joint_positions.push_back(joint_position);
    }
    return joint_positions[joint_index];
}
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    amp::ManipulatorState joint_angles;
    joint_angles.setZero(nLinks()); // Ensure joint_angles is initialized to the correct size
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {
        // Implement IK for 2-link manipulator here
        return joint_angles;
    } else if (nLinks() == 3) {
        double x = end_effector_location[0] - m_base_location[0];
        double y = end_effector_location[1] - m_base_location[0];
        double x_s = m_base_location[0];
        double y_s = m_base_location[1];
        double L1 = m_link_lengths[0];
        double L2 = m_link_lengths[1];
        double L3 = m_link_lengths[2];
        
        // Calculate the distance from the base to the end-effector
        double r = sqrt((x)*(x) + (y)*(y));
        if (r > reach()) {
            LOG("goal point greater than reach");
            throw std::runtime_error("No solution exists for the given end-effector position. [R]");
        }

        // Calculate the wrist position
        double x_w = x - L3 * (x/r);
        double y_w = y - L3 * (y/r);
        double D = ((x_w)*(x_w) + (y_w)*(y_w) - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        if (D < -1 || D > 1) {
            throw std::runtime_error("No solution exists for the given end-effector position. [D]");
        }
        
        double theta2 = atan2(sqrt(1 - D * D), D);
        double theta1 = atan2(y_w, x_w) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
        double theta3 = atan2(y - y_w, x - x_w) - theta1 - theta2;
        
        joint_angles << theta1, theta2, theta3;
        return joint_angles;
    } else {
        // Implement IK for other cases here
        return joint_angles;
    }

    return joint_angles;
}
