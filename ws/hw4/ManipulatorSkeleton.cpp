#include "ManipulatorSkeleton.h"


MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
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

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here

    amp::ManipulatorState joint_angles;
    joint_angles.setZero();
    
    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    if (nLinks() == 2) {

        return joint_angles;
    } else if (nLinks() == 3) {

        return joint_angles;
    } else {

        return joint_angles;
    }

    return joint_angles;
}