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
// Forward kinematics function using AutoDiff
template<typename Scalar>
Eigen::Matrix<Scalar, 2, 1> MyManipulator2D::forwardKinematics(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& joint_angles) const {
    std::vector<double> links = m_link_lengths;
    Eigen::Matrix<Scalar, 2, 1> ip(m_base_location[0], m_base_location[1]);
    std::vector<Eigen::Matrix<Scalar, 2, 1>> joint_positions;
    Scalar theta_accum = 0.0;
    joint_positions.push_back(ip);
    for (int i = 0; i < joint_angles.size(); ++i) {
        theta_accum += joint_angles[i];
        Eigen::Matrix<Scalar, 2, 1> joint_position;
        joint_position.x() = links[i] * cos(theta_accum) + joint_positions[i].x();
        joint_position.y() = links[i] * sin(theta_accum) + joint_positions[i].y();
        joint_positions.push_back(joint_position);
    }
    return joint_positions.back();
}

// Compute the Jacobian using AutoDiff
Eigen::MatrixXd MyManipulator2D::computeJacobian(const Eigen::VectorXd& joint_angles) const {
    using AutoDiffScalar = Eigen::AutoDiffScalar<Eigen::VectorXd>;
    Eigen::Matrix<AutoDiffScalar, Eigen::Dynamic, 1> joint_angles_ad = joint_angles.cast<AutoDiffScalar>();
    for (int i = 0; i < joint_angles.size(); ++i) {
        joint_angles_ad[i].derivatives() = Eigen::VectorXd::Unit(joint_angles.size(), i);
    }
    Eigen::Matrix<AutoDiffScalar, 2, 1> end_effector_position_ad = forwardKinematics(joint_angles_ad);
    Eigen::MatrixXd jacobian(2, joint_angles.size());
    for (int i = 0; i < 2; ++i) {
        jacobian.row(i) = end_effector_position_ad[i].derivatives();
    }
    return jacobian;
}

amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    amp::ManipulatorState joint_angles;     // state vector
    Eigen::Vector2d fp;
    fp[0] = end_effector_location[0] - m_base_location[0];
    fp[1] = end_effector_location[1] - m_base_location[1];
    
    joint_angles.setZero(nLinks());      
    // least-squares formulation of inverse kinematics for n-link planar manipulator
    const double tolerance = 1e-6; 
    const int max_iterations = 5000;
    Eigen::Vector2d current_position;
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd delta_q;
    Eigen::Vector2d error;

    for (int iter = 0; iter < max_iterations; ++iter) {
        //LOG(iter);
        // Compute the current end-effector position using forward kinematics
        current_position = forwardKinematics(joint_angles);

        // Compute the error between the desired and current end-effector positions
        error = fp - current_position;

        // Check for convergence
        if (error.norm() < tolerance) {
            break;
            LOG("error tolerance reached!");
        }

        // Compute the Jacobian
        jacobian = computeJacobian(joint_angles);

        // Solve the least-squares problem to find the change in joint angles
        delta_q = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse() * error;

        // Update the joint angles
        joint_angles += delta_q;
        if (iter==(max_iterations-1)){
        LOG("Did not converge to solution!");
        }
    }


    return joint_angles;
}