#pragma once
#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

// Derive the amp::LinkManipulator2D class
class MyManipulator2D : public amp::LinkManipulator2D {
    public:
        // Default constructor
        MyManipulator2D();
        // Override this method for implementing forward kinematics
        virtual Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;
        // Override this method for implementing inverse kinematics
        virtual amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
        // autodifferentiator for jacobian
        template<typename Scalar>
        Eigen::Matrix<Scalar, 2, 1> forwardKinematics(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& joint_angles) const;
        // compute jacobian given angles
        Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& joint_angles) const;
        // Declare the pseudoInverse function
        Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix) const;
    private:


};