#include <iostream>
#include <cmath>
#include <Eigen/Dense>


const double TOLERANCE = 1e-6;
const int MAX_ITERATIONS = 1000;

Eigen::Matrix3d rotation_matrix_x(double theta) {
    Eigen::Matrix3d R;
    R << 1, 0, 0,
         0, cos(theta), -sin(theta),
         0, sin(theta), cos(theta);
    return R;
}

Eigen::Matrix3d rotation_matrix_y(double theta) {
    Eigen::Matrix3d R;
    R << cos(theta), 0, sin(theta),
         0, 1, 0,
         -sin(theta), 0, cos(theta);
    return R;
}

Eigen::Matrix3d rotation_matrix_z(double theta) {
    Eigen::Matrix3d R;
    R << cos(theta), -sin(theta), 0,
         sin(theta), cos(theta), 0,
         0, 0, 1;
    return R;
}

Eigen::Matrix<double, 6, 4> jacobian_matrix(const Eigen::VectorXd& joint_angles, const Eigen::VectorXd& link_lengths) {
    double theta1 = joint_angles(0);
    double theta2 = joint_angles(1);
    double theta3 = joint_angles(2);
    double theta4 = joint_angles(3);

    double l1 = link_lengths(0);
    double l2 = link_lengths(1);
    double l3 = link_lengths(2);
    double l4 = link_lengths(3);

    // Calculate rotation matrices
    Eigen::Matrix3d R0_1 = rotation_matrix_z(theta1);
    Eigen::Matrix3d R1_2 = rotation_matrix_y(theta2);
    Eigen::Matrix3d R2_3 = rotation_matrix_y(theta3);
    Eigen::Matrix3d R3_4 = rotation_matrix_z(theta4);

    // Calculate joint positions
    Eigen::Vector3d P0_1(0, 0, l1);
    Eigen::Vector3d P1_2 = R0_1 * Eigen::Vector3d(l2, 0, 0);
    Eigen::Vector3d P2_3 = R0_1 * R1_2 * Eigen::Vector3d(l3, 0, 0);
    Eigen::Vector3d P3_4 = R0_1 * R1_2 * R2_3 * Eigen::Vector3d(0, 0, l4);

    // Calculate joint axes
    Eigen::Vector3d Z0(0, 0, 1);
    Eigen::Vector3d Z1 = R0_1 * Eigen::Vector3d(0, 1, 0);
    Eigen::Vector3d Z2 = R0_1 * R1_2 * Eigen::Vector3d(0, 1, 0);
    Eigen::Vector3d Z3 = R0_1 * R1_2 * R2_3 * Eigen::Vector3d(1, 0, 0);

    // Calculate the end-effector position
    Eigen::Vector3d P_end_effector = P0_1 + P1_2 + P2_3 + P3_4;

    // Calculate the Jacobian matrix
    Eigen::Matrix<double, 6, 4> J;
    J.block<3, 1>(0, 0) = Z0.cross(P_end_effector - P0_1);
    J.block<3, 1>(0, 1) = Z1.cross(P_end_effector - P1_2);
    J.block<3, 1>(0, 2) = Z2.cross(P_end_effector - P2_3);
    J.block<3, 1>(0, 3) = Z3.cross(P_end_effector - P3_4);
    J.block<3, 1>(3, 0) = Z0;
    J.block<3, 1>(3, 1) = Z1;
    J.block<3, 1>(3, 2) = Z2;
    J.block<3, 1>(3, 3) = Z3;

    return J;
}

Eigen::VectorXd inverse_kinematics(const Eigen::Vector3d& target_position, const Eigen::VectorXd& link_lengths, const Eigen::VectorXd& initial_joint_angles) {
    Eigen::VectorXd joint_angles = initial_joint_angles;
    int iteration = 0;

    while (iteration < MAX_ITERATIONS) {
        Eigen::Matrix<double, 6, 4> J = jacobian_matrix(joint_angles, link_lengths);
        Eigen::Matrix<double, 4, 6> J_pseudo_inverse = J.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::Vector3d current_position = J.block<3, 1>(0, 3);
        Eigen::Vector3d position_error = target_position - current_position;
        Eigen::VectorXd joint_angle_update = J_pseudo_inverse.block<4, 3>(0, 0) * position_error;

        joint_angles += joint_angle_update;

        if (position_error.norm() < TOLERANCE) {
            break;
        }

        iteration++;
    }

    return joint_angles;
}



int main() {
    Eigen::Vector3d target_position(3, 1, 5);
    Eigen::VectorXd link_lengths(4);
    link_lengths << 2, 3, 3, 1; 
    Eigen::VectorXd initial_joint_angles(4);
    initial_joint_angles << 0,45, 45,0;

    Eigen::VectorXd resulting_joint_angles = inverse_kinematics(target_position, link_lengths, initial_joint_angles);
    std::cout << "Resulting joint angles:\n" << resulting_joint_angles << std::endl;

    return 0;

    return 0;
}