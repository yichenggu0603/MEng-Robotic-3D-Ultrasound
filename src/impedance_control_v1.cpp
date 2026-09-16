// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example cartesian_impedance_control.cpp
 * An example showing a simple cartesian impedance controller without inertia shaping
 * that renders a spring damper system where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 *
 * @warning collision thresholds are set to high values. Make sure you have the user stop at hand!
 */

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();
    double baseline_fz = initial_state.O_F_ext_hat_K[2];
    std::cout << "Baseline Fz = " << baseline_fz << " N" << std::endl;


    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    size_t print_count = 0;
    const double target_force = 3.0;
    const double force_deadband = 0.4;
    const double z_step = 0.00003;      // 0.002 mm per 1 ms cycle = 2 mm/s max
    const double max_z_offset = 0.05;   // 3 mm

    const double initial_z = position_d.z();
    const double initial_x = position_d.x();
    const double x_step = 0.00001;      // 0.001 mm per cycle = 1 mm/s
    const double max_x_offset = 0.4;    // scan 2 cm along y
    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {
     double raw_fz = robot_state.O_F_ext_hat_K[2];
     double corrected_fz = raw_fz - baseline_fz;
     double contact_force = -corrected_fz;

     if (print_count++ % 1000 == 0) {
          std::cout << "raw Fz = " << raw_fz
                 << ", contact_force = " << contact_force
                 << ", z_offset = " << position_d.z() - initial_z
                 << ", x_offset = " << position_d.x() - initial_x
                 << std::endl;
       }
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      static Eigen::Matrix<double, 7, 1> q_initial;
      static bool q_initialized = false;

      if (!q_initialized) {
            q_initial = q;
            q_initialized = true;
      }
      
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());

      // compute error to desired equilibrium pose
      // position error
      double force_error = target_force - contact_force;

// Only start force adjustment after light contact is detected
     if (contact_force > 0.5) {
         if (std::abs(force_error) > force_deadband) {
             if (force_error > 0.0) {
      // force too small -> move equilibrium point down
                position_d.z() -= z_step;
             } else {
      // force too large -> move equilibrium point up
                position_d.z() += z_step;
             }
          }

  // safety limit
      if (position_d.z() > initial_z + max_z_offset) {
               position_d.z() = initial_z + max_z_offset;
            }
             if (position_d.z() < initial_z - max_z_offset) {
        position_d.z() = initial_z - max_z_offset;
            }
            
      // Move slowly along y while maintaining contact force
      if (std::abs(position_d.x() - initial_x) < max_x_offset) {
         position_d.x() += x_step;
        }
     }
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.rotation() * error.tail(3);

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      Eigen::Matrix<double, 7, 1> joint_stiffness;
         joint_stiffness << 0.0, 0.0, 0.0, 0.0, 0.0, 30.0, 0.0;

      Eigen::Matrix<double, 7, 1> joint_damping;
         joint_damping << 0.0, 0.0, 0.0, 0.0, 0.0, 2.0 * std::sqrt(30.0), 0.0;

      Eigen::Matrix<double, 7, 1> tau_joint_lock;
      tau_joint_lock =
            joint_stiffness.cwiseProduct(q_initial - q) -
            joint_damping.cwiseProduct(dq);

      tau_d << tau_task + tau_joint_lock + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
