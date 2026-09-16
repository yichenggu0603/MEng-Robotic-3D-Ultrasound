// Human-assisted hybrid controller for Franka/FR3.
//
// Control structure:
// - XY: human external force gives direction only; actual TCP velocity is
//   regulated toward a fixed speed by Cartesian velocity damping.
// - Z: autonomous contact-force regulation through a Z equilibrium point.
// - Orientation: low-stiffness Cartesian impedance.
//
// This is a standalone libfranka example, not a ROS 2 controller plugin.
//
// Usage:
//   ./human_assisted_hybrid_velocity_force_controller <robot-hostname>

#include <algorithm>
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

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // XY velocity intent parameters.
  const double xy_force_threshold{2.0};   // N
  const double xy_command_speed{0.0005};   // m/s, fixed TCP speed target
  const double xy_velocity_damping{175.0}; // N/(m/s), larger = stronger speed regulation
  const double xy_direction_sign{-1.0};    // Change to -1.0 if direction feels reversed.

  // Z force-control parameters.
  const double z_stiffness{30.0};         // N/m
  const double z_damping{2.0 * std::sqrt(z_stiffness)};
  const double target_force{3.0};         // N
  const double contact_threshold{0.5};    // N
  const double force_deadband{0.4};       // N
  const double z_force_speed{0.002};      // m/s, 2 mm/s equilibrium adjustment
  const double max_z_offset{0.08};       // m, +/-6 mm
  const double force_limit{10.0};          // N, software stop

  // Orientation impedance. Translational X/Y stiffness is intentionally not
  // used, because XY is velocity-controlled.
  const double rotational_stiffness{1.5};
  const double rotational_damping{2.0 * std::sqrt(rotational_stiffness)};
  const double angular_follow_speed{0.4}; // rad/s

  // Conservative Cartesian task-force limiter.
  const double max_xy_task_force{40.0};   // N
  const double max_z_task_force{10.0};    // N
  const double max_rot_task_torque{6.0};  // Nm

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();
    const double baseline_fx = initial_state.O_F_ext_hat_K[0];
    const double baseline_fy = initial_state.O_F_ext_hat_K[1];
    const double baseline_fz = initial_state.O_F_ext_hat_K[2];

    std::cout << "Force bias ready: "
              << "Fx=" << baseline_fx << " N, "
              << "Fy=" << baseline_fy << " N, "
              << "Fz=" << baseline_fz << " N" << std::endl;

    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    const double initial_z = position_d.z();

    // Keep robot safety reflexes active. Tune lower for final HRC/medical tests.
    robot.setCollisionBehavior({{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
                               {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
                               {{35.0, 35.0, 35.0, 20.0, 20.0, 20.0}},
                               {{35.0, 35.0, 35.0, 20.0, 20.0, 20.0}});

    size_t print_count = 0;

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        control_callback =
            [&](const franka::RobotState& robot_state,
                franka::Duration duration) -> franka::Torques {
      const double dt = duration.toSec();

      const double raw_fx = robot_state.O_F_ext_hat_K[0];
      const double raw_fy = robot_state.O_F_ext_hat_K[1];
      const double raw_fz = robot_state.O_F_ext_hat_K[2];

      const double corrected_fx = raw_fx - baseline_fx;
      const double corrected_fy = raw_fy - baseline_fy;
      const double corrected_fz = raw_fz - baseline_fz;

      const double contact_force = std::max(-corrected_fz, 0.0);

      if (contact_force > force_limit) {
        std::cout << "Force limit reached: " << contact_force << " N. Stop." << std::endl;
        return franka::MotionFinished(franka::Torques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      }

      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());

      Eigen::Matrix<double, 6, 1> cartesian_velocity = jacobian * dq;

      // Human intent -> fixed XY desired velocity.
      Eigen::Vector2d human_force_xy;
      human_force_xy << corrected_fx, corrected_fy;
      const double force_xy_norm = human_force_xy.norm();

      Eigen::Vector2d v_des_xy;
      v_des_xy.setZero();
      if (force_xy_norm > xy_force_threshold) {
        Eigen::Vector2d direction = xy_direction_sign * human_force_xy / force_xy_norm;
        v_des_xy = xy_command_speed * direction;
      }

      Eigen::Vector2d v_xy;
      v_xy << cartesian_velocity(0), cartesian_velocity(1);

      Eigen::Vector2d f_xy = -xy_velocity_damping * (v_xy - v_des_xy);
      const double f_xy_norm = f_xy.norm();
      if (f_xy_norm > max_xy_task_force) {
        f_xy *= max_xy_task_force / f_xy_norm;
      }

      // Integrate desired XY only for logging/reference, not for XY stiffness.
      position_d.x() += v_des_xy.x() * dt;
      position_d.y() += v_des_xy.y() * dt;

      // Orientation target follows hand-guided orientation slowly.
      const double angle_error = orientation_d.angularDistance(orientation);
      if (angle_error > 1e-6) {
        const double alpha = std::min(1.0, angular_follow_speed * dt / angle_error);
        orientation_d = orientation_d.slerp(alpha, orientation);
        orientation_d.normalize();
      }

      // Z force regulation by moving Z equilibrium.
      const double force_error = target_force - contact_force;
      if (contact_force > contact_threshold && std::abs(force_error) > force_deadband) {
        if (force_error > 0.0) {
          position_d.z() -= z_force_speed * dt;
        } else {
          position_d.z() += z_force_speed * dt;
        }
      }
      position_d.z() = std::clamp(position_d.z(), initial_z - max_z_offset,
                                  initial_z + max_z_offset);

      const double z_error = position.z() - position_d.z();
      const double z_dot = cartesian_velocity(2);
      double f_z = -z_stiffness * z_error - z_damping * z_dot;
      f_z = std::clamp(f_z, -max_z_task_force, max_z_task_force);

      // Orientation impedance.
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() = -orientation.coeffs();
      }
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      Eigen::Vector3d orientation_error;
      orientation_error << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      orientation_error = -transform.rotation() * orientation_error;

      Eigen::Vector3d omega;
      omega << cartesian_velocity(3), cartesian_velocity(4), cartesian_velocity(5);

      Eigen::Vector3d tau_rot =
          -rotational_stiffness * orientation_error - rotational_damping * omega;
      const double tau_rot_norm = tau_rot.norm();
      if (tau_rot_norm > max_rot_task_torque) {
        tau_rot *= max_rot_task_torque / tau_rot_norm;
      }

      Eigen::Matrix<double, 6, 1> task_wrench;
      task_wrench << f_xy.x(), f_xy.y(), f_z, tau_rot.x(), tau_rot.y(), tau_rot.z();

      Eigen::Matrix<double, 7, 1> tau_d = jacobian.transpose() * task_wrench + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::Matrix<double, 7, 1>::Map(&tau_d_array[0]) = tau_d;

      if (print_count++ % 1000 == 0) {
        std::cout << "Fxy_norm=" << force_xy_norm
                  << " v_des_xy=[" << v_des_xy.x() << ", " << v_des_xy.y() << "]"
                  << " v_xy=[" << v_xy.x() << ", " << v_xy.y() << "]"
                  << " contact_force=" << contact_force
                  << " z_offset=" << position_d.z() - initial_z
                  << std::endl;
      }

      return tau_d_array;
    };

    std::cout << "Human-assisted hybrid velocity/force controller." << std::endl;
    std::cout << "XY: force direction -> fixed velocity. Z: autonomous contact force."
              << std::endl;
    std::cout << "Keep one hand near the user stop. Press Enter to start..." << std::endl;
    std::cin.ignore();

    robot.control(control_callback);

  } catch (const franka::Exception& ex) {
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
