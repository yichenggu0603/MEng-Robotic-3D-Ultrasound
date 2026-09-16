// Human-assisted hybrid controller for Franka/FR3.
//
// Control structure:
// - XY: the bias-corrected FR3 wrench estimate selects a desired direction;
//   Cartesian velocity damping then applies a bounded velocity-related force.
//   The fixed XY command speed is a reference, not a hard limit on motion
//   produced by directly moving the compliant robot.
// - Z: the operator can move the end effector manually through the impedance
//   response, while the controller adjusts the Z equilibrium using a
//   torque-derived contact estimate.
// - Orientation: low-stiffness Cartesian impedance permits manual adjustment.
//
// This is a standalone libfranka example, not a ROS 2 controller plugin.
//
// Usage:
//   ./<controller-executable> <robot-hostname>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

namespace {
std::atomic_bool stop_requested{false};
}  // namespace

struct LogSample {
  double time;
  double raw_fx;
  double raw_fy;
  double raw_fz;
  double corrected_fx;
  double corrected_fy;
  double corrected_fz;
  double contact_force;
  double target_force;
  double force_xy_norm;
  double v_des_x;
  double v_des_y;
  double v_x;
  double v_y;
  double z_dot;
  double x;
  double y;
  double z;
  double x_d;
  double y_d;
  double z_d;
  double z_offset;
  double f_z;
};

std::string makeTimestampedLogPath() {
  const std::string log_directory{"/home/vasst/impedance control log"};

  std::time_t now = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&now, &local_time);

  std::ostringstream path;
  path << log_directory << "/robot_control_log_"
       << std::put_time(&local_time, "%Y%m%d_%H%M%S")
       << ".csv";
  return path.str();
}

void saveControlLog(const std::string& path, const std::vector<LogSample>& samples) {
  std::ofstream log_file(path);
  if (!log_file.is_open()) {
    std::cerr << "Could not open log file: " << path << std::endl;
    return;
  }

  log_file << std::fixed << std::setprecision(6);
  log_file << "time,raw_fx,raw_fy,raw_fz,corrected_fx,corrected_fy,corrected_fz,"
           << "contact_force,target_force,force_xy_norm,v_des_x,v_des_y,v_x,v_y,z_dot,"
           << "x,y,z,x_d,y_d,z_d,z_offset,f_z\n";

  for (const LogSample& sample : samples) {
    log_file << sample.time << ','
             << sample.raw_fx << ','
             << sample.raw_fy << ','
             << sample.raw_fz << ','
             << sample.corrected_fx << ','
             << sample.corrected_fy << ','
             << sample.corrected_fz << ','
             << sample.contact_force << ','
             << sample.target_force << ','
             << sample.force_xy_norm << ','
             << sample.v_des_x << ','
             << sample.v_des_y << ','
             << sample.v_x << ','
             << sample.v_y << ','
             << sample.z_dot << ','
             << sample.x << ','
             << sample.y << ','
             << sample.z << ','
             << sample.x_d << ','
             << sample.y_d << ','
             << sample.z_d << ','
             << sample.z_offset << ','
             << sample.f_z << '\n';
  }

  std::cout << "Saved " << samples.size() << " log samples to " << path << std::endl;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // XY velocity intent parameters.
  const double xy_force_threshold{1.0};   // N
  const double xy_command_speed{0.005};   // m/s, fixed XY velocity reference
  const double xy_velocity_damping{30.0}; // N/(m/s), larger = stronger speed regulation
  const double xy_direction_sign{1.0};    // Coordinate-direction multiplier used in v5.1.

  // Z force-control parameters.
  const double z_stiffness{18.0};         // N/m
  const double z_damping{2.0 * std::sqrt(z_stiffness)};
  const double target_force{3.0};         // N
  const double contact_threshold{1.0};    // N
  const double force_deadband{1.0};       // N
  const double z_force_speed{0.02};        // m/s, 20 mm/s equilibrium adjustment
  const double max_z_offset{0.3};          // m, +/-300 mm development bound
  const double force_limit{20.0};          // N, software stop

  // Orientation impedance. Translational X/Y stiffness is intentionally not
  // used, because XY is velocity-controlled.
  const double rotational_stiffness{0.5};
  const double rotational_damping{2.0 * std::sqrt(rotational_stiffness)};
  const double angular_follow_speed{1.0}; // rad/s

  // Cartesian task-force limits used in the reported development tests.
  const double max_xy_task_force{10.0};   // N
  const double max_z_task_force{10.0};    // N
  const double max_rot_task_torque{3.0};  // Nm

  // The separate joint-6 locking term is disabled in v5.1. Joint 6 is instead
  // made easier to rotate by scaling its Cartesian task-torque contribution.
  const double q6_stiffness{0.0};
  const double q6_damping{2.0 * std::sqrt(q6_stiffness)};
  const double q6_follow_speed{0.5};  // rad/s
  const double max_q6_lock_torque{0.0};
  const double q6_task_torque_scale{0.3};  // 0=free except Coriolis, 1=full task control

  const std::string log_path{makeTimestampedLogPath()};
  const double log_period{0.01};  // 100 Hz logging for plotting/report data.
  double control_time = 0.0;
  double last_log_time = -log_period;
  std::vector<LogSample> log_samples;
  log_samples.reserve(120000);

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

    // Keep the configured Franka collision behavior active. These are
    // development settings and are not validated medical safety limits.
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
      control_time += dt;

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
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      static double q6_d = 0.0;
      static bool q6_initialized = false;

      if (!q6_initialized) {
        q6_d = q(5);  // joint 6 = index 5
        q6_initialized = true;
      }

      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());

      Eigen::Matrix<double, 6, 1> cartesian_velocity = jacobian * dq;

      // Map the bias-corrected XY wrench direction to a fixed velocity reference.
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

      // Adjust the Z impedance equilibrium from the torque-derived contact estimate.
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

      if (control_time - last_log_time >= log_period &&
          log_samples.size() < log_samples.capacity()) {
        log_samples.push_back({control_time,
                               raw_fx,
                               raw_fy,
                               raw_fz,
                               corrected_fx,
                               corrected_fy,
                               corrected_fz,
                               contact_force,
                               target_force,
                               force_xy_norm,
                               v_des_xy.x(),
                               v_des_xy.y(),
                               v_xy.x(),
                               v_xy.y(),
                               z_dot,
                               position.x(),
                               position.y(),
                               position.z(),
                               position_d.x(),
                               position_d.y(),
                               position_d.z(),
                               position_d.z() - initial_z,
                               f_z});
        last_log_time = control_time;
      }

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

      // Reduce all Cartesian task torque acting on joint 6 while preserving
      // its Coriolis compensation. This makes joint 6 easier to rotate by hand.
      tau_d(5) =
          coriolis(5) + q6_task_torque_scale * (tau_d(5) - coriolis(5));

      // This joint-6 locking path is retained from development but contributes
      // zero torque in v5.1 because its stiffness and torque limit are both zero.
      const double q6_follow_error = q(5) - q6_d;
      const double max_q6_step = q6_follow_speed * dt;
      q6_d += std::clamp(q6_follow_error, -max_q6_step, max_q6_step);

      double tau_q6_lock =
          q6_stiffness * (q6_d - q(5)) -
          q6_damping * dq(5);

      tau_q6_lock = std::clamp(tau_q6_lock,
                         -max_q6_lock_torque,
                         max_q6_lock_torque);

      tau_d(5) += tau_q6_lock;

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

      if (stop_requested.load()) {
        std::cout << "Stop requested. Finishing control and saving log." << std::endl;
        return franka::MotionFinished(franka::Torques(tau_d_array));
      }

      return tau_d_array;
    };

    std::cout << "Human-assisted hybrid interaction controller." << std::endl;
    std::cout << "XY: wrench direction -> velocity reference. "
              << "Z: compliant manual motion with contact-estimate equilibrium adjustment."
              << std::endl;
    std::cout << "Keep one hand near the user stop. Press Enter to start..." << std::endl;
    std::cin.ignore();
    std::cout << "Recording data. Press Enter again to stop and save the log." << std::endl;

    std::thread stop_input_thread([]() {
      std::cin.get();
      stop_requested.store(true);
    });
    stop_input_thread.detach();

    robot.control(control_callback);

  } catch (const franka::Exception& ex) {
    std::cout << ex.what() << std::endl;
  }

  saveControlLog(log_path, log_samples);

  return 0;
}
