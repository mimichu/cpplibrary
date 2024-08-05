#pragma once
#include <Eigen/Dense>

#define GAP_T_SEC 0.1

namespace RUT {

/*
 * @brief A controller that interpolates linearly between two targets.
 * 
 * The controller is initialized with the current robot pose and time.
 * From that moment, the robot needs to call get_control and get a return true at
 *  every time step. If get_control returns false, the robot needs to call set_new_target
 * or keep_the_last_target to update the target, then call get_control again.
 */
class InterpolationController {
 public:
  InterpolationController() {}
  void initialize(const Eigen::VectorXd& pose0, double time0) {
    _pos0 = pose0.head<3>();
    _pos1 = pose0.head<3>();
    _quat0 = Eigen::Quaterniond(pose0(3), pose0(4), pose0(5), pose0(6));
    _quat1 = Eigen::Quaterniond(pose0(3), pose0(4), pose0(5), pose0(6));
    _time0 = time0;
    _time1 = time0;
  }

  // Try to get the control at time t, as a linear interpolation between target0 and target1.
  // t should never be less than _time0.
  // If t is smaller than _time1, the target is interpolated between _target0 and _target1.
  // If t is larger than _time1, return false. The outter loop should call set_new_target to update the target.
  bool get_control(double t, Eigen::Ref<Eigen::VectorXd> output) {
    assert(t >= _time0);
    if (t > _time1) {
      return false;
    }

    // Interpolate between _target0 and _target1
    output.head<3>() =
        _pos0 + (_pos1 - _pos0) / (_time1 - _time0) * (t - _time0);
    _output_quat = _quat0.slerp((t - _time0) / (_time1 - _time0), _quat1);
    output(3) = _output_quat.w();
    output(4) = _output_quat.x();
    output(5) = _output_quat.y();
    output(6) = _output_quat.z();

    return true;
  }

  bool set_new_target(const Eigen::VectorXd& new_target, double new_time) {
    if (new_time <= _time1) {
      std::cerr << "[InterpolationController] New target time: " << new_time
                << " is not larger than the current target time: " << _time1
                << std::endl;
      return false;
    }
    // std::cout << "[InterpolationController] New target: "
    //           << new_target.transpose() << ", new time: " << new_time
    //           << std::endl;
    // std::cout << "[InterpolationController] old: time 0 = " << _time0
    //           << ", time 1 = " << _time1 << std::endl;
    // std::cout << "[InterpolationController] old: pos 0 = " << _pos0.transpose()
    //           << ", pos 1 = " << _pos1.transpose() << std::endl;

    // now new_time > _time1
    _pos0 = _pos1;
    _quat0 = _quat1;
    _time0 = _time1;

    _pos1 = new_target.head<3>();
    _quat1.w() = new_target(3);
    _quat1.x() = new_target(4);
    _quat1.y() = new_target(5);
    _quat1.z() = new_target(6);
    _time1 = new_time;
    // std::cout << "[InterpolationController] new: time 0 = " << _time0
    //           << ", time 1 = " << _time1 << std::endl;
    // std::cout << "[InterpolationController] new: pos 0 = " << _pos0.transpose()
    //           << ", pos 1 = " << _pos1.transpose() << std::endl;

    return true;
  }

  // if there is no new waypoints to use,
  // we still need to update the current target time.
  bool keep_the_last_target(double new_time) {
    _pos0 = _pos1;
    _quat0 = _quat1;
    _time0 = _time1;
    _time1 = new_time + GAP_T_SEC;
    return true;
  }

 private:
  Eigen::Vector3d _pos0;
  Eigen::Vector3d _pos1;
  Eigen::Quaterniond _quat0;
  Eigen::Quaterniond _quat1;
  Eigen::Quaterniond _output_quat;
  double _time0;
  double _time1;
};

}  // namespace RUT