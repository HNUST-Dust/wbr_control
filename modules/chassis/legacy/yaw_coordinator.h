// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_YAW_COORDINATOR_H_
#define WBR_CONTROL_CORE_YAW_COORDINATOR_H_

namespace wbr::v2 {

struct YawCoordinatorInput {
  bool reference_enabled = false;
  bool time_reset = false;
  double control_dt = 0.001;
  double commanded_yaw_rate = 0.0;
  double measured_yaw_rate = 0.0;
  double split_angle = 0.0;
  double split_rate = 0.0;
  double roll = 0.0;
  double roll_rate = 0.0;
  double pitch = 0.0;
  double normal_force[2] = {};
};

struct YawCoordinatorOutput {
  double coordinated_yaw_rate = 0.0;
  double split_residual = 0.0;
  double split_rate_residual = 0.0;
  double predicted_split_error = 0.0;
  double predicted_roll = 0.0;
  double predicted_normal_force[2] = {};
  double split_residual_authority = 1.0;
  double split_absolute_authority = 1.0;
  double split_authority = 1.0;
  double attitude_authority = 1.0;
  double contact_authority = 1.0;
  double authority = 1.0;
};

class YawCoordinator {
 public:
  void Reset();
  YawCoordinatorOutput Update(const YawCoordinatorInput& input);

 private:
  bool initialized_ = false;
  double coordinated_rate_ = 0.0;
  double split_reference_ = 0.0;
  double filtered_force_[2] = {};
  double filtered_force_rate_[2] = {};
};

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_YAW_COORDINATOR_H_
