/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <pid.hpp>

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <example_controller_plugin/example_controllerConfig.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

// | ----------------- Calling required libraries ----------------- |
#include <math.h>
#include <Eigen/Dense>

// | ----------------- Time related variables ----------------- |
float IITGN_text_start_time = 0.0;
double initial_ros_time_custom_controller = 0.0;
float t1_IITGN_traj = 0.0;
float t2_IITGN_traj = 0.0;
// | ----------------- Position related variables ----------------- |
float sty_IITGN_traj = 0.0;
float stz_IITGN_traj = 0.0;
float eny_IITGN_traj = 0.0;
float enz_IITGN_traj = 0.0;

// | ----------------- State of the system ----------------- |
float quad_x      = 0.0;
float quad_y      = 0.0;
float quad_z      = 0.0;

float quad_x_dot  = 0.0;
float quad_y_dot  = 0.0;
float quad_z_dot  = 0.0;

float des_quad_x      = 0.0;
float des_quad_y      = 0.0;
float des_quad_z      = 2.0;

float des_quad_x_dot  = 0.0;
float des_quad_y_dot  = 0.0;
float des_quad_z_dot  = 0.0;

// | ----------------- Custom Gains ----------------- |

float kpz         = 0.1;
float kdz         = 0.1;

// | ----------------- High level commands ----------------- |
float des_roll_angle    = 0.0;
float des_pitch_angle   = 0.0;
float des_yaw_angle     = 0.0;
float thrust_force      = 0.01;

//}

namespace example_controller_plugin
{

namespace example_controller
{

/* //{ class ExampleController */

class ExampleController : public mrs_uav_managers::Controller {

public:
  bool initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command);

  ////////////////////////////////////////////////
  //// for custom controller
  ////////////////////////////////////////////////
  float norm_2(Eigen::Vector3d A, Eigen::Vector3d B);
  float min_acc_first_coefficient(float t1, float t2, float st, float en);
  float min_acc_second_coefficient(float t1, float t2, float st, float en);
  float min_acc_third_coefficient(float t1, float t2, float st, float en);
  float min_acc_fourth_coefficient(float t1, float t2, float st, float en);
  void IITGN_text_traj_planning(void);
  ////////////////////////////////////////////////
  //// for custom controller
  ////////////////////////////////////////////////

  ControlOutput updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                      mutex_drs_;
  typedef example_controller_plugin::example_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>            Drs_t;
  boost::shared_ptr<Drs_t>                                    drs_;
  void                                                        callbackDrs(example_controller_plugin::example_controllerConfig& config, uint32_t level);
  DrsConfig_t                                                 drs_params_;
  std::mutex                                                  mutex_drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool ExampleController::initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  last_update_time_ = ros::Time(0);
  initial_ros_time_custom_controller =ros::Time::now().toSec();

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  bool success = true;

  // FYI
  // This method will load the file using `rosparam get`
  //   Pros: you can the full power of the official param loading
  //   Cons: it is slower
  //
  // Alternatives:
  //   You can load the file directly into the ParamLoader as shown below.

  success *= private_handlers->loadConfigFile(ros::package::getPath("example_controller_plugin") + "/config/example_controller.yaml");

  if (!success) {
    return false;
  }

  mrs_lib::ParamLoader param_loader(nh_, "ExampleController");

  // This is the alternaive way of loading the config file.
  //
  // Files loaded using this method are prioritized over ROS params.
  //
  // param_loader.addYamlFile(ros::package::getPath("example_tracker_plugin") + "/config/example_tracker.yaml");

  param_loader.loadParam("desired_roll", drs_params_.roll);
  param_loader.loadParam("desired_pitch", drs_params_.pitch);
  param_loader.loadParam("desired_yaw", drs_params_.yaw);
  param_loader.loadParam("desired_thrust_force", drs_params_.force);
  param_loader.loadParam("kpz_value", kpz);
  param_loader.loadParam("kdz_value", kdz);

  // | ------------------ finish loading params ----------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ExampleController]: could not load all parameters!");
    return false;
  }

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&ExampleController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[ExampleController]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool ExampleController::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;

  is_active_ = true;

  ROS_INFO("[ExampleController]: activated");

  return true;
}

//}

/* //{ deactivate() */

void ExampleController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  ROS_INFO("[ExampleController]: deactivated");
}

//}

/* updateInactive() //{ */

void ExampleController::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */

ExampleController::ControlOutput ExampleController::updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  // clear all the optional parts of the result
  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};

  if (!is_active_) {
    return last_control_output_;
  }

  // | ---------- calculate dt from the last iteration ---------- |
  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (fabs(dt) < 0.001) {

    ROS_DEBUG("[ExampleController]: the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // | -------- check for the available output modalities ------- |

  // you can decide what to return, but it needs to be available
  if (common_handlers_->control_output_modalities.attitude) {
    ROS_INFO_THROTTLE(1.0, "[ExampleController]: desired attitude output modality is available");
  }

  // | ---------- extract the detailed model parameters --------- |

  if (common_handlers_->detailed_model_params) {

    mrs_uav_managers::control_manager::DetailedModelParams_t detailed_model_params = common_handlers_->detailed_model_params.value();

    ROS_INFO_STREAM_THROTTLE(1.0, "[ExampleController]: UAV inertia is: " << detailed_model_params.inertia);
  }

///////////////////////////////////////////////
///////////////////////////////////////////////
//////         Custom controller starts
///////////////////////////////////////////////
///////////////////////////////////////////////

  // | -------------- prepare the control reference ------------- |

  IITGN_text_start_time = ros::Time::now().toSec() - initial_ros_time_custom_controller;

  ROS_INFO_STREAM_THROTTLE(1, "[ExampleController]: Current Time: " << IITGN_text_start_time);

  geometry_msgs::PoseStamped position_reference;

  position_reference.header           = tracker_command.header;
  position_reference.pose.position    = tracker_command.position;
  position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);

  // | ---------------- Custom PD Controller for altitude control --------------- |
  quad_x = uav_state.pose.position.x;
  quad_y = uav_state.pose.position.y;
  quad_z = uav_state.pose.position.z;

  quad_x_dot = uav_state.velocity.linear.x;
  quad_y_dot = uav_state.velocity.linear.y;
  quad_z_dot = uav_state.velocity.linear.z;

  des_quad_z = 2.0 + sin(IITGN_text_start_time);

  thrust_force = kpz * ( des_quad_z - quad_z) + kdz * ( des_quad_z_dot - quad_z_dot);

  // | ---------------- prepare the control output --------------- |

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;

  drs_params.roll     = des_roll_angle;
  drs_params.pitch    = des_pitch_angle;
  drs_params.yaw      = des_yaw_angle;
  drs_params.force    = thrust_force;

  attitude_cmd.orientation = mrs_lib::AttitudeConverter(drs_params.roll, drs_params.pitch, drs_params.yaw);
  attitude_cmd.throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model,
                                                                             common_handlers_->getMass() * common_handlers_->g + drs_params.force);
  //////////////////////// Previous code ends ////////////////////////

  // | ----------------- set the control output ----------------- |

  last_control_output_.control_output = attitude_cmd;

  // | --------------- fill in the optional parts --------------- |

  //// it is recommended to fill the optinal parts if you know them

  /// this is used for:
  // * plotting the orientation in the control_refence topic (optional)
  // * checking for attitude control error
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(drs_params.roll, drs_params.pitch, drs_params.yaw);

  /// IMPORANT
  // The acceleration and heading rate in 3D (expressed in the "fcu" frame of reference) that the UAV will actually undergo due to the control action.
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(0, 0, 0);
  last_control_output_.desired_heading_rate          = 0;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.controller = "ExampleController";

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus ExampleController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void ExampleController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState& new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void ExampleController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr ExampleController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  ROS_INFO("[ExampleController]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void ExampleController::callbackDrs(example_controller_plugin::example_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[ExampleController]: dynamic reconfigure params updated");
}

void ExampleController::IITGN_text_traj_planning(){

  Eigen::Vector3d Z(0,0,0);
  Eigen::Vector3d A(0,0,2);
  Eigen::Vector3d B(0,0,5);
  Eigen::Vector3d C(0,1,5);
  Eigen::Vector3d D(0,1,2);
  Eigen::Vector3d E(0,2.5,2);
  Eigen::Vector3d FF(0,2.5,5);
  Eigen::Vector3d G(0,1.5,5);
  Eigen::Vector3d H(0,3.5,5);
  Eigen::Vector3d I(0,6,4.5);
  Eigen::Vector3d J(0,6,5);
  Eigen::Vector3d K(0,4,5);
  Eigen::Vector3d L(0,4,2);
  Eigen::Vector3d M(0,6,2);
  Eigen::Vector3d N(0,6,3.5);
  Eigen::Vector3d O(0,5,3.5);
  Eigen::Vector3d P(0,7,2);
  Eigen::Vector3d Q(0,7,5);
  Eigen::Vector3d R(0,9,2);
  Eigen::Vector3d S(0,9,5);
  Eigen::Vector3d Y(0,9,0);
  
  float Pos_array[21][3] = {{0,0,0},
                           {0,0,2},
                           {0,0,5},
                           {0,1,5},
                           {0,1,2},
                           {0,2.5,2},
                           {0,2.5,5},
                           {0,1.5,5},
                           {0,3.5,5},
                           {0,6,4.5},
                           {0,6,5},
                           {0,4,5},
                           {0,4,2},
                           {0,6,2},
                           {0,6,3.5},
                           {0,5,3.5},
                           {0,7,2},
                           {0,7,5},
                           {0,9,2},
                           {0,9,5},
                           {0,9,0}};
  
  // float light[21][1] = {{0},
  //              {1},
  //              {0},
  //              {1},
  //              {0},
  //              {1},
  //              {1},
  //              {1},
  //              {0},
  //              {1},
  //              {1},
  //              {1},
  //              {1},
  //              {1},
  //              {1},
  //              {0},
  //              {1},
  //              {1},
  //              {1},
  //              {0},
  //              {0}};
  
  float V_max = 0.5;
  
  float  tZ = 0;
  float  tA = tZ + (norm_2(A , Z)/V_max);
  float  tB = tA + (norm_2(B , A)/V_max);
  float  tC = tB + (norm_2(C , B)/V_max);
  float  tD = tC + (norm_2(D , C)/V_max);
  float  tE = tD + (norm_2(E , D)/V_max);
  float  tF = tE + (norm_2(FF , E)/V_max);
  float  tG = tF + (norm_2(G , FF)/V_max);
  float  tH = tG + (norm_2(H , G)/V_max);
  float  tI = tH + (norm_2(I , H)/V_max);
  float  tJ = tI + (norm_2(J , I)/V_max);
  float  tK = tJ + (norm_2(K , J)/V_max);
  float  tL = tK + (norm_2(L , K)/V_max);
  float  tM = tL + (norm_2(M , L)/V_max);
  float  tN = tM + (norm_2(N , M)/V_max);
  float  tO = tN + (norm_2(O , N)/V_max);
  float  tP = tO + (norm_2(P , O)/V_max);
  float  tQ = tP + (norm_2(Q , P)/V_max);
  float  tR = tQ + (norm_2(R , Q)/V_max);
  float  tS = tR + (norm_2(S , R)/V_max);
  float  tY = tS + (norm_2(Y , S)/V_max);
  
  // hal.console->printf("%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f\n", tO,tP,tQ,tR,tS,tY);
  
  float t_array[21][1] = {{tZ},
                          {tA},
                          {tB},
                          {tC},
                          {tD},
                          {tE},
                          {tF},
                          {tG},
                          {tH},
                          {tI},
                          {tJ},
                          {tK},
                          {tL},
                          {tM},
                          {tN},
                          {tO},
                          {tP},
                          {tQ},
                          {tR},
                          {tS},
                          {tY}};
  
  // hal.console->printf("%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n", t_array[6][0],t_array[7][0],t_array[8][0],t_array[9][0],t_array[10][0],t_array[11][0]);
  
  float tt = IITGN_text_start_time;
  ROS_INFO_STREAM_THROTTLE(1, "[ExampleController]: Current Time: " << tt);

  for (int i = 0; i < 21; i++) {
  
       if (IITGN_text_start_time >= t_array[i][0]){
          t1_IITGN_traj  = t_array[i][0];
          t2_IITGN_traj  = t_array[i+1][0];
          sty_IITGN_traj = Pos_array[i][1];
          stz_IITGN_traj = Pos_array[i][2];
          eny_IITGN_traj = Pos_array[i+1][1];
          enz_IITGN_traj = Pos_array[i+1][2];
       }
  }
  
  // hal.console->printf("%5.3f, %5.3f\n",t1_IITGN_traj,t2_IITGN_traj);
  
  float ay =  min_acc_first_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) sty_IITGN_traj, (float) eny_IITGN_traj);
  float by = min_acc_second_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) sty_IITGN_traj, (float) eny_IITGN_traj);
  float cy =  min_acc_third_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) sty_IITGN_traj, (float) eny_IITGN_traj);
  float dy = min_acc_fourth_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) sty_IITGN_traj, (float) eny_IITGN_traj);
  
  float az =  min_acc_first_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) stz_IITGN_traj, (float) enz_IITGN_traj);
  float bz = min_acc_second_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) stz_IITGN_traj, (float) enz_IITGN_traj);
  float cz =  min_acc_third_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) stz_IITGN_traj, (float) enz_IITGN_traj);
  float dz = min_acc_fourth_coefficient((float) t1_IITGN_traj, (float) t2_IITGN_traj, (float) stz_IITGN_traj, (float) enz_IITGN_traj);
  
  des_quad_x = 0.0;
  des_quad_y = ay*tt*tt*tt + by*tt*tt + cy*tt + dy;
  des_quad_z = az*tt*tt*tt + bz*tt*tt + cz*tt + dz;
  
  if (IITGN_text_start_time > 92.3101){
      des_quad_x = 0.0;
      des_quad_y = 9.0;
      des_quad_z = 0.0;
  }

  des_quad_x = 0.0;
  des_quad_y = 0.0;
  des_quad_z = 2.0;

}

float ExampleController::min_acc_first_coefficient(float t1, float t2, float st, float en){
  float a = (2*(en - st)) /  ((t1 - t2)*(t1 - t2)*(t1 - t2));
  return a;
}

float ExampleController::min_acc_second_coefficient(float t1, float t2, float st, float en){
  float b = -(3*(t1 + t2)*(en - st)) /  ((t1 - t2)*(t1 - t2)*(t1 - t2));
  return b;
}

float ExampleController::min_acc_third_coefficient(float t1, float t2, float st, float en){
  float c = (6*t1*t2*(en - st)) /  ((t1 - t2)*(t1 - t2)*(t1 - t2));
  return c;
}

float ExampleController::min_acc_fourth_coefficient(float t1, float t2, float st, float en){
  float d = (en*t1*t1*t1 - 3*en*t1*t1*t2 + 3*st*t1*t2*t2 - st*t2*t2*t2) /  ((t1 - t2)*(t1 - t2)*(t1 - t2));
  return d;
}

//}

}  // namespace example_controller

}  // namespace example_controller_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_controller_plugin::example_controller::ExampleController, mrs_uav_managers::Controller)
