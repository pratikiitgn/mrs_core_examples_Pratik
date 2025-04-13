/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

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
#include <Eigen/Geometry>

// | ----------------- Calling required libraries from gazebo ----------------- |

#include <gazebo_msgs/LinkStates.h>

// | ----------------- Basic math variables ----------------- |
Eigen::Vector3d e1(1.0,0.0,0.0);
Eigen::Vector3d e2(0.0,1.0,0.0);
Eigen::Vector3d e3(0.0,0.0,1.0);

// | ----------------- System parameters     ---------------- |

float mq                = 3.5;      // in kgs mass of the quadcopter
float mp                = 0.1;      // in kgs mass of the payload
float g_acceleration    = 9.81;     // in m/s^2
float PI_value          = 3.1415926535;

// | ----------------- Time related variables ----------------- |
float MRS_text_start_time = 0.0;
double initial_ros_time_custom_controller = 0.0;
float t1_MRS_traj = 0.0;
float t2_MRS_traj = 0.0;

// | ----------------- Position related variables ----------------- |
float sty_MRS_traj = 0.0;
float stz_MRS_traj = 0.0;
float eny_MRS_traj = 0.0;
float enz_MRS_traj = 0.0;

// | ----------------- Quadcopter State ----------------- |
float quad_x      = 0.0;
float quad_y      = 0.0;
float quad_z      = 0.0;

float quad_x_dot  = 0.0;
float quad_y_dot  = 0.0;
float quad_z_dot  = 0.0;

Eigen::Vector3d   pos_of_quad(0.0,0.0,0.0);
Eigen::Vector3d   vel_of_quad(0.0,0.0,0.0);

// | ----------------- Desired quadcopter State ----------------- |

float des_quad_x      = 0.0;
float des_quad_y      = 0.0;
float des_quad_z      = 2.0;

float des_quad_x_dot  = 0.0;
float des_quad_y_dot  = 0.0;
float des_quad_z_dot  = 0.0;

float des_quad_x_dot_dot  = 0.0;
float des_quad_y_dot_dot  = 0.0;
float des_quad_z_dot_dot  = 0.0;

Eigen::Vector3d   des_pos_of_quad(0.0,0.0,0.0);
Eigen::Vector3d   des_vel_of_quad(0.0,0.0,0.0);
Eigen::Vector3d   des_acc_of_quad(0.0,0.0,0.0);

Eigen::Vector3d b_1_des(1.0,0.0,0.0);
Eigen::Vector3d b_2_des(0.0,1.0,0.0);
Eigen::Vector3d b_3_des(0.0,0.0,1.0);

float desired_yaw_angle = 0.0;

Eigen::Vector3d b_1_c(1.0,0.0,0.0);

Eigen::Matrix3d R_quad_attitude;
Eigen::Vector3d des_rpy;
// | ----------------- Cable attitude State ----------------- |

Eigen::Vector3d     q (0.0,0.0,-1.0);
Eigen::Vector3d q_dot (0.0,0.0, 0.0);

// | ----------------- Desired cable attitude State ----------------- |

Eigen::Vector3d     q_d (0.0,0.0,-1.0);
Eigen::Vector3d q_d_dot (0.0,0.0, 0.0);

// | ----------------- Payload position State ----------------- |

Eigen::Vector3d pos_of_payload(0.0,0.0,0.0);

// | ----------------- Error definition ----------------- |
Eigen::Vector3d     e_x_q     (0.0,0.0,0.0);
Eigen::Vector3d     e_x_q_dot (0.0,0.0,0.0);

Eigen::Vector3d     e_q       (0.0,0.0,0.0);
Eigen::Vector3d     e_q_dot   (0.0,0.0,0.0);

// | ----------------- Custom Gains ----------------- |

float kx_1        = 0.0;
float kx_1_dot    = 0.0;
float kx_2        = 0.0;
float kx_2_dot    = 0.0;
float kx_3        = 0.0;
float kx_3_dot    = 0.0;

Eigen::Array3d kx(0.0,0.0,0.0);
Eigen::Array3d kx_dot(0.0,0.0,0.0);

// | ----------------- High level commands ----------------- |
float des_roll_angle        = 0.0;
float des_pitch_angle       = 0.0;
float des_yaw_angle         = 0.0;
double desired_thrust_force = 0.2;

// | ----------------- Thrust force ----------------- |
Eigen::Vector3d u_control_input(0.0,0.0,0.0);

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
  float distance_bt_two_pts(Eigen::Vector3d A, Eigen::Vector3d B);
  float min_acc_first_coefficient(float t1, float t2, float st, float en);
  float min_acc_second_coefficient(float t1, float t2, float st, float en);
  float min_acc_third_coefficient(float t1, float t2, float st, float en);
  float min_acc_fourth_coefficient(float t1, float t2, float st, float en);
  float clipping_angle(float max_value, float current_angle);
  Eigen::Vector3d Matrix_vector_mul(Eigen::Matrix3d R, Eigen::Vector3d v);
  float clipping_net_thrust_force(float max_value, float current_thrust);
  Eigen::Vector3d Rotation_matrix_to_Euler_angle(Eigen::Matrix3d R);

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

  // | ---------------------- ROS subscribers --------------------- |
  // ros::Subscriber sub_gazebo_pendulum_;
  ros::Subscriber sub_gazebo_pendulum_;
  void            callback_gazebo_pendulum(const gazebo_msgs::LinkStates& msg);
  
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
  param_loader.loadParam("kx_1_value",      kx_1);
  param_loader.loadParam("kx_1_dot_value",  kx_1_dot);
  param_loader.loadParam("kx_2_value",      kx_2);
  param_loader.loadParam("kx_2_dot_value",  kx_2_dot);
  param_loader.loadParam("kx_3_value",      kx_3);
  param_loader.loadParam("kx_3_dot_value",  kx_3_dot);

  // // | -------- initialize a subscriber -------- |
  sub_gazebo_pendulum_ = nh_.subscribe("/gazebo/link_states", 1, &ExampleController::callback_gazebo_pendulum, this, ros::TransportHints().tcpNoDelay());

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

    // ROS_INFO_STREAM_THROTTLE(1.0, "[ExampleController]: UAV inertia is: " << detailed_model_params.inertia);
  }

////////////////////////////////////////////////
//////         Custom controller starts
////////////////////////////////////////////////

  // | -------------- prepare the control reference ------------- |

  geometry_msgs::PoseStamped position_reference;

  position_reference.header           = tracker_command.header;
  position_reference.pose.position    = tracker_command.position;
  position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);

  // | ---------------- Custom PD Controller for altitude control --------------- |

  MRS_text_start_time = ros::Time::now().toSec() - initial_ros_time_custom_controller;
  // ROS_INFO_STREAM_THROTTLE(1, "[ExampleController]: Current Time: " << MRS_text_start_time);

  ////////////////////////////////////////////////////////////////////////////////////////
  //     Trajectory for tracking MRS Text
  Eigen::Vector3d Z(0,0,2);
  Eigen::Vector3d A(0,0,6);
  Eigen::Vector3d B(0,-2,4);
  Eigen::Vector3d C(0,-4,6);
  Eigen::Vector3d D(0,-4,2);

  Eigen::Vector3d E(0,-6,2);
  Eigen::Vector3d FF(0,-6,6);
  Eigen::Vector3d G(0,-9,6);
  Eigen::Vector3d H(0,-9,4);
  Eigen::Vector3d I(0,-6,4);
  Eigen::Vector3d J(0,-9,2);

  Eigen::Vector3d K(0,-11,2);
  Eigen::Vector3d L(0,-14,2);
  Eigen::Vector3d M(0,-14,4);
  Eigen::Vector3d N(0,-11,4);
  Eigen::Vector3d O(0,-11,6);
  Eigen::Vector3d P(0,-14,6);

  Eigen::Vector3d Q(0,-14,6);
  Eigen::Vector3d R(0,-14,6);
  Eigen::Vector3d S(0,-14,6);
  Eigen::Vector3d Y(0,-14,6);

  float Pos_array[21][3] = {{0,0,2},
                           {0,0,6},
                           {0,-2,4},
                           {0,-4,6},
                           {0,-4,2},
                           {0,-6,2},
                           {0,-6,6},
                           {0,-9,6},
                           {0,-9,4},
                           {0,-6,4},
                           {0,-9,2},
                           {0,-11,2},
                           {0,-14,2},
                           {0,-14,4},
                           {0,-11,4},
                           {0,-11,6},
                           {0,-14,6},
                           {0,-14,6},
                           {0,-14,6},
                           {0,-14,6},
                           {0,-14,6},};

  float V_max = 0.5;

  float  tZ = 20;
  float  tA = tZ + (distance_bt_two_pts(A , Z)/V_max);
  float  tB = tA + (distance_bt_two_pts(B , A)/V_max);
  float  tC = tB + (distance_bt_two_pts(C , B)/V_max);
  float  tD = tC + (distance_bt_two_pts(D , C)/V_max);
  float  tE = tD + (distance_bt_two_pts(E , D)/V_max);
  float  tF = tE + (distance_bt_two_pts(FF , E)/V_max);
  float  tG = tF + (distance_bt_two_pts(G , FF)/V_max);
  float  tH = tG + (distance_bt_two_pts(H , G)/V_max);
  float  tI = tH + (distance_bt_two_pts(I , H)/V_max);
  float  tJ = tI + (distance_bt_two_pts(J , I)/V_max);
  float  tK = tJ + (distance_bt_two_pts(K , J)/V_max);
  float  tL = tK + (distance_bt_two_pts(L , K)/V_max);
  float  tM = tL + (distance_bt_two_pts(M , L)/V_max);
  float  tN = tM + (distance_bt_two_pts(N , M)/V_max);
  float  tO = tN + (distance_bt_two_pts(O , N)/V_max);
  float  tP = tO + (distance_bt_two_pts(P , O)/V_max);
  float  tQ = tP + (distance_bt_two_pts(Q , P)/V_max);
  float  tR = tQ + (distance_bt_two_pts(R , Q)/V_max);
  float  tS = tR + (distance_bt_two_pts(S , R)/V_max);
  float  tY = tS + (distance_bt_two_pts(Y , S)/V_max);

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

  float tt = MRS_text_start_time;
  // ROS_INFO_STREAM_THROTTLE(1, "[ExampleController]: Current Time: " << tt);

  for (int i = 0; i < 21; i++) {
  
        if (MRS_text_start_time >= t_array[i][0]){
          t1_MRS_traj  = t_array[i][0];
          t2_MRS_traj  = t_array[i+1][0];
          sty_MRS_traj = Pos_array[i][1];
          stz_MRS_traj = Pos_array[i][2];
          eny_MRS_traj = Pos_array[i+1][1];
          enz_MRS_traj = Pos_array[i+1][2];
        }
  }
  
  float ay =  min_acc_first_coefficient((float) t1_MRS_traj, (float) t2_MRS_traj, (float) sty_MRS_traj, (float) eny_MRS_traj);
  float by = min_acc_second_coefficient((float) t1_MRS_traj, (float) t2_MRS_traj, (float) sty_MRS_traj, (float) eny_MRS_traj);
  float cy =  min_acc_third_coefficient((float) t1_MRS_traj, (float) t2_MRS_traj, (float) sty_MRS_traj, (float) eny_MRS_traj);
  float dy = min_acc_fourth_coefficient((float) t1_MRS_traj, (float) t2_MRS_traj, (float) sty_MRS_traj, (float) eny_MRS_traj);

  float az =  min_acc_first_coefficient((float) t1_MRS_traj, (float) t2_MRS_traj, (float) stz_MRS_traj, (float) enz_MRS_traj);
  float bz = min_acc_second_coefficient((float) t1_MRS_traj, (float) t2_MRS_traj, (float) stz_MRS_traj, (float) enz_MRS_traj);
  float cz =  min_acc_third_coefficient((float) t1_MRS_traj, (float) t2_MRS_traj, (float) stz_MRS_traj, (float) enz_MRS_traj);
  float dz = min_acc_fourth_coefficient((float) t1_MRS_traj, (float) t2_MRS_traj, (float) stz_MRS_traj, (float) enz_MRS_traj);

  des_quad_x      = 0.0;
  des_quad_y      = ay*tt*tt*tt + by*tt*tt + cy*tt + dy;
  des_quad_z      = az*tt*tt*tt + bz*tt*tt + cz*tt + dz;

  des_quad_x_dot  = 0;
  des_quad_y_dot  = 3*ay*tt*tt + 2*by*tt + cy;
  des_quad_z_dot  = 3*az*tt*tt + 2*bz*tt + cz;

  if (MRS_text_start_time > tY){
      des_quad_x = P[0];
      des_quad_y = P[1];
      des_quad_z = P[2];

      des_quad_x_dot  = 0;
      des_quad_y_dot  = 0;
      des_quad_z_dot  = 0;
  }

  if (MRS_text_start_time < tZ)
  {
    des_quad_x = Z[0];
    des_quad_y = Z[1];
    des_quad_z = Z[2];

    des_quad_x_dot  = 0;
    des_quad_y_dot  = 0;
    des_quad_z_dot  = 0;
  }

  // des_quad_x = 0;
  // des_quad_y = 0;
  // des_quad_z = 3;

  des_pos_of_quad[0] = des_quad_x;
  des_pos_of_quad[1] = des_quad_y;
  des_pos_of_quad[2] = des_quad_z;

  des_vel_of_quad[0] = des_quad_x_dot;
  des_vel_of_quad[1] = des_quad_y_dot;
  des_vel_of_quad[2] = des_quad_z_dot;

  des_vel_of_quad[0] = 0.0;
  des_vel_of_quad[1] = 0.0;
  des_vel_of_quad[2] = 0.0;

  des_acc_of_quad[0] = des_quad_x_dot_dot;
  des_acc_of_quad[1] = des_quad_y_dot_dot;
  des_acc_of_quad[2] = des_quad_z_dot_dot;

  ////////////////////////////////////////////////////////////////////////////////////////

  // ROS_INFO_STREAM_THROTTLE(0.2, "xd:" << des_quad_x);
  // ROS_INFO_STREAM_THROTTLE(0.2, "yd:" << des_quad_y);
  // ROS_INFO_STREAM_THROTTLE(0.2, "zd:" << des_quad_z);

  // Desired values of the position
  // des_quad_x = 1.0;
  // des_quad_y = 1.0;
  // des_quad_z = 2.0;

  // Getting positional state of the drone
  pos_of_quad[0] = uav_state.pose.position.x;
  pos_of_quad[1] = uav_state.pose.position.y;
  pos_of_quad[2] = uav_state.pose.position.z;

  vel_of_quad[0] = uav_state.velocity.linear.x;
  vel_of_quad[1] = uav_state.velocity.linear.y;
  vel_of_quad[2] = uav_state.velocity.linear.z;

  // | ---------------- Get the gains values --------------- |

  kx      << kx_1 ,     kx_2 ,    kx_3;
  kx_dot  << kx_1_dot , kx_2_dot, kx_3_dot;

  // | ---------------- Error computation --------------- |

  e_x_q       = des_pos_of_quad - pos_of_quad;
  e_x_q_dot   = des_vel_of_quad - vel_of_quad;

  e_q         = q.cross(q.cross(q_d));
  e_q_dot     = q_dot - (q_d.cross(q_d_dot)).cross(q);

  // ROS_INFO_STREAM_THROTTLE(0.5, "Just Debugging" << e_q);

  // | ---------------- prepare the control output --------------- |

  Eigen::Vector3d feed_forward      = (mq + mp) * g_acceleration * e3;
  Eigen::Vector3d position_feedback = kx * e_x_q.array();;
  Eigen::Vector3d velocity_feedback = kx_dot * e_x_q_dot.array();;

  u_control_input     = position_feedback + velocity_feedback + feed_forward;

  if (u_control_input(2) < 0) {
    ROS_WARN_THROTTLE(1.0, "[ExampleController]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", u_control_input(2));
    u_control_input << 0, 0, 1;
  }

  // Desired quadcopter attitude
  b_3_des[0]          = u_control_input[0] / u_control_input.norm();
  b_3_des[1]          = u_control_input[1] / u_control_input.norm();
  b_3_des[2]          = u_control_input[2] / u_control_input.norm();

  b_1_c[0]            = cosf(desired_yaw_angle/180.0*PI_value);
  b_1_c[1]            = sinf(desired_yaw_angle/180.0*PI_value);
  b_1_c[2]            = 0.0;

  b_2_des             = b_3_des.cross(b_1_c);
  b_1_des             = b_2_des.cross(b_3_des);

  // R_quad_attitude = common::so3transform(b_3_des, b_1_c, drs_params.rotation_type == 1);

  R_quad_attitude <<  b_1_des[0], b_2_des[0], b_3_des[0],
                      b_1_des[1], b_2_des[1], b_3_des[1],
                      b_1_des[2], b_2_des[2], b_3_des[2];

  desired_thrust_force     = u_control_input.dot(R_quad_attitude.col(2));

  double throttle          = 0.0;

  if (desired_thrust_force >= 0) {
    throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, desired_thrust_force);
  } else {
    ROS_WARN_THROTTLE(1.0, "[ExampleController]: just so you know, the desired throttle force is negative (%.2f)", desired_thrust_force);
  }

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;

  attitude_cmd.stamp       = ros::Time::now();
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(R_quad_attitude);
  attitude_cmd.throttle    = throttle;

  // All the back printing codes
  ROS_INFO_STREAM_THROTTLE(0.5, "[ExampleController]: u_control_input: " << "[" << u_control_input[0] << "," << u_control_input[1] <<"," << u_control_input[2] << "]");
  ROS_INFO_STREAM_THROTTLE(0.5, "[ExampleController]: e_x_q: " << "[" << e_x_q[0] << "," << e_x_q[1] <<"," << e_x_q[2] << "]");

  // des_rpy = Rotation_matrix_to_Euler_angle(R_quad_attitude);
  // ROS_INFO_STREAM_THROTTLE(0.3, "[ExampleController]: desired attitude rpy: " << des_rpy);

  // | ---------------- Thrust saturation --------------- |
  // As we have considered t650 frame, the maximum thrust thatcan be produced
  // by each motor is 24.9598
  // float max_thrust_force;
  // max_thrust_force    = 24.9598 * 4.0;

  // desired_thrust_force        = clipping_net_thrust_force(max_thrust_force, desired_thrust_force );

  // ROS_INFO_STREAM_THROTTLE(0.3, "[ExampleController]: Net Thrust Force: " << desired_thrust_force);

  // des_roll_angle      = 0.0;
  // des_pitch_angle     = 0.0;
  // des_yaw_angle       = 0.0;
  // des_pitch_angle     = des_rpy[0];
  // des_roll_angle      = -des_rpy[1];
  // des_yaw_angle       = des_rpy[2];

  // des_pitch_angle     = clipping_angle(0.78, des_pitch_angle);
  // des_roll_angle      = clipping_angle(0.78, des_roll_angle);

  // ROS_INFO_STREAM_THROTTLE(1, "[ExampleController]: des_roll_angle: " << des_roll_angle);

  // drs_params.roll     = des_roll_angle;
  // drs_params.pitch    = des_pitch_angle;
  // drs_params.yaw      = des_yaw_angle;
  // drs_params.force    = desired_thrust_force;

  // mrs_msgs::HwApiAttitudeCmd attitude_cmd;
  // attitude_cmd.orientation = mrs_lib::AttitudeConverter(drs_params.roll, drs_params.pitch, drs_params.yaw);
  // attitude_cmd.throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model,
  //                                                                            common_handlers_->getMass() * common_handlers_->g + drs_params.force);
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

void ExampleController::callback_gazebo_pendulum(const gazebo_msgs::LinkStates& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }
  // | --------------  ------------- |
  
  // ROS_DEBUG("[ExampleController]: Pendulum Pos data (%.2f s)!", msg.pose.Point.position.x);
  
  // ROS_INFO_STREAM_THROTTLE(1.0, "[ExampleController]: Pendulum data: " << msg.pose[12]);
  // ROS_INFO_STREAM_THROTTLE(1.0, "[ExampleController]: Pendulum data: " << msg.pose[12].position.x);


}

void ExampleController::callbackDrs(example_controller_plugin::example_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[ExampleController]: dynamic reconfigure params updated");
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

float ExampleController::clipping_angle(float max_value, float current_angle){
  if (current_angle > max_value) // 0.35 rad means 20 deg
  {
    current_angle = max_value;
  }

  if (current_angle < -max_value) // 0.35 rad means 20 deg
  {
    current_angle = -max_value;
  }
  return current_angle;
}

float ExampleController::clipping_net_thrust_force(float max_value, float current_thrust){
  if (current_thrust > max_value) // 24.959870582 * 4
  {
    current_thrust = max_value;
  }

  if (current_thrust < 0.0) // 24.959870582 * 4
  {
    current_thrust = 0.0;
  }
  return current_thrust;
}

float ExampleController::distance_bt_two_pts(Eigen::Vector3d A, Eigen::Vector3d B){

  float norm__  = (A[0] - B[0]) * (A[0] - B[0]) + (A[1] - B[1]) * (A[1] - B[1]) + (A[2] - B[2]) * (A[2] - B[2]);
  norm__ = sqrtf(norm__ );
  return norm__;

}


Eigen::Vector3d ExampleController::Matrix_vector_mul(Eigen::Matrix3d R, Eigen::Vector3d v){
  Eigen::Vector3d mul_vector (R(0,0)*v[0] + R(0,1)*v[1] + R(0,2)*v[2], R(1,0)*v[0] + R(1,1)*v[1] + R(1,2)*v[2],  R(2,0)*v[0] + R(2,1)*v[1] + R(2,2)*v[2]);
  return mul_vector;
}

Eigen::Vector3d ExampleController::Rotation_matrix_to_Euler_angle(Eigen::Matrix3d R){

  float sy = sqrtf( R(0,0) * R(0,0) +  R(1,0) * R(1,0) );
  float ph_des = atan2f(R(2,1) , R(2,2));
  float th_des = atan2f(-R(2,0), sy);
  float ps_des = atan2f(R(1,0), R(0,0));

  Eigen::Vector3d des_roll_pitch_yaw(ph_des, th_des, ps_des);
  return des_roll_pitch_yaw;
}

//}

}  // namespace example_controller

}  // namespace example_controller_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_controller_plugin::example_controller::ExampleController, mrs_uav_managers::Controller)
