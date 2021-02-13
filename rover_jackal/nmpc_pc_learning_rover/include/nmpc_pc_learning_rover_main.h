#ifndef _NMPC_PC_LEARNING_ROVER_MAIN_H
#define _NMPC_PC_LEARNING_ROVER_MAIN_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

#include <nmpc_pc_learning_rover.h>

// Subscribers
ros::Subscriber ref_trajectory_switch_sub;
ros::Subscriber ref_position_sub;
ros::Subscriber ref_velocity_sub;
ros::Subscriber ref_linvel_LRwheel_sub;
ros::Subscriber ref_heading_sub;
ros::Subscriber pos_sub;
ros::Subscriber vel_sub;
ros::Subscriber wheelLinVel_sub;
ros::Subscriber params_predInit_sub;
ros::Subscriber params_data_sub;

// Publishers
ros::Publisher nmpc_cmd_wheelLinVel_1_pub;
ros::Publisher nmpc_cmd_wheelLinVel_2_pub;
ros::Publisher nmpc_cmd_wheelLinVel_3_pub;
ros::Publisher nmpc_cmd_wheelLinVel_4_pub;
ros::Publisher nmpc_cmd_wheelLinAcc_pub;
ros::Publisher nmpc_cmd_exeTime_pub;
ros::Publisher nmpc_cmd_kkt_pub;
ros::Publisher nmpc_cmd_obj_pub;

nmpc_struct_ nmpc_struct;
std::string mocap_topic_part, params_predInit_topic, params_data_topic;

bool control_stop;
bool use_params_estimates;

Eigen::VectorXd Uref_in(NMPC_NU);
Eigen::VectorXd W_in(NMPC_NY);

int print_flag_traj_started = 0, print_flag_traj_finished = 0;

bool ref_trajectory_switch;
Eigen::Vector3d ref_position, ref_velocity; 
std::vector<double> ref_linvel_LRwheel;
double ref_heading;
std::vector<double> ref_trajectory;
double t, t_pc_loop;

tf::Quaternion current_att_quat;
tf::Matrix3x3 current_att_mat;
std::vector<double> current_pos_att;
std::vector<double> current_vel_rate;
std::vector<double> current_wheelLinVel;
std::vector<double> current_states;

struct _params_struct
{
	bool predInit;
	int print_predInit = 1;
	std::vector<double> data;
}params_struct;

#endif

