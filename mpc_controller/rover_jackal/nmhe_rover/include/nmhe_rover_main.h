#ifndef _NMHE_ROVER_MAIN_H
#define _NMHE_ROVER_MAIN_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

#include <nmhe_rover.h>

// Subscribers
ros::Subscriber estimation_switch_sub;
ros::Subscriber pos_sub;
ros::Subscriber vel_sub;
ros::Subscriber wheelvel_sub;
ros::Subscriber nmpc_cmd_sub;

// Publishers
ros::Publisher nmhe_predInit_pub;
ros::Publisher nmhe_states_pub;
ros::Publisher nmhe_params_pub;
ros::Publisher nmhe_exeTime_pub;
ros::Publisher nmhe_kkt_pub;
ros::Publisher nmhe_obj_pub;

nmhe_struct_ nmhe_struct;
std::string mocap_topic_part;

bool estimator_stop, estimation_switch;

//double m_in, g_in;

int print_flag_estimation_switch = 0;

double t, t_est_loop;

tf::Quaternion current_att_quat;
tf::Matrix3x3 current_att_mat;
std::vector<double> current_pos_att;
std::vector<double> current_vel_rate;
std::vector<double> current_wheelvel;
std::vector<double> current_states;
std::vector<double> nmpc_cmd;

#endif

