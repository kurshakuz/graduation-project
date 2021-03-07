#ifndef _TRAJECTORY_ROVER_H
#define _TRAJECTORY_ROVER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>

#define WHEEL_RADIUS_M 0.098
#define WHEEL_WIDTH_M 0.040
#define TRACK_WIDTH_M 0.37559

using namespace ros;

const double deg2rad = M_PI / 180;
struct traj_input_struct_
{
    bool traj_on_switch, est_on_switch;
    int traj_num;
    std::vector<double> xyz_setpoint;
    int heading_setpoint;
    double radius, absvel, rotvel, time_period, del_heading_rate;
};

class TRAJECTORY
{
private:
    int seq_num;
    std::vector<double> pos_xyz, pos_xyz_last, pos_xyz_atTrajStart;
    std::vector<double> vel_xyz, vel_xyz_last;
    std::vector<double> acc_xyz;
    std::vector<double> linVel_LRwheel;
    int vel_fiter_size, acc_fiter_size, headrate_fiter_size;
    std::vector<std::vector<double>> vel_xyz_unfiltered, acc_xyz_unfiltered;
    double heading, heading_last, heading_atTrajStart;
    double rate_heading;
    std::vector<double> rate_heading_unfiltered;

    double dt, t, t_last, t_last_traj, total_traj_time, traj_time;
    double heading_traj_time, t_last_heading;
    int print_flag_ref_traj_started, print_flag_origin, print_flag_setpoint, print_flag_setpointtraj, print_flag_circle,
        print_flag_square, print_flag_fig8;

public:
    // Subscriber msgs
    geometry_msgs::PoseStamped current_pos_msg;
    // Publisher msgs
    geometry_msgs::PoseStamped pos_ref_start_msg;
    geometry_msgs::Vector3 reftrajectory_msg;
    geometry_msgs::Vector3 reftrajectory_vel_msg;
    std_msgs::Float64MultiArray reftrajectory_linvel_LRwheel_msg;
    std_msgs::Float64 refheading_msg;
    std_msgs::Float64 refheading_vel_msg;
    std_msgs::Bool traj_switch_msg, est_switch_msg;
    // Subscriber
    ros::Subscriber pos_sub;
    // Publishers
    ros::Publisher ref_pos_pub;
    ros::Publisher ref_vel_pub;
    ros::Publisher ref_linvel_LRwheel_pub;
    ros::Publisher ref_head_pub;
    ros::Publisher ref_headrate_pub;
    ros::Publisher traj_switch_pub, est_switch_pub;

    tf::Quaternion setpoint_att_quat;

    int print_flag_traj_started, print_flag_est_started;
    bool traj_started_flag, est_started_flag;

    // Member functions
    void setSubscribers(ros::Subscriber _pos_sub);
    void setPublishers(ros::Publisher _ref_pos_pub,
                       ros::Publisher _ref_vel_pub,
                       ros::Publisher _ref_linvel_LRwheel_pub,
                       ros::Publisher _ref_head_pub,
                       ros::Publisher _ref_headrate_pub,
                       ros::Publisher _traj_switch_pub,
                       ros::Publisher _est_switch_pub);
    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publish(struct traj_input_struct_& _traj_input_struct);
    void core(ros::Rate rate, struct traj_input_struct_& _traj_input_struct);
    void set_ref_pos_xyz();
    std::vector<double> get_pos_xyz();
    double get_heading();
    void reset(struct traj_input_struct_& _traj_input_struct);
    TRAJECTORY(double dt, struct traj_input_struct_& _traj_input_struct);
    ~TRAJECTORY();
    int get_sign(double target, double current);
    double mean_filter(double val, std::vector<double>& data_unfiltered);
};
#endif  // TRAJECTORY_ROVER_H
