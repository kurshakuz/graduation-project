/**
 * @file   nmpc_pc_learning_rover_main.cpp
 * @author Mohit Mehndiratta
 * @date   December 2020
 *
 * @copyright
 * Copyright (C) 2020.
 */

#include <nmpc_pc_learning_rover_main.h>

using namespace Eigen;
using namespace ros;

void ref_trajectory_switch_cb(const std_msgs::Bool::ConstPtr& msg)
{
    ref_trajectory_switch = msg->data;
}
void ref_position_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void ref_linvel_LRwheel_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ref_linvel_LRwheel.clear();
    ref_linvel_LRwheel.insert(ref_linvel_LRwheel.end(), msg->data.begin(), msg->data.end());
}
void ref_heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
    ref_heading = msg->data;
}
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
    current_att_mat.setRotation(current_att_quat);
    current_att_mat.getRPY(roll, pitch, yaw);

    current_pos_att = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, roll, pitch, yaw};
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_rate = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}
void wheelLinVel_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    current_wheelLinVel.clear();
    current_wheelLinVel.insert(current_wheelLinVel.end(), msg->data.begin(), msg->data.end());
}
void params_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    params_struct.predInit = msg->data;
    if (nmpc_struct.verbose && params_struct.predInit && params_struct.print_predInit == 1)
    {
        std::cout << "Prediction initialized for estimates! \n";
        params_struct.print_predInit = 0;
    }
}
void params_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_params_estimates && params_struct.predInit)
    {
        params_struct.data.clear();
        params_struct.data.insert(params_struct.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        params_struct.data = nmpc_struct.init_params;
}

void NMPC_PC::publish_cmd(struct command_struct& commandstruct)
{
    //    commandstruct.control_vec = {-0.5,-0.5};
    std_msgs::Float64 wheel_vel_msg;
    wheel_vel_msg.data = commandstruct.control_vel_vec.at(0);
    nmpc_cmd_wheelLinVel_1_pub.publish(wheel_vel_msg);
    nmpc_cmd_wheelLinVel_3_pub.publish(wheel_vel_msg);

    wheel_vel_msg.data = commandstruct.control_vel_vec.at(1);
    nmpc_cmd_wheelLinVel_2_pub.publish(wheel_vel_msg);
    nmpc_cmd_wheelLinVel_4_pub.publish(wheel_vel_msg);

    std_msgs::Float64MultiArray acc_vec_msg;
    acc_vec_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    acc_vec_msg.layout.dim[0].size = commandstruct.control_acc_vec.size();
    acc_vec_msg.layout.dim[0].stride = 1;
    acc_vec_msg.layout.dim[0].label = "acc_l, acc_r";
    acc_vec_msg.data.clear();
    acc_vec_msg.data.insert(
        acc_vec_msg.data.end(), commandstruct.control_acc_vec.begin(), commandstruct.control_acc_vec.end());
    nmpc_cmd_wheelLinAcc_pub.publish(acc_vec_msg);

    std_msgs::Float64 exe_time_msg;
    exe_time_msg.data = commandstruct.exe_time;
    nmpc_cmd_exeTime_pub.publish(exe_time_msg);

    std_msgs::Float64 kkt_tol_msg;
    kkt_tol_msg.data = commandstruct.kkt_tol;
    nmpc_cmd_kkt_pub.publish(kkt_tol_msg);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = commandstruct.obj_val;
    nmpc_cmd_obj_pub.publish(obj_val_msg);
}

void NMPC_PC::publish_zerocmd()
{
    std_msgs::Float64 wheel_vel_msg;
    wheel_vel_msg.data = 0;
    nmpc_cmd_wheelLinVel_1_pub.publish(wheel_vel_msg);
    nmpc_cmd_wheelLinVel_2_pub.publish(wheel_vel_msg);
    nmpc_cmd_wheelLinVel_3_pub.publish(wheel_vel_msg);
    nmpc_cmd_wheelLinVel_4_pub.publish(wheel_vel_msg);

    std::vector<double> zero_acc_vec = {0.0, 0.0};
    std_msgs::Float64MultiArray acc_vec_msg;
    acc_vec_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    acc_vec_msg.layout.dim[0].size = zero_acc_vec.size();
    acc_vec_msg.layout.dim[0].stride = 1;
    acc_vec_msg.layout.dim[0].label = "acc_l, acc_r";
    acc_vec_msg.data.clear();
    acc_vec_msg.data.insert(acc_vec_msg.data.end(), zero_acc_vec.begin(), zero_acc_vec.end());
    nmpc_cmd_wheelLinAcc_pub.publish(acc_vec_msg);

    std_msgs::Float64 exe_time_msg;
    exe_time_msg.data = 0;
    nmpc_cmd_exeTime_pub.publish(exe_time_msg);

    std_msgs::Float64 kkt_tol_msg;
    kkt_tol_msg.data = 0;
    nmpc_cmd_kkt_pub.publish(kkt_tol_msg);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = 0;
    nmpc_cmd_obj_pub.publish(obj_val_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nmpc_pc_rover");
    ros::NodeHandle nh;

    ros::param::get("mocap_topic_part", mocap_topic_part);
    ros::param::get("params_predInit_topic", params_predInit_topic);
    ros::param::get("params_data_topic", params_data_topic);

    ref_trajectory_switch_sub = nh.subscribe<std_msgs::Bool>("/ref_trajectory/switch", 1, ref_trajectory_switch_cb);
    ref_position_sub = nh.subscribe<geometry_msgs::Vector3>("/ref_trajectory/pose", 1, ref_position_cb);
    ref_linvel_LRwheel_sub =
        nh.subscribe<std_msgs::Float64MultiArray>("/ref_trajectory/linearvel_wheel", 1, ref_linvel_LRwheel_cb);
    ref_heading_sub = nh.subscribe<std_msgs::Float64>("/ref_trajectory/heading", 1, ref_heading_cb);
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(mocap_topic_part + "/pose", 1, pos_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(mocap_topic_part + "/velocity_body", 1, vel_cb);
    wheelLinVel_sub =
        nh.subscribe<std_msgs::Float64MultiArray>(mocap_topic_part + "/wheel_velocity_body", 1, wheelLinVel_cb);
    params_predInit_sub = nh.subscribe<std_msgs::Bool>(params_predInit_topic, 1, params_predInit_cb);
    params_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(params_data_topic, 1, params_data_cb);

    // ----------
    // Publishers
    // ----------
    nmpc_cmd_wheelLinVel_1_pub = nh.advertise<std_msgs::Float64>("/joint1_velocity_controller/command", 1, true);
    nmpc_cmd_wheelLinVel_2_pub = nh.advertise<std_msgs::Float64>("/joint2_velocity_controller/command", 1, true);
    nmpc_cmd_wheelLinVel_3_pub = nh.advertise<std_msgs::Float64>("/joint3_velocity_controller/command", 1, true);
    nmpc_cmd_wheelLinVel_4_pub = nh.advertise<std_msgs::Float64>("/joint4_velocity_controller/command", 1, true);
    nmpc_cmd_wheelLinAcc_pub = nh.advertise<std_msgs::Float64MultiArray>("/nmpc_cmd/wheelLinAcc", 1, true);
    nmpc_cmd_exeTime_pub = nh.advertise<std_msgs::Float64>("/nmpc_cmd/exeTime", 1, true);
    nmpc_cmd_kkt_pub = nh.advertise<std_msgs::Float64>("/nmpc_cmd/kkt", 1, true);
    nmpc_cmd_obj_pub = nh.advertise<std_msgs::Float64>("/nmpc_cmd/obj", 1, true);

    nmpc_struct.init_params.resize(NMPC_NOD);
    nmpc_struct.U_ref.resize(NMPC_NU);
    nmpc_struct.W.resize(NMPC_NY);

    // To set all control references to be zero!
    nmpc_struct.U_ref.setZero();

    // Roslaunch parameters
    ros::param::get("sampleTime", nmpc_struct.sampleTime);
    ros::param::get("verbose", nmpc_struct.verbose);
    ros::param::get("use_params_estimates", use_params_estimates);
    ros::param::get("W_Wn_factor", nmpc_struct.W_Wn_factor);

    int p_idx = 0;
    ros::param::get("init_param_1", nmpc_struct.init_params.at(p_idx++));
    ros::param::get("init_param_2", nmpc_struct.init_params.at(p_idx++));
    assert(p_idx == NMPC_NOD);

    int u_idx = 0;
    ros::param::get("acc1_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("acc2_ref", nmpc_struct.U_ref(u_idx++));
    assert(u_idx == NMPC_NU);

    int w_idx = 0;
    ros::param::get("W_x", nmpc_struct.W(w_idx++));
    ros::param::get("W_y", nmpc_struct.W(w_idx++));
    ros::param::get("W_psi", nmpc_struct.W(w_idx++));
    ros::param::get("W_vl", nmpc_struct.W(w_idx++));
    ros::param::get("W_vr", nmpc_struct.W(w_idx++));
    ros::param::get("W_acc1", nmpc_struct.W(w_idx++));
    ros::param::get("W_acc2", nmpc_struct.W(w_idx++));
    assert(w_idx == NMPC_NY);

    NMPC_PC* nmpc_pc = new NMPC_PC(nmpc_struct);
    ros::Rate rate(1 / nmpc_struct.sampleTime);

    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    current_wheelLinVel.resize(4);
    params_struct.data = nmpc_struct.init_params;

    ref_position << 0, 0, 0;
    ref_linvel_LRwheel.resize(2);
    ref_heading = 0;

    control_stop = false;

    for (int i = 0; i < (int)(1 / nmpc_struct.sampleTime); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && !control_stop)
    {
        t = ros::Time::now().toSec();

        if (!nmpc_pc->return_control_init_value())
        {
            nmpc_pc->nmpc_init(nmpc_pc->nmpc_struct);
            if (nmpc_struct.verbose && nmpc_pc->return_control_init_value())
            {
                std::cout << "***********************************\n";
                std::cout << "NMPC: initialized correctly\n";
                std::cout << "***********************************\n";
            }
        }

        while (ros::ok() && ref_trajectory_switch && !control_stop)
        {
            if (ref_trajectory_switch && print_flag_traj_started == 1)
            {
                ROS_INFO("Trajectory switch turned on!");
                print_flag_traj_started = 0;
            }

            t_pc_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_pc_loop - (int)(t_pc_loop)), (double)(nmpc_struct.sampleTime)) == 0)
                std::cout << "loop time for NMPC: " << t_pc_loop << " (sec)"
                          << "\n";

            // Setting up state-feedback [x,y,psi,vx_l,vx_r]
            current_states = {current_pos_att.at(0),
                              current_pos_att.at(1),
                              current_pos_att.at(5),
                              (current_wheelLinVel.at(0) + current_wheelLinVel.at(2)) / 2.0,
                              (current_wheelLinVel.at(1) + current_wheelLinVel.at(3)) / 2.0};

            ref_trajectory = {
                ref_position(0), ref_position(1), ref_heading, ref_linvel_LRwheel.at(0), ref_linvel_LRwheel.at(1)};

            nmpc_pc->nmpc_core(
                nmpc_pc->nmpc_struct, nmpc_pc->nmpc_cmd_struct, ref_trajectory, params_struct.data, current_states);

            if (nmpc_pc->acado_feedbackStep_fb != 0)
                control_stop = true;

            for (int i = 0; i < nmpc_pc->nmpc_struct.acado_NU; i++)
            {
                if (std::isnan(nmpc_pc->nmpc_struct.u[i]) == true)
                {
                    ROS_ERROR_STREAM("Controller ERROR at time = " << ros::Time::now().toSec() - t << " (sec)");
                    control_stop = true;
                    exit(0);
                }
            }

            nmpc_pc->publish_cmd(nmpc_pc->nmpc_cmd_struct);

            ros::spinOnce();
            rate.sleep();
        }

        if (!ref_trajectory_switch && print_flag_traj_started == 0)
        {
            ROS_INFO("Waiting for trajectory switch to begin!");
            print_flag_traj_started = 1;
        }

        nmpc_pc->publish_zerocmd();

        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "Exiting NMPC code!\n";
    std::cout << "---------------------------------\n";
    delete nmpc_pc;
    int i = 0;
    while (ros::ok() && i < (int)(2 / nmpc_struct.sampleTime))
    {
        ros::spinOnce();
        rate.sleep();
        i++;
    }
    return 0;
}
