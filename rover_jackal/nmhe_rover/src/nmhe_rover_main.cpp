/**
 * @file   nmhe_rover_main.cpp
 * @author Mohit Mehndiratta
 * @date   November 2020
 *
 * @copyright
 * Copyright (C) 2020.
 */

#include <nmhe_rover_main.h>

using namespace Eigen;
using namespace ros;

void estimation_switch_cb(const std_msgs::Bool::ConstPtr& msg)
{
    estimation_switch = msg->data;
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
void wheelvel_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    current_wheelvel.clear();
    current_wheelvel.insert(current_wheelvel.end(), msg->data.begin(), msg->data.end());
}
void nmpc_cmd_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    int i = 0;
    for (std::vector<double>::const_iterator itr = msg->data.begin(); itr != msg->data.end(); ++itr)
    {
        nmpc_cmd.at(i) = *itr;
        i++;
    }
}

void NMHE::publish_est(struct estimation_struct& estimationstruct)
{
    std_msgs::Bool predInit_msg;
    predInit_msg.data = is_prediction_init;
    nmhe_predInit_pub.publish(predInit_msg);

    std_msgs::Float64MultiArray states_vec_msg;
    states_vec_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    states_vec_msg.layout.dim[0].size = estimationstruct.states_vec.size();
    states_vec_msg.layout.dim[0].stride = 1;
    states_vec_msg.layout.dim[0].label = "x, y, psi, v_l, v_r (m, deg, m/sec)";
    states_vec_msg.data.clear();
    states_vec_msg.data.insert(
        states_vec_msg.data.end(), estimationstruct.states_vec.begin(), estimationstruct.states_vec.end());
    nmhe_states_pub.publish(states_vec_msg);

    std_msgs::Float64MultiArray params_vec_msg;
    params_vec_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    params_vec_msg.layout.dim[0].size = estimationstruct.params_vec.size();
    params_vec_msg.layout.dim[0].stride = 1;
    params_vec_msg.layout.dim[0].label = "alpha_l, alpha_r";
    params_vec_msg.data.clear();
    params_vec_msg.data.insert(
        params_vec_msg.data.end(), estimationstruct.params_vec.begin(), estimationstruct.params_vec.end());
    nmhe_params_pub.publish(params_vec_msg);

    std_msgs::Float64 exe_time_cmd;
    exe_time_cmd.data = estimationstruct.exe_time;
    nmhe_exeTime_pub.publish(exe_time_cmd);

    std_msgs::Float64 kkt_tol_cmd;
    kkt_tol_cmd.data = estimationstruct.kkt_tol;
    nmhe_kkt_pub.publish(kkt_tol_cmd);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = estimationstruct.obj_val;
    nmhe_obj_pub.publish(obj_val_msg);
}

//void NMHE::publish_zeroest()
//{
//    std_msgs::Bool predInit_msg;
//    predInit_msg.data = false;
//    nmhe_predInit_pub.publish(predInit_msg);

//      std_msgs::Float64MultiArray states_vec_msg;
//      states_vec_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
//      states_vec_msg.layout.dim[0].size = std::vector<double>();
//      states_vec_msg.layout.dim[0].stride = 1;
//      states_vec_msg.layout.dim[0].label = "x, y, psi, v_l, v_r (m, deg,
//      m/sec)"; states_vec_msg.data.clear();
//      states_vec_msg.data.insert(states_vec_msg.data.end(),
//                                 estimationstruct.states_vec.begin(),
//                                 estimationstruct.states_vec.end());
//      nmhe_states_pub.publish(states_vec_msg);

//    //  std_msgs::Float64MultiArray params_vec_msg;
//    //  params_vec_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
//    //  params_vec_msg.layout.dim[0].size = estimationstruct.params_vec.size();
//    //  params_vec_msg.layout.dim[0].stride = 1;
//    //  params_vec_msg.layout.dim[0].label = "alpha_l, alpha_r";
//    //  params_vec_msg.data.clear();
//    //  params_vec_msg.data.insert(params_vec_msg.data.end(),
//    //                             estimationstruct.params_vec.begin(),
//    //                             estimationstruct.params_vec.end());
//    //  nmhe_params_pub.publish(params_vec_msg);

//    std_msgs::Float64 exe_time_msg;
//    exe_time_msg.data = 0;
//    nmhe_exeTime_pub.publish(exe_time_msg);

//    std_msgs::Float64 kkt_tol_msg;
//    kkt_tol_msg.data = 0;
//    nmhe_kkt_pub.publish(kkt_tol_msg);

//    std_msgs::Float64 obj_val_msg;
//    obj_val_msg.data = 0;
//    nmhe_obj_pub.publish(obj_val_msg);
//}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nmhe_rover");
    ros::NodeHandle nh;

    ros::param::get("mocap_topic_part", mocap_topic_part);

    estimation_switch_sub = nh.subscribe<std_msgs::Bool>("/estimation/switch", 1, estimation_switch_cb);
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(mocap_topic_part + "/pose", 1, pos_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(mocap_topic_part + "/velocity_body", 1, vel_cb);
    wheelvel_sub = nh.subscribe<std_msgs::Float64MultiArray>(mocap_topic_part + "/wheel_velocity_body", 1, wheelvel_cb);
    nmpc_cmd_sub = nh.subscribe<std_msgs::Float64MultiArray>("/nmpc_cmd/wheelAcc", 1, nmpc_cmd_cb);

    // ----------
    // Publishers
    // ----------
    nmhe_predInit_pub = nh.advertise<std_msgs::Bool>("/nmhe/predInit", 1, true);
    nmhe_states_pub = nh.advertise<std_msgs::Float64MultiArray>("/nmhe/states", 1, true);
    nmhe_params_pub = nh.advertise<std_msgs::Float64MultiArray>("/nmhe/params", 1, true);
    nmhe_exeTime_pub = nh.advertise<std_msgs::Float64>("/nmhe/exeTime", 1, true);
    nmhe_kkt_pub = nh.advertise<std_msgs::Float64>("/nmhe/kkt", 1, true);
    nmhe_obj_pub = nh.advertise<std_msgs::Float64>("/nmhe/obj", 1, true);

    nmhe_struct.X0.resize(NMHE_NX);
    nmhe_struct.W.resize(NMHE_NY);
    nmhe_struct.WN.resize(NMHE_NYN);
    nmhe_struct.process_noise_cov.resize(NMHE_NX);
    nmhe_struct.SAC.resize(NMHE_NX);
    nmhe_struct.xAC.resize(NMHE_NX);

    // Roslaunch parameters
    ros::param::get("sampleTime", nmhe_struct.sampleTime);
    ros::param::get("verbose", nmhe_struct.verbose);
    ros::param::get("num_params", nmhe_struct.num_params);

    int x_idx = 0;
    ros::param::get("x_0", nmhe_struct.X0(x_idx++));
    ros::param::get("y_0", nmhe_struct.X0(x_idx++));
    ros::param::get("psi_0", nmhe_struct.X0(x_idx++));
    ros::param::get("vl_0", nmhe_struct.X0(x_idx++));
    ros::param::get("vr_0", nmhe_struct.X0(x_idx++));
    ros::param::get("alphal_0", nmhe_struct.X0(x_idx++));
    ros::param::get("alphar_0", nmhe_struct.X0(x_idx++));

    assert(x_idx == NMHE_NX);

    nmhe_struct.W << 0.0002, 0.0002, 0.0002, 0.0001, 0.0001, 0.0005, 0.0005;
    nmhe_struct.WN << 0.0002, 0.0002, 0.0002, 0.0001, 0.0001;

    nmhe_struct.process_noise_cov << 30, 30, 30, 8, 8, 1, 1;
    nmhe_struct.SAC << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-2, 1e-2;
    nmhe_struct.xAC = nmhe_struct.X0;

    NMHE* nmhe = new NMHE(nmhe_struct);
    ros::Rate rate(1 / nmhe_struct.sampleTime);

    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    current_wheelvel.resize(4);
    nmpc_cmd.resize(NMHE_NU);

    estimator_stop = false;

    for (int i = 0; i < (int)(1 / nmhe_struct.sampleTime); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && !estimator_stop)
    {
        t = ros::Time::now().toSec();

        if (!nmhe->return_estimator_init_value())
        {
            nmhe->nmhe_init(nmhe->nmhe_struct);
            if (!nmhe_struct.verbose && nmhe->return_estimator_init_value())
            {
                std::cout << "**********************************\n";
                std::cout << "NMHE: initialized correctly\n";
                std::cout << "**********************************\n";
            }
        }

        while (ros::ok() && estimation_switch && !estimator_stop)
        {
            if (estimation_switch && print_flag_estimation_switch == 1)
            {
                ROS_INFO("Estimation switch turned on!");
                print_flag_estimation_switch = 0;
            }

            t_est_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_est_loop - (int)(t_est_loop)), (double)(nmhe_struct.sampleTime)) == 0)
                std::cout << "loop time for NMHE: " << t_est_loop << " (sec) \n";

            // Setting up state-feedback [x,y,psi,vx_l,vx_r]
            current_states = {current_pos_att.at(0),
                              current_pos_att.at(1),
                              current_pos_att.at(5),
                              (current_wheelvel.at(0) + current_wheelvel.at(2)) / 2.0,
                              (current_wheelvel.at(1) + current_wheelvel.at(3)) / 2.0};

            nmhe->nmhe_core(nmhe->nmhe_struct, nmhe->nmhe_est_struct, current_states, nmpc_cmd);

            if (nmhe->acado_feedbackStep_fb != 0)
                estimator_stop = true;

            for (int i = 0; i < nmhe->nmhe_struct.acado_NX; i++)
            {
                if (std::isnan(nmhe->nmhe_struct.x[i]) == true)
                {
                    ROS_ERROR_STREAM("Estimator ERROR at time = " << ros::Time::now().toSec() - t << " (sec)");
                    estimator_stop = true;
                    exit(0);
                }
            }

            nmhe->publish_est(nmhe->nmhe_est_struct);

            ros::spinOnce();
            rate.sleep();
        }

        if (!estimation_switch && print_flag_estimation_switch == 0)
        {
            ROS_INFO("Waiting for estimation switch to begin!");
            print_flag_estimation_switch = 1;
        }

        nmhe->publish_est(nmhe->nmhe_est_struct);

        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "Exiting NMHE code!\n";
    std::cout << "---------------------------------\n";
    delete nmhe;
    int i = 0;
    while (ros::ok() && i < (int)(2 / nmhe_struct.sampleTime))
    {
        ros::spinOnce();
        rate.sleep();
        i++;
    }
    return 0;
}
