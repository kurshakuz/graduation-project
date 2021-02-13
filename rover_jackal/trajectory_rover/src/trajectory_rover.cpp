/**
 * @file   trajectory_rover.cpp
 * @author Mohit Mehndiratta
 * @date   Novermber 2020
 *
 * @copyright
 * Copyright (C) 2020.
 */

#include <trajectory_rover.h>

TRAJECTORY::TRAJECTORY(double _dt, struct traj_input_struct_& _traj_input_struct)
{
    seq_num = 1;
    pos_xyz = {0, 0, 0};
    pos_xyz_last = {0, 0, 0};
    pos_xyz_atTrajStart = {0, 0, 0};
    vel_xyz = {0, 0, 0};
    vel_xyz_last = {0, 0, 0};
    acc_xyz = {0, 0, 0};
    linVel_LRwheel = {0.0, 0.0};
    vel_fiter_size = 30;
    acc_fiter_size = 40;
    headrate_fiter_size = 40;
    vel_xyz_unfiltered.resize(vel_xyz.size(), std::vector<double>(vel_fiter_size, 0.0));
    acc_xyz_unfiltered.resize(acc_xyz.size(), std::vector<double>(acc_fiter_size, 0.0));
    heading = 0;
    heading_last = 0;
    heading_atTrajStart = 0;
    rate_heading = 0;
    rate_heading_unfiltered.resize(headrate_fiter_size, 0.0);

    dt = _dt;
    t = 0;
    t_last = 0;
    t_last_traj = 0;
    total_traj_time = 0;
    traj_time = 0;
    print_flag_origin = 1;
    print_flag_setpoint = 1;
    print_flag_setpointtraj = 1;
    print_flag_circle = 1;
    print_flag_square = 1;
    print_flag_fig8 = 1;

    pos_ref_start_msg.pose.position.x = 0;
    pos_ref_start_msg.pose.position.y = 0;
    pos_ref_start_msg.pose.position.z = 0;

    setpoint_att_quat.setRPY(0, 0, 0);

    traj_switch_msg.data = _traj_input_struct.traj_on_switch;
    est_switch_msg.data = _traj_input_struct.est_on_switch;

    reftrajectory_linvel_LRwheel_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    reftrajectory_linvel_LRwheel_msg.layout.dim[0].size = linVel_LRwheel.size();
    reftrajectory_linvel_LRwheel_msg.layout.dim[0].stride = 1;
    reftrajectory_linvel_LRwheel_msg.layout.dim[0].label = "vx_l, vy_r [m/s]";

    print_flag_traj_started = 0;
    print_flag_est_started = 0;
    print_flag_ref_traj_started = 1;
    traj_started_flag = false;
    est_started_flag = false;

    std::cout << "Constructor of the class TRAJECTORY is created\n";
}

TRAJECTORY::~TRAJECTORY()
{
    std::cout << "*********************************\n";
    std::cout << "Destructor of the class TRAJECTORY\n";
    std::cout << "*********************************\n";
}

void TRAJECTORY::reset(struct traj_input_struct_& _traj_input_struct)
{
    pos_ref_start_msg.pose.position.x = 0;
    pos_ref_start_msg.pose.position.y = 0;
    pos_ref_start_msg.pose.position.z = 0;
    pos_xyz[0] = pos_ref_start_msg.pose.position.x;
    pos_xyz[1] = pos_ref_start_msg.pose.position.y;
    pos_xyz[2] = pos_ref_start_msg.pose.position.z;
    pos_xyz_last = pos_xyz;
    vel_xyz = {0, 0, 0};
    vel_xyz_last = vel_xyz;
    acc_xyz = {0, 0, 0};
    linVel_LRwheel = {0.0, 0.0};
    vel_xyz_unfiltered.resize(vel_xyz.size(), std::vector<double>(vel_fiter_size, 0.0));
    acc_xyz_unfiltered.resize(acc_xyz.size(), std::vector<double>(acc_fiter_size, 0.0));
    heading = _traj_input_struct.heading_setpoint;
    heading_last = heading;
    rate_heading = 0;
    rate_heading_unfiltered.resize(headrate_fiter_size, 0.0);
    print_flag_origin = 1;
    print_flag_setpoint = 1;
    print_flag_setpointtraj = 1;
    print_flag_circle = 1;
    print_flag_square = 1;
    print_flag_fig8 = 1;
    print_flag_ref_traj_started = 1;
    //    print_flag_est_started = 1;
    traj_started_flag = false;
    est_started_flag = false;
}

int TRAJECTORY::get_sign(double target, double current)
{
    if (target - current > 0)
        return 1;
    else
        return -1;
}

double TRAJECTORY::mean_filter(double val, std::vector<double>& data_unfiltered)
{
    // erase from the front!
    data_unfiltered.erase(data_unfiltered.begin());
    data_unfiltered.push_back(val);

    double new_val = 0.0;
    for (int i = 0; i < data_unfiltered.size(); ++i)
        new_val += data_unfiltered[i];
    new_val = new_val / (data_unfiltered.size());

    return new_val;
}

void TRAJECTORY::publish(struct traj_input_struct_& _traj_input_struct)
{
    traj_switch_msg.data = _traj_input_struct.traj_on_switch;
    est_switch_msg.data = _traj_input_struct.est_on_switch;
    for (int i = 0; i < 3; ++i)
    {
        vel_xyz[i] = mean_filter((pos_xyz[i] - pos_xyz_last[i]) / dt, vel_xyz_unfiltered[i]);
        acc_xyz[i] = mean_filter((vel_xyz[i] - vel_xyz_last[i]) / dt, acc_xyz_unfiltered[i]);
    }

    // hack to remove discontinuity in rate calculation at +-pi
    if (heading > -M_PI + 0.04 && heading < M_PI - 0.04)
    {
        rate_heading = mean_filter((heading - heading_last) / dt, rate_heading_unfiltered);
    }
    else
    {
        rate_heading = mean_filter(rate_heading, rate_heading_unfiltered);
    }

        // To avoid nan vaues when stationary
        //if (vel_xyz[0] == 0 && vel_xyz[1] == 0)
        //{
        //    rate_heading = mean_filter(0.0, rate_heading_unfiltered);
        //}
        //else
        //{
        //    rate_heading = (vel_xyz[0] * acc_xyz[1] - vel_xyz[1] * acc_xyz[0]) / (pow(vel_xyz[0], 2) + pow(vel_xyz[1], 2));
            //        rate_heading =
            //            mean_filter((vel_xyz[0] * acc_xyz[1] - vel_xyz[1] * acc_xyz[0]) / (pow(vel_xyz[0], 2) + pow(vel_xyz[1], 2)),
            //                        rate_heading_unfiltered);
        //}
        //std::cout << "rate_heading_temp = " << (heading - heading_last) / dt << "\n";

    pos_xyz_last = pos_xyz;
    vel_xyz_last = vel_xyz;
    heading_last = heading;

    traj_switch_pub.publish(traj_switch_msg);
    est_switch_pub.publish(est_switch_msg);

    reftrajectory_msg.x = pos_xyz[0];
    reftrajectory_msg.y = pos_xyz[1];
    reftrajectory_msg.z = pos_xyz[2];
    ref_pos_pub.publish(reftrajectory_msg);

    reftrajectory_vel_msg.x = std::abs(vel_xyz[0]) <= _traj_input_struct.absvel
                                  ? vel_xyz[0]
                                  : get_sign(vel_xyz[0], 0) * _traj_input_struct.absvel;
    reftrajectory_vel_msg.y = std::abs(vel_xyz[1]) <= _traj_input_struct.absvel
                                  ? vel_xyz[1]
                                  : get_sign(vel_xyz[1], 0) * _traj_input_struct.absvel;
    reftrajectory_vel_msg.z = std::abs(vel_xyz[2]) <= _traj_input_struct.absvel
                                  ? vel_xyz[2]
                                  : get_sign(vel_xyz[2], 0) * _traj_input_struct.absvel;
    ref_vel_pub.publish(reftrajectory_vel_msg);

    double abs_vel = 0;
    if (_traj_input_struct.traj_num > 2)
    {
        abs_vel =
            sqrt(reftrajectory_vel_msg.x * reftrajectory_vel_msg.x + reftrajectory_vel_msg.y * reftrajectory_vel_msg.y);
    }
    else
    {
        abs_vel = reftrajectory_vel_msg.x;
    }
    linVel_LRwheel[0] = abs_vel - rate_heading * TRACK_WIDTH_M / 2;
    linVel_LRwheel[1] = abs_vel + rate_heading * TRACK_WIDTH_M / 2;

    reftrajectory_linvel_LRwheel_msg.data.clear();
    reftrajectory_linvel_LRwheel_msg.data.insert(
        reftrajectory_linvel_LRwheel_msg.data.end(), linVel_LRwheel.begin(), linVel_LRwheel.end());
    ref_linvel_LRwheel_pub.publish(reftrajectory_linvel_LRwheel_msg);

    refheading_msg.data = heading;
    ref_head_pub.publish(refheading_msg);

    refheading_vel_msg.data = rate_heading;
    ref_headrate_pub.publish(refheading_vel_msg);
}

void TRAJECTORY::setSubscribers(ros::Subscriber _pos_sub)
{
    pos_sub = _pos_sub;
}

void TRAJECTORY::setPublishers(ros::Publisher _ref_pos_pub,
                               ros::Publisher _ref_vel_pub,
                               ros::Publisher _ref_linvel_LRwheel_pub,
                               ros::Publisher _ref_head_pub,
                               ros::Publisher _ref_headrate_pub,
                               ros::Publisher _traj_switch_pub,
                               ros::Publisher _est_switch_pub)
{
    ref_pos_pub = _ref_pos_pub;
    ref_vel_pub = _ref_vel_pub;
    ref_linvel_LRwheel_pub = _ref_linvel_LRwheel_pub;
    ref_head_pub = _ref_head_pub;
    ref_headrate_pub = _ref_headrate_pub;
    traj_switch_pub = _traj_switch_pub;
    est_switch_pub = _est_switch_pub;
}

void TRAJECTORY::core(ros::Rate rate, struct traj_input_struct_& _traj_input_struct)
{
    t_last = ros::Time::now().toSec();
    t = ros::Time::now().toSec();
    total_traj_time = t - t_last;
    traj_time = t - t_last_traj;
    heading_traj_time = t - t_last_heading;

    int atan_jump = 0;

    while (ros::ok() && _traj_input_struct.traj_on_switch)
    {
        if (print_flag_ref_traj_started == 1)
        {
            ROS_INFO("---------------------------------");
            ROS_INFO("Reference trajectory started!");
            print_flag_ref_traj_started = 0;
        }

        t = ros::Time::now().toSec();
        total_traj_time = t - t_last;
        traj_time = t - t_last_traj;
        heading_traj_time = t - t_last_heading;

        if (_traj_input_struct.traj_num == 0)
        {
            if (print_flag_origin == 1)
            {
                ROS_INFO("--------Hover at origin selected!--------");
                print_flag_origin = 0;
                print_flag_setpoint = 1;
                print_flag_setpointtraj = 1;
                print_flag_circle = 1;
                print_flag_fig8 = 1;
                print_flag_square = 1;

                t_last_traj = ros::Time::now().toSec();
            }
            pos_xyz[0] = pos_ref_start_msg.pose.position.x;
            pos_xyz[1] = pos_ref_start_msg.pose.position.y;
            heading = deg2rad * _traj_input_struct.heading_setpoint;
        }

        if (_traj_input_struct.traj_num == 1)
        {
            if (print_flag_setpoint == 1)
            {
                ROS_INFO("--------Setpoint selected!--------");
                print_flag_origin = 1;
                print_flag_setpoint = 0;
                print_flag_setpointtraj = 1;
                print_flag_circle = 1;
                print_flag_fig8 = 1;
                print_flag_square = 1;

                pos_xyz_atTrajStart = pos_xyz;
                heading_atTrajStart = heading;
                traj_time = 0;
                t_last_traj = ros::Time::now().toSec();
            }

            for (int i = 0; i < 2; i++)
            {
                pos_xyz[i] = pos_xyz_atTrajStart[i] + _traj_input_struct.xyz_setpoint[i];
            }
        }

        if (_traj_input_struct.traj_num == 2)
        {
            if (print_flag_setpointtraj == 1)
            {
                ROS_INFO("--------Setpoint trajectory selected!--------");
                print_flag_origin = 1;
                print_flag_setpoint = 1;
                print_flag_setpointtraj = 0;
                print_flag_circle = 1;
                print_flag_fig8 = 1;
                print_flag_square = 1;

                pos_xyz_atTrajStart = pos_xyz;
                heading_atTrajStart = heading;
                traj_time = 0;
                t_last_traj = ros::Time::now().toSec();
            }
            double target_val = 0;
            for (int i = 0; i < 2; i++)
            {
                target_val = pos_xyz_atTrajStart[i] + _traj_input_struct.xyz_setpoint[i];
                pos_xyz[i] = std::abs(pos_xyz[i] - target_val) > 0.01
                                 ? pos_xyz[i] + get_sign(target_val, pos_xyz[i]) * _traj_input_struct.absvel * dt
                                 : target_val;
            }
        }

        if (_traj_input_struct.traj_num == 3)
        {
            if (print_flag_circle == 1)
            {
                ROS_INFO("--------Circle trajectory selected!--------");
                ROS_INFO("rotational velocity = %.2f rad/s", _traj_input_struct.rotvel);
                ROS_INFO("time period = %.2f s", _traj_input_struct.time_period);
                print_flag_origin = 1;
                print_flag_setpoint = 1;
                print_flag_setpointtraj = 1;
                print_flag_circle = 0;
                print_flag_fig8 = 1;
                print_flag_square = 1;

                pos_xyz_atTrajStart[0] = pos_ref_start_msg.pose.position.x + _traj_input_struct.xyz_setpoint[0];
                pos_xyz_atTrajStart[1] = pos_ref_start_msg.pose.position.y + _traj_input_struct.xyz_setpoint[1];
                heading_atTrajStart = heading;
                traj_time = 0;
                t_last_traj = ros::Time::now().toSec();
            }
            pos_xyz[0] =
                pos_xyz_atTrajStart[0] + _traj_input_struct.radius * sin(_traj_input_struct.rotvel * traj_time);

            //            if (traj_time < 0.25 * _traj_input_struct.time_period)
            //            {
            //                pos_xyz[1] = pos_xyz_atTrajStart[1];
            //                //                heading = heading_atTrajStart - sin(_traj_input_struct.rotvel * traj_time);
            //            }
            //            else
            //            {
            pos_xyz[1] = -_traj_input_struct.radius + pos_xyz_atTrajStart[1] +
                         _traj_input_struct.radius * cos(_traj_input_struct.rotvel * traj_time);

            //                heading = heading_atTrajStart + atan2(vel_xyz[1], vel_xyz[0]);
            //            }
        }

        if (_traj_input_struct.traj_num == 4)
        {
            if (print_flag_fig8 == 1)
            {
                ROS_INFO("--------Figure-8 trajectory selected!--------");
                ROS_INFO("rotational velocity = %.2f rad/s", _traj_input_struct.rotvel);
                ROS_INFO("time period = %.2f s", _traj_input_struct.time_period);
                print_flag_origin = 1;
                print_flag_setpoint = 1;
                print_flag_setpointtraj = 1;
                print_flag_circle = 1;
                print_flag_fig8 = 0;
                print_flag_square = 1;

                pos_xyz_atTrajStart[0] = pos_ref_start_msg.pose.position.x + _traj_input_struct.xyz_setpoint[0];
                pos_xyz_atTrajStart[1] = pos_ref_start_msg.pose.position.y + _traj_input_struct.xyz_setpoint[1];
                heading_atTrajStart = heading;
                traj_time = 0;
                t_last_traj = ros::Time::now().toSec();
            }
            pos_xyz[0] = pos_xyz_atTrajStart[0] -
                         _traj_input_struct.radius * cos(_traj_input_struct.rotvel * traj_time + M_PI / 2); //-_traj_input_struct.radius + 
            pos_xyz[1] = pos_xyz_atTrajStart[1] +
                         (_traj_input_struct.radius / 2) * sin(2 * _traj_input_struct.rotvel * traj_time);

            //            heading = heading_atTrajStart + atan2(vel_xyz[1], vel_xyz[0]);
        }

        if (_traj_input_struct.traj_num == 5)
        {
            if (print_flag_square == 1)
            {
                ROS_INFO("--------Square selected!--------");
                print_flag_origin = 1;
                print_flag_setpoint = 1;
                print_flag_setpointtraj = 1;
                print_flag_circle = 1;
                print_flag_fig8 = 1;
                print_flag_square = 0;

                pos_xyz_atTrajStart[0] = pos_ref_start_msg.pose.position.x + _traj_input_struct.xyz_setpoint[0];
                pos_xyz_atTrajStart[1] = pos_ref_start_msg.pose.position.y + _traj_input_struct.xyz_setpoint[1];
                pos_xyz_last = pos_xyz_atTrajStart;
                heading_atTrajStart = heading;

                traj_time = 0;
                t_last_traj = ros::Time::now().toSec();
            }
            if (std::abs(sin(_traj_input_struct.rotvel * traj_time) - 1) < 0.001 ||
                std::abs(sin(_traj_input_struct.rotvel * traj_time) + 1) < 0.001)
                pos_xyz[0] = pos_xyz_atTrajStart[0] +
                             _traj_input_struct.radius * (sin(_traj_input_struct.rotvel * traj_time) < 0
                                                              ? std::floor(sin(_traj_input_struct.rotvel * traj_time))
                                                              : std::ceil(sin(_traj_input_struct.rotvel * traj_time)));
            else
                pos_xyz[0] = pos_xyz_last[0];
            if (traj_time < 0.25 * _traj_input_struct.time_period)
            {
                pos_xyz[1] = pos_xyz_atTrajStart[1];
            }
            else
            {
                if (std::abs(cos(_traj_input_struct.rotvel * traj_time) - 1) < 0.001 ||
                    std::abs(cos(_traj_input_struct.rotvel * traj_time) + 1) < 0.001)
                    pos_xyz[1] =
                        pos_xyz_atTrajStart[1] +
                        _traj_input_struct.radius * (cos(_traj_input_struct.rotvel * traj_time) < 0
                                                         ? std::floor(cos(_traj_input_struct.rotvel * traj_time))
                                                         : std::ceil(cos(_traj_input_struct.rotvel * traj_time)));
                else
                    pos_xyz[1] = pos_xyz_last[1];
            }
        }

        // Heading computation
        if (_traj_input_struct.traj_num > 2)
        {
            heading = heading_atTrajStart + atan2(pos_xyz[1] - pos_xyz_last[1], pos_xyz[0] - pos_xyz_last[0]);
        }

        else
        {
            heading = deg2rad * _traj_input_struct.heading_setpoint;
        }

        publish(_traj_input_struct);
        ros::spinOnce();
        rate.sleep();
    }

    print_flag_origin = 1;
    print_flag_setpoint = 1;
    print_flag_setpointtraj = 1;
    print_flag_circle = 1;
    print_flag_fig8 = 1;
    print_flag_square = 1;
}

void TRAJECTORY::set_ref_pos_xyz()
{
    pos_xyz[0] = pos_ref_start_msg.pose.position.x;
    pos_xyz[1] = pos_ref_start_msg.pose.position.y;
    pos_xyz[2] = pos_ref_start_msg.pose.position.z;
}

std::vector<double> TRAJECTORY::get_pos_xyz()
{
    return pos_xyz;
}

double TRAJECTORY::get_heading()
{
    return heading;
}

// Callback functions
void TRAJECTORY::pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos_msg = *msg;
}
