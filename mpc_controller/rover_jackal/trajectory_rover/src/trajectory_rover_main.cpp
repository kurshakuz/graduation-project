/**
 * @file   trajectory_rover_main.cpp
 * @author Mohit Mehndiratta
 * @date   November 2020
 *
 * @copyright
 * Copyright (C) 2020.
 */

#include <trajectory_rover.h>
#include <trajectory_rover/set_trajectory_roverConfig.h>

traj_input_struct_ traj_input_struct;
void dynamicReconfigureCallback(trajectory_rover::set_trajectory_roverConfig& config, uint32_t level)
{
    traj_input_struct.traj_on_switch = config.traj_on;
    traj_input_struct.est_on_switch = config.est_on;

    traj_input_struct.traj_num = config.traj_num;
    traj_input_struct.xyz_setpoint = {config.x_des, config.y_des, 0};
    traj_input_struct.heading_setpoint = config.heading_des;
    traj_input_struct.radius = config.des_radius;
    traj_input_struct.absvel = config.des_velocity;

    traj_input_struct.rotvel = traj_input_struct.absvel / traj_input_struct.radius;
    traj_input_struct.time_period = 2 * M_PI / traj_input_struct.rotvel;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "trajectory_rover");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<trajectory_rover::set_trajectory_roverConfig> server;
    dynamic_reconfigure::Server<trajectory_rover::set_trajectory_roverConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double sampleTime;
    // Roslaunch parameters
    ros::param::get("sampleTime", sampleTime);

    TRAJECTORY* trajectory = new TRAJECTORY(sampleTime, traj_input_struct);

    ros::Rate rate(1 / sampleTime);

    ros::Subscriber pos_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose", 1, &TRAJECTORY::pos_cb, trajectory);

    ros::Publisher ref_pos_pub = nh.advertise<geometry_msgs::Vector3>("/ref_trajectory/pose", 1);
    ros::Publisher ref_vel_pub = nh.advertise<geometry_msgs::Vector3>("/ref_trajectory/velocity", 1);
    ros::Publisher ref_linvel_LRwheel_pub =
        nh.advertise<std_msgs::Float64MultiArray>("/ref_trajectory/linearvel_wheel", 1);
    ros::Publisher ref_heading_pub = nh.advertise<std_msgs::Float64>("/ref_trajectory/heading", 1);
    ros::Publisher ref_headingrate_pub = nh.advertise<std_msgs::Float64>("/ref_trajectory/heading_rate", 1);
    ros::Publisher traj_switch_pub = nh.advertise<std_msgs::Bool>("/ref_trajectory/switch", 1);
    ros::Publisher est_switch_pub = nh.advertise<std_msgs::Bool>("/estimation/switch", 1);

    trajectory->setSubscribers(pos_sub);
    trajectory->setPublishers(ref_pos_pub,
                              ref_vel_pub,
                              ref_linvel_LRwheel_pub,
                              ref_heading_pub,
                              ref_headingrate_pub,
                              traj_switch_pub,
                              est_switch_pub);

    while (ros::ok())
    {
        if (traj_input_struct.traj_on_switch)
        {
            if (!trajectory->traj_started_flag)
            {
                if (trajectory->print_flag_traj_started == 1)
                {
                    ROS_INFO("---------------------------------");
                    ROS_INFO("Trajectory switch turned on!");
                    trajectory->print_flag_traj_started = 0;
                }
                trajectory->traj_started_flag = true;
            }
            // Perform the designated trajectory
            trajectory->core(rate, traj_input_struct);
        }
        else
        {
            if (trajectory->traj_started_flag)
                trajectory->reset(traj_input_struct);

            if (trajectory->print_flag_traj_started == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Default reference position only!");
                ROS_INFO("Position reference x =  %.2f m!", trajectory->pos_ref_start_msg.pose.position.x);
                ROS_INFO("Position reference y =  %.2f m!", trajectory->pos_ref_start_msg.pose.position.y);
                ROS_INFO("Position reference z =  %.2f m!", trajectory->pos_ref_start_msg.pose.position.z);
                ROS_INFO("Heading reference =  %.2f deg!", trajectory->get_heading());
                trajectory->print_flag_traj_started = 1;
            }
        }
        if (traj_input_struct.est_on_switch)
        {
            if (!trajectory->est_started_flag)
            {
                if (trajectory->print_flag_est_started == 1)
                {
                    ROS_INFO("---------------------------------");
                    ROS_INFO("Estimation switch turned on!");
                    trajectory->print_flag_est_started = 0;
                }
                trajectory->est_started_flag = true;
            }
        }
        else if (trajectory->est_started_flag || trajectory->print_flag_est_started == 0)
        {
            ROS_INFO("---------------------------------");
            ROS_INFO("Waiting for estimation switch to begin!");
            trajectory->print_flag_est_started = 1;
            trajectory->est_started_flag = false;
        }

        trajectory->publish(traj_input_struct);
        ros::spinOnce();
        rate.sleep();
    }
    delete trajectory;
    for (int i = 0; i < (int)(1 / sampleTime); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Exiting code!");
    return 0;
}
