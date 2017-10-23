/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore , Giuseppe Rigano
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it, giuseppe.rigano@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef __MISC_PLUGINS_ROS_COMM_IO_PLUGIN_H__
#define __MISC_PLUGINS_ROS_COMM_IO_PLUGIN_H__

#include <XCM/IOPlugin.h>
#include <XCM/XBotControlPlugin.h>
#include <XBotCore-interfaces/XDomainCommunication.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <XBotInterface/ModelInterface.h>

namespace MiscPlugins {

class IkRosIoPlugin : public XBot::IOPlugin {

public:

    virtual bool init(std::string path_to_config_file);
    virtual void run();
    virtual void close();

private:

    std::vector<std::string> _topic_names, _pipe_names, joint_position;
    std::vector<ros::Subscriber> _sub, _sub_joint;
    std::vector<XBot::PublisherNRT<Eigen::Affine3d>> _pub_nrt;
    std::vector<XBot::PublisherNRT<Eigen::VectorXd>> _pub_joint;
    
    std::vector<std::string> _grasp_topic_names, _grasp_pipe_names;
    std::vector<XBot::PublisherNRT<double>> _grasp_pub_nrt;
    std::vector<ros::Subscriber> _grasp_sub;

    void callback(geometry_msgs::PoseStampedConstPtr msg, int id);
    
    void joint_callback(sensor_msgs::JointState::ConstPtr msg, int id);
    
    void grasp_callback(std_msgs::Float64::ConstPtr msg, int id);

    XBot::ModelInterface::Ptr _model;


};


class IkRosRtPlugin : public XBot::XBotControlPlugin {

public:

    virtual bool init_control_plugin(std::string path_to_config_file,
                                     XBot::SharedMemory::Ptr shared_memory,
                                     XBot::RobotInterface::Ptr robot);
    virtual void on_start(double time){}
    virtual void control_loop(double time, double period);
    virtual bool close(){}

private:

    std::vector<std::string> _sharedobj_names, _pipe_names, _sharedjointposition_name;
    
    std::vector<XBot::SharedObject<Eigen::Affine3d>> _shared_obj;
    std::vector<XBot::SharedObject<Eigen::VectorXd>> _shared_jointposition;
    std::vector<XBot::SubscriberRT<Eigen::Affine3d>> _sub_rt;
    std::vector<XBot::SubscriberRT<Eigen::VectorXd>> _sub_rtjointposition;
    
    //_grasp
    std::vector<std::string> _sharedobj_grasp_names, _pipe_grasp_names;
    std::vector<XBot::SharedObject<double>> _grasp_shared_obj;
    std::vector<XBot::SubscriberRT<double>> _grasp_sub_rt;

};

}

#endif