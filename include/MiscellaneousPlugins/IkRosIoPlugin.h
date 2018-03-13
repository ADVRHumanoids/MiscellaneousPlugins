/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
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
#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotInterface/Utils.h>

namespace MiscPlugins {

class IkRosIoPlugin : public XBot::IOPlugin {

public:

    virtual bool init(std::string path_to_config_file, XBot::SharedMemory::Ptr shmem);
    virtual void run();
    virtual void close();

private:

    std::vector<std::string> _topic_names, _pipe_names;
    std::vector<ros::Subscriber> _sub;
    XBot::PublisherNRT<Eigen::Matrix<double,7,1>> _pub_nrt_joint;
    XBot::PublisherNRT<Eigen::Matrix<double,4,4>> _pub_nrt_stiff;
    XBot::PublisherNRT<Eigen::Matrix<double,4,4>> _pub_nrt_damp;

    void callback(std_msgs::Float64MultiArrayConstPtr msg, int id);

};


class IkRosRtPlugin : public XBot::XBotControlPlugin {

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);
    virtual void on_start(double time);
    virtual void control_loop(double time, double period);
    virtual bool close(){}

private:
    
    XBot::RobotInterface::Ptr _robot;

    std::vector<std::string> _sharedobj_names, _pipe_names;
    XBot::SharedObject<Eigen::Matrix<double,7,1>> _shared_obj_joint;
    XBot::SharedObject<Eigen::Matrix<double,4,4>> _shared_obj_stiff;
    XBot::SharedObject<Eigen::Matrix<double,4,4>> _shared_obj_damp;
    
    XBot::SubscriberRT<Eigen::Matrix<double,7,1>> _sub_rt_joint;
    XBot::SubscriberRT<Eigen::Matrix<double,4,4>> _sub_rt_stiff;
    XBot::SubscriberRT<Eigen::Matrix<double,4,4>> _sub_rt_damp;


};

}

#endif
