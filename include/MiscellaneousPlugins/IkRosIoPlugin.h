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
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotInterface/Utils.h>

namespace MiscPlugins {

class IkRosIoPlugin : public XBot::IOPlugin {

public:

    virtual bool init(std::string path_to_config_file);
    virtual void run();
    virtual void close();

private:

    std::vector<std::string> _topic_names, _pipe_names;
    std::vector<ros::Subscriber> _sub;
    std::vector<XBot::PublisherNRT<Eigen::Affine3d>> _pub_nrt;

    void callback(geometry_msgs::PoseStampedConstPtr msg, int id);



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
    std::vector<XBot::SharedObject<Eigen::Affine3d>> _shared_obj;
    std::vector<XBot::SubscriberRT<Eigen::Affine3d>> _sub_rt;
    
    XBot::Utils::SecondOrderFilter<Eigen::Vector3d> _filter;
    
    Eigen::Affine3d _pose_raw, _pose_ref;


};

}

#endif
