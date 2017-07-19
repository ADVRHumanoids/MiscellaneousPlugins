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

#ifndef __MISC_PLUGINS_FH_ROS_COMM_IO_PLUGIN_H__
#define __MISC_PLUGINS_FH_ROS_COMM_IO_PLUGIN_H__

#include <XCM/IOPlugin.h>
#include <XCM/XBotControlPlugin.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#include <ros/ros.h>
#include <ADVR_ROS/im_pose_array_msg.h>
#include <eigen_conversions/eigen_msg.h>

namespace MiscPlugins {

class FHRosIoPlugin : public XBot::IOPlugin {

public:

    virtual bool init(std::string path_to_config_file);
    virtual void run();
    virtual void close();

private:

    std::vector<std::string> _topic_names, _pipe_names;
    std::vector<ros::Subscriber> _sub;
    XBot::PublisherNRT<int> _pub_footholds_num;
    std::vector<XBot::PublisherNRT<Eigen::Affine3d>> _pub_nrt;

    void callback(ADVR_ROS::im_pose_array_msg msg);



};

}

#endif //__MISC_PLUGINS_FH_ROS_COMM_IO_PLUGIN_H__