/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Giuseppe Rigano
 * email:  giuseppe.rigano@iit.it
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

#ifndef __JOINT_STATE_FORWARDING_H__
#define __JOINT_STATE_FORWARDING_H__

#include <XCM/IOPlugin.h>

#include <XBotInterface/Logger.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <XBotCore/CommandAdvr.h>
#include <sstream>

namespace MiscPlugins {

class JointStateForwarding : public XBot::IOPlugin {

public:

    JointStateForwarding();

    virtual bool init(std::string path_to_config_file, 
                      XBot::SharedMemory::Ptr shmem);
    virtual void run();
    virtual void close();

protected:
  
    
private:

    void Callback(const sensor_msgs::JointState::ConstPtr& msg);


    std::shared_ptr<ros::NodeHandle> n;
    ros::Publisher chatter_pub;
    ros::Subscriber sub;
   

};

}

#endif 
