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

#include <MiscellaneousPlugins/JointStateForwarding.h>

/* Specify that the class MiscPlugins::Homing is a Xbot RT plugin with name "Homing" */
REGISTER_XBOT_IO_PLUGIN_(MiscPlugins::JointStateForwarding);

namespace MiscPlugins {

void JointStateForwarding::Callback(const sensor_msgs::JointState::ConstPtr& msg)
{ 
    XCM::CommandAdvr cmd;
    cmd.position = msg->position;
    cmd.name = msg->name;
    
    chatter_pub.publish(cmd);
}
  
JointStateForwarding::JointStateForwarding()
{

}

bool JointStateForwarding::init(std::string path_to_config_file, 
                                XBot::SharedMemory::Ptr shmem)
{
   

    int argc = 1;
    const char *arg = "ManipulationPlugin";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;

    if(!ros::isInitialized()){
      ros::init(argc, argv, "JointStateForwarding");
    }
    
    n = std::make_shared<ros::NodeHandle>();
      
    sub = n->subscribe<sensor_msgs::JointState>("/joint_states", 1,  boost::bind ( &JointStateForwarding::Callback,
                        this,
                        _1));
    
    chatter_pub = n->advertise<XCM::CommandAdvr>("/xbotcore/cogimon/command", 1);
    
    //ros::Rate loop_rate(10);
    
   
    return true;

}

void JointStateForwarding::run()
{
  
    ros::spinOnce();

    
}

void JointStateForwarding::close()
{
    return;
}



}
