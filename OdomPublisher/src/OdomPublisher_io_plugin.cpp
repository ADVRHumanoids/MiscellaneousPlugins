/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
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

#include <OdomPublisher_io_plugin.h>

/* Specify that the class XBotPlugin::OdomPublisher is a XBot RT plugin with name "OdomPublisher" */
REGISTER_XBOT_IO_PLUGIN(OdomPublisher, XBotPlugin::OdomPublisher)



bool XBotPlugin::OdomPublisher::init(std::string path_to_config_file)
{
    _world_pose_sub.init("world_odom");


    return true;
}

void XBotPlugin::OdomPublisher::run()
{
    XBot::TransformMessage odom_message;

    if(_world_pose_sub.read(odom_message)){

        tf::StampedTransform world_tf;

        tf::transformEigenToTF(odom_message.pose, world_tf);

        world_tf.child_frame_id_ = odom_message.child_frame;
        world_tf.frame_id_ = odom_message.parent_frame;
        world_tf.stamp_ = ros::Time::now();
	
        _tf_broadcaster.sendTransform(world_tf);
    }
}


void XBotPlugin::OdomPublisher::close()
{

}


