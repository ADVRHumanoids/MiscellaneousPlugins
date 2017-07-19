#include <MiscellaneousPlugins/FHRosIoPlugin.h>

REGISTER_XBOT_IO_PLUGIN(FHRosIo, MiscPlugins::FHRosIoPlugin)


bool MiscPlugins::FHRosIoPlugin::init(std::string path_to_config_file)
{
    _topic_names.push_back("walking_sequence");
    
    // number of steps NOTE id is 0
    _pub_footholds_num = XBot::PublisherNRT<int>("footholds_number");  //0
    // Left footholds
    _pub_nrt.push_back(XBot::PublisherNRT<Eigen::Affine3d>("left_footholds")); //1
    // Right footholds
    _pub_nrt.push_back(XBot::PublisherNRT<Eigen::Affine3d>("right_footholds")); //2

    ros::NodeHandle n;

    for(int i = 0; i < _topic_names.size(); i++){
        std::string topic_name = _topic_names[i];
        _sub.push_back( n.subscribe<ADVR_ROS::im_pose_array_msg>(topic_name,
                                                                1,
                                                                &MiscPlugins::FHRosIoPlugin::callback,
                                                                this
                                                                ));
    }
}

void MiscPlugins::FHRosIoPlugin::run()
{
    ros::spinOnce();
}

void MiscPlugins::FHRosIoPlugin::close()
{
    return;
}


void MiscPlugins::FHRosIoPlugin::callback(ADVR_ROS::im_pose_array_msg msg)
{
    Eigen::Affine3d pose;
    
    // send the number of footholds
    _pub_footholds_num.write(msg.im_poses.size());
    
    for(int i = 0; i < msg.im_poses.size(); i++) {
        // transform to affine3d
        tf::poseMsgToEigen(msg.im_poses.at(i).pose_stamped.pose, pose);
        
        if(msg.im_poses.at(i).name == "left") {
            // write to left
            _pub_nrt[0].write(pose);
        }
        
        if(msg.im_poses.at(i).name == "right") {
            // write to right
            _pub_nrt[1].write(pose);
        }
    };

}

