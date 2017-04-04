#include <MiscellaneousPlugins/IkRosIoPlugin.h>

REGISTER_XBOT_IO_PLUGIN(IkRosIo, MiscPlugins::IkRosIoPlugin)


bool MiscPlugins::IkRosIoPlugin::init(std::string path_to_config_file)
{
    _topic_names.push_back("w_T_left_ee");
    _topic_names.push_back("w_T_right_ee");
    _pipe_names = _topic_names;
}

void MiscPlugins::IkRosIoPlugin::run()
{
    ros::spinOnce();
}

void MiscPlugins::IkRosIoPlugin::close()
{
    return;
}


void MiscPlugins::IkRosIoPlugin::callback(geometry_msgs::PoseStampedConstPtr msg, int id)
{
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(msg->pose, pose);
    _pub_nrt[id].write(pose);
}

