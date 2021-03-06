#include <MiscellaneousPlugins/IkRosIoPlugin.h>
#include <boost/bind.hpp>

REGISTER_XBOT_IO_PLUGIN(IkRosIo, MiscPlugins::IkRosIoPlugin)


bool MiscPlugins::IkRosIoPlugin::init(std::string path_to_config_file, 
                                      XBot::SharedMemory::Ptr shmem)
{
  
    _model = XBot::ModelInterface::getModel(path_to_config_file);
  
  
    _topic_names.push_back("w_T_left_ee");
    _topic_names.push_back("w_T_right_ee");
    _pipe_names = _topic_names;
     
    for(std::string pipe_name : _pipe_names){
        _pub_nrt.push_back(XBot::PublisherNRT<Eigen::Affine3d>(pipe_name));
    }
    
    joint_position.push_back("joint_positions_desired");
    _pub_joint.push_back(XBot::PublisherNRT<MiscPlugins::Vector>(joint_position[0]));

    ros::NodeHandle n;

    for(int i = 0; i < _topic_names.size(); i++){
        std::string topic_name = _topic_names[i];
        _sub.push_back( n.subscribe<geometry_msgs::PoseStamped>(topic_name,
                                                                1,
                                                                boost::bind(&MiscPlugins::IkRosIoPlugin::callback,
                                                                            this,
                                                                            _1, i)
        ) );
    }
    
    _sub_joint.push_back(n.subscribe<sensor_msgs::JointState>(joint_position[0],1,
       boost::bind(&MiscPlugins::IkRosIoPlugin::joint_callback,this,_1, 0)));
   
    
    //grasp
    _grasp_topic_names.push_back("w_grasp_left");
    _grasp_topic_names.push_back("w_grasp_right");
    _grasp_pipe_names = _grasp_topic_names;

    for(std::string pipe_name : _grasp_pipe_names){
        _grasp_pub_nrt.push_back(XBot::PublisherNRT<double>(pipe_name));
    }

    

    
    for(int i = 0; i < _grasp_topic_names.size(); i++){
        std::string topic_name = _grasp_topic_names[i];
        _grasp_sub.push_back( n.subscribe<std_msgs::Float64>(topic_name,
                                                                1,
                                                                boost::bind(&MiscPlugins::IkRosIoPlugin::grasp_callback,
                                                                            this,
                                                                            _1, i)
        ) );
    }
    
    
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
//         std::cout << __PRETTY_FUNCTION__ << std::endl;

}

void MiscPlugins::IkRosIoPlugin::grasp_callback(std_msgs::Float64::ConstPtr msg, int id)
{
  
  
   double grasp = msg->data;
   _grasp_pub_nrt[id].write(grasp);
//         std::cout << __PRETTY_FUNCTION__ << std::endl;

}

void MiscPlugins::IkRosIoPlugin::joint_callback(sensor_msgs::JointState::ConstPtr msg, int id){
  MiscPlugins::Vector joint_position;
  
  joint_position.setZero(_model->getJointNum());
  
  for(unsigned int i = 0; i < msg->position.size(); ++i){
    joint_position[_model->getDofIndex(msg->name[i])] = msg->position[i];
  
    //std::cout<<"callback "<<msg->position[i]<<std::endl;
  }
  
  _pub_joint[id].write(joint_position);
  
}

