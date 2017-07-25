#include <MiscellaneousPlugins/IkRosIoPlugin.h>

REGISTER_XBOT_PLUGIN(IkRosSMPub, MiscPlugins::IkRosRtPlugin)

using namespace MiscPlugins;

bool IkRosRtPlugin::init_control_plugin(std::string path_to_config_file, XBot::SharedMemory::Ptr shared_memory, XBot::RobotInterface::Ptr robot)
{
    _sharedobj_names = {"w_T_left_ee", "w_T_right_ee"};
    _pipe_names = _sharedobj_names;

    for(std::string shobj_name : _sharedobj_names){
        _shared_obj.push_back(shared_memory->advertise<Eigen::Affine3d>(shobj_name));
    }

    for(std::string pipe_name : _pipe_names){
        _sub_rt.push_back(XBot::SubscriberRT<Eigen::Affine3d>(pipe_name));
    }
    
    
    //grasp
    
     _sharedobj_grasp_names = {"w_grasp_left", "w_grasp_right"};
     _pipe_grasp_names = _sharedobj_grasp_names;

    for(std::string shobj_name : _sharedobj_grasp_names){
        _grasp_shared_obj.push_back(shared_memory->advertise<double>(shobj_name));
    }

    for(std::string pipe_name : _pipe_grasp_names){
        _grasp_sub_rt.push_back(XBot::SubscriberRT<double>(pipe_name));
    }
    
    

    return true;
}

void IkRosRtPlugin::control_loop(double time, double period)
{
    for( int i = 0; i < _sharedobj_names.size(); i++ ){
        Eigen::Affine3d pose;
        if( _sub_rt[i].read(pose) ){
            *(_shared_obj[i]) = pose;
//             std::cout << pose.matrix() << std::endl;
        }
    }
    
    
    //grasp
   for( int i = 0; i < _sharedobj_grasp_names.size(); i++ ){
        double grasp;
        if( _grasp_sub_rt[i].read(grasp) ){
            *(_grasp_shared_obj[i]) = grasp;
//             std::cout << pose.matrix() << std::endl;
        }
    }
}
