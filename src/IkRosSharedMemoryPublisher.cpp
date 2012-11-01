 #include <MiscellaneousPlugins/IkRosIoPlugin.h>

REGISTER_XBOT_PLUGIN(IkRosSMPub, MiscPlugins::IkRosRtPlugin)

using namespace MiscPlugins;

bool IkRosRtPlugin::init_control_plugin(XBot::Handle::Ptr handle)
{
    _sharedobj_names = {"w_T_left_ee", "w_T_right_ee"};
    _pipe_names = _sharedobj_names;
    
   

    for(std::string shobj_name : _sharedobj_names){
        _shared_obj.push_back(handle->getSharedMemory()->getSharedObject<Eigen::Affine3d>(shobj_name));
    }

    
    
    for(std::string pipe_name : _pipe_names){
        _sub_rt.push_back(XBot::SubscriberRT<Eigen::Affine3d>(pipe_name));
    }
    
    //joints
    _sharedjointposition_name = {"joint_positions_desired"};
    _shared_jointposition.push_back(handle->getSharedMemory()->getSharedObject<MiscPlugins::Vector>(_sharedjointposition_name[0]));
    _sub_rtjointposition.push_back(XBot::SubscriberRT<MiscPlugins::Vector>(_sharedjointposition_name[0]));
    
    
    _shared_stiffness.push_back(handle->getSharedMemory()->getSharedObject<Eigen::Vector3d>("stiffness"));
    _sub_stiffness.push_back(XBot::SubscriberRT<Eigen::Vector3d>("stiffness"));
    
    
    
    //grasp
    
     _sharedobj_grasp_names = {"w_grasp_left", "w_grasp_right"};
     _pipe_grasp_names = _sharedobj_grasp_names;

    for(std::string shobj_name : _sharedobj_grasp_names){
        _grasp_shared_obj.push_back(handle->getSharedMemory()->getSharedObject<double>(shobj_name));
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
            _shared_obj[i].set(pose);
//             std::cout << pose.matrix() << std::endl;
        }
    }
    
    MiscPlugins::Vector jointvect;
    if(_sub_rtjointposition[0].read(jointvect) ){
        _shared_jointposition[0].set(jointvect);
    }
    
    
     Eigen::Vector3d stvect;
    if(_sub_stiffness[0].read(stvect) ){
        _shared_stiffness[0].set(stvect);
    }
    
    //grasp
   for( int i = 0; i < _sharedobj_grasp_names.size(); i++ ){
        double grasp;
        if( _grasp_sub_rt[i].read(grasp) ){
            _grasp_shared_obj[i].set(grasp);
//             std::cout << pose.matrix() << std::endl;
        }
    }
}
