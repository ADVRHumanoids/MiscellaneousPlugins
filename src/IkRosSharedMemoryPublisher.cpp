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
    
    _filter = XBot::Utils::SecondOrderFilter<Eigen::VectorXd>( (2*3.1415) * 10, 1.0, 0.001, Eigen::VectorXd::Zero(robot->getJointNum()));

    _robot = robot;
    
    return true;
}

void IkRosRtPlugin::on_start(double time)
{
    Eigen::Affine3d ree_start_pose;
    _robot->model().getPose(_robot->model().chain("right_arm").getTipLinkName(), 
                            _robot->model().chain("torso").getTipLinkName(), 
                            ree_start_pose); 
    _filter.reset(ree_start_pose.translation());
}

void IkRosRtPlugin::control_loop(double time, double period)
{
    for( int i = 0; i < _sharedobj_names.size(); i++ ){
        if( _sub_rt[i].read(_pose_raw) ){
            
            _pose_ref = _pose_raw;
            _pose_ref.translation() = _filter.process(_pose_raw.translation());
            
            *(_shared_obj[i]) = _pose_ref;
//             std::cout << pose.matrix() << std::endl;
        }
    }
}
