#include <MiscellaneousPlugins/CirclePublisher.h>

REGISTER_XBOT_PLUGIN(CirclePublisher, MiscPlugins::CirclePublisher)

using namespace MiscPlugins;

bool CirclePublisher::init_control_plugin(std::string path_to_config_file, XBot::SharedMemory::Ptr shared_memory, XBot::RobotInterface::Ptr robot)
{
    _sharedobj_names = {"w_T_left_ee", "w_T_right_ee"};
    _pipe_names = _sharedobj_names;

    for(std::string shobj_name : _sharedobj_names){
        _shared_obj.push_back(shared_memory->advertise<Eigen::Affine3d>(shobj_name));
    }

    for(std::string pipe_name : _pipe_names){
        _sub_rt.push_back(XBot::SubscriberRT<Eigen::Affine3d>(pipe_name));
    }
    
    _filter = XBot::Utils::SecondOrderFilter<Eigen::Vector3d>( (2*3.1415) * 10, 1.0, 0.001, Eigen::Vector3d::Zero());

    _robot = robot;
    
    return true;
}

void CirclePublisher::on_start(double time)
{
    Eigen::Affine3d ree_start_pose;
    
    _robot->model().getPose(_robot->model().chain("arm1").getTipLinkName(), 
                            _robot->model().chain("arm1").getBaseLinkName(), 
                            ree_start_pose); 
    _filter.reset(ree_start_pose.translation());
}

void CirclePublisher::control_loop(double time, double period)
{
    for( int i : {1} ){
            
            Eigen::Vector3d position_raw = Eigen::Vector3d(0.20, -0.5, 0.1);
            
            position_raw.y() += 0.1*std::sin(time);
            position_raw.z() += 0.1*std::cos(time);
            
            _pose_ref.translation() = _filter.process(position_raw);
            
            // NOTE not caring about orientation: put a fixed one
            _pose_ref.linear() << 1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1;
            
            *(_shared_obj[i]) = _pose_ref;
//             std::cout << pose.matrix() << std::endl;
    }
}
