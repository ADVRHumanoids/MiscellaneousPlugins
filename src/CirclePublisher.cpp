#include <MiscellaneousPlugins/CirclePublisher.h>

REGISTER_XBOT_PLUGIN(CirclePublisher, MiscPlugins::CirclePublisher)

using namespace MiscPlugins;

bool CirclePublisher::init_control_plugin(XBot::Handle::Ptr handle)
{
    _sharedobj_names = {"w_T_left_ee", "w_T_right_ee"};
    _pipe_names = _sharedobj_names;

    for(std::string shobj_name : _sharedobj_names){
        _shared_obj.push_back(handle->getSharedMemory()->getSharedObject<Eigen::Affine3d>(shobj_name));
    }

    for(std::string pipe_name : _pipe_names){
        _sub_rt.push_back(XBot::SubscriberRT<Eigen::Affine3d>(pipe_name));
    }
    
    _filter = XBot::Utils::SecondOrderFilter<Eigen::Vector3d>( (2*3.1415) * 10, 1.0, 0.001, Eigen::Vector3d::Zero());

    _robot = handle->getRobotInterface();
    
    return true;
}

void CirclePublisher::on_start(double time)
{
    Eigen::Affine3d ree_start_pose;
    
    _robot->model().getPose(_robot->model().chain("right_arm").getTipLinkName(), 
                            _robot->model().chain("torso").getTipLinkName(), 
                            ree_start_pose); 
    _filter.reset(ree_start_pose.translation());
}

void CirclePublisher::control_loop(double time, double period)
{
    for( int i : {1} ){
            
            Eigen::Vector3d position_raw = Eigen::Vector3d(0.55, -0.5, 0.1);
            
            position_raw.y() += 0.1*std::sin(time);
            position_raw.z() += 0.1*std::cos(time);
            
            _pose_ref.translation() = _filter.process(position_raw);
            
            // NOTE not caring about orientation: put a fixed one
            _pose_ref.linear() << 0, 0, -1,
                                  0, 1,  0,
                                  1, 0,  0;
            
            _shared_obj[i].set(_pose_ref);
//             std::cout << pose.matrix() << std::endl;
    }
}
