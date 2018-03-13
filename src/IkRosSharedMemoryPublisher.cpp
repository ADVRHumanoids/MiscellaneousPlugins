#include <MiscellaneousPlugins/IkRosIoPlugin.h>

REGISTER_XBOT_PLUGIN(IkRosSMPub, MiscPlugins::IkRosRtPlugin)

using namespace MiscPlugins;

bool IkRosRtPlugin::init_control_plugin(XBot::Handle::Ptr handle)
{
    _pipe_names = {"xsens_joint", "xsens_stiff", "xsens_damp"};

    _shared_obj_joint = handle->getSharedMemory()->getSharedObject<Eigen::Matrix<double,7,1>>(_pipe_names[0]);
    _shared_obj_stiff = handle->getSharedMemory()->getSharedObject<Eigen::Matrix<double,4,4>>(_pipe_names[1]);
    _shared_obj_damp = handle->getSharedMemory()->getSharedObject<Eigen::Matrix<double,4,4>>(_pipe_names[2]);

    
    _sub_rt_joint = XBot::SubscriberRT<Eigen::Matrix<double,7,1>>(_pipe_names[0]);
    _sub_rt_stiff = XBot::SubscriberRT<Eigen::Matrix<double,4,4>>(_pipe_names[1]);
    _sub_rt_damp = XBot::SubscriberRT<Eigen::Matrix<double,4,4>>(_pipe_names[2]);
   
    _robot = handle->getRobotInterface();
    
    return true;
}

void IkRosRtPlugin::on_start(double time)
{
    
}

void IkRosRtPlugin::control_loop(double time, double period)
{
     Eigen::Matrix<double,7,1> jnt;
     if(_sub_rt_joint.read(jnt)){
        _shared_obj_joint.set(jnt);
     }     
    
     Eigen::Matrix<double,4,4> stiff;
     if(_sub_rt_stiff.read(stiff)){
        _shared_obj_stiff.set(stiff);
     }
     
     Eigen::Matrix<double,4,4> damp;
     if(_sub_rt_damp.read(damp)){
        _shared_obj_damp.set(damp);
     }
}
