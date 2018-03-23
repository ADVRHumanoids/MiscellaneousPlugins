#include <MiscellaneousPlugins/Grasp.h>


REGISTER_XBOT_PLUGIN(Grasp, MiscPlugins::Grasp)

namespace MiscPlugins {

bool Grasp::init_control_plugin(XBot::Handle::Ptr handle)
{
    _logger = XBot::MatLogger::getLogger("/tmp/Grasp_logger");

    _robot = handle->getRobotInterface();
    
    _left_ref = handle->getSharedMemory()->getSharedObject<double>("w_grasp_left");
    
    _right_ref = handle->getSharedMemory()->getSharedObject<double>("w_grasp_right");
    
    // hands
    std::map<std::string, XBot::Hand::Ptr> hands = _robot->getHand();
    // RHand
    auto r_hand_it = hands.find("r_wrist_joint");
    if(r_hand_it != hands.end()) {
        _RHand = r_hand_it->second;
    }
    // LHand
    auto l_hand_it = hands.find("l_wrist_joint");
    if(l_hand_it != hands.end()) {
        _LHand = l_hand_it->second;
    }
    

    return true;
}

void Grasp::on_start(double time)
{

    _start_time = time;
    _left_ref.set(0); 
    _right_ref.set(0);

    std::cout << "RT_GRASP STARTED!"<< std::endl;
   
    
}

void Grasp::control_loop(double time, double period)
{
    _RHand->grasp(_right_ref.get());
    _LHand->grasp(_left_ref.get());
    _robot->move();

}

bool Grasp::close()
{
    _logger->flush();
    return true;
}

}