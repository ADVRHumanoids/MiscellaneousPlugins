#include <MiscellaneousPlugins/Grasp.h>


REGISTER_XBOT_PLUGIN(Grasp, MiscPlugins::Grasp)

namespace MiscPlugins {

bool Grasp::init_control_plugin(std::string path_to_config_file,
                                    XBot::SharedMemory::Ptr shared_memory,
                                    XBot::RobotInterface::Ptr robot)
{
    _logger = XBot::MatLogger::getLogger("/tmp/Grasp_logger");

    _robot = robot;
    
    _left_ref = shared_memory->get<double>("w_grasp_left");
    
    _right_ref = shared_memory->get<double>("w_grasp_right");
    
    // hands
    std::map<std::string, XBot::Hand::Ptr> hands = _robot->getHand();
    // RHand
    auto r_hand_it = hands.find("r_handj");
    if(r_hand_it != hands.end()) {
        _RHand = r_hand_it->second;
    }
    // LHand
    auto l_hand_it = hands.find("l_handj");
    if(l_hand_it != hands.end()) {
        _LHand = l_hand_it->second;
    }
    

    return true;
}

void Grasp::on_start(double time)
{

    _start_time = time;
    *_left_ref = *_right_ref = 0;

    std::cout << "RT_GRASP STARTED!"<< std::endl;
   
    
}

void Grasp::control_loop(double time, double period)
{
    _RHand->grasp(*_right_ref);
    _LHand->grasp(*_left_ref);
    _robot->move();

}

bool Grasp::close()
{
    _logger->flush();
    return true;
}

}