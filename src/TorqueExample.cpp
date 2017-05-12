#include <MiscellaneousPlugins/TorqueExample.h>


REGISTER_XBOT_PLUGIN(TorqueExample, MiscPlugins::TorqueExample)


bool MiscPlugins::TorqueExample::init_control_plugin(std::string path_to_config_file, 
                                                     XBot::SharedMemory::Ptr shared_memory, 
                                                     XBot::RobotInterface::Ptr robot)
{
    _robot = robot;
    _robot->getRobotState("home", _q_home);
    
    _q0 = _q_home;
    _robot->getJointPosition(_q);
    _robot->getStiffness(_k);
    _robot->getDamping(_d);
    _k0 = _k;
    _d0 = _d;
    _qref = _q0;
    _tauref.setZero(_robot->getJointNum());
    _tauref0 = _tauref;

    
    
    
    
    return true;
}

void MiscPlugins::TorqueExample::control_loop(double time, double period)
{
   
    
    if(command.read(current_command)){
        
        _starting_time = time;
        
        if( current_command.str() == "zero_torque"){
            // save current torque ref, k, d
            _robot->getEffortReference(_tauref0);
            _robot->getStiffness(_k0);
            _robot->getDamping(_d0);
            
        }
        
        if( current_command.str() == "impedance"){
            // get starting position
        }
        
        if( current_command.str() == "gcomp"){
            // get starting position
        }
    }
    
    
    if( current_command.str() == "zero_torque"){
        
        // bring torque ref, k. d to zero
        double alpha = (time - _starting_time)/6.0;
        alpha = alpha > 1 ? 1 : alpha;
        alpha = alpha < 0 ? 0 : alpha;
        
        _k = (1 - alpha) * _k0;
        _d = (1 - alpha) * _d0;
        _tauref = (1 - alpha) * _tauref0;
        
        _robot->setStiffness(_k);
        _robot->setDamping(_d);
        _robot->setEffortReference(_tauref);
        
        _robot->move();
        
        return;
    }
    
    if( current_command.str() == "impedance"){
        // command tau = imp ctrl, qref = homing
        return;
    }
    
    if( current_command.str() == "gcomp"){
        // command tau = imp ctrl (low low gains) + g(q), qref = homing
        return;
    }
    
    
    
}

void MiscPlugins::TorqueExample::on_start(double time)
{
    
}

void MiscPlugins::TorqueExample::on_stop(double time)
{
    
}

bool MiscPlugins::TorqueExample::close()
{

}
