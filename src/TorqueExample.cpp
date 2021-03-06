#include <MiscellaneousPlugins/TorqueExample.h>


REGISTER_XBOT_PLUGIN(TorqueExample, MiscPlugins::TorqueExample)


bool MiscPlugins::TorqueExample::init_control_plugin(XBot::Handle::Ptr handle)
{
    _robot = handle->getRobotInterface();
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
   
    
    if(!current_command.str().empty()){
        
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
            _robot->getStiffness(_k0);
            _robot->getDamping(_d0);
            _robot->getMotorPosition(_q0);
            
        }
        
        if( current_command.str() == "remove_gcomp"){
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
    
    if( current_command.str() == "remove_gcomp"){
        // command tau = imp ctrl, qref = homing
        _tauref = _tauref*0;
        _robot->setEffortReference(_tauref);
        _robot->move();
        return;
    }
    
    if( current_command.str() == "gcomp"){
        // command tau = imp ctrl (low low gains) + g(q), qref = homing
        
        double alpha = (time - _starting_time)/4.3;
        alpha = alpha > 1 ? 1 : alpha;
        alpha = alpha < 0 ? 0 : alpha;
        
        _robot->setPositionReference(_q0);
        
        _robot->model().computeGravityCompensation(_tauref);
        _robot->model().setJointEffort(alpha*_tauref);
        _robot->setReferenceFrom(_robot->model(), XBot::Sync::Effort);
        
        
        double beta = (time - _starting_time - 4.3)/5.4;
        beta = beta > 0.96 ? 0.96 : beta;
        beta = beta < 0 ? 0 : beta;
        
        _robot->setStiffness(_k0*(1-beta));
        _robot->setDamping(_d0*(1-beta));
        
        _robot->move();
        
        
        
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
