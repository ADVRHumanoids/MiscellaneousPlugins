/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <MiscellaneousPlugins/SimpleHoming.h>

#define PRECISION 0.02 //[radians]


/* Specify that the class MiscPlugins::SimpleHoming is a Xbot RT plugin with name "SimpleHoming" */
REGISTER_XBOT_PLUGIN(SimpleHoming, MiscPlugins::SimpleHoming)

namespace MiscPlugins {

SimpleHoming::SimpleHoming()
{

}

bool SimpleHoming::init_control_plugin(std::string path_to_config_file,
                                    XBot::SharedMemory::Ptr shared_memory,
                                    XBot::RobotInterface::Ptr robot)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = robot;

    /* Get home configuration from SRDF file */
    if(!_robot->getRobotState("home", _q_home)){
        /* If the requested configuration does not exist within the SRDF file,
         * return false. In this case, the plugin will not be executed. */
        return false;
    }

    /* Save actual robot q to a private member */
    _robot->getJointPosition(_q0);
    _q = _q0;
    _delta_q = _q0;
    
    // max velocity
    _max_vel = 0.3; // rad/s

    /* Print the homing */
    std::cout << "_q_home from SRDF : " << _q_home << std::endl;

    /* Print the robot state */
    _robot->print();

    return true;


}

void SimpleHoming::on_start(double time)
{
    _robot->getJointPosition(_q0);
    _q = _q0;
}

void SimpleHoming::on_stop(double time)
{
    
}

bool MiscPlugins::SimpleHoming::check_goal()
{
    for(unsigned int i = 0; i < _q.size(); ++i){
        if( !( fabs( _q[i] - _q_home[i] ) <= PRECISION) ) {
            return false;
        }
    }
    return true;
}


void SimpleHoming::control_loop(double time, double period)
{
    if( !check_goal() ) {

        // calculate max q increment based on period and max vel
        _max_q_increment = _max_vel * period;

        for(int i = 0; i < _robot->getJointNum(); ++i) {
            _delta_q[i] = _q_home[i] - _q[i];
            if ( fabs( _delta_q[i] ) > _max_q_increment )
                _delta_q[i] = ( _delta_q[i]/fabs(_delta_q[i] ) ) * _max_q_increment;
            _q[i] += _delta_q[i];
        }
        
        _robot->setPositionReference(_q);
        _robot->move();
    
    }
    
    return;
}

bool SimpleHoming::close()
{
    return true;
}



}
