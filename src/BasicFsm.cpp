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

#include <MiscellaneousPlugins/BasicFsm.h>

/* Specify that the class MiscPlugins::Homing is a Xbot RT plugin with name "Homing" */
REGISTER_XBOT_PLUGIN(BasicFsm, MiscPlugins::BasicFsm)

namespace MiscPlugins {

  int curr_state=0;
  double step = 0.001;
  double stepSleep = 0.0008;
  Eigen::VectorXd  state[4];
  int num_state = 3;
  double timer = 0;
  double freeze = false;
  bool stop=false;
  double sleep = 0;
  
BasicFsm::BasicFsm()
{

}

bool BasicFsm::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Get home configuration from SRDF file */
    if(!_robot->getRobotState("home", state[1])){
        /* If the requested configuration does not exist within the SRDF file,
         * return false. In this case, the plugin will not be executed. */
        return false;
    }
    
    if(!_robot->getRobotState("state1", state[0])){
        /* If the requested configuration does not exist within the SRDF file,
         * return false. In this case, the plugin will not be executed. */
        return false;
    }
    
    if(!_robot->getRobotState("state2", state[2])){
        /* If the requested configuration does not exist within the SRDF file,
         * return false. In this case, the plugin will not be executed. */
        return false;
    }

    /* Save actual robot q to a private member */
    _robot->getJointPosition(_q0);
    _qref = _q0;
   
    

    /* Print the homing */
    std::cout << "_q_home from SRDF : " << _q_home << std::endl;

    /* Set an homing time */
    _homing_time = 4;

    /* Print the robot state */
    _robot->print();

  
    return true;

}

void BasicFsm::on_start(double time)
{
   /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /Homing_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */

    /* Save the starting time to a variable */
    _first_loop_time = time;
    
    /* Save the robot starting config to a variable */
    _robot->getJointPosition(_q0);
    stop = false;
    
    //restart state
    curr_state = 0;
    timer = 0;
    sleep = 0;
}

void BasicFsm::on_stop(double time)
{
     /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /Homing_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */
    
    stop = true;
}


void BasicFsm::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-sage. */

   
    //This FSM handles sequential task, if want handle more task at the same time (e.g. grasping while moving) we should have 2 plugin on 2 different threads
    //The FSM evolves based on time (no input/ output are considered)
    //In the case you want consider external input coming from external plugin we have to use SharedMemory mechanism.
    //Using SharedMemory mechanism is possible to send output to other FSM running in other plugin.
    //In the case you want consider evolving states based on read values you have to get the value.
    
      
      if(!stop){
	
	if(!freeze)
	  timer += step;
	
        timer = timer> 1 ? 1 : timer;
    
	if(timer != 1){
	  //executes the state operation
	  _robot->setPositionReference(_q0+timer*(state[curr_state]-_q0));
	  _robot->move();
	}
	
	
	//handle input pseudocode
	/*_robot->sense();
	if (condition_met)
	  curr_state= newstate*/
	
	//handles state transition
	if(timer == 1){
	  
	  if( curr_state +1 < num_state){
	    //next state
	    curr_state += 1;
	    timer = 0;
	  }
	  else{
	    //idle state
	    curr_state = num_state ;
	    timer = 1;
	  }
	  
	  _robot->getJointPosition(_q0);
	}
	
	
	
	/*if (curr_state == desired_state && timer == 1){
	  
	  //activate output to signal end of the state
	}*/
	
	//handles operation in a particular state
	if (curr_state == 1){
	  
	  //activate output during state execution
	  
	  
	  //wait in this state
	  sleep += stepSleep;
          sleep = sleep > 1 ? 1 : sleep;
	  
	  if (sleep == 1)
	    freeze = false;
	  else
	    freeze = true;  //stop timer
	 
	  
	}
	
      }
	
    
    
}

bool BasicFsm::close()
{
    return true;
}



}
