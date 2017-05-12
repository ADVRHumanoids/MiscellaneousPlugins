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

#ifndef __MISCELLANEOUS_PLUGINS_TORQUE_EXAMPLE_H__
#define __MISCELLANEOUS_PLUGINS_TORQUE_EXAMPLE_H__

#include <XCM/XBotControlPlugin.h>

namespace MiscPlugins {

class TorqueExample : public XBot::XBotControlPlugin {

public:


    virtual bool init_control_plugin(std::string path_to_config_file,
                                     XBot::SharedMemory::Ptr shared_memory,
                                     XBot::RobotInterface::Ptr robot);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;
    Eigen::VectorXd _q0, _q_home, _q, _k, _d, _k0, _d0, _qref, _tauref0, _tauref;
    double _time, _homing_time, _first_loop_time, _starting_time;


};

}




#endif // __XCM_EXAMPLES_HOMING_EXAMPLE_H__
