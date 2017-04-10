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
 *
 */

#ifndef __MISC_PLUGINS_POSES_H__
#define __MISC_PLUGINS_POSES_H__

#include <XCM/XBotControlPlugin.h>

namespace MiscPlugins {

class Poses : public XBot::XBotControlPlugin {

public:

    virtual bool init_control_plugin(std::string path_to_config_file, XBot::SharedMemory::Ptr shared_memory, XBot::RobotInterface::Ptr robot);

    virtual void on_start(double time);

    virtual void control_loop(double time, double period);

    virtual void on_stop(double time);

    virtual bool close();

private:

    bool getJointConfigurations();
    double smootherstep(double edge0, double edge1, double x);

    XBot::MatLogger::Ptr _logger;
    XBot::RobotInterface::Ptr _robot;

    bool _read_cmd_ok;

    std::string _path_to_config_file;
    YAML::Node _root_cfg;

    bool _change_configuration;

    std::vector<Eigen::VectorXd> _joint_configuration;
    std::vector<std::string> _configuration_name;
    std::map<std::string, int> _configuration_id_map;
    int _configuration_num;
    int _current_configuration;
    std::vector<double> _move_to_configuration_time;

    Eigen::VectorXd _q0;
    double _t0;

    Eigen::VectorXd _q_ref, _current;

    std::vector< std::string> _joint_names;



};

}

#endif