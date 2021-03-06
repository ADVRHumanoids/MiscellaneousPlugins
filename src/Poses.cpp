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

#include <MiscellaneousPlugins/Poses.h>
#include <boost/algorithm/clamp.hpp>

REGISTER_XBOT_PLUGIN_(MiscPlugins::Poses)


bool MiscPlugins::Poses::getJointConfigurations()
{
    _root_cfg = YAML::LoadFile(_path_to_config_file);
    if(!_root_cfg["JointConfiguration"]) {

        XBot::Logger::error("ERROR in %s ! JointConfigurationPlugin needs mandatory node JointConfiguration in the YAML file!\n", __PRETTY_FUNCTION__);
        return false;
    }
    else if(!_root_cfg["JointConfigurationTime"]) {

        XBot::Logger::error("ERROR in %s ! JointConfigurationPlugin needs mandatory node JointConfigurationTime in the YAML file!\n", __PRETTY_FUNCTION__);
        return false;
    }
    else{

        Eigen::VectorXd _q_configuration;
        std::string _joint_configuration_name;
        double _joint_configuration_time;
        _configuration_num = 0;

        // read the possible joint configuration from config file
        for(const auto& configuration : _root_cfg["JointConfiguration"]){

            _joint_configuration_name = configuration.as<std::string>();
            // saving joint configuration name
            XBot::Logger::info("Found joint configuration : %s\n", _joint_configuration_name.c_str());
            _configuration_name.push_back(_joint_configuration_name);
            _configuration_id_map[_joint_configuration_name] = _configuration_num;
            // getting robot joint state from SRDF using the current joint configuration
            _robot->getRobotState(_joint_configuration_name, _q_configuration);
            _joint_configuration.push_back(_q_configuration);
            // config num
            _configuration_num++;
        }

        // read the joint configuration trajectory time from config file
        for(const auto& configuration_time : _root_cfg["JointConfigurationTime"]){
            _joint_configuration_time = configuration_time.as<double>();
            _move_to_configuration_time.push_back(_joint_configuration_time);
        }
    }

    return true;

}

double MiscPlugins::Poses::smootherstep(double edge0, double edge1, double x)
{
    // Scale, and clamp x to 0..1 range
    x = boost::algorithm::clamp<double>((x - edge0)/(edge1 - edge0), 0.0, 1.0);
    // Evaluate polynomial
    return x*x*x*(x*(x*6 - 15) + 10);
}


bool MiscPlugins::Poses::init_control_plugin(XBot::Handle::Ptr handle)
{
    _robot = handle->getRobotInterface();
    _path_to_config_file = handle->getPathToConfigFile();

    _change_configuration = false;
    _current_configuration = -1;
    _current_pose_id = 0;
    _read_cmd_ok = false;

    _logger = XBot::MatLogger::getLogger("/tmp/XBotCore_log");

    _robot->model().getModelOrderedJoints(_joint_names);

     // read the JointConfiguration from the YAML specified in path_to_config_file
    return getJointConfigurations();

    _robot->getSrdf();
}

void MiscPlugins::Poses::control_loop(double time, double period)
{
        
    // if we are not moving to a new joint configuration update t0 and q0
    if(!_change_configuration) {

        // NOTE continous mode
        current_command = XBot::Command(_configuration_name.at(_current_pose_id));
        _current_pose_id = _current_pose_id < (_configuration_num - 1) ? _current_pose_id + 1 : 0;
        Logger::info("Command : %s in execution\n", current_command.str().c_str());

        
//         // blocking reading: wait for a command
//         if(current_command.str().empty()) {
//             return;
//         }

        // check if the joint configuration requested in the command exists
        if(_configuration_id_map.count(current_command.str())) {

            // next configuration
            _current_configuration = _configuration_id_map.at(current_command.str());

            // update time and current joint state
            _t0 = time;
            _robot->getMotorPosition(_q0);

            // configuration changed
            _change_configuration = true;
        }
        else {
            Logger::error("Command : %s is not supported\n", current_command.str().c_str());
        }
    }

    // we received a command: we need to change configuration
    if( _change_configuration ) {

        // move to the configuration
        if( (time - _t0) <= _move_to_configuration_time.at(_current_configuration) ) {
            _q_ref = _q0 + smootherstep(_t0, _t0 + _move_to_configuration_time[_current_configuration],  time) * (_joint_configuration[_current_configuration]-_q0);

            // trajectory
            _robot->setPositionReference(_q_ref);
            _robot->move();
        }
        else {
            _change_configuration = false;
            Logger::info("Command executed!\n");
        }
    }

   
}

void MiscPlugins::Poses::on_start(double time)
{
    XBot::XBotControlPlugin::on_start(time);
}

void MiscPlugins::Poses::on_stop(double time)
{
    XBot::XBotControlPlugin::on_stop(time);
}

bool MiscPlugins::Poses::close()
{
    return true;
}



