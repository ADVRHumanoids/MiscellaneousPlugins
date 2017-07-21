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


#ifndef __MISC_PLUGINS_OPENSOT_IK_H__
#define __MISC_PLUGINS_OPENSOT_IK_H__

#include <XCM/XBotControlPlugin.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/velocity/CoM.h>

#include <MiscellaneousPlugins/TransformMessage.h>

namespace MiscPlugins {

class OpenSotIk : public XBot::XBotControlPlugin {

public:

    virtual bool init_control_plugin(std::string path_to_config_file,
                                    XBot::SharedMemory::Ptr shared_memory,
                                    XBot::RobotInterface::Ptr robot);

    virtual void on_start(double time);

    virtual void control_loop(double time, double period);

    virtual bool close();


private:

    double _start_time, _final_qdot_lim;
    
    Eigen::MatrixXd aux_matrix;

    Eigen::VectorXd _q0, _q, _dq, _qhome;

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;

    XBot::PublisherRT<XBot::TransformMessage> _world_pub;
    Eigen::Affine3d _current_world;

    XBot::SharedObject<Eigen::Affine3d> _left_ref, _right_ref;

    OpenSoT::tasks::velocity::Cartesian::Ptr _left_ee, _right_ee;
    OpenSoT::tasks::velocity::Cartesian::Ptr _l_sole, _r_sole;
    OpenSoT::tasks::velocity::CoM::Ptr _com;
    OpenSoT::tasks::velocity::Postural::Ptr _postural;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_lims;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_vel_lims;

    OpenSoT::AutoStack::Ptr _autostack;
    OpenSoT::solvers::QPOases_sot::Ptr _solver;
    
    Eigen::Affine3d left_pose, right_pose;

    XBot::MatLogger::Ptr _logger;

    std::string _floating_base_name;

};

}

#endif