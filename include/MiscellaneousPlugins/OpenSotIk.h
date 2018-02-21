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
#include <XBotInterface/Utils.h>

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

#include <atomic>

namespace MiscPlugins {

class OpenSotIk : public XBot::XBotControlPlugin {

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual void on_start(double time);

    virtual void control_loop(double time, double period);

    virtual bool close();


private:
  
    void callback(const geometry_msgs::Vector3ConstPtr& msg)
    {
        _sub_value.store(msg->z);
    }

    std::atomic<float> _sub_value;
    XBot::RosUtils::SubscriberWrapper::Ptr _sub_rt;
    
    double _start_time, _final_qdot_lim;

    Eigen::VectorXd _q0, _q, _dq, _qhome, _q_ref, _tau, _k0;
    
    XBot::Utils::SecondOrderFilter<Eigen::VectorXd> _filter_q;

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;

    XBot::SharedObject<Eigen::Affine3d> _left_ref, _right_ref;
    
    Eigen::MatrixXd _aux_matrix;

    OpenSoT::tasks::velocity::Cartesian::Ptr _left_ee, _right_ee;
    OpenSoT::tasks::velocity::Postural::Ptr _postural;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_lims;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_vel_lims;

    OpenSoT::AutoStack::Ptr _autostack;
    OpenSoT::solvers::QPOases_sot::Ptr _solver;

    XBot::MatLogger::Ptr _logger;
    
    XBot::Handle::Ptr _xbot_handle;
    XBot::JointIdMap _nrt_stiffness, _nrt_damping;
    
//     XBot::IController::Ptr _controller;
    
    float _stiffness_z;
    float _prev_stiffness_z;
    float _z_stiff;
    
    Eigen::MatrixXd K_j_star, K_c, K_j, K_offj, J, Jt, JtKc, Jfb, zeros;
    Eigen::VectorXd tau_ff,dq,qmeas,q,h;
    int _dim;

};

}

#endif