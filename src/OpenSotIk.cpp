#include <MiscellaneousPlugins/OpenSotIk.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <OpenSoT/tasks/velocity/MinimumEffort.h>

REGISTER_XBOT_PLUGIN(OpenSotIk, MiscPlugins::OpenSotIk)

namespace MiscPlugins {

bool OpenSotIk::init_control_plugin(std::string path_to_config_file,
                                    XBot::SharedMemory::Ptr shared_memory,
                                    XBot::RobotInterface::Ptr robot)
{
    _logger = XBot::MatLogger::getLogger("/tmp/OpenSotIk_logger");

    _robot = robot;
    _model = XBot::ModelInterface::getModel(path_to_config_file);
    
    _robot->sense();
    _robot->model().getJointPosition(_q0);

    _model->getRobotState("home", _qhome);

    for(unsigned int i = 0; i < _qhome.size(); ++i)
    {
        std::string name = _model->getJointByDofIndex(i)->getJointName();
        double q_min, q_max;
        _model->getJointByDofIndex(i)->getJointLimits(q_min, q_max);
        std::cout<<name<<": "<<q_min<<" < "<<_qhome(i)<<" "<<" < "<<q_max;
        if(_qhome(i) > q_max || _qhome(i) < q_min)
            std::cout<<"    VALUE NOT IN LIMITS!!!!";
        std::cout<<std::endl;
    }


    std::cout<<"home: "<<_qhome<<std::endl;

    _left_ref = shared_memory->get<Eigen::Affine3d>("w_T_left_ee");
    _right_ref = shared_memory->get<Eigen::Affine3d>("w_T_right_ee");

    _left_ref.reset(new Eigen::Affine3d);
    _right_ref.reset(new Eigen::Affine3d);

    std::vector<bool> active_joints(_model->getJointNum(), true);
    active_joints[_model->getDofIndex(_model->chain("torso").getJointId(0))] = false;
    active_joints[_model->getDofIndex(_model->chain("torso").getJointId(1))] = false;

    /* Create cartesian tasks for both hands */
    _left_ee.reset( new OpenSoT::tasks::velocity::Cartesian("CARTESIAN_LEFT",
                                                            _qhome,
                                                            *_model,
                                                            "LSoftHand",
//                                                             _model->chain("left_arm").getTipLinkName(),
                                                            "world"
                                                            ) );
     //_left_ee->setActiveJointsMask(active_joints);

    _right_ee.reset( new OpenSoT::tasks::velocity::Cartesian("CARTESIAN_RIGHT",
                                                             _qhome,
                                                             *_model,
                                                             "RSoftHand",
//                                                              _model->chain("right_arm").getTipLinkName(),
                                                             "world"
                                                             ) );
//     _right_ee->setActiveJointsMask(active_joints);

    /* Create postural task */
    _postural.reset( new OpenSoT::tasks::velocity::Postural(_qhome) );
    Eigen::VectorXd weight;
    weight.setOnes((_model->getJointNum()));
    weight(0) = 100;
//     _postural->setLambda(0.0);
    _postural->setWeight(weight.asDiagonal());

    /* Create min acc task */
    OpenSoT::tasks::velocity::MinimizeAcceleration::Ptr min_acc( new OpenSoT::tasks::velocity::MinimizeAcceleration(_qhome) );

    /* Manipulability task */
    OpenSoT::tasks::velocity::Manipulability::Ptr manipulability_right( new OpenSoT::tasks::velocity::Manipulability(_qhome, *_model, _right_ee) );
    OpenSoT::tasks::velocity::Manipulability::Ptr manipulability_left( new OpenSoT::tasks::velocity::Manipulability(_qhome, *_model, _left_ee) );

    /* Minimum effort task */
    OpenSoT::tasks::velocity::MinimumEffort::Ptr min_effort( new OpenSoT::tasks::velocity::MinimumEffort(_qhome, *_model) );
    min_effort->setLambda(0.01);


    /* Create joint limits & velocity limits */
    Eigen::VectorXd qmin, qmax, qdotmax;
    _model->getJointLimits(qmin, qmax);
    _model->getVelocityLimits(qdotmax);
    double qdotmax_min = qdotmax.minCoeff();
    Eigen::VectorXd qdotlims(_qhome.size()); qdotlims.setConstant(_qhome.size(), qdotmax_min);
    qdotlims[_model->getDofIndex(_model->chain("torso").getJointId(1))] = 0.01;
    _final_qdot_lim = 2.0;

    _joint_lims.reset( new OpenSoT::constraints::velocity::JointLimits(_qhome, qmax, qmin) );

    _joint_vel_lims.reset( new OpenSoT::constraints::velocity::VelocityLimits(qdotlims, 0.001) );

    /* Create autostack and set solver */
    _autostack = ( (_right_ee + _left_ee) / (_postural) ) << _joint_lims << _joint_vel_lims;
    _solver.reset( new OpenSoT::solvers::QPOases_sot(_autostack->getStack(), _autostack->getBounds(),1e10) );

    /* Logger */
    Eigen::Affine3d left_pose, right_pose;
    _logger->add("left_ref_pos", _left_ref->translation());
    _logger->add("right_ref_pos", _right_ref->translation());
    _logger->add("left_actual_pos", left_pose.translation());
    _logger->add("right_actual_pos", right_pose.translation());

    _logger->add("left_ref_or", _left_ref->linear());
    _logger->add("right_ref_or", _right_ref->linear());
    _logger->add("left_actual_or", left_pose.linear());
    _logger->add("right_actual_or", right_pose.linear());
    _logger->add("computed_q", _q0);

    _logger->add("computed_qdot", _q0);

    _logger->add("time", 0.0);


    return true;
}

void OpenSotIk::on_start(double time)
{

    _model->syncFrom(*_robot);
    _model->getJointPosition(_q);


    _model->getPose(_left_ee->getDistalLink(), *_left_ref);
    _model->getPose(_right_ee->getDistalLink(), *_right_ref);

    /* Set cartesian tasks reference */
    _left_ee->setReference(_left_ref->matrix());
    _right_ee->setReference(_right_ref->matrix());

    _left_ee->setLambda(0);
    _right_ee->setLambda(0);
    _postural->setLambda(0);

    _start_time = time;

    std::cout << "OpenSotIkTestPlugin STARTED!\nInitial q is " << _q.transpose() << std::endl;
    std::cout << "Home q is " << _qhome.transpose() << std::endl;
}

void OpenSotIk::control_loop(double time, double period)
{
    /* Model update */
    _model->setJointPosition(_q);
    _model->update();

    /* HACK: shape IK gain to avoid discontinuity */
    double alpha = 0;
    alpha = (time - _start_time)/100;
    alpha = alpha > 1 ? 1 : alpha;

    _left_ee->setLambda(alpha);
    _right_ee->setLambda(alpha);
    _postural->setLambda(alpha);


    /* Set cartesian tasks reference */
    _left_ee->setReference(_left_ref->matrix());
    _right_ee->setReference(_right_ref->matrix());

    /* Log data */
    Eigen::Affine3d left_pose, right_pose;
    _model->getPose(_left_ee->getDistalLink(), left_pose);
    _model->getPose(_right_ee->getDistalLink(), right_pose);

    _logger->add("left_ref_pos", _left_ref->translation());
    _logger->add("right_ref_pos", _right_ref->translation());
    _logger->add("left_actual_pos", left_pose.translation());
    _logger->add("right_actual_pos", right_pose.translation());

    _logger->add("left_ref_or", _left_ref->linear());
    _logger->add("right_ref_or", _right_ref->linear());
    _logger->add("left_actual_or", left_pose.linear());
    _logger->add("right_actual_or", right_pose.linear());
    _logger->add("computed_q", _q);

    _logger->add("time", time);


    /* Stack update and solve */
    _autostack->update(_q);

    _dq.setZero(_model->getJointNum());

    if( !_solver->solve(_dq) ){
        std::cerr << "UNABLE TO SOLVE" << std::endl;
        return;
    }

    _logger->add("computed_qdot", _dq/period);

    /* Update q */
    _q += _dq;


    /* Send command to motors */
    _robot->setReferenceFrom(*_model, XBot::Sync::Position);
    _robot->move();

}

bool OpenSotIk::close()
{
    _logger->flush();
    return true;
}

}
