#include <MiscellaneousPlugins/OpenSotIkXSense.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <OpenSoT/SubTask.h>

REGISTER_XBOT_PLUGIN(OpenSotIkXSense, MiscPlugins::OpenSotIkXSense)

namespace MiscPlugins {

void setWorld(const KDL::Frame& l_sole_T_Waist, Eigen::VectorXd& q, XBot::ModelInterface::Ptr _model_ptr)
{
        _model_ptr->setFloatingBasePose(l_sole_T_Waist);

        _model_ptr->getJointPosition(q);
}


bool OpenSotIkXSense::init_control_plugin(XBot::Handle::Ptr handle)
{
    _logger = XBot::MatLogger::getLogger("/tmp/OpenSotIk_logger");

    _robot = handle->getRobotInterface();
    _model = XBot::ModelInterface::getModel(handle->getPathToConfigFile());

    _robot->sense();
    _robot->getPositionReference(_q0);
    _robot->model().setJointPosition(_q0);
    
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

    _model->setJointPosition(_qhome);
    _model->update();

    //Here we set the world in the middle of the feet
    KDL::Frame l_sole_T_Waist;
    this->_model->getPose("Waist", "l_sole", l_sole_T_Waist);
    std::cout<<"l_sole_T_Waist: " <<  l_sole_T_Waist << std::endl;

    l_sole_T_Waist.p.x(0.0);
    l_sole_T_Waist.p.y(0.0);

    setWorld(l_sole_T_Waist, _q, _model);
    _model->setJointPosition(this->_q);
    _model->update();


    KDL::Frame world_T_bl;
    _model->getPose("Waist",world_T_bl);

    std::cout<<"world_T_bl: " << world_T_bl << std::endl;
    ///////////

    std::cout<<"home: "<<_qhome<<std::endl;

    _left_ref = handle->getSharedMemory()->getSharedObject<Eigen::Affine3d>("w_T_left_ee");
    _right_ref = handle->getSharedMemory()->getSharedObject<Eigen::Affine3d>("w_T_right_ee");
    
    _joint_ref = handle->getSharedMemory()->getSharedObject<MiscPlugins::Vector>("joint_positions_desired");

    std::vector<bool> active_joints(_model->getJointNum(), true);
    active_joints[_model->getDofIndex(_model->chain("torso").getJointId(0))] = false;
    active_joints[_model->getDofIndex(_model->chain("torso").getJointId(1))] = true;

    /* Create cartesian tasks for both hands */
    _left_ee.reset( new OpenSoT::tasks::velocity::Cartesian("CARTESIAN_LEFT",
                                                            _qhome,
                                                            *_model,
                                                             "LSoftHand",
//                                                            _model->chain("left_arm").getTipLinkName(),
                                                            "world"
                                                            ) );
     _left_ee->setActiveJointsMask(active_joints);
     _left_ee->setLambda(0.1);
     _left_ee->setOrientationErrorGain(0.05);

    _right_ee.reset( new OpenSoT::tasks::velocity::Cartesian("CARTESIAN_RIGHT",
                                                             _qhome,
                                                             *_model,
                                                              "RSoftHand",
//                                                             _model->chain("right_arm").getTipLinkName(),
                                                             "world"
                                                             ) );
    _right_ee->setActiveJointsMask(active_joints);
    _right_ee->setLambda(0.1);
    _right_ee->setOrientationErrorGain(0.05);

    /* Create postural task */
    _postural.reset( new OpenSoT::tasks::velocity::Postural(_qhome) );
    Eigen::MatrixXd W(_model->getJointNum(), _model->getJointNum()); W.setIdentity(W.rows(), W.cols());
    W(_model->getDofIndex(_model->chain("torso").getJointId(1)),_model->getDofIndex(_model->chain("torso").getJointId(1))) = 100000.;
    _postural->setWeight(W);
    _postural->setLambda(0.01);
    _postural->setActiveJointsMask(active_joints);


    /* Create joint limits & velocity limits */ //CHECK!
    Eigen::VectorXd qmin, qmax, qdotmax;
    _model->getJointLimits(qmin, qmax);
    qmax[_model->getDofIndex("LElbj")] = -0.1;
    qmax[_model->getDofIndex("RElbj")] = -0.1;
    qmax[_model->getDofIndex(_model->chain("torso").getJointId(1))] = 0.5;
    qmin[_model->getDofIndex(_model->chain("torso").getJointId(1))] = -0.5;
    _model->getVelocityLimits(qdotmax);
    double qdotmax_min = qdotmax.minCoeff();
    Eigen::VectorXd qdotlims(_qhome.size()); qdotlims.setConstant(_qhome.size(),1.75); // 1.0); // qdotmax_min);
    _final_qdot_lim = 2.0;

    _joint_lims.reset( new OpenSoT::constraints::velocity::JointLimits(_qhome, qmax, qmin) );

    _joint_vel_lims.reset( new OpenSoT::constraints::velocity::VelocityLimits(qdotlims, 0.003) );


    /* Create cartesian tasks for both feet */
    _l_sole.reset( new OpenSoT::tasks::velocity::Cartesian("CARTESIAN_L_SOLE",
                                                            _qhome,
                                                            *_model,
                                                            "l_sole",
                                                            "world"
                                                            ) );
    _r_sole.reset( new OpenSoT::tasks::velocity::Cartesian("CARTESIAN_R_SOLE",
                                                            _qhome,
                                                            *_model,
                                                            "r_sole",
                                                            "world"
                                                            ) );
    
    _waist.reset( new OpenSoT::tasks::velocity::Cartesian("CARTESIAN_WAIST",
                                                            _qhome,
                                                            *_model,
                                                            "Waist",
                                                            "world"
                                                            ) );
    std::list<unsigned int> orient_indices;
    orient_indices.push_back(3);
    orient_indices.push_back(4);
    orient_indices.push_back(5);
    OpenSoT::SubTask::Ptr waist_orient(new OpenSoT::SubTask(_waist, orient_indices));

    _com.reset( new OpenSoT::tasks::velocity::CoM(_qhome,*_model));
    
    std::vector<bool> com_active_joints(_model->getJointNum(), true);
    com_active_joints[_model->getDofIndex(_model->chain("torso").getJointId(0))] = false;
    com_active_joints[_model->getDofIndex(_model->chain("torso").getJointId(1))] = false;
    for(unsigned int i = 0; i < 7; ++i)
    {
        com_active_joints[_model->getDofIndex(_model->chain("left_arm").getJointId(i))] = false;
        com_active_joints[_model->getDofIndex(_model->chain("right_arm").getJointId(i))] = false;
    }
    _com->setActiveJointsMask(com_active_joints);
    _com->setLambda(0.1);



    std::list<unsigned int> neck_indices;
    neck_indices.push_back(_model->getDofIndex("NeckYawj"));
    neck_indices.push_back(_model->getDofIndex("NeckPitchj"));
    OpenSoT::SubTask::Ptr joint_space_gaze(
                new OpenSoT::SubTask(_postural, neck_indices));
    
     std::list<unsigned int> body_indices;
     for(unsigned int i = 0; i < _model->getJointNum(); ++i)
     {
        std::string joint_name = _model->getJointByDofIndex(i)->getJointName();
        if(joint_name.compare("NeckYawj")!= 0 && joint_name.compare("NeckPitchj")!= 0)
          body_indices.push_back(_model->getDofIndex(joint_name));
     }
    OpenSoT::SubTask::Ptr joint_space_body(
                new OpenSoT::SubTask(_postural, body_indices));
        
    std::list<unsigned int> id_com;
    id_com.push_back(0);id_com.push_back(1);
    std::list<unsigned int> id_waist;
    id_waist.push_back(2);id_waist.push_back(3);id_waist.push_back(4);id_waist.push_back(5);
    /* Create autostack and set solver */
    _autostack = (  (_l_sole + _r_sole)/
                    (_com%id_com + joint_space_gaze + _waist%id_waist)/
                    (_right_ee + _left_ee)/
                    (joint_space_body) ) << _joint_lims << _joint_vel_lims;
    _autostack->update(_qhome);               

    _solver.reset( new OpenSoT::solvers::QPOases_sot(_autostack->getStack(), _autostack->getBounds(),1e7) );

    /* Logger */
    Eigen::Affine3d left_pose, right_pose;
    _logger->add("left_ref_pos", _left_ref.get().translation());
    _logger->add("right_ref_pos", _right_ref.get().translation());
    _logger->add("left_actual_pos", left_pose.translation());
    _logger->add("right_actual_pos", right_pose.translation());
    
    _logger->add("com", _com->getActualPosition());

    _logger->add("left_ref_or", _left_ref.get().linear());
    _logger->add("right_ref_or", _right_ref.get().linear());
    _logger->add("left_actual_or", left_pose.linear());
    _logger->add("right_actual_or", right_pose.linear());
    _logger->add("computed_q", _q0);

    _logger->add("computed_qdot", _q0);

    _logger->add("time", 0.0);


    // NOTE initializing world RT publisher with "world" pipe name
    if(_model->getFloatingBasePose( _current_world ))
        _world_pub.init("world_odom");
    
    aux_matrix.resize(4,4);
    aux_vector.resize(_model->getJointNum());

    return true;
}

void OpenSotIkXSense::on_start(double time)
{

    //_model->syncFrom(*_robot);
    Eigen::VectorXd q_ref;
    _robot->getPositionReference(q_ref);
    //_robot->model().setJointPosition(_q);
    
    _model->getJointPosition(_q);
    _q.segment(6, _model->getJointNum()) = q_ref;
    _model->setJointPosition(_q);
    _model->update();

    Eigen::Affine3d left_ee_pose, right_ee_pose;
   
    _model->getPose(_left_ee->getDistalLink(), left_ee_pose);
    _model->getPose(_right_ee->getDistalLink(), right_ee_pose);
    
     _left_ref.set(left_ee_pose);
     _right_ref.set(right_ee_pose);

    /* Set cartesian tasks reference */
    _postural->setReference(_joint_ref.get());
    _left_ee->setReference(_left_ref.get().matrix());
    _right_ee->setReference(_right_ref.get().matrix());

    _left_ee->setLambda(0);
    _right_ee->setLambda(0);
    //_postural->setLambda(0);

    _start_time = time;
    
    std::cout << "OpenSotIkTestPlugin STARTED!"<<std::endl;
    for(unsigned int i = 0; i < _q.size(); ++i)
        std::cout<<"q sensed "<<i<<": "<<_q[i]<<"    expected from home: "<<_qhome[i]<<" --> "<<_model->getJointByDofIndex(i)->getJointName()<<std::endl;
}

void OpenSotIkXSense::control_loop(double time, double period)
{
    /* Model update */
    _model->setJointPosition(_q);
    _model->update();

    // NOTE compute current world and send it trough the Publisher RT
    XBot::TransformMessage odom_message;
    _model->getFloatingBaseLink(_floating_base_name);
    odom_message.parent_frame = _floating_base_name; 
    odom_message.child_frame = std::string("world_odom");
    if(_model->getFloatingBasePose( odom_message.pose )){
        odom_message.pose = odom_message.pose.inverse();
        _world_pub.write(odom_message);
    }


    /* HACK: shape IK gain to avoid discontinuity */
    double alpha = 0;
    alpha = (time - _start_time)/100;
    alpha = alpha > 1 ? 1 : alpha;

    _left_ee->setLambda(alpha);
    _right_ee->setLambda(alpha);
    //_postural->setLambda(alpha);


    /* Set cartesian tasks reference */
    aux_matrix = _left_ref.get().matrix();
    _left_ee->setReference(aux_matrix);
    aux_matrix= _right_ref.get().matrix();
    _right_ee->setReference(aux_matrix);
    aux_vector = _joint_ref.get();
    _postural->setReference(aux_vector);
//     std::cout<<"_joint_ref: "<<_joint_ref.get()<<std::endl;
    

    /* Log data */
  
//     _model->getPose(_left_ee->getDistalLink(), left_pose);
//     _model->getPose(_right_ee->getDistalLink(), right_pose);

//     _logger->add("left_ref_pos", _left_ref.get().translation());
//     _logger->add("right_ref_pos", _right_ref.get().translation());
//     _logger->add("left_actual_pos", left_pose.translation());
//     _logger->add("right_actual_pos", right_pose.translation());
//     
//     _logger->add("com", _com->getActualPosition());
// 
//     _logger->add("left_ref_or", _left_ref.get().linear());
//     _logger->add("right_ref_or", _right_ref.get().linear());
//     _logger->add("left_actual_or", left_pose.linear());
//     _logger->add("right_actual_or", right_pose.linear());
//     _logger->add("computed_q", _q);
// 
//     _logger->add("time", time);


    /* Stack update and solve */
    _postural->update(_q);
    _waist->update(_q);
    _autostack->update(_q);

    _dq.setZero(_model->getJointNum());

    if( !_solver->solve(_dq) ){
        std::cerr << "UNABLE TO SOLVE" << std::endl;
        return;
    }

//     _logger->add("computed_qdot", _dq/period);

    /* Update q */
    _q += _dq;


    /* Send command to motors */
    _robot->setReferenceFrom(*_model, XBot::Sync::Position);
    _robot->move();

}

bool OpenSotIkXSense::close()
{
    _logger->flush();
    return true;
}

}
