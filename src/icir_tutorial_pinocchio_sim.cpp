#include "icir_tutorial_pinocchio/icir_tutorial_pinocchio_sim.hpp"

using namespace std;
using namespace Eigen;
using namespace pinocchio;

int main(int argc, char **argv)
{
    /////////////////////////// Setting ////////////////////////////////////////////////    
    ros::init(argc, argv, "icir_gen3_pinocchio_sim");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(SAMPLING_RATE);

    // Mujoco Subs
    jointState = n_node.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_command_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_time_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));
    sam_approach_sub_ = n_node.subscribe("/sam/approach_point", 1, &samApproachCallback);
    sam_grasp_sub_    = n_node.subscribe("/sam/grasp_point",    1, &samGraspCallback);

    // Mujoco Pubs
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("mujoco_ros/mujoco_ros_interface/joint_set", 5);
    mujoco_run_pub_ = n_node.advertise<std_msgs::Bool>("mujoco_ros/mujoco_ros_interface/sim_run", 5);

    //topic echo
    pos_des_pub_ = n_node.advertise<std_msgs::Float64MultiArray>("pos_des_topic", 10);
    pos_cur_pub_ = n_node.advertise<std_msgs::Float64MultiArray>("pos_cur_topic", 10);

    // Mujoco Msg
    robot_command_msg_.position.resize(GEN3_DOF); 
    robot_command_msg_.torque.resize(GEN3_DOF); 

    // Ros Param
    string urdf_name, urdf_path;
    n_node.getParam("urdf_path", urdf_path);
    n_node.getParam("urdf_name", urdf_name);    

    // Pinocchio
    vector<string> package_dirs;
    package_dirs.push_back(urdf_path);
    std::string urdfFileName = package_dirs[0] + "/" + urdf_name;
    robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false);  
    model_ = robot_->model();
    Data data(model_);
    data_ = data;

    cout << "nq : " << robot_->nq() << endl;
    cout << "nv : " << robot_->nv() << endl;
    cout << "na : " << robot_->na() << endl;

    // Control Variable
    ctrl_mode_ = 0;
    chg_flag_ = false;
    gripper_pos_des_ = 0.0;
    state_.J.resize(6, GEN3_DOF);   

    state_.q_des.setZero();
    state_.q_des_pre.setZero();   
    state_.v_des.setZero();
    state_.ddq_des.setZero();
    state_.tau_des.setZero(); 

    state_.task_jog_offset_.setZero();
    state_.task_rot_offset_ = Eigen::Matrix3d::Identity();
    
    m_p_.resize(12);
    m_v.resize(6);
    m_a.resize(6);
    m_v_ref = Motion(m_v.setZero());
    m_a_ref = Motion(m_a.setZero());
    m_frame_id = model_.getFrameId("Actuator6");    
    m_joint_id = model_.getJointId("Actuator6");    
    cout << "m_frame_id : " << m_frame_id << endl;
    cout << "m_joint_id : " << m_joint_id << endl;
    
    cout << "nframes : " << model_.nframes << endl;
    for (pinocchio::FrameIndex i = 0; i < model_.nframes; ++i)
    {
    const auto & f = model_.frames[i];
    std::cout << "Frame ID: " << i
              << ", Name: " << f.name
              << ", Type: " << f.type
              << ", Parent Joint: " << f.parent
              << std::endl;
    }
    ////////////////////////////////////////////////////////////////////////////////////////      

    Home << 0.00, 45.0, 90.0, 0.0, 45.0, -90.0, 0.0;
    Home2 << 45.00, 45.0, 90.0, 30.0, 50.0, 0.0, 0.0;
    Home3 << 90.00, -25.0, 45.0, 0.0, 45.0, -90.0, 0.0;
    Pick << 180.0, -15.0, 125.0, 0.0, 40.0, -90.0, 0.0;
    Place << 0.0, -15.0, 125.0, 0.0, 40.0, -90.0, 0.0;
    ////////////////////////////////////////////////////////////////////////////////////////
            
    posture_Kp << 40000., 40000., 40000., 40000., 40000., 40000., 40000.;
    posture_Kd << 2.0*posture_Kp.cwiseSqrt();
    
    // ee_Kp << 1000., 1000., 1000., 2000., 2000., 2000.;
    ee_Kp << 5000., 5000., 5000., 50000., 50000., 50000., 50000.;
    ee_Kd << 2.0*ee_Kp.cwiseSqrt();
    ////////////////////////////////////////////////////////////////////////////////////////

    while (ros::ok()){
        keyboard_event();

        // ── SAM Pick State Machine ────────────────────────────────────────
        // 카메라 → 월드 변환 헬퍼
        auto cam_to_world = [&](double cx, double cy, double cz) -> Eigen::Vector3d {
            // 카메라 오프셋 in Bracelet_Link frame (XML: pos="0 0.08 -0.04")
            pinocchio::SE3 T_world_bl  = robot_->position(data_, m_joint_id);
            pinocchio::SE3 T_bl_cam(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.08, -0.04));
            pinocchio::SE3 T_world_cam = T_world_bl * T_bl_cam;
            return T_world_cam.act(Eigen::Vector3d(cx, cy, cz));
        };

        // 그리퍼팁이 target_world에 오도록 BL offset 계산
        auto compute_offset = [&](const Eigen::Vector3d& target_world) {
            pinocchio::SE3 T_world_bl = robot_->position(data_, m_joint_id);
            Eigen::Vector3d gripper_offset_world = T_world_bl.rotation() * T_BL_GRIPPER_TIP;
            Eigen::Vector3d target_bl = target_world - gripper_offset_world;
            Eigen::Vector3d offset    = target_bl - T_world_bl.translation();
            const double MAX_OFFSET = 0.6;
            if (offset.norm() > MAX_OFFSET) {
                ROS_WARN("[SAM] Offset %.3f m clamped to %.1f m", offset.norm(), MAX_OFFSET);
                offset = offset.normalized() * MAX_OFFSET;
            }
            return offset;
        };

        // approach 수신 → 포인트 저장 + APPROACH 시작 (IDLE일 때만)
        if (pick_state_ == PICK_IDLE && sam_approach_received_ && sam_grasp_received_) {
            sam_approach_received_ = false;
            sam_grasp_received_    = false;

            approach_world_ = cam_to_world(sam_approach_cam_[0], sam_approach_cam_[1], sam_approach_cam_[2]);
            grasp_world_    = cam_to_world(sam_grasp_cam_[0],    sam_grasp_cam_[1],    sam_grasp_cam_[2]);

            cout << "[SAM] approach world: " << approach_world_.transpose() << endl;
            cout << "[SAM] grasp    world: " << grasp_world_.transpose()    << endl;

            // 그리퍼 열기
            gripper_pos_des_ = 0.0;

            // APPROACH 이동
            state_.task_jog_offset_ = compute_offset(approach_world_);
            ctrl_mode_ = 3;
            chg_flag_  = true;

            pick_state_            = PICK_APPROACH;
            pick_state_start_time_ = time_;
        }

        // State machine 전환
        if (pick_state_ == PICK_APPROACH && (time_ - pick_state_start_time_) > 3.0) {
            // DESCEND: 그리퍼팁을 물체 윗면으로
            state_.task_jog_offset_ = compute_offset(grasp_world_);
            state_.task_rot_offset_ = Eigen::Matrix3d::Identity();
            ctrl_mode_ = 3;
            chg_flag_  = true;
            pick_state_            = PICK_DESCEND;
            pick_state_start_time_ = time_;
            cout << "[SAM] State: DESCEND" << endl;
        }
        else if (pick_state_ == PICK_DESCEND && (time_ - pick_state_start_time_) > 3.0) {
            // GRASP: 그리퍼 닫기
            gripper_pos_des_ = 2.5;
            pick_state_            = PICK_GRASP;
            pick_state_start_time_ = time_;
            cout << "[SAM] State: GRASP (closing gripper)" << endl;
        }
        else if (pick_state_ == PICK_GRASP && (time_ - pick_state_start_time_) > 1.0) {
            // LIFT: approach 높이로 다시 올라가기
            state_.task_jog_offset_ = compute_offset(approach_world_);
            state_.task_rot_offset_ = Eigen::Matrix3d::Identity();
            ctrl_mode_ = 3;
            chg_flag_  = true;
            pick_state_            = PICK_LIFT;
            pick_state_start_time_ = time_;
            cout << "[SAM] State: LIFT" << endl;
        }
        else if (pick_state_ == PICK_LIFT && (time_ - pick_state_start_time_) > 3.0) {
            pick_state_ = PICK_IDLE;
            cout << "[SAM] State: IDLE (pick complete)" << endl;
        }
        // ─────────────────────────────────────────────────────────────────

        std_msgs::String sim_run_msg_;
        sim_run_msg_.data = true;
        mujoco_run_pub_.publish(sim_run_msg_);
        
        robot_->computeAllTerms(data_, state_.q, state_.v);        

        if (ctrl_mode_== 0){
            state_.q_des.setZero();
            state_.tau_des.setZero();    
        }

        if (ctrl_mode_ == 1){ // joint task
            if (chg_flag_){
                cubic_.stime = time_;
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;                
                state_.q_des_pre = state_.q; // Initialize q_des_pre to prevent velocity spike

                for (int i = 0; i<GEN3_DOF; i++)
                {
                    // q_target_(i) = Home(i) * M_PI / 180.;
                    q_target_(i) = state_.q_goal(i);
                }               

                chg_flag_ = false;
            }            
            for (int i=0; i<GEN3_DOF; i++)
            {
                // state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);                            
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), 0.0, 0.0);                            

                state_.v_des(i) = (state_.q_des(i) - state_.q_des_pre(i)) * SAMPLING_RATE;
                state_.q_des_pre(i) = state_.q_des(i);
                
                state_.ddq_des(i) = -posture_Kp(i)*(state_.q(i) - state_.q_des(i)) -posture_Kd(i)*(state_.v(i) - state_.v_des(i));
            }
        }
        
        if (ctrl_mode_ == 3){ // ee task //k ee jog -0.05z
            if (chg_flag_){
                SE3Cubic_.stime = time_;
                SE3Cubic_.duration = 2.0;                

                //set init ee pose
                H_ee_ = robot_->position(data_, m_joint_id);
                SE3Cubic_.m_init = H_ee_;                  

                //set desired ee pose
                H_ee_ref_ = H_ee_;

                // H_ee_ref_.translation()(2) -= 0.05; 
                H_ee_ref_.translation() += state_.task_jog_offset_;
                H_ee_ref_.rotation() = H_ee_.rotation() * state_.task_rot_offset_;

                state_.task_jog_offset_.setZero();
                state_.task_rot_offset_ = Eigen::Matrix3d::Identity();
                
                chg_flag_ = false;                
            }

            //desired trajectory [VectorXd];
            sampleEE_ = SE3Cubic(time_, SE3Cubic_.stime, SE3Cubic_.duration, SE3Cubic_.m_init, H_ee_ref_);
            vectorToSE3(sampleEE_, m_M_ref);  //desired trajectory in SE3 [pinocchio::SE3]                                 

            SE3 oMi;
            Motion v_frame;                                                            
            robot_->framePosition(data_, m_frame_id, oMi);                //frame position in global frame
            robot_->frameVelocity(data_, m_frame_id, v_frame);            //frame velocity in local frame
            robot_->frameClassicAcceleration(data_, m_frame_id, m_drift); //m_drift in local frame which is identical to J_dot * q_dot            
            robot_->jacobianWorld(data_, m_joint_id, state_.J);           //jacobian in global frame     
            robot_->frameJacobianLocal(data_, m_frame_id, m_J_local_);    //frame jacobian in local frame

            // cout << "robot_->position(data_, m_joint_id)" << robot_->position(data_, m_joint_id) << endl;
            // cout << "robot_->framePosition(data_, m_frame_id, oMi)" << oMi << endl;

            SE3ToVector(oMi, m_p_);                                       // current pos in vector form            
            errorInSE3(oMi, m_M_ref, m_p_error);                          // pos erorr represented in local frame, oMi_inv*m_M_ref                                    

            // Transformation from local to world
            m_wMl.translation(oMi.translation());                               // use rotation only for vel&acc transformation
            m_wMl.rotation(oMi.rotation());                               // use rotation only for vel&acc transformation

            m_p_error_vec = m_p_error.toVector();                         // pos err vector in local frame            
            m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;                 // vel err vector in local frame                         

            // desired acc in local frame
            m_a_des =   ee_Kp.cwiseProduct(m_p_error_vec)
                        + ee_Kd.cwiseProduct(m_v_error.toVector())
                        + m_wMl.actInv(m_a_ref).toVector();                             

            //transformation from ee to joint            
            // state_.ddq_des = m_J_local_.completeOrthogonalDecomposition().pseudoInverse() * (m_a_des - m_drift.toVector());                                            

            Eigen::Matrix<double, 6, 7> J_fixed = m_J_local_;
            Eigen::Matrix<double, 6, 1> a_error = m_a_des.head<6>() - m_drift.toVector();
            state_.ddq_des = J_fixed.completeOrthogonalDecomposition().solve(a_error);

            //publish
            pos_des_pub();            
            pos_cur_pub();
        }
        if (ctrl_mode_ == 4) { // sine motion
            if (chg_flag_) {
                sine_stime_ = time_;
                H_ee_init_ = robot_->position(data_, m_joint_id);

                chg_flag_ = false;
            }

            double t = time_ -sine_stime_;
            double A = 0.05;
            double f = 0.5;

            m_M_ref = H_ee_init_;
            m_M_ref.translation()(2) += A * sin(2.0 * M_PI * f * t);

            SE3 oMi;
            Motion v_frame;
            robot_->framePosition(data_, m_frame_id, oMi);
            robot_->frameVelocity(data_, m_frame_id, v_frame);
            robot_->frameClassicAcceleration(data_, m_frame_id, m_drift);
            robot_->jacobianWorld(data_, m_joint_id, state_.J);
            robot_->frameJacobianLocal(data_, m_frame_id, m_J_local_);

            SE3ToVector(oMi, m_p_);
            errorInSE3(oMi, m_M_ref, m_p_error);

            // Transformation from local to world
            m_wMl.translation(oMi.translation());
            m_wMl.rotation(oMi.rotation());

            m_p_error_vec = m_p_error.toVector();
            m_v_error = m_wMl.actInv(m_v_ref) - v_frame;

            // Task Space PD Control Law
            m_a_des = ee_Kp.cwiseProduct(m_p_error_vec)
                    + ee_Kd.cwiseProduct(m_v_error.toVector())
                    + m_wMl.actInv(m_a_ref).toVector();

            // 자코비안 의사 역행렬을 이용한 관절 가속도 분배
            // state_.ddq_des = m_J_local_.completeOrthogonalDecomposition().pseudoInverse() * (m_a_des - m_drift.toVector());

            Eigen::Matrix<double, 6, 7> J_fixed = m_J_local_;
            Eigen::Matrix<double, 6, 1> a_error = m_a_des.head<6>() - m_drift.toVector();
            state_.ddq_des = J_fixed.completeOrthogonalDecomposition().solve(a_error);

            // publish (rqt_plot으로 궤적을 확인하기 위해 꼭 필요)
            // Sine 궤적의 Z축 변화를 보기 위해 sampleEE_ 대신 m_M_ref를 벡터로 변환해서 퍼블리시
            // SE3ToVector(m_M_ref, sampleEE_);
            pos_des_pub();
            pos_cur_pub();
        }

        if (ctrl_mode_ == 10){ // impedance control //v
            if (chg_flag_){
                SE3Cubic_.stime = time_;
                SE3Cubic_.duration = 2.0;                

                //set init ee pose
                H_ee_ = robot_->position(data_, m_joint_id);
                SE3Cubic_.m_init = H_ee_;                  

                //set desired ee pose
                H_ee_ref_ = H_ee_;                

                chg_flag_ = false;                
            }

            //to make K(x-xd)=0, put xd=x 
            H_ee_ref_.translation() = robot_->position(data_, m_joint_id).translation();
            m_M_ref = H_ee_ref_;
            
            // SE3ToVector(H_ee_ref_, sampleEE_);
            // vectorToSE3(sampleEE_, m_M_ref);  //desired trajectory in SE3 [pinocchio::SE3]                                 

            SE3 oMi;
            Motion v_frame;                                                            
            robot_->framePosition(data_, m_frame_id, oMi);                //frame position in global frame
            robot_->frameVelocity(data_, m_frame_id, v_frame);            //frame velocity in local frame
            robot_->frameClassicAcceleration(data_, m_frame_id, m_drift); //m_drift in local frame which is identical to J_dot * q_dot                        
            robot_->frameJacobianLocal(data_, m_frame_id, m_J_local_);    //frame jacobian in local frame
            
            SE3ToVector(oMi, m_p_);                                       // current pos in vector form            
            errorInSE3(oMi, m_M_ref, m_p_error);                          // pos erorr represented in local frame, oMi_inv*m_M_ref                                    

            // Transformation from local to world
            m_wMl.rotation(oMi.rotation());                               // use rotation only for vel&acc transformation

            m_p_error_vec = m_p_error.toVector();                         // pos err vector in local frame            
            m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;                 // vel err vector in local frame                         

            cout << m_p_error_vec.transpose() << endl;

            // desired acc in local frame
            m_a_des =   ee_Kp.cwiseProduct(m_p_error_vec)
                        + ee_Kd.cwiseProduct(m_v_error.toVector())
                        + m_wMl.actInv(m_a_ref).toVector();                             

            //transformation from ee to joint            
            // state_.ddq_des = m_J_local_.completeOrthogonalDecomposition().pseudoInverse() * (m_a_des - m_drift.toVector());                                            

            Eigen::Matrix<double, 6, 7> J_fixed = m_J_local_;
            Eigen::Matrix<double, 6, 1> a_error = m_a_des.head<6>() - m_drift.toVector();
            state_.ddq_des = J_fixed.completeOrthogonalDecomposition().solve(a_error);
            
            //publish
            pos_des_pub();            
            pos_cur_pub();
        }

        double Kp_grip = 8000.0;
        double Kd_grip = 0.8 * sqrt(Kp_grip);
        state_.ddq_des(6) = Kp_grip * (gripper_pos_des_ - state_.q(6)) - Kd_grip * state_.v(6);
    
        ////////////////////////////////////////////////////////////////////////////////////////
        state_.tau_des = data_.M * state_.ddq_des + data_.nle;
        robot_command(); //send torque command state_.tau_des to mujoco

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        int msg = 0;
        switch (key){
            case 'h': //home joint
                ctrl_mode_ = 1;
                state_.q_goal = Home * M_PI / 180.;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Home Position" << endl;
                cout << " " << endl;
                break;
            case 'j': //a joint
                ctrl_mode_= 1;
                state_.q_goal = Home2 * M_PI / 180.;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Home2 Position" << endl;
                cout << " " << endl;
                break;

            case 'u': //a joint
                ctrl_mode_= 1;
                state_.q_goal = Home3 * M_PI / 180.;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Home3 Position" << endl;
                cout << " " << endl;
                break;
                      
            case 'x': //k ee taskk
                ctrl_mode_= 3;
                state_.task_jog_offset_ << 0.05, 0.0, 0.0;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move ee 0.05 x" << endl;
                cout << " " << endl;
                break;
            case 'X': //k ee taskk
                ctrl_mode_= 3;
                state_.task_jog_offset_ << -0.05, 0.0, 0.0;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move ee -0.05 x" << endl;
                cout << " " << endl;
                break;
            case 'y': //k ee taskk
                ctrl_mode_= 3;
                state_.task_jog_offset_ << 0.0, 0.05, 0.0;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move ee 0.05 y" << endl;
                cout << " " << endl;
                break;     
            case 'Y': //k ee taskk
                ctrl_mode_= 3;
                state_.task_jog_offset_ << 0.0, -0.05, 0.0;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move ee -0.05 y" << endl;
                cout << " " << endl;
                break;
            case 'z': //k ee taskk
                ctrl_mode_= 3;
                state_.task_jog_offset_ << 0.0, 0.0, 0.05;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move ee 0.05 z" << endl;
                cout << " " << endl;
                break;      
            case 'Z': //k ee taskk
                ctrl_mode_= 3;
                state_.task_jog_offset_ << 0.0, 0.0, -0.05;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move ee -0.05 z" << endl;
                cout << " " << endl;
                break;

            case 'a':
                ctrl_mode_= 3;
                state_.task_rot_offset_ = Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Rotate ee 5 deg about x" << endl;
                cout << " " << endl;
                break;
            case 'A':
                ctrl_mode_= 3;
                state_.task_rot_offset_ = Eigen::AngleAxisd(-5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Rotate ee -5 deg about x" << endl;
                cout << " " << endl;
                break;
            case 's':
                ctrl_mode_= 3;
                state_.task_rot_offset_ = Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Rotate ee 5 deg about y" << endl;
                cout << " " << endl;
                break;
            case 'S':
                ctrl_mode_= 3;
                state_.task_rot_offset_ = Eigen::AngleAxisd(-5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Rotate ee -5 deg about y" << endl;
                cout << " " << endl;
                break;
            case 'd':
                ctrl_mode_= 3;
                state_.task_rot_offset_ = Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Rotate ee 5 deg about z" << endl;
                cout << " " << endl;
                break;
            case 'D':
                ctrl_mode_= 3;
                state_.task_rot_offset_ = Eigen::AngleAxisd(-5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Rotate ee -5 deg about z" << endl;
                cout << " " << endl;
                break;

            case 'w':
                ctrl_mode_ = 4;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Start Sine Motion (Z-axis)" << endl;
                cout << " " << endl;
                break;

            case 'o':
                gripper_pos_des_ = 0.0;
                cout << "Gripper Open!" << endl;
                break;
            case 'c':
                gripper_pos_des_ = 2.5;
                cout << "Gripper Close!" << endl;
                break;

            case 'g': //gravity
                ctrl_mode_ = 0;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "garvity mode" << endl;
                cout << " " << endl;
                break;

            case 'i': //i ee task
                ctrl_mode_= 3;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move ee -0.05 z" << endl;
                cout << " " << endl;
                break;            
            
            case 'v': //v impedance control
                ctrl_mode_= 10;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "impedance control" << endl;
                cout << " " << endl;
                break;  

            case 'p': //pick pose
                ctrl_mode_ = 1;
                state_.q_goal = Pick * M_PI / 180.;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Pick Position (Box)" << endl;
                cout << " " << endl;
                break;

            case 'l': //place pose
                ctrl_mode_ = 1;
                state_.q_goal = Place * M_PI / 180.;
                chg_flag_ = true;
                cout << " " << endl;
                cout << "Move to Place Position" << endl;
                cout << " " << endl;
                break;
        }
    }
}

void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
  for (int i=0; i<GEN3_DOF; i++){
    state_.q(i) = msg->position[i];
    state_.v(i) = msg->velocity[i];
  }
}

void robot_command()
{
    robot_command_msg_.MODE = 1;
        robot_command_msg_.header.stamp = ros::Time::now();
    robot_command_msg_.time = time_;
    for (int i=0; i<GEN3_DOF; i++)
    {
        //robot_command_msg_.position[i] = state_.q_des(i);
        robot_command_msg_.torque[i] = state_.tau_des(i);
    }
    robot_command_pub_.publish(robot_command_msg_); 
}

void pos_des_pub()
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(sampleEE_.size());
    for(int i=0; i<sampleEE_.size(); i++){
        msg.data[i] = sampleEE_[i];
    }
    pos_des_pub_.publish(msg);
}

void pos_cur_pub()
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(m_p_.size());
    for(int i=0; i<m_p_.size(); i++){
        msg.data[i] = m_p_[i];
    }
    pos_cur_pub_.publish(msg);
}

void samApproachCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (pick_state_ != PICK_IDLE) {
        ROS_WARN("[SAM] Pick in progress (state=%d), ignoring approach point", pick_state_);
        return;
    }
    sam_approach_cam_[0] = msg->point.x;
    sam_approach_cam_[1] = msg->point.y;
    sam_approach_cam_[2] = msg->point.z;
    sam_approach_received_ = true;
    ROS_INFO("[SAM] approach (cam): x=%.3f y=%.3f z=%.3f",
             sam_approach_cam_[0], sam_approach_cam_[1], sam_approach_cam_[2]);
}

void samGraspCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (pick_state_ != PICK_IDLE) {
        ROS_WARN("[SAM] Pick in progress (state=%d), ignoring grasp point", pick_state_);
        return;
    }
    sam_grasp_cam_[0] = msg->point.x;
    sam_grasp_cam_[1] = msg->point.y;
    sam_grasp_cam_[2] = msg->point.z;
    sam_grasp_received_ = true;
    ROS_INFO("[SAM] grasp    (cam): x=%.3f y=%.3f z=%.3f",
             sam_grasp_cam_[0], sam_grasp_cam_[1], sam_grasp_cam_[2]);
}

