//
// Created by shuoy on 10/19/21.
//

#include "A1RobotControl.h"

A1RobotControl::A1RobotControl() {
    std::cout << "init A1RobotControl" << std::endl;
    // init QP solver
    // init some parameters
    Q.diagonal() << 1.0, 1.0, 1.0, 400.0, 400.0, 100.0;
    R = 1e-3;
    mu = 0.7;
    F_min = 0;
    F_max = 180;
    hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
    gradient.resize(3 * NUM_LEG);
    linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
    lowerBound.resize(NUM_LEG + 4 * NUM_LEG);
    lowerBound.setZero();
    upperBound.resize(NUM_LEG + 4 * NUM_LEG);
    upperBound.setZero();

    // init mpc skip counter
    mpc_init_counter = 0;

    // constraint matrix fixed
    for (int i = 0; i < NUM_LEG; ++i) {
        // extract F_zi
        linearMatrix.insert(i, 2 + i * 3) = 1;
        // friction pyramid
        // 1. F_xi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4, i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4) = -OsqpEigen::INFTY;
        // 2. F_xi > -uF_zi    ===> -F_xi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4 + 1, i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 1, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 1) = -OsqpEigen::INFTY;
        // 3. F_yi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 2) = -OsqpEigen::INFTY;
        // 4. -F_yi > uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 3) = -OsqpEigen::INFTY;
    }
    // debug linearMatrix
//    std::cout << Eigen::MatrixXd(linearMatrix) << std::endl;

    for (int i = 0; i < NUM_LEG; ++i) {
        recent_contact_x_filter[i] = MovingWindowFilter(60);
        recent_contact_y_filter[i] = MovingWindowFilter(60);
        recent_contact_z_filter[i] = MovingWindowFilter(60);
    }
}

A1RobotControl::A1RobotControl(ros::NodeHandle &_nh) : A1RobotControl() {
    std::cout << "init nh" << std::endl;
    nh = _nh;
    _nh.param("use_sim_time", use_sim_time);
    // initial debug publisher
    for (int i = 0; i < NUM_LEG; ++i) {
        std::string id = std::to_string(i);
        std::string start_topic = "/isaac_a1/foot" + id + "/start_pos";
        std::string end_topic = "/isaac_a1/foot" + id + "/end_pos";
        std::string path_topic = "/isaac_a1/foot" + id + "/swing_path";

        pub_foot_start[i] = nh.advertise<visualization_msgs::Marker>(start_topic, 100);
        pub_foot_end[i] = nh.advertise<visualization_msgs::Marker>(end_topic, 100);
        pub_foot_path[i] = nh.advertise<visualization_msgs::Marker>(path_topic, 100);

        // set basic info of markers
        foot_start_marker[i].header.frame_id = "a1_world";
        foot_start_marker[i].ns = "basic_shapes";
        foot_start_marker[i].id = 10 + i;
        foot_start_marker[i].type = visualization_msgs::Marker::CYLINDER;
        foot_start_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_start_marker[i].scale.x = 0.08;
        foot_start_marker[i].scale.y = 0.08;
        foot_start_marker[i].scale.z = 0.02;
        foot_start_marker[i].pose.orientation.x = 0.0;
        foot_start_marker[i].pose.orientation.y = 0.0;
        foot_start_marker[i].pose.orientation.z = 0.0;
        foot_start_marker[i].pose.orientation.w = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        foot_start_marker[i].color.r = 1.0f;
        foot_start_marker[i].color.g = 0.0f;
        foot_start_marker[i].color.b = 0.0f;
        foot_start_marker[i].color.a = 1.0;

        foot_end_marker[i].lifetime = ros::Duration();

        foot_end_marker[i].header.frame_id = "a1_world";
        foot_end_marker[i].ns = "basic_shapes";
        foot_end_marker[i].id = 20 + i;
        foot_end_marker[i].type = visualization_msgs::Marker::CYLINDER;
        foot_end_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_end_marker[i].scale.x = 0.08;
        foot_end_marker[i].scale.y = 0.08;
        foot_end_marker[i].scale.z = 0.02;
        foot_end_marker[i].pose.orientation.x = 0.0;
        foot_end_marker[i].pose.orientation.y = 0.0;
        foot_end_marker[i].pose.orientation.z = 0.0;
        foot_end_marker[i].pose.orientation.w = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        foot_end_marker[i].color.r = 0.0f;
        foot_end_marker[i].color.g = 0.0f;
        foot_end_marker[i].color.b = 1.0f;
        foot_end_marker[i].color.a = 1.0;

        foot_end_marker[i].lifetime = ros::Duration();

        foot_path_marker[i].header.frame_id = "a1_world";
        foot_path_marker[i].ns = "basic_shapes";
        foot_path_marker[i].id = 30 + i;
        foot_path_marker[i].type = visualization_msgs::Marker::LINE_STRIP;
        foot_path_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_path_marker[i].scale.x = 0.02;
        foot_path_marker[i].pose.position.x = 0.0;
        foot_path_marker[i].pose.position.y = 0.0;
        foot_path_marker[i].pose.position.z = 0.0;
        foot_path_marker[i].pose.orientation.w = 1.0;
        foot_path_marker[i].pose.orientation.x = 0.0;
        foot_path_marker[i].pose.orientation.y = 0.0;
        foot_path_marker[i].pose.orientation.z = 0.0;
        foot_path_marker[i].points.resize(10); // fix to be 10 points
        foot_path_marker[i].colors.resize(10); // fix to be 10 points
        for (int k = 0; k < 10; k++) {
            foot_path_marker[i].colors[k].r = 0.0f;
            foot_path_marker[i].colors[k].g = 1.0f;
            foot_path_marker[i].colors[k].b = 0.0f;
            foot_path_marker[i].colors[k].a = 1.0f;
        }

        foot_path_marker[i].lifetime = ros::Duration();
    }
}

void A1RobotControl::update_plan(Go1CtrlStates &state, double dt) {
    state.counter += 1;
    if (!state.movement_mode) {
        // movement_mode == 0, standstill with all feet in contact with ground
        for (bool &plan_contact: state.plan_contacts) plan_contact = true;
        state.gait_counter_reset();
    } else {
        // movement_mode == 1, walk
        for (int i = 0; i < NUM_LEG; ++i) {
            state.gait_counter(i) = state.gait_counter(i) + state.gait_counter_speed(i);
            state.gait_counter(i) = std::fmod(state.gait_counter(i), state.counter_per_gait);
            if (state.gait_counter(i) <= state.counter_per_swing) {
                state.plan_contacts[i] = true;
            } else {
                state.plan_contacts[i] = false;
            }
        }
    }

    // update foot plan: state.foot_pos_target_world
    Eigen::Vector3d lin_vel_world = state.root_lin_vel; // world frame linear velocity
    Eigen::Vector3d lin_vel_rel = state.root_rot_mat_z.transpose() * lin_vel_world; // robot body frame linear velocity

    // Raibert Heuristic, calculate foothold position
    state.foot_pos_target_rel = state.default_foot_pos;
    for (int i = 0; i < NUM_LEG; ++i) {
        double delta_x =
                std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(0) - state.root_lin_vel_d(0)) +
                ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(0);
        double delta_y =
                std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(1) - state.root_lin_vel_d(1)) +
                ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(1);

        if (delta_x < -FOOT_DELTA_X_LIMIT) {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        if (delta_x > FOOT_DELTA_X_LIMIT) {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT) {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        if (delta_y > FOOT_DELTA_Y_LIMIT) {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }

        state.foot_pos_target_rel(0, i) += delta_x;
        state.foot_pos_target_rel(1, i) += delta_y;

        state.foot_pos_target_abs.block<3, 1>(0, i) = state.root_rot_mat * state.foot_pos_target_rel.block<3, 1>(0, i);
        state.foot_pos_target_world.block<3, 1>(0, i) = state.foot_pos_target_abs.block<3, 1>(0, i) + state.root_pos;
    }
}

void A1RobotControl::generate_swing_legs_ctrl(Go1CtrlStates &state, double dt) {
    state.joint_torques.setZero();

    // get current foot pos and target foot pose
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;
    spline_time.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    foot_pos_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    foot_vel_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

    // the foot force of swing foot and stance foot, both are in robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;

    for (int i = 0; i < NUM_LEG; ++i) {
        foot_pos_cur.block<3, 1>(0, i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3, 1>(0, i);

        // from foot_pos_cur to foot_pos_final computes an intermediate point using BezierUtils
        if (state.gait_counter(i) <= state.counter_per_swing) {
            // stance foot
            spline_time(i) = 0.0;
            // in this case the foot should be stance
            // keep refreshing foot_pos_start in stance mode
            state.foot_pos_start.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);
        } else {
            // in this case the foot should be swing
            spline_time(i) = float(state.gait_counter(i) - state.counter_per_swing) / float(state.counter_per_swing);
        }

        foot_pos_target.block<3, 1>(0, i) = bezierUtils[i].get_foot_pos_curve(spline_time(i),
                                                                              state.foot_pos_start.block<3, 1>(0, i),
                                                                              state.foot_pos_target_rel.block<3, 1>(0, i),
                                                                              0.0);

        foot_vel_cur.block<3, 1>(0, i) = (foot_pos_cur.block<3, 1>(0, i) - state.foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

        foot_vel_target.block<3, 1>(0, i) = (foot_pos_target.block<3, 1>(0, i) - state.foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

        foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
        foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
        foot_forces_kin.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                                            foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i));
    }
    state.foot_pos_cur = foot_pos_cur;

    // detect early contact
    bool last_contacts[NUM_LEG];

    for (int i = 0; i < NUM_LEG; ++i) {
        if (state.gait_counter(i) <= state.counter_per_swing * 1.5) {
            state.early_contacts[i] = false;
        }
        if (!state.plan_contacts[i] &&
            (state.gait_counter(i) > state.counter_per_swing * 1.5) &&
            (state.foot_force(i) > FOOT_FORCE_LOW)) {
            state.early_contacts[i] = true;
        }

        // actual contact
        last_contacts[i] = state.contacts[i];
        state.contacts[i] = state.plan_contacts[i] || state.early_contacts[i];

        // record recent contact position if the foot is in touch with the ground
        if (state.contacts[i]) {
//            state.foot_pos_recent_contact.block<3, 1>(0, i) = state.root_rot_mat.transpose() * (state.foot_pos_world.block<3, 1>(0, i));
//            state.foot_pos_recent_contact.block<3, 1>(0, i) = state.foot_pos_abs.block<3, 1>(0, i);
            state.foot_pos_recent_contact.block<3, 1>(0, i)
                    << recent_contact_x_filter[i].CalculateAverage(state.foot_pos_abs(0, i)),
                    recent_contact_y_filter[i].CalculateAverage(state.foot_pos_abs(1, i)),
                    recent_contact_z_filter[i].CalculateAverage(state.foot_pos_abs(2, i));
        }
    }

    std::cout << "foot_pos_recent_contact z: " << state.foot_pos_recent_contact.block<1, 4>(2, 0) << std::endl;

    state.foot_forces_kin = foot_forces_kin;
}

void A1RobotControl::compute_joint_torques(Go1CtrlStates &state) {
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    joint_torques.setZero();
    mpc_init_counter++;
    // for the first 10 ticks, just return zero torques.
    if (mpc_init_counter < 10) {
        state.joint_torques = joint_torques;
    } else {
        // for each leg, if it is a swing leg (contact[i] is false), use foot_force_kin to get joint_torque
        // for each leg, if it is a stance leg (contact[i] is true), use foot_forces_grf to get joint_torque
        for (int i = 0; i < NUM_LEG; ++i) {
            Eigen::Matrix3d jac = state.j_foot.block<3, 3>(3 * i, 3 * i);
            if (state.contacts[i]) {
                // stance leg
                joint_torques.segment<3>(i * 3) = jac.transpose() * -state.foot_forces_grf.block<3, 1>(0, i);
            } else {
                // swing leg
                Eigen::Vector3d force_tgt = state.km_foot.cwiseProduct(state.foot_forces_kin.block<3, 1>(0, i));
                joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt);   // jac * tau = F
            }
        }
        // gravity compensation
        joint_torques += state.torques_gravity;

        // prevent nan
        for (int i = 0; i < 12; ++i) {
            if (!isnan(joint_torques[i]))
                state.joint_torques[i] = joint_torques[i];
        }
    }
}

Eigen::Matrix<double, 3, NUM_LEG> A1RobotControl::compute_grf(Go1CtrlStates &state, double dt) {
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    // first get parameters needed to construct the solver hessian and gradient
    // use euler angle to get desired angle
    Eigen::Vector3d euler_error = state.root_euler_d - state.root_euler;

    // limit euler error to pi/2
    if (euler_error(2) > 3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) - 3.1415926 * 2 - state.root_euler(2);
    } else if (euler_error(2) < -3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) + 3.1415926 * 2 - state.root_euler(2);
    }

    // desired acc in world frame
    root_acc.setZero();
    root_acc.block<3, 1>(0, 0) = state.kp_linear.cwiseProduct(state.root_pos_d - state.root_pos);

    root_acc.block<3, 1>(0, 0) += state.root_rot_mat * state.kd_linear.cwiseProduct(
            state.root_lin_vel_d - state.root_rot_mat.transpose() * state.root_lin_vel);

    root_acc.block<3, 1>(3, 0) = state.kp_angular.cwiseProduct(euler_error);

    root_acc.block<3, 1>(3, 0) += state.kd_angular.cwiseProduct(
            state.root_ang_vel_d - state.root_rot_mat.transpose() * state.root_ang_vel);

    // add gravity
    root_acc(2) += state.robot_mass * 9.8;

    // Create inverse inertia matrix
    Eigen::Matrix<double, 6, DIM_GRF> inertia_inv;
    for (int i = 0; i < NUM_LEG; ++i) {
        inertia_inv.block<3, 3>(0, i * 3).setIdentity();
        // TODO: confirm this should be root_rot_mat instead of root_rot_mat
        inertia_inv.block<3, 3>(3, i * 3) = state.root_rot_mat_z.transpose() * Utils::skew(state.foot_pos_abs.block<3, 1>(0, i));
    }
    Eigen::Matrix<double, DIM_GRF, DIM_GRF> dense_hessian;
    dense_hessian.setIdentity();
    dense_hessian *= R;
    dense_hessian += inertia_inv.transpose() * Q * inertia_inv;
    hessian = dense_hessian.sparseView();
    // accidentally wrote this as -2* before. Huge problem
    gradient.block<3 * NUM_LEG, 1>(0, 0) = -inertia_inv.transpose() * Q * root_acc;

    // adjust bounds according to contact flag
    for (int i = 0; i < NUM_LEG; ++i) {
        double c_flag = state.contacts[i] ? 1.0 : 0.0;
        lowerBound(i) = c_flag * F_min;
        upperBound(i) = c_flag * F_max;
    }

    // instantiate the solver
    OsqpEigen::Solver solver;
    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(3 * NUM_LEG);
    solver.data()->setNumberOfConstraints(NUM_LEG + 4 * NUM_LEG);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    auto t1 = std::chrono::high_resolution_clock::now();
    solver.initSolver();
    auto t2 = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto t3 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
    std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;

    std::cout << "qp solver init time: " << ms_double_1.count() << "ms; solve time: " << ms_double_2.count() << "ms" << std::endl;

    Eigen::VectorXd QPSolution = solver.getSolution(); //12x1
    for (int i = 0; i < NUM_LEG; ++i) {
        // the QP solves for world frame force
        // here we convert the force into robot frame
        foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * QPSolution.segment<3>(i * 3);
    }

    return foot_forces_grf;
}

