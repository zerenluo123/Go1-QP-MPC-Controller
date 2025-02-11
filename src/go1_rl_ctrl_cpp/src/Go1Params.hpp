//
// Created by shuoy on 10/18/21.
//

#ifndef GO1_CPP_A1PARAMS_H
#define GO1_CPP_A1PARAMS_H

// control time related
//#define CTRL_FREQUENCY 2.5  // ms
#define ACTION_UPDATE_FREQUENCY 0.5 // ms
#define DEPLOYMENT_FREQUENCY 0.5 // ms
#define HARDWARE_FEEDBACK_FREQUENCY 0.5  // ms

// constant define
// joy stick command interprate
#define JOY_CMD_BODY_HEIGHT_MAX 0.32     // m
#define JOY_CMD_BODY_HEIGHT_MIN 0.1     // m
#define JOY_CMD_BODY_HEIGHT_VEL 0.04    // m/s
#define JOY_CMD_VELX_MAX 0.6         // m/s
#define JOY_CMD_VELY_MAX 0.6           // m/s
#define JOY_CMD_YAW_MAX 0.7             // rad
#define JOY_CMD_PITCH_MAX 0.4           // rad
#define JOY_CMD_ROLL_MAX 0.4            // rad

// mpc
#define PLAN_HORIZON 10
#define MPC_STATE_DIM 13
#define MPC_CONSTRAINT_DIM 20

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

#define LOWER_LEG_LENGTH 0.21

#define FOOT_FORCE_LOW 30.0
#define FOOT_FORCE_HIGH 80.0

#define FOOT_SWING_CLEARANCE1 0.0f
#define FOOT_SWING_CLEARANCE2 0.4f

#define FOOT_DELTA_X_LIMIT 0.1
#define FOOT_DELTA_Y_LIMIT 0.1

#define ERROR_CURVE_ALREADY_SET 184
#define ERROR_CURVE_NOT_SET 185

#endif //GO1_CPP_A1PARAMS_H
