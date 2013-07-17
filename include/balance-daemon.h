
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: May 30, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef BALANCE_DAEMON_H
#define BALANCE_DAEMON_H

#include <hubo.h>

#define BALANCE_CMD_CHAN "balance-cmd"
#define BALANCE_STATE_CHAN "balance-state"
#define BALANCE_PARAM_CHAN "balance-param"

typedef enum {

    BAL_READY=0,
    BAL_LEGS_ONLY,
    BAL_ZMP_WALKING
    
/*
    STATE_INVALID,
    S_HORSE,
    S_CRANE,
    Q_SHIFTDIST,
    Q_LIFTLEG,
    Q_CROUCH
*/

} balance_mode_t;

typedef enum {

    WALK_INACTIVE=0,
    WALK_WAITING,
    WALK_INITIALIZING,
    WALK_IN_PROGRESS

} walk_mode_t;

typedef enum {

    NO_WALK_ERROR=0,
    WALK_INIT_FAILED,
    WALK_FAILED_SWAP

} walk_error_t;


typedef struct balance_gains {

    double flattening_gain[2];
    double decay_gain[2];
    double force_min_threshold[2];
    double force_max_threshold[2];

    double straightening_pitch_gain[2];
    double straightening_roll_gain[2];
    double single_support_straightening_gain; 
    double double_support_straightening_gain; 

    double spring_gain[2];
    double damping_gain[2];
    double fz_response[2];

    double single_support_hip_nudge_kp;
    double single_support_hip_nudge_kd;
    double double_support_hip_nudge_kp;
    double double_support_hip_nudge_kd;

} balance_gains_t;




typedef struct balance_cmd {

    balance_mode_t cmd_request;
    
    double height;

} balance_cmd_t;

typedef struct balance_state {

    balance_mode_t m_balance_mode;

    walk_mode_t m_walk_mode;
    walk_error_t m_walk_error;

    double jointOffset[HUBO_JOINT_COUNT];

} balance_state_t;


typedef enum {
    
    T_INVALID,
    T_INCOMPLETE,
    T_COMPLETE

} transition_result_t;


#endif // BALANCE_DAEMON_H

