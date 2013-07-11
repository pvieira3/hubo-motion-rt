
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


#include <Hubo_Control.h>
#include "Walker.h"
#include "balance-daemon.h"

/**
 * \brief Set value to zero if within the deadband defined by
 * the interval -tau_dead_band to tau_dead_band, otherwise
 * take the value that x exceeds either end of the deadband.
 * \param x Number to apply deadband to.
 * \return Number after deadband is applied.
*/
/*double applyDeadband( double x )
{
  if (x > tau_dead_band)
    return x - tau_dead_band;
  else if (x < -tau_dead_band)
    return x + tau_dead_band;
  else
    return 0;
}*/

/**
 * \brief Clamp the input value to +/- some value
 * defined by 'cap'.
 * \param x Value to be capped
 * \param cap Value to keep x within +/- itself
 * \return void
*/
static inline void clamp(double& x, double cap) {
  x = std::max(-cap, std::min(x, cap));
}
/*
void Walker::legWorkspaceControl( Hubo_Control &hubo, zmp_traj_element_t &elem,
            nudge_state_t &state, blaance_gains_t &gains, double dt, COMPLY_KNEE )
{
    // Store leg joint angels for current trajectory timestep
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > qPrev(2);
    qPrev[LEFT](HY) = elem.angles[LHY],
    qPrev[LEFT](HR) = elem.angles[LHR],
    qPrev[LEFT](HP) = elem.angles[LHP],
    qPrev[LEFT](KN) = elem.angles[LKN],
    qPrev[LEFT](AP) = elem.angles[LAP],
    qPrev[LEFT](AR) = elem.angles[LAR];

    qPrev[RIGHT](HY) = elem.angles[RHY],
    qPrev[RIGHT](HR) = elem.angles[RHR],
    qPrev[RIGHT](HP) = elem.angles[RHP],
    qPrev[RIGHT](KN) = elem.angles[RKN],
    qPrev[RIGHT](AP) = elem.angles[RAP],
    qPrev[RIGHT](AR) = elem.angles[RAR];

    // TF for body to each foot
    std::vector< Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > footTF(2);
    // New joint angles for both legs
    std::vector< Vector6d, Eigen::aligned_allocator<Vector6d> > qNew(2);
    // Ankle torque error XYZ (ie. Roll/Pitch/Yaw), but just setting Z to zero.

    // Get TFs for feet
    hubo.huboLegFK( footTF[LEFT], qPrev[LEFT], LEFT ); 
    hubo.huboLegFK( footTF[RIGHT], qPrev[RIGHT], RIGHT );

    switch(CONTROLLER)
    {
        case COMPLY_KNEE:
            complyKnee();
            break;
        case NUDGE_HIPS:
            nudgeHips();
            break;
    }

    // Run IK on the adjusted feet TF to get new joint angles
    hubo.huboLegIK(qNew[LEFT], footTF[LEFT], qPrev[LEFT], LEFT);
    hubo.huboLegIK(qNew[RIGHT], footTF[RIGHT], qPrev[RIGHT], RIGHT);

    // Prevent giant joint angle changes
    bool ok = true;
    double jointTol = 0.01; // radians

    for(int i=0; i<2; i++)
    {
        std::string s = i==LEFT ? "LEFT" : "RIGHT";
        for(int j=0; j<6; j++)
        {
            double qDiff = qNew[i](j) - qPrev[i](j);
            if(fabs(qDiff) > jointTol)
            {
                // Cap joint change
                qNew[i](j) = qDiff > 0 ? qPrev[i](j) + jointTol : qPrev[i](j) - jointTol;
                if(debug)
                {
                    std::cout << "Change in joint " << j << " of " << s << " side" << " = " << qDiff
                              << " , which is greater than Joint Tolerance of " << jointTol
                              << "\n";
                }
            }
        }
    }

    // Set leg joint angles for current timestep of trajectory
    if(ok)
    {
        elem.angles[LHY] = qNew[LEFT](HY);
        elem.angles[LHR] = qNew[LEFT](HR);
        elem.angles[LHP] = qNew[LEFT](HP);
        elem.angles[LKN] = qNew[LEFT](KN);
        elem.angles[LAP] = qNew[LEFT](AP);
        elem.angles[LAR] = qNew[LEFT](AR);

        elem.angles[RHY] = qNew[RIGHT](HY);
        elem.angles[RHR] = qNew[RIGHT](HR);
        elem.angles[RHP] = qNew[RIGHT](HP);
        elem.angles[RKN] = qNew[RIGHT](KN);
        elem.angles[RAP] = qNew[RIGHT](AP);
        elem.angles[RAR] = qNew[RIGHT](AR);
    }


}*/

void Walker::nudgeHips( Hubo_Control &hubo, zmp_traj_element_t &elem,
            nudge_state_t &state, balance_gains_t &gains, double dt )
{
    bool debug = true;
    // Figure out if we're in single or double support stance and which leg
    double kP, kD;
    double side;
    switch(elem.stance)
    {
        case SINGLE_LEFT:
            side = LEFT;
            kP = gains.single_support_hip_nudge_kp;
            kD = gains.single_support_hip_nudge_kd;
            break;
        case SINGLE_RIGHT:
            side = RIGHT;
            kP = gains.single_support_hip_nudge_kp;
            kD = gains.single_support_hip_nudge_kd;
            break;
        case DOUBLE_LEFT:
        case DOUBLE_RIGHT:
            side = 100;
            kP = gains.double_support_hip_nudge_kp;
            kD = gains.double_support_hip_nudge_kd;
            break;
        default:
            return;
    }
    if(debug)
    {
        //std::cout << "kP, kD: " << kP << ", " << kD << "\n";
    }
    // Store leg joint angels for current trajectory timestep
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > qPrev(2);
    qPrev[LEFT](HY) = elem.angles[LHY],
    qPrev[LEFT](HR) = elem.angles[LHR],
    qPrev[LEFT](HP) = elem.angles[LHP],
    qPrev[LEFT](KN) = elem.angles[LKN],
    qPrev[LEFT](AP) = elem.angles[LAP],
    qPrev[LEFT](AR) = elem.angles[LAR];

    qPrev[RIGHT](HY) = elem.angles[RHY],
    qPrev[RIGHT](HR) = elem.angles[RHR],
    qPrev[RIGHT](HP) = elem.angles[RHP],
    qPrev[RIGHT](KN) = elem.angles[RKN],
    qPrev[RIGHT](AP) = elem.angles[RAP],
    qPrev[RIGHT](AR) = elem.angles[RAR];

    // Skew matrix for torque reaction logic
    Eigen::Matrix3d skew; 
//    skew << 0, 1, 0,
//           -1, 0, 0,
//            0, 0, 0;
    //FIXME The version below is opposite b/c hubomz computes reaction torque at ankle
    // instead of torque at F/T sensor
    skew << 0, -1, 0,
            1, 0, 0,
            0, 0, 0;

    // Gain matrix for ankle roll and pitch
    Eigen::Matrix3d shiftGains;
    shiftGains << dt*kP,       0, 0,
                      0,   dt*kP, 0,
                      0,       0, 0;

    // Get rotation matrix for each hip yaw
    std::vector< Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > yawRot(2);
    yawRot[LEFT] = Eigen::AngleAxisd(hubo.getJointAngle(LHY), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    yawRot[RIGHT]= Eigen::AngleAxisd(hubo.getJointAngle(RHY), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // TF for body to each foot
    std::vector< Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > footTF(2);
    // New joint angles for both legs
    std::vector< Vector6d, Eigen::aligned_allocator<Vector6d> > qNew(2);
    // Ankle torque error XYZ (ie. Roll/Pitch/Yaw), but just setting Z to zero.
    Vector3d torqueErr;

    // Determine how much we need to nudge to hips over to account for
    // error in ankle torques about the x- and y- axes.
    // If Roll torque is positive (ie. leaning left) we want hips to go right (ie. negative y-direction)
    // If Pitch torque is positive (ie. leaning back) we want hips to go forward (ie. positive x-direction)
    // Get TFs for feet
    hubo.huboLegFK( footTF[LEFT], qPrev[LEFT], LEFT ); 
    hubo.huboLegFK( footTF[RIGHT], qPrev[RIGHT], RIGHT );
    // Averaged torque error in ankles (roll and pitch) (yaw is always zero)
    if(side != LEFT && side != RIGHT)
    {
        torqueErr(0) = ((elem.torque[LEFT][0] - hubo.getLeftFootMx()) + (elem.torque[RIGHT][0] - hubo.getRightFootMx()))/2;
        torqueErr(1) = ((elem.torque[LEFT][1] - hubo.getLeftFootMy()) + (elem.torque[RIGHT][1]) - hubo.getRightFootMy())/2;
    }
    else
    {
        torqueErr(0) = side = LEFT ? (elem.torque[LEFT][0] - hubo.getLeftFootMx()) : (elem.torque[RIGHT][0] - hubo.getRightFootMx());
        torqueErr(1) = side = LEFT ? (elem.torque[LEFT][1] - hubo.getLeftFootMy()) : (elem.torque[RIGHT][1] - hubo.getRightFootMy());
    }
    torqueErr(2) = 0;
    // Feet position errors (x,y)
    Vector3d footErr = shiftGains * skew * torqueErr;
    // Rotate by hip yaws and then translate by footErr to get body
    // translation for both feet and average them to get total body translation.
    if(side != LEFT && side != RIGHT)
        state.bodyErr += ((yawRot[LEFT] * footErr) + (yawRot[RIGHT] * footErr)) / 2;
    else
        state.bodyErr += yawRot[side] * footErr;

    const double bodyErrTol = 0.02;
    double n = state.bodyErr.norm();
    if (n > bodyErrTol) {
      state.bodyErr *= bodyErrTol/n;
    }

    // Pretranslate feet TF by body error translation vector
    footTF[LEFT].pretranslate(state.bodyErr);
    footTF[RIGHT].pretranslate(state.bodyErr);
    // Run IK on the adjusted feet TF to get new joint angles
    hubo.huboLegIK(qNew[LEFT], footTF[LEFT], qPrev[LEFT], LEFT);
    hubo.huboLegIK(qNew[RIGHT], footTF[RIGHT], qPrev[RIGHT], RIGHT);

    if(debug)
    {
        std::cout << " K: " << kP
                  << " TdL: " << elem.torque[LEFT][0] << ", " << elem.torque[LEFT][1]
                  << " TdR: " << elem.torque[RIGHT][0] << ", " << elem.torque[RIGHT][1]
                  << " MyLR: " << hubo.getLeftFootMy() << ", " << hubo.getRightFootMy()
                  << " Te: " << torqueErr.transpose()
                  //<< " Fte: " << footErr.transpose()
                  //<< " qDfL: " << (qNew[LEFT] - qPrev[LEFT]).transpose()
                  //<< " qDfR: " << (qNew[RIGHT] - qPrev[RIGHT]).transpose()
                  << " bE: " << state.bodyErr.transpose()
                  << "\n";
    }

    bool ok = false;

    // Set leg joint angles for current timestep of trajectory
    if(ok)
    {
        elem.angles[LHY] = qNew[LEFT](HY);
        elem.angles[LHR] = qNew[LEFT](HR);
        elem.angles[LHP] = qNew[LEFT](HP);
        elem.angles[LKN] = qNew[LEFT](KN);
        elem.angles[LAP] = qNew[LEFT](AP);
        elem.angles[LAR] = qNew[LEFT](AR);

        elem.angles[RHY] = qNew[RIGHT](HY);
        elem.angles[RHR] = qNew[RIGHT](HR);
        elem.angles[RHP] = qNew[RIGHT](HP);
        elem.angles[RKN] = qNew[RIGHT](KN);
        elem.angles[RAP] = qNew[RIGHT](AP);
        elem.angles[RAR] = qNew[RIGHT](AR);
    }
}

void Walker::flattenFoot( Hubo_Control &hubo, zmp_traj_element_t &elem,
			nudge_state_t &state, balance_gains_t &gains, double dt )
{
     
    state.ankle_roll_compliance[LEFT] -= gains.decay_gain[LEFT]*state.ankle_roll_compliance[LEFT];
    state.ankle_roll_compliance[RIGHT] -= gains.decay_gain[RIGHT]*state.ankle_roll_compliance[RIGHT];

    state.ankle_pitch_compliance[LEFT] -= gains.decay_gain[LEFT]*state.ankle_pitch_compliance[LEFT];
    state.ankle_pitch_compliance[RIGHT] -= gains.decay_gain[RIGHT]*state.ankle_pitch_compliance[RIGHT];

    if( gains.force_min_threshold[RIGHT] < hubo.getRightFootFz() 
     && hubo.getRightFootFz() < gains.force_max_threshold[RIGHT] )
    {
        state.ankle_roll_compliance[RIGHT] += dt*gains.flattening_gain[RIGHT]
                                                *( hubo.getRightFootMx() );
        state.ankle_pitch_compliance[RIGHT] += dt*gains.flattening_gain[RIGHT]
                                                 *( hubo.getRightFootMy() );
    }

    if( gains.force_min_threshold[LEFT] < hubo.getLeftFootFz()
     && hubo.getLeftFootFz() < gains.force_max_threshold[LEFT] )
    {
        state.ankle_roll_compliance[LEFT] += dt*gains.flattening_gain[LEFT]
                                               *( hubo.getLeftFootMx() );
        state.ankle_pitch_compliance[LEFT] += dt*gains.flattening_gain[LEFT]
                                                *( hubo.getLeftFootMy() );
    }


    elem.angles[RAR] += state.ankle_roll_compliance[RIGHT];
    elem.angles[RAP] += state.ankle_pitch_compliance[RIGHT];
    elem.angles[LAR] += state.ankle_roll_compliance[LEFT];
    elem.angles[LAP] += state.ankle_pitch_compliance[LEFT];

}

void Walker::straightenBack( Hubo_Control &hubo, zmp_traj_element_t &elem,
        nudge_state_t &state, balance_gains_t &gains, double dt )
{
    if( elem.stance == SINGLE_LEFT )
    {
        state.ankle_pitch_resistance[LEFT] += dt*gains.straightening_pitch_gain[LEFT]
                                                *( hubo.getAngleY() );
        state.ankle_roll_resistance[LEFT]  += dt*gains.straightening_roll_gain[LEFT]
                                                *( hubo.getAngleX() );
    }

    if( elem.stance == SINGLE_RIGHT )
    {
        state.ankle_pitch_resistance[RIGHT] += dt*gains.straightening_pitch_gain[RIGHT]
                                                 *( hubo.getAngleY() );
        state.ankle_roll_resistance[RIGHT]  += dt*gains.straightening_roll_gain[RIGHT]
                                                 *( hubo.getAngleX() );
    }
    
    elem.angles[RAR] += state.ankle_roll_resistance[RIGHT];
    elem.angles[RAP] += state.ankle_pitch_resistance[RIGHT];
    elem.angles[LAR] += state.ankle_roll_resistance[LEFT];
    elem.angles[LAP] += state.ankle_pitch_resistance[LEFT];

}
/*
void Walker::complyKnee( Hubo_Control &hubo, Vector6d &angles,
        nudge_state_t &state, balance_gains_t &gains, double dt )
{
    if(elem.stance == SINGLE_RIGHT)
    {
        state.bodyErr(2) =
                       gains.spring_gain[LEFT]*( angles[LKN] - hubo.getJointAngle(LKN) )
                     + gains.damping_gain[LEFT]*( -state.knee_velocity_offset[LEFT] )
                     + gains.fz_response[LEFT]*( hubo.getLeftFootFz() );
    }
    else if(elem.stance == SINGLE_LEFT)
    {
        state.bodyErr(2) =
                       gains.spring_gain[RIGHT]*( angles[RKN] - hubo.getJointAngle(RKN) )
                     + gains.damping_gain[RIGHT]*( -state.knee_velocity_offset[RIGHT] )
                     + gains.fz_response[RIGHT]*( hubo.getRightFootFz() );
    }
}
*/
Walker::Walker(double maxInitTime, double jointSpaceTolerance, double jointVelContinuityTolerance) :
        m_maxInitTime(maxInitTime),
        m_jointSpaceTolerance( jointSpaceTolerance ),
        m_jointVelContTol( jointVelContinuityTolerance ),
        keepWalking(true),
        hubo()
{
    ach_status_t r = ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                HUBO_CHAN_ZMP_TRAJ_NAME, ach_result_to_string(r), (int)r );
    
    r = ach_open( &param_chan, BALANCE_PARAM_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_PARAM_CHAN, ach_result_to_string(r), (int)r );

    r = ach_open( &bal_cmd_chan, BALANCE_CMD_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_CMD_CHAN, ach_result_to_string(r), (int)r );

    r = ach_open( &bal_state_chan, BALANCE_STATE_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_STATE_CHAN, ach_result_to_string(r), (int)r );

    memset( &cmd, 0, sizeof(cmd) );
    memset( &bal_state, 0, sizeof(bal_state) );
} 

Walker::~Walker()
{
    ach_close( &zmp_chan );
    ach_close( &param_chan );
    ach_close( &bal_cmd_chan );
    ach_close( &bal_state_chan );
}

void Walker::commenceWalking(balance_state_t &parent_state, nudge_state_t &state, balance_gains_t &gains)
{
    int timeIndex=0, nextTimeIndex=0, prevTimeIndex=0;
    keepWalking = true;
    size_t fs;
 
    zmp_traj_t *prevTrajectory, *currentTrajectory, *nextTrajectory;
    prevTrajectory = new zmp_traj_t;
    currentTrajectory = new zmp_traj_t;
    nextTrajectory = new zmp_traj_t;

    memset( prevTrajectory, 0, sizeof(*prevTrajectory) );
    memset( currentTrajectory, 0, sizeof(*currentTrajectory) );
    memset( nextTrajectory, 0, sizeof(*nextTrajectory) );
    
    // TODO: Consider making these values persistent
    memset( &state, 0, sizeof(state) );


    memcpy( &bal_state, &parent_state, sizeof(bal_state) );

    bal_state.m_balance_mode = BAL_ZMP_WALKING; 
    bal_state.m_walk_mode = WALK_WAITING;
    bal_state.m_walk_error = NO_WALK_ERROR;
    sendState();

    currentTrajectory->reuse = true;
    fprintf(stdout, "Waiting for first trajectory\n"); fflush(stdout);
    ach_status_t r;
    do {
        struct timespec t;
        clock_gettime( ACH_DEFAULT_CLOCK, &t );
        t.tv_sec += 1;
        r = ach_get( &zmp_chan, currentTrajectory, sizeof(*currentTrajectory), &fs,
                    &t, ACH_O_WAIT | ACH_O_LAST );

        checkCommands();
        if( cmd.cmd_request != BAL_ZMP_WALKING )
            keepWalking = false;
    } while(!daemon_sig_quit && keepWalking && (r==ACH_TIMEOUT
                || !currentTrajectory->reuse) ); // TODO: Replace this with something more intelligent

    if(!keepWalking || !currentTrajectory->reuse) // TODO: Take out the reuse condition here
    {
        bal_state.m_walk_mode = WALK_INACTIVE;
        sendState();
        return;
    }

    if(!daemon_sig_quit)
        fprintf(stdout, "First trajectory acquired\n");
    
        
    daemon_assert( !daemon_sig_quit, __LINE__ );

    bal_state.m_walk_mode = WALK_INITIALIZING;
    sendState();

    // Get the balancing gains from the ach channel
    ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

    hubo.update(true);

    // Set all the joints to the initial posiiton in the trajectory
    // using the control daemon to interpolate in joint space.
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        // Don't worry about where these joint are
        if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
         && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
         && NK1!=i && NK2!=i && NKY!=i && LWR!=i && RWR!=i && RWY!=i && RWP!=i) //FIXME
        {
            hubo.setJointAngle( i, currentTrajectory->traj[0].angles[i] + bal_state.jointOffset[i] );
            hubo.setJointNominalSpeed( i, 0.4 );
            hubo.setJointNominalAcceleration( i, 0.4 );
        }
        //std::cout << jointNames[i] << " = " << currentTrajectory->traj[0].angles[i] + bal_state.jointOffset[i] << "\n";
    }
    
    hubo.setJointNominalSpeed( RKN, 0.8 );
    hubo.setJointNominalAcceleration( RKN, 0.8 );
    hubo.setJointNominalSpeed( LKN, 0.8 );
    hubo.setJointNominalAcceleration( LKN, 0.8 );

//    hubo.setJointAngle( RSR, currentTrajectory->traj[0].angles[RSR] + hubo.getJointAngleMax(RSR) );
//    hubo.setJointAngle( LSR, currentTrajectory->traj[0].angles[LSR] + hubo.getJointAngleMin(LSR) );

    hubo.sendControls();

    // Wait specified time for joints to get into initial configuration,
    // otherwise time out and alert user.
    double m_maxInitTime = 10;
    double biggestErr = 0;
    int worstJoint=-1;

    double dt, time, stime; stime=hubo.getTime(); time=hubo.getTime();
    double norm = m_jointSpaceTolerance+1; // make sure this fails initially
    while( !daemon_sig_quit && (norm > m_jointSpaceTolerance && time-stime < m_maxInitTime)) {
//    while(false) { // FIXME TODO: SWITCH THIS BACK!!!
        hubo.update(true);
        norm = 0;
        for(int i=0; i<HUBO_JOINT_COUNT; i++)
        {
            double err=0;
            // Don't worry about waiting for these joints to get into position.
            if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
             && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
             && NK1!=i && NK2!=i && NKY!=i && LWR!=i && RWR!=i && RWY!=i && RWP!=i) //FIXME
                err = (hubo.getJointAngleState( i )-currentTrajectory->traj[0].angles[i] + bal_state.jointOffset[i]);
//            if( LSR == i )
//                err -= hubo.getJointAngleMin(i);
//            if( RSR == i )
//                err -= hubo.getJointAngleMax(i);

            norm += err*err;
            if( fabs(err) > fabs(biggestErr) )
            {
                biggestErr = err;
                worstJoint = i;
            }
        }
        time = hubo.getTime();
    }
    // Print timeout error if joints don't get to initial positions in time
    if( time-stime >= m_maxInitTime )
    {
        fprintf(stderr, "Warning: could not reach the starting Trajectory within %f seconds\n"
                        " -- Biggest error was %f radians in joint %s\n",
                        m_maxInitTime, biggestErr, jointNames[worstJoint] );

        keepWalking = false;
        
        bal_state.m_walk_error = WALK_INIT_FAILED;
    }

    timeIndex = 1;
    bool haveNewTrajectory = false;
    if( keepWalking )
        fprintf(stdout, "Beginning main walking loop\n"); fflush(stdout);
    while(keepWalking && !daemon_sig_quit)
    {
        haveNewTrajectory = checkForNewTrajectory(*nextTrajectory, haveNewTrajectory);
        ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );
        hubo.update(true);

        bal_state.m_walk_mode = WALK_IN_PROGRESS;

        dt = hubo.getTime() - time;
        time = hubo.getTime();
        if( dt <= 0 )
        {
            fprintf(stderr, "Something unnatural has happened... %f\n", dt);
            continue;
        }

        if( timeIndex==0 )
        {
            bal_state.m_walk_error = NO_WALK_ERROR;
            nextTimeIndex = timeIndex+1;
            executeTimeStep( hubo, prevTrajectory->traj[prevTimeIndex],
                                   currentTrajectory->traj[timeIndex],
                                   currentTrajectory->traj[nextTimeIndex],
                                   state, gains, dt );
            
        }
        else if( timeIndex == currentTrajectory->periodEndTick && haveNewTrajectory )
        {
            if( validateNextTrajectory( currentTrajectory->traj[timeIndex],
                                        nextTrajectory->traj[0], dt ) )
            {
                nextTimeIndex = 0;
                executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                       currentTrajectory->traj[timeIndex],
                                       nextTrajectory->traj[nextTimeIndex],
                                       state, gains, dt );
                
                memcpy( prevTrajectory, currentTrajectory, sizeof(*prevTrajectory) );
                memcpy( currentTrajectory, nextTrajectory, sizeof(*nextTrajectory) );
                fprintf(stderr, "Notice: Swapping in new trajectory\n");
            }
            else
            {
                fprintf(stderr, "WARNING: Discontinuous trajectory passed in. Walking to a stop.\n");
                bal_state.m_walk_error = WALK_FAILED_SWAP;

                nextTimeIndex = timeIndex+1;
                executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                       currentTrajectory->traj[timeIndex],
                                       currentTrajectory->traj[nextTimeIndex],
                                       state, gains, dt );
            }
            haveNewTrajectory = false;
        }
        else if( timeIndex == currentTrajectory->periodEndTick && currentTrajectory->reuse )
        {
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                currentTrajectory->reuse = false;

            if( currentTrajectory->reuse == true )
                nextTimeIndex = currentTrajectory->periodStartTick;
            else
                nextTimeIndex = timeIndex+1;

            executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                   currentTrajectory->traj[timeIndex],
                                   currentTrajectory->traj[nextTimeIndex],
                                   state, gains, dt );
        }
        else if( timeIndex < currentTrajectory->count-1 )
        {
            nextTimeIndex = timeIndex+1;
            executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                   currentTrajectory->traj[timeIndex],
                                   currentTrajectory->traj[nextTimeIndex],
                                   state, gains, dt );
        }
        else if( timeIndex == currentTrajectory->count-1 && haveNewTrajectory )
        {
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                keepWalking = false;

            if( keepWalking )
            {
                if( validateNextTrajectory( currentTrajectory->traj[timeIndex],
                                            nextTrajectory->traj[0], dt ) )
                {
                    bal_state.m_walk_error = NO_WALK_ERROR;
                    nextTimeIndex = 0;
                    executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                           currentTrajectory->traj[timeIndex],
                                           nextTrajectory->traj[nextTimeIndex],
                                           state, gains, dt );
                    
                    memcpy( prevTrajectory, currentTrajectory, sizeof(*prevTrajectory) );
                    memcpy( currentTrajectory, nextTrajectory, sizeof(*nextTrajectory) );
                }
                else
                {
                    bal_state.m_walk_mode = WALK_WAITING;
                    bal_state.m_walk_error = WALK_FAILED_SWAP;
                    fprintf(stderr, "WARNING: Discontinuous trajectory passed in. Discarding it.\n");
                }
                haveNewTrajectory = false;
            }
        }
        else
        {
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                keepWalking = false;
        }

        prevTimeIndex = timeIndex;
        timeIndex = nextTimeIndex;
        sendState();
    }

    bal_state.m_walk_mode = WALK_INACTIVE;
    sendState();
}




bool Walker::checkForNewTrajectory(zmp_traj_t &newTrajectory, bool haveNewTrajAlready)
{
    size_t fs;
    
    ach_status_t r = ach_get( &zmp_chan, &newTrajectory, sizeof(newTrajectory), &fs, NULL, ACH_O_LAST );

    if( ACH_OK==r || ACH_MISSED_FRAME==r )
    {
        fprintf(stdout, "Noticed new trajectory: ID #%d\n", (int)newTrajectory.trajNumber);
        return true;
    }
    else
        return haveNewTrajAlready || false;

}

bool Walker::validateNextTrajectory( zmp_traj_element_t &current, zmp_traj_element_t &next, double dt )
{
    bool valid = true;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        if( fabs(next.angles[i]-current.angles[i])/fabs(dt) > fabs(m_jointVelContTol) )
            valid = false;

    return valid;
}


void Walker::executeTimeStep( Hubo_Control &hubo, zmp_traj_element_t &prevElem,
            zmp_traj_element_t &currentElem, zmp_traj_element &nextElem,
            nudge_state_t &state, balance_gains_t &gains, double dt )
{
    //flattenFoot( hubo, nextElem, state, gains, dt );
    //straightenBack( hubo, nextElem, state, gains, dt );
    //complyKnee( hubo, nextElem, state, gains, dt );
    nudgeHips( hubo, nextElem, state, gains, dt );
    //nudgeRefs( hubo, nextElem, state, dt, hkin ); //vprev, verr, dt );
    double vel, accel;

    // For each joint set it's position to that in the trajectory for the
    // current timestep, which has been adjusted based on feedback.
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
//        hubo.setJointTraj( i, currentElem.angles[i] );
//        hubo.setJointTraj( i, nextElem.angles[i] );
//        hubo.setJointAngle( i, nextElem.angles[i] );
        hubo.passJointAngle( i, nextElem.angles[i] + bal_state.jointOffset[i] );
        
        // Compute and set joint velocity, used by the control daemon,
        // based on current and next joint angles.
        //FIXME vel = (nextElem.angles[i]-currentElem.angles[i])*ZMP_TRAJ_FREQ_HZ;
        vel = (nextElem.angles[i]-currentElem.angles[i])/dt;
        hubo.setJointVelocity( i, vel );
//        hubo.setJointNominalSpeed(i, vel);
//        hubo.setJointNominalSpeed( i, 1*
//                (nextElem.angles[i]-currentElem.angles[i])*ZMP_TRAJ_FREQ_HZ/2.0 );
//               (nextElem.angles[i]-currentElem.angles[i])*ZMP_TRAJ_FREQ_HZ );
//               (nextElem.angles[i]-currentElem.angles[i])/dt );

        // Compute and set joint acceleration, used by the control daemon,
        // based on current and next joint velocities. Save new velocity in
        // the state.V0 variable for use on next timestep.
        //FIXME accel = (vel-state.V0[i])*ZMP_TRAJ_FREQ_HZ;
        accel = (vel-state.V0[i])/dt;
        state.V0[i] = vel;
        hubo.setJointNominalAcceleration( i, 2*accel );
//        if( i == RHY || i == RHR || i == RHP || i == RKN || i == RAR || i==RAP )
//            std::cout << "(" << vel << ":" << accel << ")" << "\t";
    }
//    std::cout << std::endl;

//    hubo.setJointAngle( RSR, nextElem.angles[RSR] + hubo.getJointAngleMax(RSR) );
//    hubo.setJointAngle( LSR, nextElem.angles[LSR] + hubo.getJointAngleMin(LSR) );
//    hubo.setJointTraj( RSR, currentElem.angles[RSR] + hubo.getJointAngleMax(RSR) );
//    hubo.setJointTraj( LSR, currentElem.angles[LSR] + hubo.getJointAngleMin(LSR) );

    hubo.setJointAngleMin( LHR, currentElem.angles[RHR]-M_PI/2.0 );
    hubo.setJointAngleMax( RHR, currentElem.angles[LHR]+M_PI/2.0 );

    //LegVector lL, rL;
    // Send all the new commands
    //hubo.getLeftLegAngles(lL);
    //hubo.getRightLegAngles(rL);
    //std::cout << "refL: " << lL.transpose() << "\trefR: " << rL.transpose() << "\n";
    hubo.sendControls();
}


void Walker::sendState()
{
    ach_put( &bal_state_chan, &bal_state, sizeof(bal_state) );
}


void Walker::checkCommands()
{
    size_t fs;
    ach_get( &bal_cmd_chan, &cmd, sizeof(cmd), &fs, NULL, ACH_O_LAST );
}

