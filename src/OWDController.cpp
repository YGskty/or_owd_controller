/***********************************************************************

Copyright (c) 2012, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

/** \file OWDController.cpp
 * \brief OpenRAVE controller for controlling a Barrett WAM arm using OWD.
 * \author Michael Koval
 * \date 2012
 */
#include "OWDController.h"

OWDController::OWDController(OpenRAVE::EnvironmentBasePtr env, std::string const &ns)
    : OpenRAVE::ControllerBase(env)
    , owd_ns_(ns)
{
    RegisterCommand("WaitForUpdate", boost::bind(&OWDController::waitForUpdate, this, _1, _2),
                    "Block for an update.");
    RegisterCommand("Servo", boost::bind(&OWDController::servoCommand, this, _1, _2),
                    "Servo with an instantaneous joint velocity.");
    RegisterCommand("SetStiffness", boost::bind(&OWDController::setStiffnessCommand, this, _1, _2),
                    "Change the stiffness of the joints; 0 is gravity compensation and 1 is stiff.");
}

bool OWDController::Init(OpenRAVE::RobotBasePtr robot, std::vector<int> const &dof_indices, int ctrl_transform)
{
    BOOST_ASSERT(robot && ctrl_transform == 0);
    robot_ = robot;

    nh_.setCallbackQueue(&queue_);
    ros::NodeHandle nh_owd(nh_, owd_ns_);

    dof_indices_ = dof_indices;
    sub_wamstate_ = nh_owd.subscribe("wamstate", 1, &OWDController::wamstateCallback, this);
    pub_servo_ = nh_owd.advertise<owd_msgs::Servo>("wamservo", 1);
    srv_add_traj_ = nh_owd.serviceClient<owd_msgs::AddTrajectory>("AddTrajectory");
    srv_delete_traj_ = nh_owd.serviceClient<owd_msgs::DeleteTrajectory>("DeleteTrajectory");
    srv_set_stiffness_ = nh_owd.serviceClient<owd_msgs::SetStiffness>("SetStiffness");
    return true;
}

void OWDController::SimulationStep(OpenRAVE::dReal time_ellapsed)
{
    queue_.callAvailable();

    // Update the DOF values from the most recent WAMState message.
    if (current_wamstate_) {
        std::vector<OpenRAVE::dReal> dof_values;
        robot_->GetDOFValues(dof_values);

        for (size_t owd_index = 0; owd_index < dof_indices_.size(); ++owd_index) {
            size_t const dof_index = dof_indices_[owd_index];
            BOOST_ASSERT(dof_index < dof_values.size());
            dof_values[dof_index] = current_wamstate_->positions[owd_index];
        }
        robot_->SetDOFValues(dof_values);
    }
}

void OWDController::Reset(int options)
{
    current_wamstate_ = owd_msgs::WAMState::ConstPtr();

    // Attempt to cancel the current trajectory.
    if (!traj_id_.empty()) {
        owd_msgs::DeleteTrajectory msg_delete;
        msg_delete.request.ids.push_back(traj_id_);

        bool const success = srv_delete_traj_.call(msg_delete) && msg_delete.response.ok;
        if (!success && !msg_delete.response.reason.empty()) {
            RAVELOG_ERROR("Deleting the trajectory '%s' from OWD failed: %s\n",
                traj_id_.c_str(), msg_delete.response.reason.c_str()
            );
        } else if (!success) {
            RAVELOG_ERROR("Deleting the trajectory '%s' from OWD failed with an unknown error.\n",
                traj_id_.c_str()
            );
        }
    }
}

bool OWDController::IsDone(void)
{
    return !current_wamstate_ || (current_wamstate_->state != owd_msgs::WAMState::state_traj_active
                              &&  current_wamstate_->state != owd_msgs::WAMState::state_traj_stalled
                              &&  current_wamstate_->state != owd_msgs::WAMState::state_traj_paused);
}

OpenRAVE::RobotBasePtr OWDController::GetRobot(void) const
{
    return robot_;
}

std::vector<int> const &OWDController::GetControlDOFIndices(void) const
{
    return dof_indices_;
}

int OWDController::IsControlTransformation(void) const
{
    return 0;
}

void OWDController::GetVelocity(std::vector<OpenRAVE::dReal> &velocities) const
{
    velocities.resize(dof_indices_.size());

    if (current_wamstate_) {
        for (size_t i = 0; i < velocities.size(); ++i) {
            velocities[i] = static_cast<OpenRAVE::dReal>(current_wamstate_->velocities[i]);
        }
    } else {
        velocities.assign(dof_indices_.size(), 0);
    }
}

bool OWDController::SetDesired(std::vector<OpenRAVE::dReal> const &values,
                               OpenRAVE::TransformConstPtr transform)
{
    return true;
}

bool OWDController::SetPath(OpenRAVE::TrajectoryBaseConstPtr traj)
{
    RAVELOG_DEBUG("OWDController::SetPath: Starting.\n");
    size_t const num_waypoints = traj->GetNumWaypoints();
    size_t const num_dofs = dof_indices_.size();
    OpenRAVE::ConfigurationSpecification const config_spec = traj->GetConfigurationSpecification();
    RAVELOG_DEBUG("OWDController::SetPath: Processing trajectory with %d waypoints defined for %d DOFs.\n",
                   num_waypoints, num_dofs);

    owd_msgs::AddTrajectory::Request request;
    // FIXME: Which options should I pass here?
    request.traj.options = owd_msgs::JointTraj::opt_CancelOnStall;
    request.traj.id = "";
    request.traj.positions.resize(num_waypoints);
    request.traj.blend_radius.resize(num_waypoints);
    request.traj.blend_radius.assign(num_waypoints, 0);

    // Check if this is a blended trajectory.
    bool is_blending;
    OpenRAVE::ConfigurationSpecification::Group blend_group;
    try {
        blend_group = config_spec.GetGroupFromName("owd_blend_radius");

        if (static_cast<size_t>(blend_group.dof) != num_dofs) {
            RAVELOG_ERROR("Trajectory blend radii have the incorrect number of DOFs; expected %d got %d.\n",
                static_cast<int>(num_dofs), blend_group.dof
            );
            return false;
        }
        is_blending = true;
    }
    // GetGroupFromName throws an openrave_exception, but catching it by the
    // specific type doesn't work. I'm not sure why...
    catch (...) {
        RAVELOG_INFO("Did not find blend radii in the trajectory. Using a zero blend radius.\n");
        is_blending = false;
    }

    for (size_t i = 0; i < num_waypoints; ++i) {
        std::vector<OpenRAVE::dReal> full_waypoint;
        traj->GetWaypoint(i, full_waypoint, config_spec);

        // Extract the joint values from the waypoint. The full waypoint may
        // include extra fields that we don't care about (e.g. timestamps,
        // joint velocities).
        std::vector<OpenRAVE::dReal> waypoint(num_dofs);
        if (!config_spec.ExtractJointValues(waypoint.begin(), full_waypoint.begin(), robot_, dof_indices_)) {
            RAVELOG_ERROR("Unable to extract joint values from waypoint.\n");
            return false;
        } else if (waypoint.size() != num_dofs) {
            RAVELOG_ERROR("Unable to extract joint values from waypoint; expected %d, got %d.\n",
                static_cast<int>(num_dofs), static_cast<int>(waypoint.size())
            );
            return false;
        }

        // Optionally extract the blend radius. If it doesn't exist we default
        // to a zero blend radius (i.e. come to a complete stop at each
        // waypoint).
        request.traj.positions[i].j.resize(num_dofs);
        for (size_t j = 0; j < num_dofs; ++j) {
            request.traj.positions[i].j[j] = waypoint[j];
            if (is_blending) {
                request.traj.blend_radius[j] = full_waypoint[blend_group.offset + j];
            }
        }
    }

    // Add the trajectory to OWD.
    owd_msgs::AddTrajectory::Response response;
    bool const success = srv_add_traj_.call(request, response) && response.ok;
    if (success) {
        RAVELOG_DEBUG("Successfully added the trajectory to OWD.\n");
        traj_id_ = response.id;
    } else if (!response.reason.empty()) {
        RAVELOG_ERROR("Adding the trajectory to OWD failed with error: %s\n", response.reason.c_str());
        return false;
    } else {
        RAVELOG_ERROR("Adding the trajectory to OWD failed with an unknown error.\n");
        return false;
    }
    return true;
}

bool OWDController::waitForUpdate(std::ostream &out, std::istream &in)
{
    // Block until we can update the controlled DOFs. Otherwise the user could
    // hit a race condition by planning using the robot's default configuration.
    RAVELOG_DEBUG("Waiting for WAMState message.\n");
    ros::NodeHandle nh_owd(nh_, owd_ns_);
    owd_msgs::WAMState::ConstPtr wamstate = ros::topic::waitForMessage<owd_msgs::WAMState>("wamstate", nh_owd);

    OpenRAVE::EnvironmentMutex::scoped_lock lock(robot_->GetEnv()->GetMutex());
    wamstateCallback(wamstate);
    SimulationStep(0);
    RAVELOG_DEBUG("Received WAMState message. Initialization is complete.\n");
    return true;
}

bool OWDController::servoCommand(std::ostream &out, std::istream &in)
{
    // OWD will silently fail if we send a servo command while in grav-comp. We
    // can't simply check if the arm is holding position because it will cause
    // problems if we send the commands too quickly.
    if (current_wamstate_ && current_wamstate_->state == owd_msgs::WAMState::state_free) {
        RAVELOG_ERROR("Servoing is only possible when the arm is holding position.\n");
        return false;
    }

    size_t const num_dofs = dof_indices_.size();
    owd_msgs::Servo msg_servo;
    msg_servo.joint.resize(num_dofs);
    msg_servo.velocity.resize(num_dofs);

    for (size_t i = 0; i < num_dofs; ++i) {
        msg_servo.joint[i] = i + 1;
        in >> msg_servo.velocity[i];
    }

    if (in.fail()) {
        RAVELOG_ERROR("Servo command received missing or malformed joint velocities.\n");
        return false;
    }

    // Verify that we don't exceed any velocity limits. This should, hopefully,
    // help avoid velocity faulting the arm.
    std::vector<OpenRAVE::dReal> velocity_limits;
    robot_->GetDOFVelocityLimits(velocity_limits, dof_indices_);

    for (size_t i = 0; i < num_dofs; ++i) {
        if (std::abs(msg_servo.velocity[i]) > velocity_limits[i]) {
            RAVELOG_ERROR("Joint J%d exceeds the joint velocity limit; %f > %f.\n",
                msg_servo.velocity[i], velocity_limits[i]
            );
            return false;
        }
    }

    pub_servo_.publish(msg_servo);
    return true;
}

bool OWDController::setStiffnessCommand(std::ostream &out, std::istream &in)
{
    owd_msgs::SetStiffness msg_stiffness;
    float &stiffness = msg_stiffness.request.stiffness;
    in >> stiffness;

    if (in.fail()) {
        RAVELOG_ERROR("SetStiffness command received missing or malformed argument.\n");
        return false;
    } else if (stiffness < 0 || stiffness > 1) {
        RAVELOG_ERROR("Stiffness must be in the range [0, 1]; got %f.\n",
            static_cast<double>(msg_stiffness.request.stiffness)
        );
        return false;
    } else if (stiffness != 0 && stiffness != 1) {
        RAVELOG_WARN("Stiffness values in the range (0, 1) are experimental.\n");
    }

    bool const success = srv_set_stiffness_.call(msg_stiffness) && msg_stiffness.response.ok;
    if (!success && !msg_stiffness.response.reason.empty()) {
        RAVELOG_ERROR("Setting stiffness failed with error: %s\n", msg_stiffness.response.reason.c_str());
        return false;
    } else if (!success) {
        RAVELOG_ERROR("Setting stiffness failed with an unknown error.\n");
        return false;
    }
    return true;
}

void OWDController::wamstateCallback(owd_msgs::WAMState::ConstPtr const &new_wamstate)
{
    // Verify that we received the WAMState messages in sequential order.
    if (current_wamstate_ && new_wamstate->header.stamp < current_wamstate_->header.stamp) {
        RAVELOG_WARN("Received WAMState message with an out-of-order timestamp.\n");
        current_wamstate_ = owd_msgs::WAMState::ConstPtr();
        return;
    }
    // Verify that the message contains the correct number of DOFs.
    else if (new_wamstate->positions.size() != dof_indices_.size()) {
        RAVELOG_WARN("Received WAMState message with %d DOFs; expected %d.\n",
            static_cast<int>(new_wamstate->positions.size()),
            static_cast<int>(dof_indices_.size())
        );
        return;
    }
    current_wamstate_ = new_wamstate;
}
