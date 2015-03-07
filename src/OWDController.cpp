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
#include <boost/foreach.hpp>
#include "OWDController.h"

OWDController::OWDController(OpenRAVE::EnvironmentBasePtr env, std::string const &ns)
    : OpenRAVE::ControllerBase(env)
    , initialized_(false)
    , owd_ns_(ns)
{
    RegisterCommand("WaitForUpdate", boost::bind(&OWDController::waitForUpdate, this, _1, _2),
                    "Block for an update.");
    RegisterCommand("Servo", boost::bind(&OWDController::servoCommand, this, _1, _2),
                    "Servo with an instantaneous joint velocity.");
    RegisterCommand("SetStiffness", boost::bind(&OWDController::setStiffnessCommand, this, _1, _2),
                    "Change the stiffness of the joints; 0 is gravity compensation and 1 is stiff.");
    RegisterCommand("SetSpeed", boost::bind(&OWDController::setSpeedCommand, this, _1, _2),
                    "Change the min_accel_time and joint velocity limits.");
    RegisterCommand("GetStatus", boost::bind(&OWDController::getStatusCommand, this, _1, _2),
                    "Get the trajectory execution status.");
    RegisterCommand("ClearStatus", boost::bind(&OWDController::clearStatusCommand, this, _1, _2),
                    "Clear the trajectory execution status.");
}

bool OWDController::Init(OpenRAVE::RobotBasePtr robot, std::vector<int> const &dof_indices, int ctrl_transform)
{
    if (initialized_) {
        return true;
    }

    BOOST_ASSERT(robot && ctrl_transform == 0);
    robot_ = robot;
    execution_time_ = ros::Time::now();

    nh_.setCallbackQueue(&queue_);
    ros::NodeHandle nh_owd(nh_, owd_ns_);

    dof_indices_ = dof_indices;
    sub_wamstate_ = nh_owd.subscribe("wamstate", 1, &OWDController::wamstateCallback, this);
    pub_servo_ = nh_owd.advertise<owd_msgs::Servo>("wamservo", 1);
    srv_add_traj_ = nh_owd.serviceClient<owd_msgs::AddTrajectory>("AddTrajectory");
    srv_add_or_traj_ = nh_owd.serviceClient<owd_msgs::AddOrTrajectory>("AddOrTrajectory");
    srv_add_timed_traj_ = nh_owd.serviceClient<owd_msgs::AddTimedTrajectory>("AddTimedTrajectory");
    srv_cancel_all_traj_ = nh_owd.serviceClient<owd_msgs::CancelAllTrajectories>("CancelAllTrajectories");
    srv_set_stiffness_ = nh_owd.serviceClient<owd_msgs::SetStiffness>("SetStiffness");
    srv_set_speed_ = nh_owd.serviceClient<owd_msgs::SetSpeed>("SetSpeed");
    srv_force_threshold_ = nh_owd.serviceClient<owd_msgs::SetForceInputThreshold>("SetForceInputThreshold");
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

            // Use target_positions if available. This is necessary for newer
            // versions of OWD that publish joint encoders as positions.
            if (!current_wamstate_->target_positions.empty()) {
                dof_values[dof_index] = current_wamstate_->target_positions[owd_index];
            } else {
                dof_values[dof_index] = current_wamstate_->positions[owd_index];
            }
        }

        // Set the actual DOF values, even if they violate joint limits. This
        // prevents SetDOFValues from clamping the joint values to the
        // conservative OpenRAVE joint limits. This could potentially cause
        // trajectory execution values in OWD.
        robot_->SetDOFValues(dof_values, OpenRAVE::KinBody::CLA_Nothing);
    }
}

void OWDController::Reset(int options)
{
    current_wamstate_ = owd_msgs::WAMState::ConstPtr();

    // Attempt to cancel all trajectories
    owd_msgs::CancelAllTrajectories msg_cancel_all;
    
    bool const success = srv_cancel_all_traj_.call(msg_cancel_all) && msg_cancel_all.response.ok;
    if (!success) {
      RAVELOG_ERROR("Cancelling all OWD trajectories failed: %s\n",
                    msg_cancel_all.response.reason.c_str()
                    );
    }
}

bool OWDController::IsDone(void)
{
    OpenRAVE::EnvironmentMutex::scoped_lock lock(robot_->GetEnv()->GetMutex());
    if (!current_wamstate_) {
        throw OpenRAVE::openrave_exception("Have not received any wamstate messages from OWD",
                                           OpenRAVE::ORE_InvalidState);
    }
    // Check if we still don't have a recent enough wamstate
    if (current_wamstate_->header.stamp <= execution_time_) {
        return false;
    }

    // Check to see if there are still any trajectories running
    if (current_wamstate_->trajectory_queue.size() > 0) {
        return false;
    }

    // Check to make sure the arm is not being moved by Servo commands
    return current_wamstate_->state != owd_msgs::WAMState::state_traj_active;
    
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
    // OpenRAVE sends a NULL trajectory when you click on the robot.
    if (!traj) {
        return true;
    }

    // If we have or_mac_trajectory and the trajectory is of the correct type,
    // then we'll directly forward the binary blob to OWD.
#ifndef NO_MAC_TRAJECTORY
    or_mac_trajectory::MacTrajectoryConstPtr mac_traj = boost::dynamic_pointer_cast<or_mac_trajectory::MacTrajectory const>(traj);
    if (mac_traj) {
        RAVELOG_DEBUG("Executing timed trajectory.\n");
        return ExecuteTimedTrajectory(mac_traj);
    } else
#endif
    {
        // Check whether the trajectory has a deltatime group.
        OpenRAVE::ConfigurationSpecification const cspec
            = traj->GetConfigurationSpecification();
        std::vector<OpenRAVE::dReal> waypoint(cspec.GetDOF());
        OpenRAVE::dReal placeholder;

        bool const has_deltatime = cspec.ExtractDeltaTime(
            placeholder, waypoint.begin());

        // If the trajectory is timed, then we'll execute it with owd_ortraj.
        if (has_deltatime) {
            RAVELOG_DEBUG("Executing owd_ortraj trajectory.\n");
            return ExecuteORTrajectory(traj);
        }
        // Otherwise, we'll let OWD do the timing itself.
        else {
            RAVELOG_DEBUG("Executing generic trajectory.\n");
            return ExecuteGenericTrajectory(traj);
        }
    }
}

bool OWDController::ExecuteORTrajectory(OpenRAVE::TrajectoryBaseConstPtr traj)
{
    std::stringstream ss;
    traj->serialize(ss);

    owd_msgs::AddOrTrajectory::Request request;
    request.id = "";
    request.traj = ss.str();
    request.xml_id = traj->GetXMLId();

    // TODO: How do we know whether synchronization is required?
    request.synchronize = false;

    owd_msgs::AddOrTrajectory::Response response;
    bool const success = srv_add_or_traj_.call(request, response) && response.ok;
    if (success) {
        RAVELOG_DEBUG("Successfully added the trajectory to OWD.\n");
    } else if (!response.reason.empty()) {
        RAVELOG_ERROR("Adding the trajectory to OWD failed with error: %s\n", response.reason.c_str());
        return false;
    } else {
        RAVELOG_ERROR("Adding the trajectory to OWD failed with an unknown error.\n");
        return false;
    }

    // TODO: We need to add a time_added field to avoid a race condition here.
    //execution_time_ = response.time_added;
    execution_time_ = ros::Time::now();

    status_cleared_ = false;
    return true;
}

bool OWDController::ExecuteGenericTrajectory(OpenRAVE::TrajectoryBaseConstPtr traj)
{
    RAVELOG_DEBUG("OWDController::SetPath: Starting.\n");
    size_t const num_waypoints = traj->GetNumWaypoints();
    size_t const num_dofs = dof_indices_.size();
    OpenRAVE::ConfigurationSpecification const config_spec = traj->GetConfigurationSpecification();
    RAVELOG_DEBUG("OWDController::SetPath: Processing trajectory with %d waypoints defined for %d DOFs.\n",
                   num_waypoints, num_dofs);

    owd_msgs::AddTrajectory::Request request;
    request.traj.options = parseTrajectoryFlags(traj);
    request.traj.id = "";
    request.traj.positions.resize(num_waypoints);
    request.traj.blend_radius.resize(num_waypoints);
    request.traj.blend_radius.assign(num_waypoints, 0);

    // Check if this is a blended trajectory.
    bool is_blending;
    OpenRAVE::ConfigurationSpecification::Group blend_group;
    try {
        blend_group = config_spec.GetGroupFromName("owd_blend_radius");

        if (blend_group.dof != 1) {
            RAVELOG_ERROR("Trajectory blend radii have the incorrect number of DOFs; expected %d got %d.\n",
                1, blend_group.dof
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


    std::vector<OpenRAVE::dReal> current_dofs(num_dofs);
    robot_->GetDOFValues(current_dofs, dof_indices_);
    bool dofs_changed = false;

    for (size_t i = 0; i < num_waypoints; ++i) {
        std::vector<OpenRAVE::dReal> full_waypoint;
        traj->GetWaypoint(i, full_waypoint, config_spec);

        // Initialize the waypoint to the current joint values. We'll only
        // change the DOF values included in the trajectory from these default
        // values.
        std::vector<OpenRAVE::dReal> waypoint = current_dofs;

        // Extract the joint values from the waypoint. The full waypoint may
        // include extra fields that we don't care about (e.g. timestamps,
        // joint velocities).
        if (!config_spec.ExtractJointValues(waypoint.begin(), full_waypoint.begin(), robot_, dof_indices_)) {
            return true;
        } else if (waypoint.size() != num_dofs) {
            RAVELOG_ERROR("Unable to extract joint values from waypoint; expected %d, got %d.\n",
                static_cast<int>(num_dofs), static_cast<int>(waypoint.size())
            );
            return false;
        }

        // Check if any of the DOFs changed.
        for (size_t j = 0; j < num_dofs; ++j) {
            if (waypoint[j] != current_dofs[j]) {
                dofs_changed = true;
                break;
            }
        }

        // Optionally extract the blend radius. If it doesn't exist we default
        // to a zero blend radius (i.e. come to a complete stop at each
        // waypoint).
        request.traj.positions[i].j.resize(num_dofs);
        for (size_t j = 0; j < num_dofs; ++j) {
            request.traj.positions[i].j[j] = waypoint[j];
            if (is_blending) {
                request.traj.blend_radius[i] = full_waypoint[blend_group.offset];
            }
        }
    }

    // Short-circuit if none of the DOF values changed.
    if (!dofs_changed) {
        RAVELOG_DEBUG("Not executing trajectory. No DOF values changed.\n");
        return true;
    }

    // Add the trajectory to OWD.
    owd_msgs::AddTrajectory::Response response;
    bool const success = srv_add_traj_.call(request, response) && response.ok;
    if (success) {
        RAVELOG_DEBUG("Successfully added the trajectory to OWD.\n");
    } else if (!response.reason.empty()) {
        RAVELOG_ERROR("Adding the trajectory to OWD failed with error: %s\n", response.reason.c_str());
        return false;
    } else {
        RAVELOG_ERROR("Adding the trajectory to OWD failed with an unknown error.\n");
        return false;
    }
    execution_time_ = response.time_added;
    status_cleared_ = false;
    return true;
}

#ifndef NO_MAC_TRAJECTORY
bool OWDController::ExecuteTimedTrajectory(or_mac_trajectory::MacTrajectoryConstPtr traj)
{
    // Check if this trajectory includes this controller's DOFs.
    std::vector<int> traj_dof_indices, executing_dofs;
    traj->GetDOFIndices(traj_dof_indices);
    std::set_intersection(dof_indices_.begin(), dof_indices_.end(),
                          traj_dof_indices.begin(), traj_dof_indices.end(),
                          std::back_inserter(executing_dofs));

    // Debug information.
    std::stringstream controller_stream, traj_stream, executing_stream;
    controller_stream << "[";
    traj_stream << "[";
    executing_stream << "[";
    BOOST_FOREACH (int const index, dof_indices_) { controller_stream << " " << index; }
    BOOST_FOREACH (int const index, traj_dof_indices) { traj_stream << " " << index; }
    BOOST_FOREACH (int const index, executing_dofs) { executing_stream << " " << index; }
    controller_stream << "]";
    traj_stream << "]";
    executing_stream << "]";
    RAVELOG_DEBUG("Controlled DOFs = %s\n", controller_stream.str().c_str());
    RAVELOG_DEBUG("Trajectory DOFs = %s\n", traj_stream.str().c_str());
    RAVELOG_DEBUG("Executing DOFs = %s\n", executing_stream.str().c_str());

    uint32_t const synchronized = traj->GetExecutionFlags() & owd_msgs::JointTraj::opt_Synchronize;
    if (executing_dofs.empty() && !synchronized) {
        RAVELOG_DEBUG("Skipping TimedTrajectory that contains not controlled DOFs.\n");
        return true;
    }
#if 0
    // Verify that all of the controlled joints are included in the trajectory.
    else if (executing_dofs.size() != dof_indices_.size()) {
        throw OpenRAVE::openrave_exception("Partial controller DOF trajectories are not supported.",
                                           OpenRAVE::ORE_InvalidArguments);
    }
#endif

    // Set the force/torque threshold for guarded moves.
    if (traj->GetExecutionFlags() & owd_msgs::JointTraj::opt_CancelOnForceInput) {
        // Split the force vector into a magnitude and a direction.
        OpenRAVE::Vector force_direction = traj->GetThresholdForce();
        OpenRAVE::Vector torque_threshold = traj->GetThresholdTorque();
        double const force_magnitude = std::sqrt(force_direction.lengthsqr3());
        force_direction.normalize3();

        RAVELOG_INFO("stopping on force/torque\n");
        RAVELOG_INFO("force = [ %f %f %f ], magnitude = %f\n", force_direction.x, force_direction.y, force_direction.z, force_magnitude);
        RAVELOG_INFO("torque = [ %f %f %f ]\n", torque_threshold.x, torque_threshold.y, torque_threshold.z);

        owd_msgs::SetForceInputThresholdRequest ft_request;
        ft_request.direction.x = force_direction.x;
        ft_request.direction.y = force_direction.y;
        ft_request.direction.z = force_direction.z;
        ft_request.force = force_magnitude;
        ft_request.torques.x = torque_threshold.x;
        ft_request.torques.y = torque_threshold.y;
        ft_request.torques.z = torque_threshold.z;

        owd_msgs::SetForceInputThresholdResponse ft_response;
        bool success = srv_force_threshold_.call(ft_request, ft_response) && ft_response.ok;
        if (!success) {
            throw OPENRAVE_EXCEPTION_FORMAT("Unable to set force/torque threshold: %s",
                                            ft_response.reason.c_str(), OpenRAVE::ORE_Failed);
        }
    }

    // Extract the timed MacJointTraj from the OpenRAVE trajectory.
    boost::shared_ptr<OWD::MacJointTraj> mac_traj = boost::const_pointer_cast<OWD::MacJointTraj>(traj->GetOWDTrajectory());
    if (!mac_traj) {
        throw OpenRAVE::openrave_exception("Unable to execute an untimed MacTrajectory.",
                                           OpenRAVE::ORE_Failed);
    }

    // Execute a synchronized full DOF trajectory.
    size_t index_min = 0;
    size_t index_max = 0;
    if (synchronized) {
        size_t const num_mac_dof = mac_traj->start_position.size();
        if (num_mac_dof != static_cast<size_t>(robot_->GetDOF())) {
            throw OpenRAVE::openrave_exception("Synchronized trajectories must be full DOF.",
                                               OpenRAVE::ORE_InvalidArguments);
        }

        // Extract the correct DOFs from the trajectory.
        // FIXME: This assumes that traj_dof_indices is sorted.
        index_min = dof_indices_.front();
        index_max = dof_indices_.back();
        RAVELOG_DEBUG("Executing a synchronized trajectory.\n");
    }
    // Unsynchronized single controller trajectory.
    else {
        index_min = 0;
        index_max = traj_dof_indices.size() - 1;
        RAVELOG_DEBUG("Executing an unsynchronized trajectory.\n");
    }
    RAVELOG_DEBUG("Extracting DOFs %d-%d from the MacJointTraj.\n",
        static_cast<int>(index_min), static_cast<int>(index_max)
    );

    {
        std::ofstream trajstream("/tmp/macjointtraj.bin", std::ios::out | std::ios::binary);
        try {
            trajstream << mac_traj->serialize();
        } catch (char const *error_message) {
            RAVELOG_WARN("Failed serializing MacJointTraj: %s\n", error_message);
        }
    }

    // Extract this arm's DOFs from the full trajectory.
    BinaryData serialized_trajectory;
    try {
        serialized_trajectory.PutInt(OWD::Trajectory::TRAJTYPE_MACJOINTTRAJ);
        serialized_trajectory.PutString(mac_traj->serialize(index_min, index_max));
    } catch (char const *error_message) {
        throw OPENRAVE_EXCEPTION_FORMAT("Failed serializing MacJointTraj: %s",
                                        error_message, OpenRAVE::ORE_Failed);
    }
        
    // Directly execute the timed trajectory.
    owd_msgs::AddTimedTrajectory::Request request;
    request.SerializedTrajectory = serialized_trajectory;
    request.options = traj->GetExecutionFlags();
    request.id = traj->GetTrajectoryID();

    owd_msgs::AddTimedTrajectory::Response response;
    bool const success = srv_add_timed_traj_.call(request, response);
    if (!success || !response.ok) {
        throw OPENRAVE_EXCEPTION_FORMAT("Adding a timed trajectory failed: %s",
                                        response.reason.c_str(), OpenRAVE::ORE_Failed);
    }
    execution_time_ = response.time_added;
    status_cleared_ = false;
    return true;
}
#endif

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

bool OWDController::setSpeedCommand(std::ostream &out, std::istream &in)
{
    size_t const num_dofs = dof_indices_.size();

    owd_msgs::SetSpeed msg_speed;
    msg_speed.request.velocities.resize(num_dofs);

    in >> msg_speed.request.min_accel_time;
    for (size_t i = 0; i < num_dofs; ++i) {
        in >> msg_speed.request.velocities[i];
    }

    if (in.fail()) {
        RAVELOG_ERROR("Unable to parse arugments to SetSpeed.\n");
        return false;
    }

    // Call the OWD SetSpeed service.
    bool const success = srv_set_speed_.call(msg_speed) && msg_speed.response.ok;
    if (!success) {
        RAVELOG_ERROR("Setting speed failed with error: %s\n", msg_speed.response.reason.c_str());
        return false;
    }
    return true;
}

void OWDController::wamstateCallback(owd_msgs::WAMState::ConstPtr const &new_wamstate)
{
    // Verify that we received the WAMState messages in sequential order.
    if (current_wamstate_ && new_wamstate->header.stamp < current_wamstate_->header.stamp) {
        //RAVELOG_WARN("Received WAMState message with an out-of-order timestamp.\n");
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

bool OWDController::getStatusCommand(std::ostream &out, std::istream &in)
{
    if (!current_wamstate_) {
        RAVELOG_ERROR("Attempted to query trajectory status with no active WAMState message.\n");
        return false;
    }else if(status_cleared_) {
        out << "cleared";
	return true;
    }

    if (current_wamstate_->state == owd_msgs::WAMState::state_traj_stalled) {
        out << "stalled"; 
        return true;
    }

    uint8_t const state = current_wamstate_->prev_trajectory.state;
    switch (state) {
    case owd_msgs::TrajInfo::state_pending:
        out << "pending";
        break;
    case owd_msgs::TrajInfo::state_active:
        out << "active";
        break;
    case owd_msgs::TrajInfo::state_done:
        out << "done";
        break;
    case owd_msgs::TrajInfo::state_aborted:
        out << "aborted";
        break;
    default:
        RAVELOG_ERROR("Unknown trajectory state %d.\n", state);
        return false;
    }
    return true;
}

bool OWDController::clearStatusCommand(std::ostream &out, std::istream &in)
{
    status_cleared_ = true;
    return true;
}

int OWDController::parseTrajectoryFlags(OpenRAVE::TrajectoryBaseConstPtr traj)
{
    static std::string const group_prefix = "or_owd_controller";
    bool stop_on_stall = true;
    bool stop_on_ft = false;
    double force_magnitude = 0.0;
    std::vector<double> force_direction(3);
    std::vector<double> torque(3);

    // Extract the flags from a bogus trajectory group. This is a hack since
    // we can't read trajectory UserData that is set in Python.
    RAVELOG_DEBUG("Reading configuration specification.\n");
    OpenRAVE::ConfigurationSpecification config_spec = traj->GetConfigurationSpecification();
    OpenRAVE::ConfigurationSpecification::Group owd_group;
    try {
        owd_group = config_spec.GetGroupFromName(group_prefix);
    }
    // GetGroupFromName throws an openrave_exception, but catching it by the
    // specific type doesn't work. I'm not sure why...
    catch (...) {
        RAVELOG_WARN("Trajectory is missing the %s group.\n", group_prefix.c_str());
        return 0;
    }

    // Verify that we found the right group. All of this is possible because
    // OpenRAVE only matches group prefixes.
    std::stringstream flags_stream;
    std::string found_group_prefix;
    flags_stream << owd_group.name;
    flags_stream >> found_group_prefix;

    if (found_group_prefix != group_prefix) {
        RAVELOG_WARN("Expected group %s, but found %s.\n",
                     group_prefix.c_str(), found_group_prefix.c_str()); 
        return 0;
    }
    RAVELOG_DEBUG("Found trajectory flags: %s\n", flags_stream.str().c_str());

    while (flags_stream.good()) {
        std::string key;
        flags_stream >> key; 

        if (key == "stop_on_stall") {
            flags_stream >> stop_on_stall;
        } else if (key == "stop_on_ft") {
            flags_stream >> stop_on_ft;
        } else if (key == "force_magnitude") {
            flags_stream >> force_magnitude;
        } else if (key == "force_direction") {
            flags_stream >> force_direction[0] >> force_direction[1] >> force_direction[2];
        } else if (key == "torque") {
            flags_stream >> torque[0] >> torque[1] >> torque[2];
        } else {
            RAVELOG_ERROR("Unknown option '%s'.\n", key.c_str());
            throw OpenRAVE::openrave_exception("Unknown trajectory execution option.",
                                               OpenRAVE::ORE_InvalidArguments);
        }
    }

    if (flags_stream.fail()) {
        RAVELOG_ERROR("Unable to extract options from the trajectory.\n");
        throw OpenRAVE::openrave_exception("Unable to extract options from the trajectory.",
                                           OpenRAVE::ORE_InvalidArguments);
    }
    RAVELOG_DEBUG("Flags: stop_on_stall %d stop_on_ft %d\n", stop_on_stall, stop_on_ft);

    // Create the trajectory options bitmask.
    int flags = 0;
    if (stop_on_stall) {
        flags |= owd_msgs::JointTraj::opt_CancelOnStall;
    }
    if (stop_on_ft) {
        flags |= owd_msgs::JointTraj::opt_CancelOnForceInput;
    }

    // Set the force/torque thresholds.
    if (stop_on_ft) {
        owd_msgs::SetForceInputThreshold msg;
        msg.request.direction.x = force_direction[0];
        msg.request.direction.y = force_direction[1];
        msg.request.direction.z = force_direction[2];
        msg.request.force = force_magnitude;
        msg.request.torques.x = torque[0];
        msg.request.torques.y = torque[1];
        msg.request.torques.z = torque[2];

        bool success = srv_force_threshold_.call(msg) && msg.response.ok;
        if (!success && !msg.response.reason.empty()) {
            RAVELOG_ERROR("Setting force/torque threshold failed with error: %s\n",
                          msg.response.reason.c_str());
            throw OpenRAVE::openrave_exception("Failed setting force/torque threshold in OWD.",
                                               OpenRAVE::ORE_Failed);
        } else if (!success) {
            RAVELOG_ERROR("Setting force/torque threshold failed with an unknown error.\n");
            throw OpenRAVE::openrave_exception("Failed setting force/torque threshold in OWD.",
                                               OpenRAVE::ORE_Failed);
        }
        RAVELOG_DEBUG("Set thresholds: direction = [ %f %f %f ], magnitude = %f, torque = [ %f %f %f ]\n",
                      msg.request.direction.x, msg.request.direction.y, msg.request.direction.z,
                      msg.request.force,
                      msg.request.torques.x, msg.request.torques.y, msg.request.torques.z);
    }
    return flags;
}
