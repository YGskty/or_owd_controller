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

/** \file BHController.cpp
 * \brief OpenRAVE controller for controlling a BarrettHand using OWD.
 * \author Michael Koval
 * \date 2012
 */
#include "BHController.h"

BHController::BHController(OpenRAVE::EnvironmentBasePtr env, std::string const &ns)
    : OpenRAVE::ControllerBase(env)
   ,  initialized_(false)
    , bhd_ns_(ns)
{
    RegisterCommand("WaitForUpdate", boost::bind(&BHController::waitForUpdate, this, _1, _2),
                    "Block for an update.");
    RegisterCommand("ResetHand", boost::bind(&BHController::resetHand, this, _1, _2),
                    "Send a reset command to the hand.");
}

bool BHController::Init(OpenRAVE::RobotBasePtr robot, std::vector<int> const &dof_indices, int ctrl_transform)
{
    if (initialized_) {
        return true;
    }

    BOOST_ASSERT(robot && ctrl_transform == 0);
    BOOST_ASSERT(dof_indices.size() == 4);
    robot_ = robot;
    execution_time_ = ros::Time::now();

    nh_.setCallbackQueue(&queue_);
    ros::NodeHandle nh_bhd(nh_, bhd_ns_);

    dof_indices_ = dof_indices;
    sub_bhstate_ = nh_bhd.subscribe("handstate", 1, &BHController::bhstateCallback, this);
    srv_move_ = nh_bhd.serviceClient<owd_msgs::MoveHand>("MoveHand");
    srv_reset_ = nh_bhd.serviceClient<owd_msgs::ResetHand>("ResetHand");
    initialized_ = true;
    return true;
}

void BHController::SimulationStep(OpenRAVE::dReal time_ellapsed)
{
    queue_.callAvailable();

    // Update the DOF values from the most recent WAMState message.
    if (current_bhstate_) {
        std::vector<OpenRAVE::dReal> dof_values;
        robot_->GetDOFValues(dof_values);

        for (size_t index = 0; index < dof_indices_.size(); ++index) {
            size_t const dof_index = dof_indices_[index];
            BOOST_ASSERT(dof_index < dof_values.size());
            dof_values[dof_index] = current_bhstate_->positions[index];
        }


        // This prevents OpenRAVE from spamming "DOF is not in limits" warnings.
        int const debug_level = OpenRAVE::RaveGetDebugLevel();
        OpenRAVE::RaveSetDebugLevel(OpenRAVE::Level_Fatal);
        robot_->SetDOFValues(dof_values);
        OpenRAVE::RaveSetDebugLevel(debug_level);
    }
}

void BHController::Reset(int options)
{
    current_bhstate_ = owd_msgs::BHState::ConstPtr();

    // TODO: Move this to a SendCommand.
#if 0
    owd_msgs::ResetHand::Request request;
    owd_msgs::ResetHand::Response response;
    bool const success = srv_reset_.call(request, response) && response.ok;
    if (!success && !response.reason.empty()) {
        RAVELOG_ERROR("Resetting hand failed: %s\n", response.reason.c_str());
    } else if (!success) {
        RAVELOG_ERROR("Resetting hand failed.\n");
    }
#endif
}

bool BHController::IsDone(void)
{
    // Check if we're still waiting for the fingers to move.
    if (current_bhstate_->header.stamp <= execution_time_) {
        return false;
    }
    return !current_bhstate_ || current_bhstate_->state == owd_msgs::BHState::state_done;
}

OpenRAVE::RobotBasePtr BHController::GetRobot(void) const
{
    return robot_;
}

std::vector<int> const &BHController::GetControlDOFIndices(void) const
{
    return dof_indices_;
}

int BHController::IsControlTransformation(void) const
{
    return 0;
}

bool BHController::SetDesired(std::vector<OpenRAVE::dReal> const &values,
                               OpenRAVE::TransformConstPtr transform)
{
    OpenRAVE::EnvironmentMutex::scoped_lock lock(robot_->GetEnv()->GetMutex());
    BOOST_ASSERT(values.size() == dof_indices_.size() && !transform);
    size_t const num_dofs = dof_indices_.size();

    owd_msgs::MoveHand::Request request;
    request.movetype = owd_msgs::MoveHand::Request::movetype_position;
    request.positions.resize(num_dofs);

    for (size_t i = 0; i < num_dofs; ++i) {
        request.positions[i] = values[i];
    }

    owd_msgs::MoveHand::Response response;
    bool const success = srv_move_.call(request, response) && response.ok;
    if (!success && !response.reason.empty()) {
        RAVELOG_ERROR("Moving hand failed with error: %s\n", response.reason.c_str());
        return false;
    } else if (!success) {
        RAVELOG_ERROR("Moving hand failed with unknown error.\n");
        return false;
    }
    execution_time_ = ros::Time::now();
    return true;
}

bool BHController::SetPath(OpenRAVE::TrajectoryBaseConstPtr traj)
{
    // TODO: warn if the trajectory contains the finger DOFs.
    return true;
}

bool BHController::waitForUpdate(std::ostream &out, std::istream &in)
{
    // Block until we can update the controlled DOFs. Otherwise the user could
    // hit a race condition by planning using the robot's default configuration.
    RAVELOG_DEBUG("Waiting for BHState message.\n");
    ros::NodeHandle nh_bhd(nh_, bhd_ns_);
    owd_msgs::BHState::ConstPtr bhstate = ros::topic::waitForMessage<owd_msgs::BHState>("handstate", nh_bhd);

    OpenRAVE::EnvironmentMutex::scoped_lock lock(robot_->GetEnv()->GetMutex());
    bhstateCallback(bhstate);
    SimulationStep(0);
    RAVELOG_DEBUG("Received BHState message. Initialization is complete.\n");
    return true;
}

bool BHController::resetHand(std::ostream &out, std::istream &in)
{

    //Send a request to reset the hand
    owd_msgs::ResetHand::Request request;
    owd_msgs::ResetHand::Response response;
    bool const success = srv_reset_.call(request, response) && response.ok;
    if (!success && !response.reason.empty()) {
        RAVELOG_ERROR("Resetting hand failed: %s\n", response.reason.c_str());
    } else if (!success) {
        RAVELOG_ERROR("Resetting hand failed.\n");
    }
}

void BHController::bhstateCallback(owd_msgs::BHState::ConstPtr const &new_bhstate)
{
    // Verify that we received the WAMState messages in sequential order.
    if (current_bhstate_ && new_bhstate->header.stamp < current_bhstate_->header.stamp) {
        RAVELOG_WARN("Received BHState message with an out-of-order timestamp.\n");
        return;
    }
    // Verify that the message contains the correct number of DOFs.
    else if (new_bhstate->positions.size() != dof_indices_.size()) {
        RAVELOG_WARN("Received BHState message with %d DOFs; expected %d.\n",
            static_cast<int>(new_bhstate->positions.size()),
            static_cast<int>(dof_indices_.size())
        );
        return;
    }
    current_bhstate_ = new_bhstate;
}
