#include "OWDController.h"

OWDController::OWDController(OpenRAVE::EnvironmentBasePtr env, std::string const &ns)
    : OpenRAVE::ControllerBase(env)
    , owd_ns_(ns)
{
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
    throw OpenRAVE::openrave_exception("SetDesired is not implemented.", OpenRAVE::ORE_NotImplemented);
}

bool OWDController::SetPath(OpenRAVE::TrajectoryBaseConstPtr traj)
{
    size_t const num_waypoints = traj->GetNumWaypoints();
    size_t const num_dofs = dof_indices_.size();
    OpenRAVE::ConfigurationSpecification const config_spec = traj->GetConfigurationSpecification();

    owd_msgs::AddTrajectory::Request request;
    // FIXME: Which options should I pass here?
    request.traj.options = owd_msgs::JointTraj::opt_CancelOnStall;
    request.traj.id = "";
    request.traj.positions.resize(num_waypoints);
    request.traj.blend_radius.resize(num_waypoints);

    // FIXME: Implement trajectory blending.
    request.traj.blend_radius.assign(num_waypoints, 0);

    for (size_t i = 0; i < num_waypoints; ++i) {
        std::vector<OpenRAVE::dReal> full_waypoint;
        traj->GetWaypoint(i, full_waypoint, config_spec);

        // Extract only the joint values from the waypoint. The full waypoint
        // may include extra fields that we don't care about (e.g. timestamps,
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

        for (size_t j = 0; j < num_dofs; ++j) {
            request.traj.positions[i].j.resize(num_dofs);
            request.traj.positions[i].j[j] = waypoint[j];
        }
    }

    // Add the trajectory to OWD.
    owd_msgs::AddTrajectory::Response response;
    bool const success = srv_add_traj_.call(request, response) && response.ok;
    if (success) {
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

bool OWDController::servoCommand(std::ostream &out, std::istream &in)
{
    // OWD will silently fail if we send a servo command while in grav-comp.
    if (current_wamstate_ && current_wamstate_->state != owd_msgs::WAMState::state_fixed) {
        RAVELOG_ERROR("Servoing is only possible when the arm is holding position.\n");
        return false;
    }

    size_t const num_dofs = dof_indices_.size();
    owd_msgs::Servo msg_servo;
    msg_servo.joint.resize(num_dofs);
    msg_servo.velocity.resize(num_dofs);

    for (size_t i = 0; i < num_dofs; ++i) {
        msg_servo.joint[i] = i;
        in >> msg_servo.velocity[i];
    }

    if (in.fail()) {
        RAVELOG_ERROR("Servo command received missing or malformed joint velocities.\n");
        return false;
    }

    pub_servo_.publish(msg_servo);
    return true;
}

bool OWDController::setStiffnessCommand(std::ostream &out, std::istream &in)
{
    throw OpenRAVE::openrave_exception("setStiffnessCommand is not implemented");
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

