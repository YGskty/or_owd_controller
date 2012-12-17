#include "OWDController.h"

OWDController::OWDController(OpenRAVE::EnvironmentBasePtr env, std::string const &ns)
    : OpenRAVE::ControllerBase(env)
    , owd_ns_(ns)
{
}

bool OWDController::Init(OpenRAVE::RobotBasePtr robot, std::vector<int> const &dof_indices, int ctrl_transform)
{
    BOOST_ASSERT(robot && ctrl_transform == 0);
    robot_ = robot;

    nh_.setCallbackQueue(&queue_);
    ros::NodeHandle nh_owd(nh_, owd_ns_);

    dof_indices_ = dof_indices;
    sub_wamstate_ = nh_owd.subscribe("wamstate", 1, &OWDController::wamstateCallback, this);
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
    // TODO: What else do we need to reinitialize?
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
    throw OpenRAVE::openrave_exception("SetPath is not implemented.", OpenRAVE::ORE_NotImplemented);
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
