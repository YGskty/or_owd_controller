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

/** \file OWDController.h
 * \brief OpenRAVE controller for controlling a Barrett WAM arm using OWD.
 * \author Michael Koval
 * \date 2012
 */
#ifndef OWDCONTROLLER_H_
#define OWDCONTROLLER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <owd_msgs/AddTrajectory.h>
#include <owd_msgs/AddTimedTrajectory.h>
#include <owd_msgs/DeleteTrajectory.h>
#include <owd_msgs/WAMState.h>
#include <owd_msgs/Servo.h>
#include <owd_msgs/SetForceInputThreshold.h>
#include <owd_msgs/SetStiffness.h>
#include <owd_msgs/SetSpeed.h>
#include <openrave/openrave.h>
#include <or_mac_trajectory/MacTrajectory.h>

class OWDController : public OpenRAVE::ControllerBase {
public:
    OWDController(OpenRAVE::EnvironmentBasePtr env, std::string const &ns);

    virtual bool Init(OpenRAVE::RobotBasePtr robot, std::vector<int> const &dof_indices, int ctrl_transform);
    virtual void SimulationStep(OpenRAVE::dReal time_ellapsed);
    virtual void Reset(int options = 0);
    virtual bool IsDone(void);

    virtual OpenRAVE::RobotBasePtr GetRobot(void) const;
    virtual std::vector<int> const &GetControlDOFIndices(void) const;
    virtual int IsControlTransformation(void) const;

    virtual void GetVelocity(std::vector<OpenRAVE::dReal> &velocities) const;

    virtual bool SetDesired(std::vector<OpenRAVE::dReal> const &values,
                            OpenRAVE::TransformConstPtr transform = OpenRAVE::TransformConstPtr());

    virtual bool SetPath(OpenRAVE::TrajectoryBaseConstPtr traj);
    virtual bool ExecuteGenericTrajectory(OpenRAVE::TrajectoryBaseConstPtr traj);
    virtual bool ExecuteTimedTrajectory(or_mac_trajectory::MacTrajectoryConstPtr traj);
    

private:
    bool initialized_;
    bool status_cleared_;
    ros::Time execution_time_;
    std::string owd_ns_;
    std::string traj_id_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    ros::Subscriber sub_wamstate_;
    ros::Publisher pub_servo_;
    ros::ServiceClient srv_add_traj_;
    ros::ServiceClient srv_add_timed_traj_;
    ros::ServiceClient srv_delete_traj_;
    ros::ServiceClient srv_set_stiffness_;
    ros::ServiceClient srv_force_threshold_;
    ros::ServiceClient srv_set_speed_;
    OpenRAVE::RobotBasePtr robot_;
    std::vector<int> dof_indices_;
    owd_msgs::WAMState::ConstPtr current_wamstate_;

    bool waitForUpdate(std::ostream &out, std::istream &in);
    bool servoCommand(std::ostream &out, std::istream &in);
    bool setStiffnessCommand(std::ostream &out, std::istream &in);
    bool setSpeedCommand(std::ostream &out, std::istream &in);
    bool getStatusCommand(std::ostream &out, std::istream &in);
    bool clearStatusCommand(std::ostream &out, std::istream &in);

    void wamstateCallback(owd_msgs::WAMState::ConstPtr const &wamstate);

    int parseTrajectoryFlags(OpenRAVE::TrajectoryBaseConstPtr traj);
    
};

#endif
