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

#ifndef BHCONTROLLER_H_
#define BHCONTROLLER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <openrave/openrave.h>
#include <owd_msgs/BHState.h>
#include <owd_msgs/MoveHand.h>
#include <owd_msgs/ResetHand.h>

class BHController : public OpenRAVE::ControllerBase {
public:
    BHController(OpenRAVE::EnvironmentBasePtr env, std::string const &ns);

    virtual bool Init(OpenRAVE::RobotBasePtr robot, std::vector<int> const &dof_indices, int ctrl_transform);
    virtual void SimulationStep(OpenRAVE::dReal time_ellapsed);
    virtual void Reset(int options = 0);
    virtual bool IsDone(void);

    virtual OpenRAVE::RobotBasePtr GetRobot(void) const;
    virtual std::vector<int> const &GetControlDOFIndices(void) const;
    virtual int IsControlTransformation(void) const;

    virtual bool SetDesired(std::vector<OpenRAVE::dReal> const &values,
                            OpenRAVE::TransformConstPtr transform = OpenRAVE::TransformConstPtr());
    virtual bool SetPath(OpenRAVE::TrajectoryBaseConstPtr traj);

private:
    std::string bhd_ns_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    ros::Subscriber sub_bhstate_;
    ros::ServiceClient srv_move_;
    ros::ServiceClient srv_reset_;
    OpenRAVE::RobotBasePtr robot_;
    std::vector<int> dof_indices_;
    owd_msgs::BHState::ConstPtr current_bhstate_;

    void bhstateCallback(owd_msgs::BHState::ConstPtr const &bhstate);
};

#endif
