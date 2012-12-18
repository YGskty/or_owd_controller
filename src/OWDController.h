#ifndef OWDCONTROLLER_H_
#define OWDCONTROLLER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <owd_msgs/AddTrajectory.h>
#include <owd_msgs/DeleteTrajectory.h>
#include <owd_msgs/WAMState.h>
#include <openrave/openrave.h>

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

private:
    std::string owd_ns_;
    std::string traj_id_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    ros::Subscriber sub_wamstate_;
    ros::ServiceClient srv_add_traj_;
    ros::ServiceClient srv_delete_traj_;
    OpenRAVE::RobotBasePtr robot_;
    std::vector<int> dof_indices_;
    owd_msgs::WAMState::ConstPtr current_wamstate_;

    void wamstateCallback(owd_msgs::WAMState::ConstPtr const &wamstate);
};

#endif
