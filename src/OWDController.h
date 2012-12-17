#ifndef OWDCONTROLLER_H_
#define OWDCONTROLLER_H_

#include <ros/ros.h>
#include <owd_msgs/WAMState.h>
#include <openrave/openrave.h>
//#include <openrave/controller.h>

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
    OpenRAVE::RobotBasePtr robot_;
    std::vector<int> dof_indices_;
    ros::Subscriber sub_wamstate_;
    owd_msgs::WAMState::ConstPtr current_wamstate_;

    void wamstateCallback(owd_msgs::WAMState::ConstPtr const &wamstate);
};

#endif
