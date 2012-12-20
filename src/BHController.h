#ifndef BHCONTROLLER_H_
#define BHCONTROLLER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <openrave/openrave.h>
#include <owd_msgs/BHState.h>
#include <owd_msgs/MoveHand.h>

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
    ros::ServiceClient srv_movehand_;
    OpenRAVE::RobotBasePtr robot_;
    std::vector<int> dof_indices_;
    owd_msgs::BHState::ConstPtr current_bhstate_;

    void bhstateCallback(owd_msgs::BHState::ConstPtr const &bhstate);
};

#endif
