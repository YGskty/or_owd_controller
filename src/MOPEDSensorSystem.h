#ifndef MOPEDSENSORSYSTEM_H_
#define MOPEDSENSORSYSTEM_H_

#include <map>
#include <set>
#include <openrave/openrave.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf/transform_listener.h>
#include <pr_msgs/ObjectPoseList.h>

class MOPEDSensorSystem : public OpenRAVE::SensorSystemBase {
public:
    MOPEDSensorSystem(OpenRAVE::EnvironmentBasePtr env, std::string const &ns, std::string const &frame_id);
    virtual ~MOPEDSensorSystem(void);

    virtual void Reset(void);
    virtual void AddRegisteredBodies(std::vector<OpenRAVE::KinBodyPtr> const &bodies);
    virtual OpenRAVE::KinBody::ManageDataPtr AddKinBody(OpenRAVE::KinBodyPtr body, OpenRAVE::XMLReadableConstPtr data);
    virtual bool RemoveKinBody(OpenRAVE::KinBodyPtr body);
    virtual bool IsBodyPresent(OpenRAVE::KinBodyPtr body);
    virtual bool EnableBody(OpenRAVE::KinBodyPtr body, bool enable);
    virtual bool SwitchBody(OpenRAVE::KinBodyPtr body1, OpenRAVE::KinBodyPtr body2);

private:
    std::string frame_id_;
    std::string model_path_;
    std::string model_suffix_;
    std::string name_prefix_;

    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    tf::TransformListener sub_tf_;
    ros::Subscriber sub_moped_;
    ros::AsyncSpinner spinner_;

    boost::mutex mutex_;
    std::set<OpenRAVE::KinBodyPtr> bodies_;
    std::map<std::string, size_t> numbers_;

    size_t getSequenceNumber(std::string const &name);
    std::string getUniqueName(std::string const &name);
    std::string getKinBodyPath(std::string const &name) const;

    void mopedCallback(pr_msgs::ObjectPoseList const &detections);
};

#endif
