#include <sstream>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include "MOPEDSensorSystem.h"

MOPEDSensorSystem::MOPEDSensorSystem(OpenRAVE::EnvironmentBasePtr env, std::string const &ns,
                                     std::string const &frame_id)
    : OpenRAVE::SensorSystemBase(env)
    , frame_id_(frame_id)
    , model_path_("/pr/pr-ros-pkg/data/pr_ordata/ordata/objects/household")
    , model_suffix_(".kinbody.xml")
    , name_prefix_("moped_")
    , sub_tf_(nh_)
    , spinner_(0, &queue_)
{
    nh_.setCallbackQueue(&queue_);

    ros::NodeHandle moped_nh(nh_, ns);
    sub_moped_ = moped_nh.subscribe("object_poses", 10, &MOPEDSensorSystem::mopedCallback, this);
    spinner_.start();
}

MOPEDSensorSystem::~MOPEDSensorSystem(void)
{
}

void MOPEDSensorSystem::Reset(void)
{
    // TODO: Implement this.
}

void MOPEDSensorSystem::AddRegisteredBodies(std::vector<OpenRAVE::KinBodyPtr> const &bodies)
{
    throw OpenRAVE::openrave_exception("MOPEDSensorSystem does not implement AddRegisteredBodies.",
                                       OpenRAVE::ORE_NotImplemented);
}

OpenRAVE::KinBody::ManageDataPtr MOPEDSensorSystem::AddKinBody(OpenRAVE::KinBodyPtr body,
                                                               OpenRAVE::XMLReadableConstPtr data)
{
    throw OpenRAVE::openrave_exception("MOPEDSensorSystem does not implement AddKinBody.",
                                       OpenRAVE::ORE_NotImplemented);
}

bool MOPEDSensorSystem::RemoveKinBody(OpenRAVE::KinBodyPtr body)
{
    size_t const num_erased = bodies_.erase(body);
    return num_erased > 0;
}

bool MOPEDSensorSystem::IsBodyPresent(OpenRAVE::KinBodyPtr body)
{
    return bodies_.count(body) > 0;
}

bool MOPEDSensorSystem::EnableBody(OpenRAVE::KinBodyPtr body, bool enable)
{
    throw OpenRAVE::openrave_exception("MOPEDSensorSystem does not implement EnableBody.",
                                       OpenRAVE::ORE_NotImplemented);
}

bool MOPEDSensorSystem::SwitchBody(OpenRAVE::KinBodyPtr body1, OpenRAVE::KinBodyPtr body2)
{
    throw OpenRAVE::openrave_exception("MOPEDSensorSystem does not implement SwitchBody.",
                                       OpenRAVE::ORE_NotImplemented);
}

size_t MOPEDSensorSystem::getSequenceNumber(std::string const &name)
{
    std::pair<std::map<std::string, size_t>::iterator, bool> tmp = numbers_.insert(
        std::make_pair(name, 0)
    );

    // The C++ standard guarantees that unsigned integers wrap on overflow.
    size_t &number = tmp.first->second;
    size_t const current_number = number;
    ++number;
    return current_number;
}

std::string MOPEDSensorSystem::getUniqueName(std::string const &name)
{
    size_t const number = getSequenceNumber(name);
    std::stringstream ss;
    ss << name_prefix_ << name << number;
    return ss.str();
}

std::string MOPEDSensorSystem::getKinBodyPath(std::string const &name) const
{
    std::stringstream ss;
    ss << model_path_ << "/" << name << model_suffix_;
    return ss.str();
}

void MOPEDSensorSystem::mopedCallback(pr_msgs::ObjectPoseList const &detections)
{
    OpenRAVE::EnvironmentBasePtr env = GetEnv();

    RAVELOG_INFO("MOPED Callback\n");

    BOOST_FOREACH (pr_msgs::ObjectPose const &detection, detections.object_list) {
        // Transform the pose into the OpenRAVE frame.
        geometry_msgs::PoseStamped pose_moped, pose_or;
        pose_moped.header = detections.header;
        pose_moped.pose = detection.pose;

        try {
            sub_tf_.transformPose(frame_id_, pose_moped, pose_or);
        } catch (tf::TransformException const &e) {
            // It's very unlikely that any of the other transformations in this
            // message will succeed, so we'll give up.
            RAVELOG_ERROR("Unable to transform MOPED pose: %s\n", e.what());
            return;
        }

        OpenRAVE::Transform tf;
        tf.trans.x = pose_or.pose.position.x;
        tf.trans.y = pose_or.pose.position.y;
        tf.trans.z = pose_or.pose.position.z;
        tf.rot.w = pose_or.pose.orientation.w;
        tf.rot.x = pose_or.pose.orientation.x;
        tf.rot.y = pose_or.pose.orientation.y;
        tf.rot.z = pose_or.pose.orientation.z;

        // Attempt to create a new KinBody and skip any objects that are
        // missing KinBody files. ReadKinBodyXMLFile is thread-safe.
        std::string const kinbody_path = getKinBodyPath(detection.name);
        OpenRAVE::KinBodyPtr body = env->ReadKinBodyXMLFile(kinbody_path);
        if (!body) {
            RAVELOG_WARN("Unable to create KinBody for object of type '%s'\n", detection.name.c_str());
            continue;
        }

        // Assign the KinBody a unique name and add it to the environment. There
        // are potential concurrency issues between the OpenRAVE threads and the
        // AsyncSpinner thread, so we must be extra careful.
        std::string kinbody_name;
        {
            boost::mutex::scoped_lock lock(mutex_);
            kinbody_name = getUniqueName(detection.name);
        }

        try {
            OpenRAVE::EnvironmentMutex::scoped_lock lock(env->GetMutex());
            body->SetName(kinbody_name);
            env->Add(body);
        } catch (OpenRAVE::openrave_exception const &e) {
            RAVELOG_ERROR("Unable to add object '%s' to the environment.\n", kinbody_name.c_str());
            continue;
        }

        {
            boost::mutex::scoped_lock lock(mutex_);
            bodies_.insert(body);
        }
    }
}
