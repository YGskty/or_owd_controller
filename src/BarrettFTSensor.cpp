/** \file BarrettFTSensor.cpp
 * \brief OpenRAVE sensor plugin for the Barrett force/torque sensor.
 * \author Michael Koval
 * \date 2013
 */
#include <boost/make_shared.hpp>
#include "BarrettFTSensor.h"

BarrettFTSensor::BarrettFTSensor(OpenRAVE::EnvironmentBasePtr env, std::string const &owd_ns)
    : OpenRAVE::SensorBase(env)
    , owd_ns_(owd_ns)
{
    nh_.setCallbackQueue(&queue_);
    ros::NodeHandle nh_owd(nh_, owd_ns_);
    sub_ft_ = nh_owd.subscribe("filtered_forcetorque", 1, &BarrettFTSensor::forcetorqueCallback, this);
    srv_tare_ = nh_owd.serviceClient<owd_msgs::Reset>("ft_tare");
    current_wrench_ = geometry_msgs::WrenchStampedConstPtr();

    RegisterCommand("Tare", boost::bind(&BarrettFTSensor::tareCallback, this, _1, _2),
                    "Tare the force/torque sensor.");

    // FIXME: This is a hack to work around the lack of a SetName function in Python.
    SetName("forcetorque");
}

int BarrettFTSensor::Configure(OpenRAVE::SensorBase::ConfigureCommand, bool blocking)
{
    return 0;
}

OpenRAVE::SensorBase::SensorGeometryPtr BarrettFTSensor::GetSensorGeometry(SensorType type)
{
    return boost::make_shared<OpenRAVE::SensorBase::Force6DGeomData>();
}

OpenRAVE::SensorBase::SensorDataPtr BarrettFTSensor::CreateSensorData(OpenRAVE::SensorBase::SensorType type)
{
    return boost::make_shared<OpenRAVE::SensorBase::Force6DSensorData>();
}

bool BarrettFTSensor::GetSensorData(OpenRAVE::SensorBase::SensorDataPtr sensor_data)
{
    if (sensor_data->GetType() != OpenRAVE::SensorBase::ST_Force6D) {
        RAVELOG_ERROR("Received sensor data of incorect type.\n");
        return false;
    } else if (!current_wrench_) {
        RAVELOG_ERROR("There is no force/torque data available.\n");
        return false;
    }

    boost::shared_ptr<Force6DSensorData> wrench = boost::dynamic_pointer_cast<Force6DSensorData>(sensor_data);
    wrench->force.x = current_wrench_->wrench.force.x;
    wrench->force.y = current_wrench_->wrench.force.y;
    wrench->force.z = current_wrench_->wrench.force.z;
    wrench->torque.x = current_wrench_->wrench.torque.x;
    wrench->torque.y = current_wrench_->wrench.torque.y;
    wrench->torque.z = current_wrench_->wrench.torque.z;
    return true;
}

bool BarrettFTSensor::Supports(OpenRAVE::SensorBase::SensorType type)
{
    return type == OpenRAVE::SensorBase::ST_Force6D;
}

void BarrettFTSensor::SetTransform(OpenRAVE::Transform const &trans)
{
    transform_ = trans;
}

OpenRAVE::Transform BarrettFTSensor::GetTransform()
{
    // TODO: Populate this from TF.
    return transform_;
}

bool BarrettFTSensor::SimulationStep(OpenRAVE::dReal fTimeElapsed)
{
    queue_.callAvailable();
    return true;
}

void BarrettFTSensor::forcetorqueCallback(geometry_msgs::WrenchStampedConstPtr wrench_msg)
{

    if (current_wrench_ && wrench_msg->header.stamp < current_wrench_->header.stamp) {
        RAVELOG_WARN("Dropping force/torque message with out-of-order timestamp.\n");
        current_wrench_ = geometry_msgs::WrenchStampedConstPtr();
        return;
    }
    current_wrench_ = wrench_msg;
}

bool BarrettFTSensor::tareCallback(std::ostream &out, std::istream &in)
{
    owd_msgs::Reset msg_reset;
    bool const success = srv_tare_.call(msg_reset) && msg_reset.response.ok;

    if (!success && !msg_reset.response.reason.empty()) {
        RAVELOG_ERROR("Tareing the force/torque sensor failed: %s\n",
                      msg_reset.response.reason.c_str());
        return false;
    } else if (!success) {
        RAVELOG_ERROR("Tareing the force/torque sensor failed with an unknown error.\n");
        return false;
    }
    return true;
}
