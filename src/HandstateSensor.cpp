/** \file HandstateSensor.cpp
 * \brief OpenRAVE sensor plugin for the Barrett force/torque sensor.
 * \author Michael Koval
 * \date 2013
 */
#include <boost/make_shared.hpp>
#include "HandstateSensor.h"

HandstateSensor::HandstateSensor(OpenRAVE::EnvironmentBasePtr env, std::string const &owd_ns)
    : OpenRAVE::SensorBase(env)
    , owd_ns_(owd_ns)
{

    nh_.setCallbackQueue(&queue_);
    ros::NodeHandle nh_owd(nh_, owd_ns_);
    sub_ft_ = nh_owd.subscribe("handstate", 1, &HandstateSensor::handstateCallback, this);

    // FIXME: This is a hack to work around the lack of a SetName function in Python.
    SetName("handstate");

    RegisterCommand("GetState", boost::bind(&HandstateSensor::getState, this, _1, _2),
		    "Get the current handstate status.");
}

int HandstateSensor::Configure(OpenRAVE::SensorBase::ConfigureCommand, bool blocking)
{
    return 0;
}

OpenRAVE::SensorBase::SensorGeometryPtr HandstateSensor::GetSensorGeometry(SensorType type)
{
    return boost::make_shared<OpenRAVE::SensorBase::Force6DGeomData>();
}

OpenRAVE::SensorBase::SensorDataPtr HandstateSensor::CreateSensorData(OpenRAVE::SensorBase::SensorType type)
{
    return boost::make_shared<OpenRAVE::SensorBase::Force6DSensorData>();
}

bool HandstateSensor::GetSensorData(OpenRAVE::SensorBase::SensorDataPtr sensor_data)
{
    if (sensor_data->GetType() != OpenRAVE::SensorBase::ST_Force6D) {
        RAVELOG_ERROR("Received sensor data of incorect type.\n");
        return false;
    } else if (!current_bhstate_) {
        RAVELOG_ERROR("There is no handstate data available.\n");
        return false;
    } else if (!sensor_data) {
        RAVELOG_ERROR("Sensor data is NULL; this should never happen..\n");
        return false;
    }

    // TODO: This isn't great. but its the best bad idea i have right now.
    boost::shared_ptr<Force6DSensorData> wrench = boost::dynamic_pointer_cast<Force6DSensorData>(sensor_data);
    BOOST_ASSERT(wrench);
    wrench->force.x = current_bhstate_->strain[0];
    wrench->force.y = current_bhstate_->strain[1];
    wrench->force.z = current_bhstate_->strain[2];

    // The BH260 can't detect breakway.
    if (!current_bhstate_->breakaway.empty()) {
        BOOST_ASSERT(current_bhstate_->breakaway.size() == 3);
        wrench->torque.x = current_bhstate_->breakaway[0];
        wrench->torque.y = current_bhstate_->breakaway[1];
        wrench->torque.z = current_bhstate_->breakaway[2];
    } else {
        wrench->torque.x = 0;
        wrench->torque.y = 0;
        wrench->torque.z = 0;
    }
    return true;
}

bool HandstateSensor::Supports(OpenRAVE::SensorBase::SensorType type)
{
    return type == OpenRAVE::SensorBase::ST_Force6D;
}

void HandstateSensor::SetTransform(OpenRAVE::Transform const &trans)
{
    transform_ = trans;
}

OpenRAVE::Transform HandstateSensor::GetTransform()
{
    // TODO: Populate this from TF.
    return transform_;
}

bool HandstateSensor::SimulationStep(OpenRAVE::dReal fTimeElapsed)
{
    queue_.callAvailable();
    return true;
}

void HandstateSensor::handstateCallback(owd_msgs::BHStateConstPtr bhstate_msg)
{
    if (current_bhstate_ && bhstate_msg->header.stamp < current_bhstate_->header.stamp) {
        RAVELOG_WARN("Dropping handstate message with out-of-order timestamp.\n");
        current_bhstate_ = owd_msgs::BHStateConstPtr();
        return;
    }
    current_bhstate_ = bhstate_msg;
}

 bool HandstateSensor::getState(std::ostream &out, std::istream &in)
 {
   if (!current_bhstate_){
     RAVELOG_ERROR("Attempted to query handstate status with no active Handstate message.\n");
     out << "unknown";
     return true;
   }

   if (current_bhstate_->state == owd_msgs::BHState::state_done) {
     out << "done";
   }else if (current_bhstate_->state == owd_msgs::BHState::state_moving) {
     out << "moving";
   }else if (current_bhstate_->state == owd_msgs::BHState::state_stalled) {
     out << "stalled";
   }else if (current_bhstate_->state == owd_msgs::BHState::state_uninitialized) {
     out << "uninitialized";
   }

   return true;
 }
