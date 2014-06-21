/** \file HandstateSensor.h
 * \brief OpenRAVE sensor plugin for the handstate
 * \author Jennifer King
 * \date 2013
 */
#ifndef HANDSTATESENSOR_H_ 
#define HANDSTATESENSOR_H_ 

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <owd_msgs/BHState.h>
#include <owd_msgs/Reset.h>
#include <openrave/openrave.h>

class HandstateSensor : public OpenRAVE::SensorBase {
public:
    HandstateSensor(OpenRAVE::EnvironmentBasePtr env, std::string const &ns);
    virtual int Configure(OpenRAVE::SensorBase::ConfigureCommand, bool blocking = false);
    virtual OpenRAVE::SensorBase::SensorGeometryPtr GetSensorGeometry(SensorType type = ST_Invalid);
    virtual SensorDataPtr CreateSensorData(OpenRAVE::SensorBase::SensorType type = ST_Invalid);
    virtual bool GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata);
    virtual bool Supports(OpenRAVE::SensorBase::SensorType type);
    virtual void SetTransform(OpenRAVE::Transform const &trans);
    virtual OpenRAVE::Transform GetTransform();
    virtual bool SimulationStep(OpenRAVE::dReal fTimeElapsed);

private:
    std::string owd_ns_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    ros::Subscriber sub_ft_;
    ros::ServiceClient srv_tare_;
    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::Transform transform_;
    owd_msgs::BHStateConstPtr current_bhstate_;

    void handstateCallback(owd_msgs::BHStateConstPtr bhstate_msg);
    bool getState(std::ostream &out, std::istream &in);
};

#endif
