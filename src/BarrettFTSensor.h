/** \file BarrettFTSensor.h
 * \brief OpenRAVE sensor plugin for the Barrett force/torque sensor.
 * \author Michael Koval
 * \date 2013
 */
#ifndef BARRETTFTSENSOR_H_ 
#define BARRETTFTSENSOR_H_ 

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/WrenchStamped.h>
#include <owd_msgs/Reset.h>
#include <openrave/openrave.h>

class BarrettFTSensor : public OpenRAVE::SensorBase {
public:
    BarrettFTSensor(OpenRAVE::EnvironmentBasePtr env, std::string const &ns);
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
    geometry_msgs::WrenchStampedConstPtr current_wrench_;

    void forcetorqueCallback(geometry_msgs::WrenchStampedConstPtr wrench_msg);
    bool tareCallback(std::ostream &out, std::istream &in);
};

#endif
