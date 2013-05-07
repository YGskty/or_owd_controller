/** \file BHTactileSensor.h
 * \brief OpenRAVE sensor plugin for the BarrettHand's tactile sensors.
 * \author Michael Koval
 * \date 2013
 */
#ifndef BHTACTILESENSOR_H_ 
#define BHTACTILESENSOR_H_ 

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <owd_msgs/BHTactile.h>
#include <openrave/openrave.h>

struct TactileCell {
    OpenRAVE::RobotBase::LinkPtr parent;
    OpenRAVE::Vector position;
    OpenRAVE::Vector normal;
};

class BHTactileSensor : public OpenRAVE::SensorBase {
public:
    BHTactileSensor(OpenRAVE::EnvironmentBasePtr env, OpenRAVE::RobotBasePtr robot,
                    std::string const &ns, std::string const &link_prefix);
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
    ros::Subscriber sub_tactile_;
    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::RobotBasePtr robot_;
    OpenRAVE::Transform transform_;
    std::vector<TactileCell> cell_offsets_;
    owd_msgs::BHTactileConstPtr current_tactile_;

    void loadGeometry(std::string const &path, std::string const &link_prefix,
                      std::vector<TactileCell> &cells) const;
    OpenRAVE::Vector transformData(size_t index, double pressure) const;
    OpenRAVE::Vector transformGeometry(size_t index) const;
    void tactileCallback(owd_msgs::BHTactileConstPtr tactile_msg);
};

#endif
