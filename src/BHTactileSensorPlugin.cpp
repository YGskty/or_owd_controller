/** \file BHTactileSensorPlugin.cpp
 * \brief OpenRAVE sensor plugin for the BarrettHand's tactile sensors.
 * \author Michael Koval
 * \date 2013
 */
#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <ros/ros.h>
#include "BHTactileSensor.h"

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO &info)
{ 
    info.interfacenames[OpenRAVE::PT_Sensor].push_back("BHTactileSensor");
    return;
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type,
    std::string const &interface_name, std::istream &sinput, OpenRAVE::EnvironmentBasePtr env)
{

    if (type == OpenRAVE::PT_Sensor && interface_name == "bhtactilesensor") {
        std::string node_name, owd_namespace, robot_name, link_prefix;
        sinput >> node_name >> owd_namespace >> robot_name >> link_prefix;

        // Initialize the ROS node.
        RAVELOG_DEBUG("name = %s  namespace = %s\n", node_name.c_str(), owd_namespace.c_str());
        if (sinput.fail()) {
            RAVELOG_ERROR("BHTactileSensor is missing the node_name, owd_namespace, robot_name, or link_prefix parameter(s).\n");
            return OpenRAVE::InterfaceBasePtr();
        }

        if (!ros::isInitialized()) {
            int argc = 0;
            ros::init(argc, NULL, node_name, ros::init_options::AnonymousName);
            RAVELOG_DEBUG("Starting ROS node '%s'.\n", node_name.c_str());
        } else {
            RAVELOG_DEBUG("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
        }

        OpenRAVE::RobotBasePtr robot = env->GetRobot(robot_name);
        if (!robot) {
            throw OPENRAVE_EXCEPTION_FORMAT("There is no robot named '%s'.",
                                            robot_name.c_str(), OpenRAVE::ORE_InvalidArguments);
        }
        return boost::make_shared<BHTactileSensor>(env, robot, owd_namespace, link_prefix);
    } else {
        return OpenRAVE::InterfaceBasePtr();
    }
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
