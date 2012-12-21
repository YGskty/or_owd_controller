#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <ros/ros.h>
#include "MOPEDSensorSystem.h"

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO &info)
{ 
    info.interfacenames[OpenRAVE::PT_SensorSystem].push_back("MOPEDSensorSystem");
    return;
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type,
    std::string const &interface_name, std::istream &sinput, OpenRAVE::EnvironmentBasePtr env)
{

    if (type == OpenRAVE::PT_SensorSystem && interface_name == "mopedsensorsystem") {
        std::string node_name, moped_namespace, frame_id;
        sinput >> node_name >> moped_namespace >> frame_id;

        RAVELOG_INFO("name = %s  namespace = %s, frame_id = %s\n",
            node_name.c_str(), moped_namespace.c_str(), frame_id.c_str()
        );

        if (sinput.fail()) {
            RAVELOG_ERROR("MOPEDSensorSystem is missing the node_name, moped_namespace, and/or frame_id parameter(s).\n");
            return OpenRAVE::InterfaceBasePtr();
        }

        if (!ros::isInitialized()) {
            int argc = 0;
            ros::init(argc, NULL, node_name, ros::init_options::AnonymousName);
            RAVELOG_INFO("Starting ROS node '%s'.\n", node_name.c_str());
        } else {
            RAVELOG_INFO("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
        }

        return boost::make_shared<MOPEDSensorSystem>(env, moped_namespace, frame_id);
    } else {
        return OpenRAVE::InterfaceBasePtr();
    }
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
