#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <ros/ros.h>
#include "BHController.h"

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO &info)
{ 
    info.interfacenames[OpenRAVE::PT_Controller].push_back("BHController");
    return;
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type,
    std::string const &interface_name, std::istream &sinput, OpenRAVE::EnvironmentBasePtr env)
{

    if (type == OpenRAVE::PT_Controller && interface_name == "bhcontroller") {
        std::string node_name, owd_namespace;
        sinput >> node_name >> owd_namespace;

        if (sinput.fail()) {
            RAVELOG_ERROR("BHController is missing the node_name and/or bh_namespace parameter(s).\n");
            return OpenRAVE::InterfaceBasePtr();
        }

        if (!ros::isInitialized()) {
            int argc = 0;
            ros::init(argc, NULL, node_name, ros::init_options::AnonymousName);
            RAVELOG_INFO("Starting ROS node '%s'.\n", node_name.c_str());
        } else {
            RAVELOG_INFO("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
        }

        return boost::make_shared<BHController>(env, owd_namespace);
    } else {
        return OpenRAVE::InterfaceBasePtr();
    }
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
