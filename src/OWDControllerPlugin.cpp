#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <ros/ros.h>
#include "OWDController.h"

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO &info)
{ 
    info.interfacenames[OpenRAVE::PT_Controller].push_back("OWDController");
    return;
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type,
    std::string const &interface_name, std::istream &sinput, OpenRAVE::EnvironmentBasePtr env)
{

    if (type == OpenRAVE::PT_Controller && interface_name == "owdcontroller") {
        std::string owd_namespace;
        sinput >> owd_namespace;

        if (!sinput.good()) {
            RAVELOG_ERROR("OWDController is missing the required namespace parameter.\n");
            return OpenRAVE::InterfaceBasePtr();
        }

        return boost::make_shared<OWDController>(env, owd_namespace);
    } else {
        return OpenRAVE::InterfaceBasePtr();
    }
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
