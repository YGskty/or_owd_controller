/* or_owdcontroller entry point from OpenRAVE */

#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <ros/ros.h>
#include <owd_msgs/WAMState.h>

#include "OrOwdController.h"

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{ 
   info.interfacenames[OpenRAVE::PT_Controller].push_back("OrOwdController");
   return;
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
   if((type == OpenRAVE::PT_Controller)&&(interfacename == "orowdcontroller"))
      return OpenRAVE::InterfaceBasePtr(new or_owdcontroller::OrOwdController(penv, sinput));
   return OpenRAVE::InterfaceBasePtr();
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
   RAVELOG_INFO("destroying or_owdcontroller plugin ...\n");
   return;
}
