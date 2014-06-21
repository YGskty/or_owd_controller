/***********************************************************************

Copyright (c) 2012, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <ros/ros.h>
#include "BHController.h"
#include "BHTactileSensor.h"
#include "HandstateSensor.h"
#include "OWDController.h"

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO &info)
{ 
    info.interfacenames[OpenRAVE::PT_Controller].push_back("BHController");
    info.interfacenames[OpenRAVE::PT_Controller].push_back("OWDController");
    info.interfacenames[OpenRAVE::PT_Sensor].push_back("BHTactileSensor");
    info.interfacenames[OpenRAVE::PT_Sensor].push_back("HandstateSensor");
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type,
    std::string const &interface_name, std::istream &sinput, OpenRAVE::EnvironmentBasePtr env)
{
    std::string node_name, owd_namespace;

    // Initialize the ROS node.
    if (interface_name == "owdcontroller" || interface_name == "bhcontroller"
     || interface_name == "bhtacitlesensor" || interface_name == "handstatesensor")
    {
        sinput >> node_name >> owd_namespace;

        RAVELOG_DEBUG("name = %s  namespace = %s\n", node_name.c_str(), owd_namespace.c_str());
        if (sinput.fail()) {
            RAVELOG_ERROR("OWDController is missing the node_name and/or owd_namespace parameter(s).\n");
            return OpenRAVE::InterfaceBasePtr();
        }

        if (!ros::isInitialized()) {
            int argc = 0;
            ros::init(argc, NULL, node_name, ros::init_options::AnonymousName);
            RAVELOG_DEBUG("Starting ROS node '%s'.\n", node_name.c_str());
        } else {
            RAVELOG_DEBUG("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
        }
    } else {
        return OpenRAVE::InterfaceBasePtr();
    }

    // Create the plugin.
    if (type == OpenRAVE::PT_Controller && interface_name == "owdcontroller") {
        return boost::make_shared<OWDController>(env, owd_namespace);
    } else if (type == OpenRAVE::PT_Controller && interface_name == "bhcontroller") {
        return boost::make_shared<BHController>(env, owd_namespace);
    } else if (type == OpenRAVE::PT_Sensor && interface_name == "bhtactilesensor") {
        std::string robot_name, link_prefix;
        sinput >> robot_name >> link_prefix;

        OpenRAVE::RobotBasePtr robot = env->GetRobot(robot_name);
        if (!robot) {
            throw OPENRAVE_EXCEPTION_FORMAT("There is no robot named '%s'.",
                    robot_name.c_str(), OpenRAVE::ORE_InvalidArguments);
        }
        return boost::make_shared<BHTactileSensor>(env, robot, owd_namespace,
                                                   link_prefix);
    } else if (type == OpenRAVE::PT_Sensor && interface_name == "Handstatesensor") {
        return boost::make_shared<HandstateSensor>(env, owd_namespace);
    } else {
        throw OpenRAVE::openrave_exception("This should never happen.");
    }
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

