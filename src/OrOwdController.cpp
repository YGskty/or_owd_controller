#include <openrave/openrave.h>
#include <ros/ros.h>
#include <owd_msgs/WAMState.h>
#include "OrOwdController.h"

namespace or_owdcontroller
{
   
OrOwdController::OrOwdController(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput)
   : OpenRAVE::ControllerBase(penv)
{
   this->np = 0;
   this->is_initialized = false;
   this->__description = "OrOwdController: controller for an OWD robot.";
   sinput >> this->owdns;
   printf("Constructor got owdns: |%s|\n", this->owdns.c_str());
}

OrOwdController::~OrOwdController()
{
   if (this->np) delete this->np;
   printf("OrOwdController Destructed!\n");
}

bool OrOwdController::Init(OpenRAVE::RobotBasePtr r, const std::vector<int> &dofs, int nControlTransformation)
{
   std::map<std::string, std::string> remappings;
   
   printf("OrOwdController Init called!\n");
   this->r = r;
   this->dofs = dofs;
   
   /* initialize ROS;
    * note, since we're an openrave plugin among potentially several others
    * with their own "ROS nodes", only the first init will actually succeed ... */
   try {
      printf("initializing node ...\n");
      ros::init(remappings, "orowdcontroller", 0);
      printf("done!\n");
   }
   catch (ros::InvalidNodeNameException ex) {
      RAVELOG_ERROR("Could not initialize ROS!\n");
      return false;
   }
   
   /* get a nodehandle */
   printf("getting nodehandle ...\n");
   this->np = new ros::NodeHandle();
   printf("got nodehandle %p!\n", this->np);
   
   /* set up some sweet subscribers */
   printf("waiting for first message ...\n");
   owd_msgs::WAMStateConstPtr m = ros::topic::waitForMessage<owd_msgs::WAMState>(this->owdns + "/wamstate");
   if (!m)
   {
      RAVELOG_ERROR("didn't receive a message! are we shutting down?\n");
      delete this->np;
      this->np = 0;
      return false;
   }
   this->wamstate_last = *m;
   if (this->dofs.size() != this->wamstate_last.positions.size())
   {
      RAVELOG_ERROR("number of dofs doesn't match!\n");
      delete this->np;
      this->np = 0;
      return false;
   }
   this->wamstate_sub = this->np->subscribe(this->owdns + "/wamstate", 1000, &OrOwdController::wamstate_cb, this);
   
   this->is_initialized = true;
   return true;
}

const std::vector<int>& OrOwdController::GetControlDOFIndices() const
{
   return this->dofs;
}

int OrOwdController::IsControlTransformation() const
{
   return 0;
}

OpenRAVE::RobotBasePtr OrOwdController::GetRobot() const
{
   return OpenRAVE::RobotBasePtr();
}

void OrOwdController::Reset(int options)
{
}

bool OrOwdController::SetDesired(const std::vector<double> &des, OpenRAVE::TransformConstPtr t)
{
   return false;
}

bool OrOwdController::SetPath(OpenRAVE::TrajectoryBaseConstPtr traj)
{
   return false;
}

void OrOwdController::SimulationStep(OpenRAVE::dReal dt)
{
   std::vector< OpenRAVE::dReal > dofvals;
   
   if (!this->is_initialized) return;

   ros::spinOnce();
   
   assert(this->dofs.size() == this->wamstate_last.positions.size());
   
   printf("updating robot from %s ...\n", this->owdns.c_str());
   this->r->GetDOFValues(dofvals);
   for (unsigned int i=0; i<this->dofs.size(); i++)
   {
      dofvals[this->dofs[i]] = this->wamstate_last.positions[i];
   }
   this->r->SetDOFValues(dofvals);
}

bool OrOwdController::IsDone()
{
   return true;
}

void OrOwdController::wamstate_cb(const owd_msgs::WAMState::ConstPtr& msg)
{
   this->wamstate_last = *msg;
}

} /* namespace or_owdcontroller */

