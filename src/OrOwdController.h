/* requires: openrave/openrave.h
 *           ros/ros.h
 *           owd_msgs/WAMState.h */

namespace or_owdcontroller
{

class OrOwdController : public OpenRAVE::ControllerBase
{
   OpenRAVE::RobotBasePtr r;
   std::vector<int> dofs;
   std::string owdns;
   bool is_initialized;
   ros::NodeHandle * np;
   owd_msgs::WAMState wamstate_last;
   ros::Subscriber wamstate_sub;
public:
   /* openrave controller interface */
   OrOwdController(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
   ~OrOwdController();
   bool Init(OpenRAVE::RobotBasePtr r, const std::vector<int> &dofs, int nControlTransformation);
   const std::vector<int>& GetControlDOFIndices() const;
   int IsControlTransformation() const;
   OpenRAVE::RobotBasePtr GetRobot() const;
   void Reset(int options);
   bool SetDesired(const std::vector<double> &des, OpenRAVE::TransformConstPtr t);
   bool SetPath(OpenRAVE::TrajectoryBaseConstPtr traj);
   void SimulationStep(OpenRAVE::dReal dt);
   bool IsDone();
   /* other stuff */
   void wamstate_cb(const owd_msgs::WAMState::ConstPtr& msg);
};

} /* namespace or_owdcontroller */
