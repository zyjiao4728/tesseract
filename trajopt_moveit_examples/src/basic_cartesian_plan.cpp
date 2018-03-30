#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <trajopt_moveit/trajopt_moveit_env.h>
#include <trajopt_scene/kdl_chain_kin.h>
#include <trajopt/problem_description.hpp>
#include <trajopt/plot_callback.hpp>

#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/config.hpp>


#include <jsoncpp/json/json.h>

using namespace trajopt;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string TRAJOPT_DESCRIPTION_PARAM = "trajopt_description"; /**< Default ROS parameter for trajopt description */

bool plotting_ = false;
int steps_ = 5;
std::string method_ = "json";
robot_model_loader::RobotModelLoaderPtr loader_;  /**< Used to load the robot model */
moveit::core::RobotModelPtr robot_model_;         /**< Robot model */
planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene for the current robot model */
trajopt_moveit::TrajOptMoveItEnvPtr env_;         /**< Trajopt Basic Environment */


TrajOptProbPtr jsonMethod()
{
  ros::NodeHandle nh;
  std::string trajopt_config;

  nh.getParam(TRAJOPT_DESCRIPTION_PARAM, trajopt_config);

  Json::Value root;
  Json::Reader reader;
  bool parse_success = reader.parse(trajopt_config.c_str(), root);
  if (!parse_success)
  {
    ROS_FATAL("Failed to load trajopt json file from ros parameter");
  }

  return ConstructProblem(root, env_);
}

TrajOptProbPtr cppMethod()
{
  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps_;
  pci.basic_info.manip = "manipulator";
  pci.basic_info.start_fixed = false;
//  pci.basic_info.dofs_fixed

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());

  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Populate Cost Info
  boost::shared_ptr<JointVelCostInfo> jv = boost::shared_ptr<JointVelCostInfo>(new JointVelCostInfo);
  jv->coeffs = std::vector<double>(7, 5.0);
  jv->name = "joint_vel";
  jv->term_type = TT_COST;
  pci.cost_infos.push_back(jv);

  boost::shared_ptr<CollisionCostInfo> collision = boost::shared_ptr<CollisionCostInfo>(new CollisionCostInfo);
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  collision->coeffs = DblVec(pci.basic_info.n_steps, 20.0);
  collision->dist_pen = DblVec(pci.basic_info.n_steps, 0.025);
  pci.cost_infos.push_back(collision);

  // Populate Constraints
  double delta = 0.5/pci.basic_info.n_steps;
  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
  {
    boost::shared_ptr<PoseCostInfo> pose = boost::shared_ptr<PoseCostInfo>(new PoseCostInfo);
    pose->term_type = TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "tool0";
    pose->timestep = i;
    pose->xyz = Eigen::Vector3d(0.5, -0.2 + delta * i, 0.62);
    pose->wxyz = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
    pci.cnt_infos.push_back(pose);
  }

  return ConstructProblem(pci);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_cartesian_plan");
  ros::NodeHandle pnh("~");

  // Initial setup
  loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
  robot_model_ = loader_->getModel();
  env_ = trajopt_moveit::TrajOptMoveItEnvPtr(new trajopt_moveit::TrajOptMoveItEnv);
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  assert(robot_model_ != nullptr);
  assert(planning_scene_ != nullptr);

  // Now assign collision detection plugin
  bool success;
  collision_detection::CollisionPluginLoader cd_loader;
  std::string class_name = "BULLET";

  success = cd_loader.activate(class_name, planning_scene_, true);
  assert(success);

  success = env_->init(planning_scene_);
  assert(success);

  // Get ROS Parameters
  pnh.param("plotting", plotting_, plotting_);
  pnh.param<std::string>("method", method_, method_);
  pnh.param<int>("steps", steps_, steps_);

  // Set the robot initial state
  robot_state::RobotState &rs = planning_scene_->getCurrentStateNonConst();
  std::map<std::string, double> ipos;
  ipos["joint_a1"] = -0.4;
  ipos["joint_a2"] = 0.2762;
  ipos["joint_a3"] = 0.0;
  ipos["joint_a4"] = -1.3348;
  ipos["joint_a5"] = 0.0;
  ipos["joint_a6"] = 1.4959;
  ipos["joint_a7"] = 0.0;
  rs.setVariablePositions(ipos);

  // Set Log Level
  gLogLevel = util::LevelError;

  // Setup Problem
  TrajOptProbPtr prob;
  if (method_ == "cpp")
    prob = cppMethod();
  else
    prob = jsonMethod();

  // Solve Trajectory
  ROS_INFO("basic cartesian plan example");

  trajopt_scene::DistanceResultVector collisions;
  const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
  const std::vector<std::string>& link_names = prob->GetKin()->getLinkNames();

  env_->continuousCollisionCheckTrajectory(joint_names, link_names, prob->GetInitTraj(), collisions);
  ROS_INFO("Initial trajector number of continuous collisions: %lui\n", collisions.size());

  BasicTrustRegionSQP opt(prob);
  if (plotting_)
  {
    opt.addCallback(PlotCallback(*prob));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  ros::Time tStart = ros::Time::now();
  opt.optimize();
  ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
  {
    prob->GetEnv()->plotClear();
  }

  collisions.clear();
  env_->continuousCollisionCheckTrajectory(joint_names, link_names, getTraj(opt.x(), prob->GetVars()), collisions);
  ROS_INFO("Final trajectory number of continuous collisions: %lui\n", collisions.size());

}
