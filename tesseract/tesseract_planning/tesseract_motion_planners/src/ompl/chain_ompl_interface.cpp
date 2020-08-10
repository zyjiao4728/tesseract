#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_motion_planners/ompl/chain_ompl_interface.h"

namespace tesseract_motion_planners
{
ChainOmplInterface::ChainOmplInterface(tesseract_environment::Environment::ConstPtr env,
                                       tesseract_kinematics::ForwardKinematics::ConstPtr kin)
  : env_(std::move(env)), kin_(std::move(kin))
{
  joint_names_ = kin_->getJointNames();

  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  tesseract_environment::AdjacencyMap adj_map(
      env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->transforms);
  link_names_ = adj_map.getActiveLinkNames();

  const auto dof = kin_->numJoints();
  const auto& limits = kin_->getLimits();

  // Construct the OMPL state space for this manipulator
  ompl::base::RealVectorStateSpace* space = new ompl::base::RealVectorStateSpace();
  for (unsigned i = 0; i < dof; ++i)
  {
    space->addDimension(joint_names_[i], limits(i, 0), limits(i, 1));
  }

  ompl::base::StateSpacePtr state_space_ptr(space);

  state_space_ptr->setValidSegmentCountFactor(2);
  state_space_ptr->setLongestValidSegmentFraction(0.3);

  // std::cout << "state space dimension is " << (int)dof << std::endl;

  ss_.reset(new ompl::geometric::SimpleSetup(state_space_ptr));

  // std::cout << " reset passed " << std::endl;

  // Setup state checking functionality
  ss_->setStateValidityChecker(std::bind(&ChainOmplInterface::isStateValid, this, std::placeholders::_1));

  // ss_->getSpaceInformation()->setMinMaxControlDuration(n_step, n_step);

  contact_manager_ = env_->getDiscreteContactManager();

  // tesseract_environment::AdjacencyMap::Ptr adjacency_map =
  //   std::make_shared<tesseract_environment::AdjacencyMap>(env->getSceneGraph(),
  //                                                         kin->getActiveLinkNames(),
  //                                                         env->getCurrentState()->transforms);

  // contact_manager_->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  // contact_manager_->setContactDistanceThreshold(0);

  contact_manager_->setActiveCollisionObjects(link_names_);
  contact_manager_->setContactDistanceThreshold(0);

  // std::cout << "coi finished " << std::endl;
  // We need to set the planner and call setup before it can run
}

void ChainOmplInterface::setAdjacencyMap()
{
  // std::cout << "setting map" << std::endl;
  this->contact_manager_ = this->env_->getDiscreteContactManager();

  // std::cout << "setting map" << std::endl;
  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      this->env_->getSceneGraph(), this->kin_->getActiveLinkNames(), this->env_->getCurrentState()->transforms);

  // std::cout << "setting map" << std::endl;
  this->contact_manager_->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  this->contact_manager_->setContactDistanceThreshold(0);
}

boost::optional<ompl::geometric::PathGeometric> ChainOmplInterface::plan(ompl::base::PlannerPtr planner,
                                                                         const std::vector<double>& from,
                                                                         const std::vector<double>& to,
                                                                         const OmplPlanParameters& params)
{
  // ss_->setPlanner(planner);
  // std::cout << "debug 1" << std::endl;
  // planner->clear();
  // std::cout << "planner cleared" << std::endl;
  std::cout << ss_->getStateSpace()->getDimension() << std::endl;

  const auto dof = ss_->getStateSpace()->getDimension();

  ompl::base::ScopedState<> start_state(ss_->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = from[i];

  // std::cout << "debug 2" << std::endl;

  ompl::base::ScopedState<> goal_state(ss_->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    goal_state[i] = to[i];

  ss_->setStartAndGoalStates(start_state, goal_state);

  // std::cout << "debug 3" << std::endl;

  ompl::base::PlannerStatus status = ss_->solve(params.planning_time);

  // std::cout << "debug 4" << std::endl;
  if (status)
  {
    if (params.simplify)
    {
      ss_->simplifySolution();
    }
    else
    {
    }

    int num_output_states = params.n_steps;
    if (ss_->getSolutionPath().getStateCount() < num_output_states)
    {
      ss_->getSolutionPath().interpolate(num_output_states);
    }

    ompl::geometric::PathGeometric& path = ss_->getSolutionPath();
    return boost::optional<ompl::geometric::PathGeometric>{ path };
  }
  else
  {
    return {};
  }
}

ompl::base::SpaceInformationPtr ChainOmplInterface::spaceInformation() { return ss_->getSpaceInformation(); }

bool ChainOmplInterface::isStateValid(const ompl::base::State* state) const
{
  const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto dof = joint_names_.size();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

  // Need to get thread id
  tesseract_collision::DiscreteContactManager::Ptr cm = contact_manager_->clone();
  cm->setCollisionObjectsTransform(env_state->transforms);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  bool constric_sat = checkConstraints(state);


  if (contact_map.size() ==1 ){
    if (contact_map.begin()->first.first.compare("ur_arm_forearm_link")==0 && contact_map.begin()->first.second.compare("ur_arm_wrist_3_link")==0){
      contact_map.clear();
    }
  }

  if (contact_map.empty() && constric_sat)
    return true;

  return false;
  // return contact_map.empty();
}

void ChainOmplInterface::setConstraints(std::string link_name, const ompl::base::State* state, double tor)
{
  const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto dof = joint_names_.size();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

  this->constraint_name_ = link_name;
  this->goal_transform_ = env_state->transforms.find(link_name)->second;
  this->tor_ = tor;
}

void ChainOmplInterface::setConstraintsStd(std::string link_name, Eigen::Isometry3d & trans, double tor)
{
  // const auto dof = joint_names_.size();

  // Eigen::Map<Eigen::VectorXd> joint_angles(state.data(), long(dof));

  // tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

  this->constraint_name_ = link_name;
  // this->goal_transform_ = env_state->transforms.find(link_name)->second;
  this->goal_transform_ = trans;
  this->tor_ = tor;
}

bool ChainOmplInterface::checkConstraints(const ompl::base::State* state) const
{
  const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto dof = joint_names_.size();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

  if (env_state->transforms.find(this->constraint_name_) != env_state->transforms.end())
  {
    Eigen::Isometry3d temp = env_state->transforms.find(this->constraint_name_)->second;

    // std::cout << "current transform is" << std::endl;
    // std::cout << temp.matrix() << std::endl;

    // std::cout << "goal transform is" << std::endl;
    // std::cout << this->goal_transform_.matrix() << std::endl;
    Eigen::Isometry3d diff_trans = this->goal_transform_ * temp.inverse();
    Eigen::Vector3d lin = diff_trans.translation();
    Eigen::Vector3d rot = diff_trans.rotation().eulerAngles(0, 1, 2);
    if (lin.norm() > this->tor_)
      return false;
  }

  return true;
}

bool ChainOmplInterface::isStateValidStd(std::vector<double> wp) const
{
  const auto dof = joint_names_.size();

  Eigen::Map<Eigen::VectorXd> joint_angles(wp.data(), long(dof));
  std::cout << "checking states" << joint_angles << std::endl;

  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

  // Need to get thread id
  tesseract_collision::DiscreteContactManager::Ptr cm = contact_manager_->clone();
  cm->setCollisionObjectsTransform(env_state->transforms);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);


  if (contact_map.size() ==1 ){
    if (contact_map.begin()->first.first.compare("ur_arm_forearm_link")==0 && contact_map.begin()->first.second.compare("ur_arm_wrist_3_link")==0){
      return true;
    }
  }
  // for (auto& x : contact_map)
  // {
  //   std::cout << x.first.first << " => " << x.first.second << '\n';
  // }
  return contact_map.empty();
}

}  // namespace tesseract_motion_planners
