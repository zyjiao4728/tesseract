#ifndef TESSERACT_MOTION_PLANNERS_CHAIN_OMPL_INTERFACE_H
#define TESSERACT_MOTION_PLANNERS_CHAIN_OMPL_INTERFACE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_motion_planners
{
struct OmplPlanParameters
{
  double planning_time = 5.0;
  bool simplify = true;
  int n_steps = 1;
};

class ChainOmplInterface
{
public:
  ChainOmplInterface(tesseract_environment::Environment::ConstPtr env,
                     tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  boost::optional<ompl::geometric::PathGeometric> plan(ompl::base::PlannerPtr planner,
                                                       const std::vector<double>& from,
                                                       const std::vector<double>& to,
                                                       const OmplPlanParameters& params);

  ompl::base::SpaceInformationPtr spaceInformation();

  void setMotionValidator(ompl::base::MotionValidatorPtr mv)
  {
    ss_->getSpaceInformation()->setMotionValidator(std::move(mv));
  }

  void setAdjacencyMap();


  bool isStateValidStd(std::vector<double> wp) const;

  void setConstraints(std::string link_name , const ompl::base::State* state , double tor = 0.05);

  void setConstraintsStd(std::string link_name, Eigen::Isometry3d & trans, double tor = 0.05);

  bool checkConstraints(const ompl::base::State* state) const;

  ompl::geometric::SimpleSetupPtr ss_;

  
private:
  bool isStateValid(const ompl::base::State* state) const;

private:
  
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  std::string constraint_name_;
  Eigen::Isometry3d goal_transform_;
  double tor_;
};
}  // namespace tesseract_motion_planners

#endif
