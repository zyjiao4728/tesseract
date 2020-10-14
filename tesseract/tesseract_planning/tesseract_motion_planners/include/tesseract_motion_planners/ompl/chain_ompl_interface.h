#ifndef TESSERACT_MOTION_PLANNERS_CHAIN_OMPL_INTERFACE_H
#define TESSERACT_MOTION_PLANNERS_CHAIN_OMPL_INTERFACE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>

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


  void resetKinematicsLimit(Eigen::MatrixX2d joint_limit);

  boost::optional<ompl::geometric::PathGeometric> plan(ompl::base::PlannerPtr planner,
                                                       const std::vector<double>& from,
                                                       const std::vector<double>& to,
                                                       const OmplPlanParameters& params);

  boost::optional<ompl::geometric::PathGeometric> plan(const std::vector<double>& from,
                                                       const std::vector<double>& to,
                                                       const OmplPlanParameters& params);

  ompl::base::SpaceInformationPtr spaceInformation();

  Eigen::MatrixXd toEigenArray(const ompl::geometric::PathGeometric &path);

  Eigen::MatrixXd L2NormWeightedInterpolation( ompl::geometric::PathGeometric & wpts , const OmplPlanParameters& params);

  void useL2NormWeightedInterpolation(bool option);

  void setMotionValidator(ompl::base::MotionValidatorPtr mv)
  {
    ss_->getSpaceInformation()->setMotionValidator(std::move(mv));
  }

  void setAdjacencyMap();

  void populateAdjacent();

  void populateInclusion(std::vector<std::string> link_names);

  std::map<std::string, std::string> getAdjacent() const { return this->adj_map; };

  std::vector<std::string> getInclusion() const { return this->inclusive_names_; };

  bool filterAdjacent(std::string link_name1, std::string link_name2) const;

  bool filterInclusion(std::string link_name1, std::string link_name2) const;

  bool checkResultSatisfy(ompl::geometric::PathGeometric& path, const std::vector<double>& to);

  bool isStateValidStd(std::vector<double> wp) const;

  void setConstraints(std::string link_name, const ompl::base::State* state, double tor = 0.05);

  void setConstraintsStd(std::string link_name, Eigen::Isometry3d& trans, double tor = 0.05);

  bool checkConstraints(const ompl::base::State* state) const;

  bool satisfyVelocity(boost::optional<ompl::geometric::PathGeometric>& sol);

  ompl::geometric::SimpleSetupPtr ss_;

  void setSamplerWeights(Eigen::VectorXd wts);
  ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space) const;


tesseract_kinematics::ForwardKinematics::ConstPtr GetKin();
private:
  bool isStateValid(const ompl::base::State* state) const;

private:
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  Eigen::VectorXd weights;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<std::string> inclusive_names_;
  std::map<std::string, std::string> adj_map;

  ompl::base::StateSamplerAllocator state_sampler_allocator;
  
  bool use_l2_weighted_norm;

  std::string constraint_name_;
  Eigen::Isometry3d goal_transform_;
  double tor_;
};
}  // namespace tesseract_motion_planners

#endif
