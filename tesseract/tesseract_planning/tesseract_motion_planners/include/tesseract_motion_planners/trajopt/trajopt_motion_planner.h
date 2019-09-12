/**
 * @file trajopt_planner.h
 * @brief Tesseract ROS Trajopt planner
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/planner.h>

namespace tesseract_motion_planners
{
struct TrajOptPlannerConfig
{
  TrajOptPlannerConfig(trajopt::TrajOptProb::Ptr prob) : prob(prob) {}
  virtual ~TrajOptPlannerConfig() {}
  /** @brief Trajopt problem to be solved (Required) */
  trajopt::TrajOptProb::Ptr prob;

  /** @brief Optimization parameters to be used (Optional) */
  sco::BasicTrustRegionSQPParameters params;

  /** @brief Callback functions called on each iteration of the optimization (Optional) */
  std::vector<sco::Optimizer::Callback> callbacks;
};

class TrajOptMotionPlanner : public MotionPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajOptMotionPlanner(const std::string& name = "TRAJOPT");

  ~TrajOptMotionPlanner() {}

  /**
   * @brief Set the configuration for the planner
   *
   * This must be called prior to calling solve.
   *
   * @param config The planners configuration
   * @return True if successful otherwise false
   */
  bool setConfiguration(const TrajOptPlannerConfig& config);

  /**
   * @brief Sets up the opimizer and solves a SQP problem read from json with no callbacks and dafault parameterss
   * @param response The results of the optimization. Primary output is the optimized joint trajectory
   * @return true if optimization complete
   */
  bool solve(PlannerResponse& response) override;

  bool solve(PlannerResponse& response, std::vector<double>& cost_vals, std::vector<double>& cnt_viols, double& total_cost);


  bool terminate() override;

  void clear() override;

  /**
   * @brief checks whether the planner is properly configure for solving a motion plan
   * @return True when it is configured correctly, false otherwise
   */
  bool isConfigured() const override;

protected:
  std::shared_ptr<TrajOptPlannerConfig> config_;
};
}  // namespace tesseract_motion_planners
#endif  // TESSERACT_PLANNING_TRAJOPT_PLANNER_H
