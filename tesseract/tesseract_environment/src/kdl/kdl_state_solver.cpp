/**
 * @file kdl_state_solver.cpp
 * @brief Tesseract environment kdl solver implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <tesseract_scene_graph/parser/kdl_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_environment/kdl/kdl_state_solver.h"
#include "tesseract_environment/kdl/kdl_utils.h"
namespace tesseract_environment
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

bool KDLStateSolver::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph)
{
  scene_graph_ = std::move(scene_graph);
  return createKDETree();
}

void KDLStateSolver::setState(const std::unordered_map<std::string, double>& joints)
{
  for (auto& joint : joints)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint.first, joint.second))
    {
      current_state_->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(
      current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
}

void KDLStateSolver::setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values)
{
  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
    {
      current_state_->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(
      current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
}

void KDLStateSolver::setState(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
    {
      current_state_->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(
      current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
}

EnvState::Ptr KDLStateSolver::getState(const std::unordered_map<std::string, double>& joints) const
{
  EnvState::Ptr state(new EnvState(*current_state_));
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto& joint : joints)
  {
    if (setJointValuesHelper(jnt_array, joint.first, joint.second))
    {
      state->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(state->transforms, jnt_array, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

EnvState::Ptr KDLStateSolver::getState(const std::vector<std::string>& joint_names,
                                       const std::vector<double>& joint_values) const
{
  EnvState::Ptr state(new EnvState(*current_state_));
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, joint_names[i], joint_values[i]))
    {
      state->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(state->transforms, jnt_array, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

EnvState::Ptr KDLStateSolver::getState(const std::vector<std::string>& joint_names,
                                       const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  EnvState::Ptr state(new EnvState(*current_state_));
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0u; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, joint_names[i], joint_values[i]))
    {
      state->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(state->transforms, jnt_array, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

bool KDLStateSolver::createKDETree()
{
  kdl_tree_.reset(new KDL::Tree());
  if (!tesseract_scene_graph::parseSceneGraph(*scene_graph_, *kdl_tree_))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return false;
  }

  current_state_ = EnvState::Ptr(new EnvState());
  kdl_jnt_array_.resize(kdl_tree_->getNrOfJoints());
  size_t j = 0;
  for (const auto& seg : kdl_tree_->getSegments())
  {
    const KDL::Joint& jnt = seg.second.segment.getJoint();

    if (jnt.getType() == KDL::Joint::None)
      continue;

    joint_to_qnr_.insert(std::make_pair(jnt.getName(), seg.second.q_nr));
    kdl_jnt_array_(seg.second.q_nr) = 0.0;
    current_state_->joints.insert(std::make_pair(jnt.getName(), 0.0));

    j++;
  }

  calculateTransforms(
      current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Isometry3d::Identity());
  return true;
}

bool KDLStateSolver::setJointValuesHelper(KDL::JntArray& q,
                                          const std::string& joint_name,
                                          const double& joint_value) const
{
  auto qnr = joint_to_qnr_.find(joint_name);
  if (qnr != joint_to_qnr_.end())
  {
    q(qnr->second) = joint_value;
    // std::cout << joint_name << ":\t" << joint_value << std::endl;
    // std::cout << qnr->second << std::endl;
    return true;
  }
  else
  {
    CONSOLE_BRIDGE_logError("Tried to set joint name %s which does not exist!", joint_name.c_str());
    return false;
  }
}

void KDLStateSolver::calculateTransformsHelper(tesseract_common::TransformMap& transforms,
                                               const KDL::JntArray& q_in,
                                               const KDL::SegmentMap::const_iterator& it,
                                               const Eigen::Isometry3d& parent_frame) const
{
  if (it != kdl_tree_->getSegments().end())
  {
    const KDL::TreeElementType& current_element = it->second;

    auto qnr = joint_to_qnr_.find(current_element.segment.getJoint().getName());
    unsigned int joint_idx = 0;
    if (qnr == joint_to_qnr_.end())
    {
      // ROS_ERROR("Cannot find joint %s for link %s.",current_element.segment.getJoint().getName().c_str(), current_element.segment.getName().c_str());
      joint_idx = GetTreeElementQNr(current_element);
    }
    else{
      joint_idx = qnr->second;
    }
    
    KDL::Frame current_frame = GetTreeElementSegment(current_element).pose(q_in(joint_idx));

    Eigen::Isometry3d local_frame, global_frame;
    KDLToEigen(current_frame, local_frame);
    global_frame = parent_frame * local_frame;
    transforms[current_element.segment.getName()] = global_frame;
    // if (current_element.segment.getName() == "door0_base_link")
    //   {
    //     std::cout << current_element.segment.getJoint().getName() << std::endl;
    //     std::cout << q_in.data << std::endl;
    //     std::cout << GetTreeElementQNr(current_element) << std::endl;
    //     std::cout << "current q_in:\t" << q_in(GetTreeElementQNr(current_element)) << std::endl;
    //     std::cout << parent_frame.translation() << std::endl;
    //     std::cout << parent_frame.linear() << std::endl;
    //     std::cout << global_frame.translation() << std::endl;
    //     std::cout << global_frame.linear() << std::endl;
    //   }

    for (auto& child : current_element.children)
    {
      calculateTransformsHelper(transforms, q_in, child, global_frame);
    }
  }
}

void KDLStateSolver::calculateTransforms(tesseract_common::TransformMap& transforms,
                                         const KDL::JntArray& q_in,
                                         const KDL::SegmentMap::const_iterator& it,
                                         const Eigen::Isometry3d& parent_frame) const
{
  calculateTransformsHelper(transforms, q_in, it, parent_frame);
}

}  // namespace tesseract_environment
