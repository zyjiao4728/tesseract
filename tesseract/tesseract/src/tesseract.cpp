/**
 * @file common.h
 * @brief This is a collection of common methods
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
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>

namespace tesseract
{
Tesseract::Tesseract() { clear(); }

bool Tesseract::isInitialized() const { return initialized_; }

bool Tesseract::init(tesseract_scene_graph::SceneGraph::Ptr scene_graph)
{
  clear();

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;

  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(tesseract_scene_graph::SceneGraph::Ptr scene_graph,
                     tesseract_scene_graph::SRDFModel::Ptr srdf_model)
{
  clear();

  srdf_model_ = srdf_model;

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;

  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const std::string& urdf_string, tesseract_scene_graph::ResourceLocatorFn locator)
{
  clear();

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_scene_graph::parseURDFString(urdf_string, locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return false;
  }

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const std::string& urdf_string,
                     const std::string& srdf_string,
                     tesseract_scene_graph::ResourceLocatorFn locator)
{
  clear();

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_scene_graph::parseURDFString(urdf_string, locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return false;
  }

  // Parse srdf string into SRDF Model
  tesseract_scene_graph::SRDFModel::Ptr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  if (!srdf->initString(*scene_graph, srdf_string))
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    return false;
  }
  srdf_model_ = srdf;

  // Add allowed collision matrix to scene graph
  tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf);

  // Construct Environment
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const boost::filesystem::path& urdf_path, tesseract_scene_graph::ResourceLocatorFn locator)
{
  clear();

  // Parse urdf file into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph =
      tesseract_scene_graph::parseURDFFile(urdf_path.string(), locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return false;
  }

  // Construct Environment from Scene Graph
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

bool Tesseract::init(const boost::filesystem::path& urdf_path,
                     const boost::filesystem::path& srdf_path,
                     tesseract_scene_graph::ResourceLocatorFn locator)
{
  clear();

  // Parse urdf file into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph =
      tesseract_scene_graph::parseURDFFile(urdf_path.string(), locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return false;
  }

  // Parse srdf file into SRDF Model
  tesseract_scene_graph::SRDFModel::Ptr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  if (!srdf->initFile(*scene_graph, srdf_path.string()))
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    return false;
  }
  srdf_model_ = srdf;

  // Add allowed collision matrix to scene graph
  tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf);

  // Construct Environment
  environment_ = std::make_shared<tesseract_environment::KDLEnv>();
  if (!environment_->init(scene_graph))
  {
    CONSOLE_BRIDGE_logError("Failed to initialize environment.");
    return false;
  }
  environment_const_ = environment_;
  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();

  initialized_ = true;
  return true;
}

const tesseract_environment::Environment::Ptr& Tesseract::getEnvironment() { return environment_; }

const tesseract_environment::Environment::ConstPtr& Tesseract::getEnvironmentConst() const
{
  return environment_const_;
}

tesseract_scene_graph::SRDFModel::Ptr& Tesseract::getSRDFModel() { return srdf_model_; }

const ForwardKinematicsManager::Ptr& Tesseract::getFwdKinematicsManager() { return fwd_kin_manager_; }

const ForwardKinematicsManager::ConstPtr& Tesseract::getFwdKinematicsManagerConst() const
{
  return fwd_kin_manager_const_;
}

const InverseKinematicsManager::Ptr& Tesseract::getInvKinematicsManager() { return inv_kin_manager_; }

const InverseKinematicsManager::ConstPtr& Tesseract::getInvKinematicsManagerConst() const
{
  return inv_kin_manager_const_;
}

/** @brief registerDefaultContactManagers */
bool Tesseract::registerDefaultContactManagers()
{
  using namespace tesseract_collision;

  // Register contact manager
  environment_->registerDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                               &tesseract_collision_bullet::BulletDiscreteBVHManager::create);
  environment_->registerContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                                 &tesseract_collision_bullet::BulletCastBVHManager::create);

  // Set Active contact manager
  environment_->setActiveDiscreteContactManager(tesseract_collision_bullet::BulletDiscreteBVHManager::name());
  environment_->setActiveContinuousContactManager(tesseract_collision_bullet::BulletCastBVHManager::name());

  return true;
}

bool Tesseract::registerDefaultFwdKinSolvers()
{
  bool success = true;

  fwd_kin_manager_ = std::make_shared<ForwardKinematicsManager>();
  fwd_kin_manager_const_ = fwd_kin_manager_;

  auto chain_factory = std::make_shared<tesseract_kinematics::KDLFwdKinChainFactory>();
  fwd_kin_manager_->registerFwdKinematicsFactory(chain_factory);

  auto tree_factory = std::make_shared<tesseract_kinematics::KDLFwdKinTreeFactory>();
  fwd_kin_manager_->registerFwdKinematicsFactory(tree_factory);

  for (const auto& group : srdf_model_->getGroups())
  {
    if (!group.chains_.empty())
    {
      assert(group.chains_.size() == 1);
      tesseract_kinematics::ForwardKinematics::Ptr solver = chain_factory->create(
          environment_->getSceneGraph(), group.chains_.front().first, group.chains_.front().second, group.name_);
      if (solver != nullptr)
      {
        if (!fwd_kin_manager_->addFwdKinematicSolver(solver))
        {
          CONSOLE_BRIDGE_logError("Failed to add inverse kinematic chain solver %s for manipulator %s to manager!",
                                  solver->getSolverName().c_str(),
                                  group.name_.c_str());
          success = false;
        }
      }
      else
      {
        CONSOLE_BRIDGE_logError("Failed to create inverse kinematic chain solver %s for manipulator %s!",
                                solver->getSolverName().c_str(),
                                group.name_.c_str());
        success = false;
      }
    }

    if (!group.joints_.empty())
    {
      assert(group.joints_.size() > 0);
      tesseract_kinematics::ForwardKinematics::Ptr solver =
          tree_factory->create(environment_->getSceneGraph(), group.joints_, group.name_);
      if (solver != nullptr)
      {
        if (!fwd_kin_manager_->addFwdKinematicSolver(solver))
        {
          CONSOLE_BRIDGE_logError("Failed to add inverse kinematic tree solver %s for manipulator %s to manager!",
                                  solver->getSolverName().c_str(),
                                  group.name_.c_str());
          success = false;
        }
      }
      else
      {
        CONSOLE_BRIDGE_logError("Failed to create inverse kinematic tree solver %s for manipulator %s!",
                                solver->getSolverName().c_str(),
                                group.name_.c_str());
        success = false;
      }
    }

    // TODO: Need to add other options
    if (!group.links_.empty())
    {
      CONSOLE_BRIDGE_logError("Link groups are currently not supported!");
      success = false;
    }

    if (!group.subgroups_.empty())
    {
      CONSOLE_BRIDGE_logError("Subgroups are currently not supported!");
      success = false;
    }
  }

  return success;
}

bool Tesseract::registerDefaultInvKinSolvers()
{
  bool success = true;

  inv_kin_manager_ = std::make_shared<InverseKinematicsManager>();
  inv_kin_manager_const_ = inv_kin_manager_;

  auto factory = std::make_shared<tesseract_kinematics::KDLInvKinChainLMAFactory>();
  inv_kin_manager_->registerInvKinematicsFactory(factory);

  for (const auto& group : srdf_model_->getGroups())
  {
    if (!group.chains_.empty())
    {
      assert(group.chains_.size() == 1);
      tesseract_kinematics::InverseKinematics::Ptr solver = factory->create(
          environment_->getSceneGraph(), group.chains_.front().first, group.chains_.front().second, group.name_);
      if (solver != nullptr)
      {
        if (!inv_kin_manager_->addInvKinematicSolver(solver))
        {
          CONSOLE_BRIDGE_logError("Failed to add inverse kinematic chain solver %s for manipulator %s to manager!",
                                  solver->getSolverName().c_str(),
                                  group.name_.c_str());
          success = false;
        }
      }
      else
      {
        CONSOLE_BRIDGE_logError("Failed to create inverse kinematic chain solver %s for manipulator %s!",
                                solver->getSolverName().c_str(),
                                group.name_.c_str());
        success = false;
      }
    }

    if (!group.joints_.empty())
    {
      CONSOLE_BRIDGE_logError("Joint groups are currently not supported by inverse kinematics!");
      success = false;
    }

    // TODO: Need to add other options
    if (!group.links_.empty())
    {
      CONSOLE_BRIDGE_logError("Link groups are currently not supported by inverse kinematics!");
      success = false;
    }

    if (!group.subgroups_.empty())
    {
      CONSOLE_BRIDGE_logError("Subgroups are currently not supported!");
      success = false;
    }
  }

  return success;
}

void Tesseract::clear()
{
  initialized_ = false;
  environment_ = nullptr;
  environment_const_ = nullptr;
  srdf_model_ = nullptr;
  inv_kin_manager_ = nullptr;
  fwd_kin_manager_ = nullptr;
}

void Tesseract::clearKinematics()
{
  inv_kin_manager_ = nullptr;
  fwd_kin_manager_ = nullptr;
  fwd_kin_manager_const_ = nullptr;
  inv_kin_manager_const_ = nullptr;
}

}  // namespace tesseract