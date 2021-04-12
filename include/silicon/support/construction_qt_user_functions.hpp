/**
 * \file construction_qt_user_functions.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of SILICON.
 *
 * SILICON is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * SILICON is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * SILICON.  If not, see <http://www.gnu.org/licenses/
 */
#ifndef INCLUDE_SILICON_SUPPORT_CONSTRUCTION_QT_USER_FUNCTIONS_HPP_
#define INCLUDE_SILICON_SUPPORT_CONSTRUCTION_QT_USER_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "cosm/hal/robot.hpp"

#include "silicon/controller/controller_fwd.hpp"
#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class construction_qt_user_functions
 * \ingroup support
 *
 * \brief Contains hooks for Qt to draw the the following visualization' for
 * each robot, per configuration:
 *
 * - Block currently carried
 */
class construction_qt_user_functions : public argos::CQTOpenGLUserFunctions {
 public:
  /**
   * \brief How far above the center of the robot to draw the carried block (if
   * the robot is carrying a block)
   */
  static constexpr double kBLOCK_VIS_OFFSET = 0.3;

  /**
   * \brief How far above the center of the robot to draw text (robot id, task,
   * etc.)
   */
  static constexpr double kTEXT_VIS_OFFSET = 0.5;

  construction_qt_user_functions(void);

  ~construction_qt_user_functions(void) override = default;

  void Draw(chal::robot& c_entity);

 private:
  void los_render(const controller::constructing_controller* controller,
                  const argos::CQuaternion& orientation);
  void nearest_ct_render(const controller::constructing_controller* controller,
                         const argos::CQuaternion& orientation);
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_CONSTRUCTION_QT_USER_FUNCTIONS_HPP_ */
