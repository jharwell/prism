/**
 * \file construction_qt_user_functions.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_OVERLOADED_VIRTUAL()
#include "silicon/support/construction_qt_user_functions.hpp"
RCPPSW_WARNING_DISABLE_POP()

#include <argos3/core/simulator/entity/controllable_entity.h>

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/vis/block_carry_visualizer.hpp"
#include "cosm/vis/los_visualizer.hpp"
#include "cosm/vis/steer2D_visualizer.hpp"

#include "silicon/controller/fcrw_bst_controller.hpp"
#include "silicon/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(silicon, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
construction_qt_user_functions::construction_qt_user_functions(void) {
  RegisterUserFunction<construction_qt_user_functions, argos::CFootBotEntity>(
      &construction_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void construction_qt_user_functions::Draw(argos::CFootBotEntity& c_entity) {
  auto* controller = dynamic_cast<const controller::constructing_controller*>(
      &c_entity.GetControllableEntity().GetController());

  if (controller->display_id()) {
    DrawText(argos::CVector3(0.0, 0.0, 0.5), c_entity.GetId());
  }

  if (controller->is_carrying_block()) {
    cvis::block_carry_visualizer(this, kBLOCK_VIS_OFFSET, kTEXT_VIS_OFFSET)
        .draw(controller->block(), controller->GetId().size());
  }
  if (controller->display_steer2D()) {
    auto steering = cvis::steer2D_visualizer(this, kTEXT_VIS_OFFSET);
    steering(controller->rpos3D(),
             c_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation,
             controller->saa()->steer_force2D().tracker());
  }
  if (controller->display_los() && nullptr != controller->perception()->los()) {
    auto* los = controller->perception()->los();
    auto* ct = controller->perception()->nearest_ct();

    /*
     * ARGos Qt user functions draw things relative to the robot's current
     * position, and the LOS corners are absolute coordinates, so we need to
     * transform. Also, the absolute cell location returned is the location of
     * the lower left coordinate of the cell, which is fine for the minimum X/Y
     * coordinates of the LOS corners, but off by the size of a cell for the
     * maximum X/Y coordinates of the LOS corners, so we need to account for
     * that too.
     */
    double correction = ct->block_unit_dim();
    std::vector<rmath::vector2d> points = {
        ct->cell_loc_abs(los->abs_ll()).to_2D() - controller->rpos2D(),
        ct->cell_loc_abs(los->abs_ul()).to_2D() - controller->rpos2D() +
            rmath::vector2d(0.0, correction),
        ct->cell_loc_abs(los->abs_ur()).to_2D() - controller->rpos2D() +
            rmath::vector2d(correction, correction),
        ct->cell_loc_abs(los->abs_lr()).to_2D() - controller->rpos2D() +
            rmath::vector2d(correction, 0.0)};
    cvis::los_visualizer(this)(points);
  }
}

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()
REGISTER_QTOPENGL_USER_FUNCTIONS(construction_qt_user_functions,
                                 "construction_qt_user_functions"); // NOLINT
RCPPSW_WARNING_DISABLE_POP()

NS_END(support, silicon);
