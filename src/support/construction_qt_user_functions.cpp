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

#include "cosm/vis/block_carry_visualizer.hpp"

#include "silicon/controller/fcrw_bst_controller.hpp"

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
  auto* base = dynamic_cast<const controller::constructing_controller*>(
      &c_entity.GetControllableEntity().GetController());

  if (base->display_id()) {
    DrawText(argos::CVector3(0.0, 0.0, 0.5), c_entity.GetId());
  }

  if (base->is_carrying_block()) {
    cvis::block_carry_visualizer(this, kBLOCK_VIS_OFFSET, kTEXT_VIS_OFFSET)
        .draw(base->block(), base->GetId().size());
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
