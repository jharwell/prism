/**
 * \file construction_qt_user_functions.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of PRISM.
 *
 * PRISM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * PRISM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * PRISM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/algorithm/convex_hull2D.hpp"
RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_OVERLOADED_VIRTUAL()
#include "prism/support/construction_qt_user_functions.hpp"
RCPPSW_WARNING_DISABLE_POP()

#include <argos3/core/simulator/entity/controllable_entity.h>

#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/vis/block_carry_visualizer.hpp"
#include "cosm/vis/polygon2D_visualizer.hpp"
#include "cosm/vis/steer2D_visualizer.hpp"

#include "prism/controller/fcrw_bst_controller.hpp"
#include "prism/controller/perception/builder_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
construction_qt_user_functions::construction_qt_user_functions(void) {
  RegisterUserFunction<construction_qt_user_functions, chal::robot>(
      &construction_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void construction_qt_user_functions::Draw(chal::robot& c_entity) {
  const auto* controller =
      dynamic_cast<const controller::constructing_controller*>(
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
    los_render(controller,
               c_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation);
  }

  if (controller->display_nearest_ct() &&
      nullptr != controller->perception()->nearest_ct()) {
    nearest_ct_render(controller,
                      c_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation);
  }
}

void construction_qt_user_functions::los_render(
    const controller::constructing_controller* controller,
    const argos::CQuaternion& orientation) {
  const auto* los = controller->perception()->los();
  const auto* ct = controller->perception()->nearest_ct();

  auto [v_begin, v_end] = los->vertices();
  std::vector<rmath::vector2d> points;
  for (auto vd = v_begin; vd != v_end; ++vd) {
    auto ll = ct->roriginr().to_2D() + rmath::zvec2dvec(los->access(*vd)->coord,
                                                        ct->block_unit_dim().v()).to_2D();
    auto ul = ll + rmath::vector2d::Y * ct->block_unit_dim().v();
    auto ur = ul + rmath::vector2d::X * ct->block_unit_dim().v();
    auto lr = ur - rmath::vector2d::Y * ct->block_unit_dim().v();
    points.push_back(ll);
    points.push_back(ul);
    points.push_back(ur);
    points.push_back(lr);
  } /* for(vd..) */
  auto hull = ralgorithm::convex_hull2D<rmath::vector2d>()(std::move(points));

  std::vector<rmath::vector2d> vec(hull->begin(), hull->end());
  cvis::polygon2D_visualizer(this).abs_draw(
      controller->rpos3D(), orientation, vec, rutils::color::kYELLOW);
} /* los_render() */

void construction_qt_user_functions::nearest_ct_render(
    const controller::constructing_controller* controller,
    const argos::CQuaternion& orientation) {
  const auto* ct = controller->perception()->nearest_ct();
  auto bbr = ct->bbd(false);
  auto bbv = ct->bbd(true);

  /*
   * Because of all values in a given cell "snapping" to the LL corner, we need
   * to correct for that to render the actual boundaries of the target.
   */
  auto corr_x = rmath::vector2d::X * ct->block_unit_dim().v();
  auto corr_y = rmath::vector2d::Y * ct->block_unit_dim().v();

  auto llr = ct->as_rcoord(rmath::vector3z(0, 0, 0));
  auto ulr = ct->as_rcoord(rmath::vector3z(0, bbr.y(), 0));
  auto urr = ct->as_rcoord(rmath::vector3z(bbr.x(), bbr.y(), 0));
  auto lrr = ct->as_rcoord(rmath::vector3z(bbr.x(), 0, 0));

  auto llv = ct->as_vcoord(rmath::vector3z(0, 0, 0));
  auto ulv = ct->as_vcoord(rmath::vector3z(0, bbv.y(), 0));
  auto urv = ct->as_vcoord(rmath::vector3z(bbv.x(), bbv.y(), 0));
  auto lrv = ct->as_vcoord(rmath::vector3z(bbv.x(), 0, 0));

  std::vector<rmath::vector2d> ct_rpoints = {
    ct->anchor_loc_abs(llr).to_2D(),
    ct->anchor_loc_abs(ulr).to_2D(),
    ct->anchor_loc_abs(urr).to_2D(),
    ct->anchor_loc_abs(lrr).to_2D()
  };
  std::vector<rmath::vector2d> ct_vpoints = {
    ct->anchor_loc_abs(llv).to_2D(),
    ct->anchor_loc_abs(ulv).to_2D(),
    ct->anchor_loc_abs(urv).to_2D(),
    ct->anchor_loc_abs(lrv).to_2D()
  };
  /* Draw BOTH virtual and real bounding boxes in 2D */
  cvis::polygon2D_visualizer v(this);
  v.abs_draw(controller->rpos3D(), orientation, ct_rpoints, rutils::color::kRED);
  v.abs_draw(
      controller->rpos3D(), orientation, ct_vpoints, rutils::color::kORANGE);
} /* nearest_ct_render() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()
REGISTER_QTOPENGL_USER_FUNCTIONS(construction_qt_user_functions,
                                 "construction_qt_user_functions"); // NOLINT
RCPPSW_WARNING_DISABLE_POP()

NS_END(support, prism);
