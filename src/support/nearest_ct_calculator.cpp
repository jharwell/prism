/**
 * \file nearest_ct_calculator.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "prism/support/nearest_ct_calculator.hpp"

#include "prism/controller/constructing_controller.hpp"
#include "prism/gmt/spc_gmt.hpp"
#include "prism/gmt/repr/vshell.hpp"
#include "prism/gmt/ds/connectivity_graph.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, support);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
gmt::spc_gmt* nearest_ct_calculator::nearest_ct(
    const controller::constructing_controller* c) const {
  /*
   * We check if the robot is currently inside the a given target's boundaries
   * INCLUDING the virtual cells that surround it; this is necessary so that
   * robots will get a LOS and correctly compute the placement paths for the
   * last block in a lane on a given level.
   */
  auto target_it = std::find_if(mc_targets.begin(),
                                mc_targets.end(),
                                [&](auto* target) {
                                  return target->contains(c->rpos2D(), true);
                                });

  if (mc_targets.end() == target_it) {
    return nullptr;
  } else {
    return (*target_it);
  }
} /* robot_target() */

boost::optional<rmath::vector3z> nearest_ct_calculator::nearest_ct_cell(
    const controller::constructing_controller* c,
    const gmt::spc_gmt* ct) const {
  if (nullptr == ct) {
    return boost::none;
  }
  rmath::vector3z cell;
  /* easy case: robot currently inside the target bounding box */
  if (ct->contains(c->rpos2D(), false)) {
    cell = c->dpos3D() - ct->vshell()->real()->danchor3D();
  } else if (ct->contains(c->rpos2D(), true)) {
    /*
     * Harder case: robot outside target bounding box, but inside virtual shell,
     * Need to find the closest cell inside the actual structure (relative to
     * structure real origin), which will be straight ahead in either X or Y.
     */
    rmath::vector3d plus_sh;
    if (rmath::radians::kZERO == ct->orientation()) {
      plus_sh = rmath::vector3d::X * ct->vshell()->sh_sizer().v();
    } else if (rmath::radians::kPI_OVER_TWO == ct->orientation()) {
      plus_sh = rmath::vector3d::Y * ct->vshell()->sh_sizer().v();
    } else if (rmath::radians::kPI == ct->orientation()) {
      plus_sh = -rmath::vector3d::X * ct->vshell()->sh_sizer().v();
    } else if (rmath::radians::kTHREE_PI_OVER_TWO == ct->orientation()) {
      plus_sh = -(rmath::vector3d::Y * ct->vshell()->sh_sizer().v());
    }
    cell = rmath::dvec2zvec(c->rpos3D() + plus_sh,
                            ct->arena_grid_resolution().v()) -
           ct->vshell()->real()->danchor3D();
  }

  /*
   * Only if the spec actually contains the cell we calculated do we return
   * it. It is possible that we are inside the virtual shell but not on the
   * side which the construction lanes enter/exit on.
   */
  if(ct->spec()->contains(cell)) {
    return boost::make_optional(cell);
  }
  return boost::none;
} /* nearest_ct_cell() */

NS_END(support, prism);
