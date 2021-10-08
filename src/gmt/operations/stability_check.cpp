/**
 * \file stability_check.cpp
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
#include "prism/gmt/operations/stability_check.hpp"

#include "prism/gmt/operations/topology_check.hpp"
#include "prism/gmt/repr/vshell.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, gmt, operations);


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool stability_check::operator()(const pgds::connectivity_graph* graph,
                                     const pgrepr::vshell* vshell) const {
  for (size_t z = 0; z < vshell->real()->zdsize() - 1; ++z) {
    auto lower = pgrepr::slice2D(pgrepr::slice2D::spec_calc(rmath::vector3z::Z,
                                                            z,
                                                            vshell),
                                 graph);
    auto upper = pgrepr::slice2D(pgrepr::slice2D::spec_calc(rmath::vector3z::Z,
                                                            z + 1,
                                                            vshell),
                                 graph);

    ER_CHECK(is_composable(lower, upper, vshell),
             "Layer%zu -> layer%zu not composable",
             z,
             z + 1);
    ER_DEBUG("Layer%zu -> layer%zu OK", z, z + 1);
  } /* for(z..) */
  return true;

error:
  return false;
} /* operator()() */

bool stability_check::is_composable(const pgrepr::slice2D& lower,
                                        const pgrepr::slice2D& upper,
                                        const pgrepr::vshell* vshell) const {
  auto topo_checker = topology_check();

  /*
   * \todo For right now, we disallow ALL holes in structures, because even
   * simple holes are not feasible unless beam blocks exist, which they do not
   * yet. This may be relaxed at some point in the future.
   */
  ER_CHECK(topo_checker.topological_holes(lower, vshell).empty() &&
           topo_checker.topological_holes(upper, vshell).empty(),
           "One or both layers contain topological holes");

  ER_CHECK(!has_overhangs(lower, upper), "Upper layer has overhangs");
  /*
   * \todo For now,  since only cube blocks are implemented, no more checking is
   * needed.
   */
  return true;

error:
  return false;
} /* is_composable() */

bool stability_check::has_overhangs(const pgrepr::slice2D& lower,
                                        const pgrepr::slice2D& upper) const {
  auto lower_offset = lower.dcenter3D().mask(lower.spec().axis()).length();
  auto upper_offset = upper.dcenter3D().mask(upper.spec().axis()).length();

  auto [v_begin, v_end] = upper.vertices();
  for (auto vd = v_begin; vd != v_end; ++vd) {
    ER_INFO("Upper coord: %s", rcppsw::to_string(upper.access(*vd)->coord).c_str());
    ER_ASSERT(upper.contains(upper.access(*vd)->coord),
              "Upper layer with axis offset=%zu does not contain %s",
              static_cast<size_t>(upper_offset),
              rcppsw::to_string(upper.access(*vd)->coord).c_str());

    rmath::vector3z check(upper.access(*vd)->coord.project_on_xy(),
                          lower.danchor3D().z());
    /* ER_CHECK(lower.contains(check), */
    /*          "Lower layer with axis offset=%zu does not contain block anchor %s", */
    /*          static_cast<size_t>(lower_offset), */
    /*          rcppsw::to_string(check).c_str()); */
  } /* (for vd...) */
  return false;

error:
  return true;
} /* has_overhangs() */

NS_END(operations, gmt, prism);
