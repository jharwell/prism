/**
 * \file robot_configurer.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_ROBOT_CONFIGURER_HPP_
#define INCLUDE_SILICON_SUPPORT_ROBOT_CONFIGURER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>

#include "cosm/vis/config/visualization_config.hpp"

#include "silicon/controller/controller_fwd.hpp"
#include "silicon/controller/construction_lane_allocator.hpp"
#include "silicon/controller/config/lane_alloc_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon::structure {
class structure3D;
} /* namespace silicon::structure */

NS_START(silicon, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct robot_configurer
 * \ingroup support depth0
 *
 * \brief Functor to perform controller configuration during initialization.
 */
template <typename TControllerType>
class robot_configurer : public boost::static_visitor<void> {
 public:
  explicit robot_configurer(const cvconfig::visualization_config* const vis_config,
                            const scconfig::lane_alloc_config* const alloc_config,
                            const sstructure::structure3D* structure)
      : mc_vis_config(vis_config),
        mc_alloc_config(alloc_config){}

  void operator()(TControllerType* const c) const {
    if (nullptr != mc_vis_config) {
      c->display_los(mc_vis_config->robot_los);
      c->display_id(mc_vis_config->robot_id);
    }
    auto allocator = std::make_unique<controller::construction_lane_allocator>(
        mc_alloc_config,
        mc_structure);
    /* @todo Send this to the controller */
  }

 private:
  /* clang-format off */
  const sstructure::structure3D*               mc_structure;
  const cvconfig::visualization_config * const mc_vis_config;
  const scconfig::lane_alloc_config * const    mc_alloc_config;
  /* clang-format on */
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_ROBOT_CONFIGURER_HPP_ */
