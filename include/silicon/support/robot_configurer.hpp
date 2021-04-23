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
#include <memory>
#include <utility>

#include "silicon/controller/controller_fwd.hpp"
#include "silicon/controller/perception/perception_receptor.hpp"
#include "silicon/ds/ct_vector.hpp"
#include "silicon/support/config/visualization_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
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
template <typename TController>
class robot_configurer : public boost::static_visitor<void> {
 public:
  robot_configurer(const config::visualization_config* const vis_config,
                   const sds::ct_vectorro& targets)
      : mc_vis_config(vis_config), mc_targets(targets) {}

  robot_configurer(robot_configurer&&) = default;

  /* Not copy constructible/assignable by default  */
  robot_configurer(const robot_configurer&) = delete;
  const robot_configurer& operator=(const robot_configurer&) = delete;

  void operator()(TController* const c) const {
    if (nullptr != mc_vis_config) {
      c->display_id(mc_vis_config->robot_id);
      c->display_steer2D(mc_vis_config->robot_steer2D);
      c->display_los(mc_vis_config->robot_los);
      c->display_nearest_ct(mc_vis_config->robot_nearest_ct);
    }
    scperception::perception_receptor::ct_info_vector infos;
    for (const auto* t : mc_targets) {
      infos.push_back(scperception::ct_skel_info(t));
    } /* for(&t..) */

    auto receptor = std::make_unique<scperception::perception_receptor>(infos);
    c->perception()->receptor(std::move(receptor));
  }

 private:
  /* clang-format off */
  const config::visualization_config * const mc_vis_config;
  const sds::ct_vectorro                     mc_targets;
  /* clang-format on */
};

NS_END(support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_ROBOT_CONFIGURER_HPP_ */
