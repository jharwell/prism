/**
 * \file prism_metrics_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/config/metrics_config.hpp"

#include "cosm/ds/config/grid2D_config.hpp"
#include "cosm/argos/metrics/fs_output_manager.hpp"

#include "prism/controller/controller_fwd.hpp"
#include "prism/ds/ct_vector.hpp"
#include "prism/prism.hpp"
#include "prism/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace prism::gmt {
class ct_manager;
} /* namespace prism::gmt */

NS_START(prism, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class prism_metrics_manager
 * \ingroup metrics
 *
 * \brief Extends the \ref cargos::metrics::fs_output_manager for the PRISM
 * project.
 */
class prism_metrics_manager : public rer::client<prism_metrics_manager>,
                              public cargos::metrics::fs_output_manager {
 public:
  prism_metrics_manager(const rmconfig::metrics_config* mconfig,
                          const cdconfig::grid2D_config* gconfig,
                          const fs::path& output_root,
                          const ds::ct_vectorno& targets);
  ~prism_metrics_manager(void) override = default;

  void collect_from_tv(const support::tv::tv_manager* tvm);
  void collect_from_ct(const gmt::ct_manager* manager);

  template <typename TController>
  void collect_from_controller(const TController* c,
                               const rtypes::type_uuid& ct_id);

 private:
  /**
   * \brief Register collectors that don't need extra arguments that do not
   * pertain to construction targets.
   */
  void register_standard(const rmconfig::metrics_config* mconfig);

  /**
   * \brief Register collectors that don't need extra arguments that pertain to
   * construction targets.
   */
  void register_with_ct(const rmconfig::metrics_config* mconfig,
                                 const pgmt::spc_gmt* ct);

  /**
   * \brief Register collectors that need extra arguments that pertain to
   * construction targets (# construction lanes).
   */
  void register_with_ct_and_lanes(const rmconfig::metrics_config* mconfig,
                                  const pgmt::spc_gmt* ct);

  /**
   * \brief Register collectors that need extra arguments that pertain to
   * construction targets (# construction lanes and ct ID).
   */
  void
  register_with_ct_and_lanes_and_id(const rmconfig::metrics_config* mconfig,
                                    const pgmt::spc_gmt* ct);

  /**
   * \brief Register collectors that need extra arguments that pertain to
   * construction targets (structure dimensions).
   */
  void register_with_ct_and_dims(const rmconfig::metrics_config* mconfig,
                                 const pgmt::spc_gmt* ct);
};

NS_END(metrics, prism);
