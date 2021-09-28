/**
 * \file cubic_spacefill.hpp
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

#ifndef INCLUDE_PRISM_FSM_CALCULATORS_FS_ACQ_CUBIC_SPACEFILL_HPP_
#define INCLUDE_PRISM_FSM_CALCULATORS_FS_ACQ_CUBIC_SPACEFILL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/fsm/calculators/fs_acq/base_strategy.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, fsm, calculators, fs_acq);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cubic_spacefill
 * \ingroup fsm calculators fs_acq
 *
 * \brief Calculate whether or not a robot has reached a stygmergic
 * configuration yet as it proceeds along the path from the ingress point to the
 * back of the lane, for structures composed of only cubic blocks where only
 * space filling is required (i.e., 2.5D structures composed of at most one
 * level of cubic blocks).
 */
class cubic_spacefill : public rer::client<cubic_spacefill>,
                        public pfcalculators::fs_acq::base_strategy {
 public:
  cubic_spacefill(const csubsystem::sensing_subsystemQ3D* sensing,
                 const pcperception::builder_perception_subsystem* perception);

  prepr::fs_acq_result operator()(const prepr::construction_lane* lane) const override;

  /* Not move/copy constructable/assignable by default */
  cubic_spacefill(const cubic_spacefill&) = delete;
  const cubic_spacefill& operator=(const cubic_spacefill&) = delete;
  cubic_spacefill(cubic_spacefill&&) = delete;
  cubic_spacefill& operator=(cubic_spacefill&&) = delete;

 private:
  prepr::fs_acq_result acq_result_calc(const prepr::construction_lane* lane,
                                       const prepr::builder_los* los,
                                       size_t lookahead) const;

  prepr::fs_configuration
  configuration_calc(const prepr::fs_acq_result& result,
                     const prepr::construction_lane* lane,
                     const prepr::builder_los* los) const;

  prepr::fs_acq_positions acq_positions_calc(const rmath::vector3d& rpos,
                                             const prepr::construction_lane* lane,
                                             size_t lookahead) const;

  bool acq_empty_lane(const prepr::construction_lane* lane,
                      const prepr::builder_los* los) const;

  /* clang-format off */
  /* clang-format on */
};

NS_END(fs_acq, calculators, fsm, prism);

#endif /* INCLUDE_PRISM_FSM_CALCULATORS_FS_ACQ_CUBIC_SPACEFILL_HPP_ */
