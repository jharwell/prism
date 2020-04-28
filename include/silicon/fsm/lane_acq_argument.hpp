/**
 * \file lane_acq_data.hpp
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

#ifndef INCLUDE_SILICON_FSM_LANE_ACQ_DATA_HPP_
#define INCLUDE_SILICON_FSM_LANE_ACQ_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/ta/taskable_argument.hpp"

#include "silicon/silicon.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lane_acq_argument
 * \ingroup fsm
 *
 * \brief An argument that can be passed to a \ref ta::taskable function
 * containing information necessary for acquisition of ingress/egress lanes of a
 * construction lane.
 */
class lane_acq_argument final : public cta::taskable_argument,
                                public rpfsm::event_data {
 public:
  lane_acq_argument(void) = default;
  lane_acq_argument(const rmath::vector2d& ingress_point,
                    const rmath::vector2d& egress_point,
                    const rmath::radians& orientation)
      : m_ingress_point(ingress_point),
        m_egress_point(egress_point),
        m_orientation(orientation) {}

  ~lane_acq_argument(void) override = default;

  const rmath::vector2d& ingress_point(void) const { return m_ingress_point; }
  const rmath::vector2d& egress_point(void) const { return m_egress_point; }
  const rmath::radians& orientation(void) const { return m_orientation; }

 private:
  /* clang-format off */
  rmath::vector2d m_ingress_point{};
  rmath::vector2d m_egress_point{};
  rmath::radians  m_orientation{};
  /* clang-format on */
};

NS_END(fsm, silicon);

#endif /* INCLUDE_SILICON_FSM_LANE_ACQ_ARGUMENT_HPP_ */
