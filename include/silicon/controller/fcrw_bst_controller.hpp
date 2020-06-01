/**
 * \file fcrw_bst_controller.hpp
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

#ifndef INCLUDE_SILICON_CONTROLLER_FCRW_BST_CONTROLLER_HPP_
#define INCLUDE_SILICON_CONTROLLER_FCRW_BST_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "silicon/controller/constructing_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace silicon::fsm { class fcrw_bst_fsm; }

NS_START(silicon, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class fcrw_bst_controller
 * \ingroup controller
 *
 * \brief The most basic constructing controller: CRW foraging + memory-less
 * building of a single target structure.
 */
class fcrw_bst_controller : public constructing_controller,
                            public rer::client<fcrw_bst_controller> {
 public:
  fcrw_bst_controller(void) RCSW_COLD;
  ~fcrw_bst_controller(void) override RCSW_COLD;

  /* constructing_controller overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void control_step(void) override;
  void reset(void) override RCSW_COLD;
  std::type_index type_index(void) const override { return typeid(*this); }

  /* Foraging goal acquisition metrics */
  bool is_vectoring_to_goal(void) const override { return false; }
  RCPPSW_WRAP_OVERRIDE_DECL(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, goal_acquired, const);
  RCPPSW_WRAP_OVERRIDE_DECL(csmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, acquisition_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, vector_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rtypes::type_uuid, entity_acquired_id, const);

  /* block transportation */
  RCPPSW_WRAP_OVERRIDE_DECL(fsm::construction_transport_goal,
                            block_transport_goal,
                            const);

  /* block placement */
  RCPPSW_WRAP_OVERRIDE_DECL(boost::optional<block_placer::placement_intent>,
                            block_placement_intent,
                            const);


  const fsm::fcrw_bst_fsm* fsm(void) const { return m_fsm.get(); }
  fsm::fcrw_bst_fsm* fsm(void) { return m_fsm.get(); }

 private:
  /* clang-format off */
  std::unique_ptr<fsm::fcrw_bst_fsm> m_fsm;
  /* clang-format on */
};

NS_END(controller, silicon);

#endif /* INCLUDE_SILICON_CONTROLLER_FCRW_BST_CONTROLLER_HPP_ */
