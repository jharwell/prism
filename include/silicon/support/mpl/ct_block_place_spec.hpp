/**
 * \file ct_block_place_spec.hpp
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

#ifndef INCLUDE_SILICON_SUPPORT_MPL_CT_BLOCK_PLACE_SPEC_HPP_
#define INCLUDE_SILICON_SUPPORT_MPL_CT_BLOCK_PLACE_SPEC_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/mpl/map.hpp>
#include <boost/mpl/fold.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "cosm/arena/base_arena_map.hpp"

#include "silicon/support/tv/block_op_penalty_handler.hpp"
#include "silicon/support/interactor_status.hpp"
#include "silicon/controller/operations/ct_block_place.hpp"
#include "silicon/structure/structure3D.hpp"
#include "silicon/metrics/silicon_metrics_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, support, mpl, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
struct ct_block_place_spec_value {
  using structure_type = sstructure::structure3D;
  using penalty_handler_type = tv::block_op_penalty_handler;
  using interactor_status_type = interactor_status;
  using robot_block_place_visitor_type = scops::ct_block_place_visitor;
  using metrics_manager_type = smetrics::silicon_metrics_manager;
  using arena_map_type = carena::base_arena_map;
};

/*
 * First argu is the map as it is built, second in the thing to insert,
 * built from each type in the specified typelist.
 */
using ct_block_place_inserter = boost::mpl::insert<boost::mpl::_1,
                                                boost::mpl::pair<boost::mpl::_2,
                                                                 ct_block_place_spec_value>>;

NS_END(detail);

template<typename TTypelist>
using ct_block_place_spec = typename boost::mpl::fold<TTypelist,
                                                        boost::mpl::map0<>,
                                                        detail::ct_block_place_inserter>::type;
NS_END(mpl, support, silicon);

#endif /* INCLUDE_SILICON_SUPPORT_MPL_CT_BLOCK_PLACE_SPEC_HPP_ */
