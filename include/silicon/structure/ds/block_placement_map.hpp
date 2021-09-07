/**
 * \file block_placement_map.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_SILICON_STRUCTURE_DS_BLOCK_PLACEMENT_MAP_HPP_
#define INCLUDE_SILICON_STRUCTURE_DS_BLOCK_PLACEMENT_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>

#include "rcppsw/patterns/decorator/decorator.hpp"

#include "silicon/silicon.hpp"
#include "silicon/structure/repr/block_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(silicon, structure, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_placement_map
 * \ingroup structure ds
 *
 * \brief Maps a
 */

class block_placement_map : public rpdecorator::decorator<
  std::map<rmath::vector3z,
           ssrepr::block_placement_spec>
  > {
 public:
  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(find, const);
  RCPPSW_DECORATE_DECLDEF(find);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(size, const);
};

NS_END(ds, structure, silicon);

#endif /* INCLUDE_SILICON_STRUCTURE_DS_BLOCK_PLACEMENT_MAP_HPP_ */
