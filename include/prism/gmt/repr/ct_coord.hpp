/**
 * \file ct_coord.hpp
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

#ifndef INCLUDE_PRISM_GMT_REPR_CT_COORD_HPP_
#define INCLUDE_PRISM_GMT_REPR_CT_COORD_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/stringizable.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism::gmt {
class spc_gmt;
} /* namespace prism::gmt */

NS_START(prism, gmt, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ct_coord
 * \ingroup gmt repr
 *
 * \brief Representation of the location of a cell within a \ref
 * spc_gmt. Because of the real/virtual origin, offsets within the 3D grid
 * need to be tagged with what they are relative to, to make it VERY clear what
 * the \a implied origin is when an offset is passed to a different
 * function/class/etc.*
 */
class ct_coord : public rer::client<ct_coord>, public rer::stringizable {
 public:
  /**
   * \brief Type tag specifying if the coordinates for a cell within a \ref
   * spc_gmt are relative to the virtual or real origin of the structure.
   */
  enum relativity {
    ekUNDEFINED = -1,
    ekVORIGIN,
    ekRORIGIN
  };

  static ct_coord from_arena(rmath::vector3d pos,
                             const spc_gmt* ct);
  static ct_coord from_arena(rmath::vector3z arena_cell,
                             const spc_gmt* ct);
  static ct_coord to_virtual(const ct_coord& coord,
                             const spc_gmt* ct);
  static ct_coord to_real(const ct_coord& coord,
                          const spc_gmt* ct);

  ct_coord(const rmath::vector3z& offset,
           const relativity& relative_to,
           const spc_gmt* ct);

  ct_coord(void) : ER_CLIENT_INIT("prism.gmt.ct_coord") {}

  /* Copy constructible/assignable by default */
  ct_coord& operator=(const ct_coord& other) = default;
  ct_coord(const ct_coord&) = default;

  const rmath::vector3z& offset(void) const { return m_offset; }
  const relativity& relative_to(void) const { return m_relative_to; }

  /* operators */
  ct_coord operator-(const rmath::vector3z& other) const {
    return ct_coord{m_offset - other, m_relative_to, mc_ct};
  }

  ct_coord operator+(const rmath::vector3z& other) const {
    return ct_coord{m_offset + other, m_relative_to, mc_ct};
  }

  ct_coord to_real(void) const;
  ct_coord to_virtual(void) const;
  std::string to_str(void) const override;

 private:
  /* clang-format off */
  const spc_gmt* mc_ct{nullptr};

  rmath::vector3z    m_offset{};
  relativity         m_relative_to{relativity::ekUNDEFINED};
  /* clang-format on */
};


NS_END(repr, gmt, prism);

#endif /* INCLUDE_PRISM_GMT_REPR_CT_COORD_HPP_ */
