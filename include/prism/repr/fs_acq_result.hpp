/**
 * \file fs_acq_result.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_PRISM_REPR_FS_ACQ_RESULT_HPP_
#define INCLUDE_PRISM_REPR_FS_ACQ_RESULT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/repr/fs_configuration.hpp"
#include "prism/gmt/repr/block_spec.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct fs_acq_positions {
  rmath::vector3d ingress{};
  rmath::vector3d egress{};
};

struct fs_acq_specs {
  const pgrepr::block_anchor_spec* ingress{ nullptr };
  const pgrepr::block_anchor_spec* egress{ nullptr };
};

/**
 * \class fs_acq_result
 * \ingroup repr
 *
 * \brief The result of acquiring a frontier set configuration (i.e., a \ref
 * prepr::fs_configuration and associated data).
 */
struct fs_acq_result {
  /* clang-format off */
  fs_acq_positions        positions{};
  fs_acq_specs            specs{};
  size_t                  lookahead{0};
  prepr::fs_configuration configuration{prepr::fs_configuration::ekNONE};
  /* clang-format on */
};

NS_END(repr, prism);

#endif /* INCLUDE_PRISM_REPR_FS_ACQ_RESULT_HPP_ */
