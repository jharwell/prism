/**
 * \file silicon.hpp
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

#ifndef INCLUDE_SILICON_SILICON_HPP_
#define INCLUDE_SILICON_SILICON_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace silicon {
namespace ds {}

namespace repr {}

namespace structure {
namespace config {}
namespace operations {}
} /* namespace structure */

namespace support {
namespace tv {}
} /* namespace support */

namespace controller {
namespace config {}
namespace perception {}
namespace operations {}
} /* namespace controller */

namespace lane_alloc {
namespace metrics {}
namespace config {}
}/* namespace lane_alloc */

namespace metrics {}

} /* namespace silicon */

namespace sds = silicon::ds;

namespace srepr = silicon::repr;

namespace smetrics = silicon::metrics;

namespace sstructure = silicon::structure;
namespace ssops = sstructure::operations;
namespace ssconfig = sstructure::config;

namespace ssupport = silicon::support;
namespace sstv = ssupport::tv;

namespace scontroller = silicon::controller;
namespace scconfig = scontroller::config;;
namespace scperception = scontroller::perception;
namespace scops = scontroller::operations;

namespace slane_alloc = silicon::lane_alloc;
namespace slametrics = slane_alloc::metrics;
namespace slaconfig = slane_alloc::config;

#endif /* INCLUDE_SILICON_SILICON_HPP_ */
