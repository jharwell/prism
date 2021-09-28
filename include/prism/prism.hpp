/**
 * \file prism.hpp
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

#ifndef INCLUDE_PRISM_PRISM_HPP_
#define INCLUDE_PRISM_PRISM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace prism {
namespace ds {}

namespace algorithm {
namespace constants {}
} /* namespace algorithm */

namespace repr {}

namespace gmt {
namespace config {}
namespace operations {}
namespace ds {}
namespace metrics {}
namespace repr {}
} /* namespace gmt */

namespace fsm {
namespace calculators {}
}

namespace support {
namespace tv {}
} /* namespace support */

namespace controller {
namespace perception {
namespace config {}
}
namespace operations {}
} /* namespace controller */

namespace lane_alloc {
namespace metrics {}
namespace config {}
}/* namespace lane_alloc */

namespace metrics {}

} /* namespace prism */

namespace pds = prism::ds;

namespace pfsm = prism::fsm;
namespace pfcalculators = pfsm::calculators;

namespace prepr = prism::repr;

namespace pmetrics = prism::metrics;

namespace pgmt = prism::gmt;
namespace pgmetrics = pgmt::metrics;
namespace pgops = pgmt::operations;
namespace pgconfig = pgmt::config;
namespace pgds = pgmt::ds;
namespace pgrepr = pgmt::repr;

namespace palgorithm = prism::algorithm;
namespace paconstants = palgorithm::constants;

namespace psupport = prism::support;
namespace pstv = psupport::tv;

namespace pcontroller = prism::controller;
namespace pcperception = pcontroller::perception;
namespace pcpconfig = pcperception::config;
namespace pcops = pcontroller::operations;

namespace plane_alloc = prism::lane_alloc;
namespace plametrics = plane_alloc::metrics;
namespace placonfig = plane_alloc::config;

#endif /* INCLUDE_PRISM_PRISM_HPP_ */
