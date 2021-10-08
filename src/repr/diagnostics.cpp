/**
 * \file diagnostics.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "prism/repr/diagnostics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(prism, repr, diagnostics);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
chactuators::diagnostic_actuator::map_type kColorMap = {
  {
    chactuators::diagnostics::ekEXPLORE,
    rutils::color::kMAGENTA
  },
  {
    chactuators::diagnostics::ekSUCCESS,
    rutils::color::kGREEN
  },
  {
    chactuators::diagnostics::ekTAXIS,
    rutils::color::kYELLOW
  },
  {
    chactuators::diagnostics::ekLEAVING_NEST,
    rutils::color::kGRAY50
  },
  {
    chactuators::diagnostics::ekWAIT_FOR_SIGNAL,
    rutils::color::kWHITE
  },
  {
    chactuators::diagnostics::ekEXP_INTERFERENCE,
    rutils::color::kRED
  },
  {
    prepr::diagnostics::ekBUILDER,
    rutils::color::kBLUE
  },
  {
    prepr::diagnostics::ekCT_APPROACH,
    rutils::color::kPURPLE
  }
};

NS_END(diagnostics, repr, prism);
