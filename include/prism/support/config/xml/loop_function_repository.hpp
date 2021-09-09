/**
 * \file loop_function_repository.hpp
 *
 * Handles parsing of all XML parameters at runtime.
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

#ifndef INCLUDE_PRISM_SUPPORT_CONFIG_XML_LOOP_FUNCTION_REPOSITORY_HPP_
#define INCLUDE_PRISM_SUPPORT_CONFIG_XML_LOOP_FUNCTION_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/xml/base_sm_repository.hpp"

#include "prism/prism.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(prism, support, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class loop_function_repository
 * \ingroup support config xml
 *
 * \brief Extra loop function XML parsers/results specific to the PRISM
 * project.
 */
class loop_function_repository : public cpal::config::xml::base_sm_repository {
 public:
  loop_function_repository(void) noexcept;
};

NS_END(xml, config, support, prism);

#endif /* INCLUDE_PRISM_SUPPORT_CONFIG_XML_LOOP_FUNCTION_REPOSITORY_HPP_ */
