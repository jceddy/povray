//******************************************************************************
///
/// @file core/math/minimindistance.h
///
/// Declarations related to computing the minumum distance.
///
/// @copyright
/// @parblock
///
/// Persistence of Vision Ray Tracer ('POV-Ray') version 3.8.
/// Copyright 1991-2019 Persistence of Vision Raytracer Pty. Ltd.
///
/// POV-Ray is free software: you can redistribute it and/or modify
/// it under the terms of the GNU Affero General Public License as
/// published by the Free Software Foundation, either version 3 of the
/// License, or (at your option) any later version.
///
/// POV-Ray is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/// GNU Affero General Public License for more details.
///
/// You should have received a copy of the GNU Affero General Public License
/// along with this program.  If not, see <http://www.gnu.org/licenses/>.
///
/// ----------------------------------------------------------------------------
///
/// POV-Ray is based on the popular DKB raytracer version 2.12.
/// DKBTrace was originally written by David K. Buck.
/// DKBTrace Ver 2.0-2.12 were written by David K. Buck & Aaron A. Collins.
///
/// @endparblock
///
//******************************************************************************

#ifndef POVRAY_CORE_MINIMUMDISTANCE_H
#define POVRAY_CORE_MINIMUMDISTANCE_H

// Module config header file must be the first file included within POV-Ray unit header files
#include "core/configcore.h"

// C++ variants of C standard header files
// C++ standard header files
#include <random>

// POV-Ray header files (base module)
// POV-Ray header files (core module)
#include "core/coretypes.h"

namespace pov
{
	/*

	MINIMUM_DISTANCE:
		minumum_distance { OBJECT [MINIMUM_DISTANCE_ITEMS] }

	MINIMUM_DISTANCE_ITEMS:
		method Number | quality Number | t_min Value | alpha Value | iterations Number

	method 1 - brute force
	method 2 - gradient descent
	method 3 - simulated annealing (default)

	quality (1-6) controls the samples taken for brute force (including truncated brute force at the beginning of other methods)
	  - defaults to 2

	t_min - range from .0001 to .1, temperature to stop for simulated annealing
	alpha - range from 0.1 to 0.9
	  - for method 3, amount to adjust temperature by at each simulated annealing step
	  - for method 2, the size of each gradient descent step
	iterations - range from 10 to 100, number of iterations at each simulated annealing step

	*/
	struct MinimumDistance_Struct final
	{
		ObjectPtr object;
		int method = 3;
		int quality = 2;
	};
}
// end of namespace pov

#endif // POVRAY_CORE_MINIMUMDISTANCE_H
