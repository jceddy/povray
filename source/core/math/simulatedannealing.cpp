//******************************************************************************
///
/// @file core/math/simulatedannealing.cpp
///
/// Implementations related to the simulated annealing solver.
///
/// @author Joseph Eddy
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
/// ----------------------------------------------------------------------------
///
/// The author reserves the right to distribute this material elsewhere under
/// different terms. This file is provided within POV-Ray under the following
/// terms:
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///
///  1. Redistributions of source code must retain the above copyright notice,
///     this list of conditions and the following disclaimer.
///  2. Redistributions in binary form must reproduce the above copyright notice,
///     this list of conditions and the following disclaimer in the documentation
///     and/or other materials provided with the distribution.
///  3. Neither the names of the copyright holders nor the names of contributors
///     may be used to endorse or promote products derived from this software
///     without specific prior written permission.
///
///  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
///  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
///  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
///  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
///  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
///  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
///  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
///  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
///  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
///  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
///  POSSIBILITY OF SUCH DAMAGE.
///
/// @endparblock
///
//******************************************************************************

// Unit header file must be the first file included within POV-Ray *.cpp files (pulls in config)
#include "core/math/simulatedannealing.h"

// C++ variants of C standard header files
// C++ standard header files
#include <limits>

// POV-Ray header files (base module)
#include "base/pov_err.h"

// POV-Ray header files (core module)
#include "core/render/ray.h"
#include "core/render/trace.h"
#include "core/scene/object.h"

// this must be the last file included
#include "base/povdebug.h"

namespace pov
{
	MinimumDistanceInput::MinimumDistanceInput() { }

	MinimumDistanceInput::MinimumDistanceInput(Vector3d pt, ObjectPtr obj, TraceThreadData *tData, int initQuality) {
		point = pt;
		object = obj;
		initialSolutionQuality = initQuality;
		traceThreadData = tData;
		
		if (initialSolutionQuality < 1) { initialSolutionQuality = 1; }
		if (initialSolutionQuality > 6) { initialSolutionQuality = 6; }
	}

	MinimumDistanceState::MinimumDistanceState() { }

	MinimumDistanceState::MinimumDistanceState(DBL p, DBL t) {
		phi = p;
		theta = t;
	}

	MinimumDistanceSolution::MinimumDistanceSolution(DBL val, const MinimumDistanceInput &in, const MinimumDistanceState &st, DBL out) : SimulatedAnnealingSolution<MinimumDistanceInput, MinimumDistanceState, DBL>(val, in, st, out) { }

	SimulatedAnnealingSolution<MinimumDistanceInput, MinimumDistanceState, DBL> MinimimDistanceSolver::FindInitialSolution(const MinimumDistanceInput &in) {
		bool depthFound = false;
		Vector3d direction;
		TraceTicket ticket(1, 0.0);
		Ray ray(ticket, in.point, direction);
		Intersection intersection;
		int slices = pow(2, in.initialSolutionQuality + 1);
		DBL minPhi, minTheta;
		DBL mdist = maxDistance;

		direction[X] = 0;
		direction[Y] = 1;
		direction[Z] = 0;

		ray.Direction = direction;
		if (Find_Intersection(&intersection, in.object, ray, in.traceThreadData)) {
			if (!depthFound || (intersection.Depth < mdist)) {
				mdist = intersection.Depth;
				minPhi = 0.0;
				minTheta = 0.0;
				depthFound = true;
			}
		}

		for (int i = 0; i < slices - 1; i++) {
			DBL phi = M_PI * double(i + 1) / double(slices);
			for (int j = 0; j < slices; j++) {
				DBL theta = 2.0 * M_PI * double(j) / double(slices);
				DBL x = std::sin(phi) * std::cos(theta);
				DBL y = std::cos(phi);
				DBL z = std::sin(phi) * std::sin(theta);

				direction[X] = x;
				direction[Y] = y;
				direction[Z] = z;

				ray.Direction = direction;
				if (Find_Intersection(&intersection, in.object, ray, in.traceThreadData)) {
					if (!depthFound || (intersection.Depth < mdist)) {
						mdist = intersection.Depth;
						minPhi = phi;
						minTheta = theta;
						depthFound = true;
					}
				}
			}
		}

		direction[X] = 0;
		direction[Y] = -1;
		direction[Z] = 0;

		ray.Direction = direction;
		if (Find_Intersection(&intersection, in.object, ray, in.traceThreadData)) {
			if (!depthFound || (intersection.Depth < mdist)) {
				minPhi = M_PI;
				minTheta = 0.0;
				mdist = intersection.Depth;
				depthFound = true;
			}
		}

		MinimumDistanceState state = MinimumDistanceState(minPhi, minTheta);

		return MinimumDistanceSolution(mdist, in, state, mdist);
	}

	SimulatedAnnealingSolution<MinimumDistanceInput, MinimumDistanceState, DBL> MinimimDistanceSolver::GetNeighbor(const SimulatedAnnealingSolution<MinimumDistanceInput, MinimumDistanceState, DBL> &solution) {
		Vector3d direction;
		TraceTicket ticket(1, 0.0);
		Ray ray(ticket, solution.input.point, direction);
		Intersection intersection;
		DBL dist = maxDistance;

		DBL phi = solution.state.phi + smallStepDistribution(re);
		DBL theta = solution.state.theta + smallStepDistribution(re);

		if (phi < 0.0) {
			phi = 0 - phi;
		}
		if (phi > M_PI) {
			phi = M_PI - phi;
		}
		if (theta < 0.0) {
			theta = theta + 2.0 * M_PI;
		}
		if (theta > 2.0 * M_PI) {
			theta = theta - 2.0 * M_PI;
		}

		MinimumDistanceState state = MinimumDistanceState(phi, theta);

		DBL x = std::sin(phi) * std::cos(theta);
		DBL y = std::cos(phi);
		DBL z = std::sin(phi) * std::sin(theta);

		direction[X] = x;
		direction[Y] = y;
		direction[Z] = z;

		ray.Direction = direction;
		if (Find_Intersection(&intersection, solution.input.object, ray, solution.input.traceThreadData)) {
			dist = intersection.Depth;
		}

		return MinimumDistanceSolution(dist, solution.input, state, dist);
	}
}
// end of namespace pov
