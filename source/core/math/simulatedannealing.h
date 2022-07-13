//******************************************************************************
///
/// @file core/math/simulatedannealing.h
///
/// Declarations related to the simulated annealing solver.
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

#ifndef POVRAY_CORE_SIMULATEDANNEALING_H
#define POVRAY_CORE_SIMULATEDANNEALING_H

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
	// Simulated Annealing solution base class
	// bundles input, state, output, and value
	template <class TInput, class TState, class TOutput>
	struct SimulatedAnnealingSolution {
		// this is an "error" value...lower values means we are closer to an optimum solution
		DBL value;

		// array containing solution argument(s)
		TInput input;

		// array containing the internal state associated with this solution instance
		TState state;

		// the output for this instance
		TOutput output;

		// constructors
		SimulatedAnnealingSolution() { };

		SimulatedAnnealingSolution(DBL val, const TInput &in, const TState &st, TOutput out) {
			value = val;
			input = in;
			state = st;
			output = out;
		}

		SimulatedAnnealingSolution(const SimulatedAnnealingSolution<TInput, TState, TOutput> &other) {
			value = other.value;
			input = other.input;
			state = other.state;
			output = other.output;
		}
	};

	// Simulated Annealing solver base class
	template <class TInput, class TState, class TOutput>
	struct SimulatedAnnealingSolver
	{
		// initial temperature
		DBL initialTemperature = 1;

		// temperature at which iteration terminates
		DBL minimumTemperature = .0001;

		// decrease in temperature
		DBL alpha = 0.9;

		// number of iterations of annealing before decreasing temperature
		int numIterations = 100;

		// virtual methods to be overridden
		virtual SimulatedAnnealingSolution<TInput, TState, TOutput> FindInitialSolution(const TInput &in) {
			SimulatedAnnealingSolution<TInput, TState, TOutput> solution;
			solution.input = in;
			solution.value = -1.0;
			return solution;
		}

		virtual SimulatedAnnealingSolution<TInput, TState, TOutput> GetNeighbor(const SimulatedAnnealingSolution<TInput, TState, TOutput> &solution) {
			return solution;
		}

		// main solve method
		SimulatedAnnealingSolution<TInput, TState, TOutput> Solve(const TInput &in) {
			// random number generator to be used below
			std::uniform_real_distribution<double> oneDistribution(0.0, 1.0);
			std::default_random_engine re;

			// global minimum
			SimulatedAnnealingSolution<TInput, TState, TOutput> min;
			min.value = std::numeric_limits<DBL>::max();

			// get initial solution before annealing process
			SimulatedAnnealingSolution<TInput, TState, TOutput> currentSolution = FindInitialSolution(in);

			// continute annealing until minimum temperature is reached
			DBL temperature = initialTemperature;
			while (temperature > minimumTemperature) {
				for (int i = 0; i < numIterations; i++) {
					// reassign global minimum if we have a better solution
					if (currentSolution.value < min.value) {
						min = currentSolution;
					}

					SimulatedAnnealingSolution<TInput, TState, TOutput> newSolution = GetNeighbor(currentSolution);
					DBL ap = exp((currentSolution.value - newSolution.value) / temperature);
					if (ap > oneDistribution(re)) {
						currentSolution = newSolution;
					}
				}

				temperature *= alpha;
			}

			return min;
		}
	};

	// input for minimum distance classes
	struct MinimumDistanceInput final {
		Vector3d point;
		ObjectPtr object;
		int initialSolutionQuality;
		TraceThreadData *traceThreadData;
		bool initialDirProvided = false;
		DBL initialTheta = 0.0;
		DBL initialPhi = 0.0;

		MinimumDistanceInput();
		MinimumDistanceInput(Vector3d pt, ObjectPtr obj, TraceThreadData *tData, int initQuality = 2);
		MinimumDistanceInput(Vector3d pt, ObjectPtr obj, TraceThreadData *tData, DBL theta, DBL phi);
	};

	// state for minimum distance classes
	// phi, theta represent direction from a point in spherical coordinates
	struct MinimumDistanceState final {
		DBL phi;
		DBL theta;

		MinimumDistanceState();
		MinimumDistanceState(DBL p, DBL t);
	};

	// minimum distance solution class
	struct MinimumDistanceSolution final : public SimulatedAnnealingSolution<MinimumDistanceInput, MinimumDistanceState, DBL>
	{
		MinimumDistanceSolution(DBL val, const MinimumDistanceInput &in, const MinimumDistanceState &st, DBL out);
	};

	// minimum distance solver class
	struct MinimimDistanceSolver final : public SimulatedAnnealingSolver<MinimumDistanceInput, MinimumDistanceState, DBL>
	{
	private:
		DBL maxDistance = 20000000000.0;
		std::uniform_real_distribution<DBL> smallStepDistribution = std::uniform_real_distribution<DBL>(0.0 - M_PI / 360.0, M_PI / 360.0);
		std::default_random_engine re;

	public:
		virtual SimulatedAnnealingSolution<MinimumDistanceInput, MinimumDistanceState, DBL> FindInitialSolution(const MinimumDistanceInput &in) override;
		virtual SimulatedAnnealingSolution<MinimumDistanceInput, MinimumDistanceState, DBL> GetNeighbor(const SimulatedAnnealingSolution<MinimumDistanceInput, MinimumDistanceState, DBL> &solution) override;
	};
}
// end of namespace pov

#endif // POVRAY_CORE_SIMULATEDANNEALING_H
