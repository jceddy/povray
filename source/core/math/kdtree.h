//******************************************************************************
///
/// @file core/math/kdtree.h
///
/// Declarations related to K-D Trees.
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
/// This is an adaptation of the KD - tree implementation in rosetta code
/// https://rosettacode.org/wiki/K-d_tree
///
/// @endparblock
///
//******************************************************************************

#ifndef POVRAY_CORE_KDTREE_H
#define POVRAY_CORE_KDTREE_H

// Module config header file must be the first file included within POV-Ray unit header files
#include "core/configcore.h"

// C++ variants of C standard header files
// C++ standard header files
#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

// POV-Ray header files (base module)
// POV-Ray header files (core module)
#include "core/math/vector.h"

using indexArr = std::vector<int>;
using pointIndex = typename std::pair<pov::Vector3d, int>;

namespace pov
{

//##############################################################################
///
/// @defgroup PovCoreMath Mathematic Utilities
/// @ingroup PovCore
///
/// @{

class KDNode {
public:
	using KDNodePtr = std::shared_ptr< KDNode >;
	int index;
	bool empty = true;
	Vector3d x;
	KDNodePtr left;
	KDNodePtr right;

	// initializer
	KDNode();
	KDNode(const Vector3d &, const int &, const KDNodePtr &,
		const KDNodePtr &);
	KDNode(const pointIndex &, const KDNodePtr &, const KDNodePtr &);
	~KDNode();

	// getter
	DBL coord(const int &);
	DBL getX();
	DBL getY();
	DBL getZ();

	// conversions
	explicit operator bool();
	explicit operator Vector3d();
	explicit operator int();
	explicit operator pointIndex();
};

using KDNodePtr = std::shared_ptr< KDNode >;

KDNodePtr NewKDNodePtr();

// square euclidean distance
inline DBL dist2(const Vector3d &, const Vector3d &);
inline DBL dist2(const KDNodePtr &, const KDNodePtr &);

// euclidean distance
inline DBL dist(const Vector3d &, const Vector3d &);
inline DBL dist(const KDNodePtr &, const KDNodePtr &);

// Need for sorting
class comparer {
public:
	int idx;
	explicit comparer(int idx_);
	inline bool compare_idx(
		const pointIndex &,  //
		const pointIndex &   //
	);
};

using pointIndexArr = typename std::vector< pointIndex >;

inline void sort_on_idx(const pointIndexArr::iterator &,  //
	const pointIndexArr::iterator &,  //
	int idx);

using pointVec = std::vector< Vector3d >;

class KDTree {
	KDNodePtr root;
	KDNodePtr leaf;

	KDNodePtr make_tree(const pointIndexArr::iterator &begin,  //
		const pointIndexArr::iterator &end,    //
		const size_t &length,                  //
		const int &level                    //
	);

public:
	KDTree() = default;
	explicit KDTree(pointVec point_array);

private:
	KDNodePtr nearest_(           //
		const KDNodePtr &branch,  //
		const Vector3d &pt,        //
		const int &level,      //
		const KDNodePtr &best,    //
		const DBL &best_dist   //
	);

	// default caller
	KDNodePtr nearest_(const Vector3d &pt);

public:
	Vector3d nearest_point(const Vector3d &pt);
	int nearest_index(const Vector3d &pt);
	pointIndex nearest_pointIndex(const Vector3d &pt);

private:
	pointIndexArr neighborhood_(  //
		const KDNodePtr &branch,  //
		const Vector3d &pt,        //
		const DBL &rad,        //
		const int &level       //
	);

public:
	pointIndexArr neighborhood(  //
		const Vector3d &pt,       //
		const DBL &rad);

	pointVec neighborhood_points(  //
		const Vector3d &pt,         //
		const DBL &rad);

	indexArr neighborhood_indices(  //
		const Vector3d &pt,          //
		const DBL &rad);
};

/// @}
///
//##############################################################################

}
// end of namespace pov

#endif // POVRAY_CORE_KDTREE_H
