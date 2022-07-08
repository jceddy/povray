//******************************************************************************
///
/// @file core/math/kdtree.cpp
///
/// Implementations related to K-D Trees.
///
/// @author Joseph Eddy, J. Frederico Carvalho
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
#include "core/math/kdtree.h"

// C++ variants of C standard header files
// C++ standard header files
#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>

// POV-Ray header files (base module)
#include "base/pov_err.h"

// POV-Ray header files (core module)
//  (none at the moment)

// this must be the last file included
#include "base/povdebug.h"

namespace pov
{

KDNode::KDNode() = default;

KDNode::KDNode(const Vector3d &pt, const int &idx_, const KDNodePtr &left_,
	const KDNodePtr &right_) {
	x = pt;
	index = idx_;
	left = left_;
	right = right_;
	empty = false;
}

KDNode::KDNode(const pointIndex &pi, const KDNodePtr &left_,
	const KDNodePtr &right_) {
	x = pi.first;
	index = pi.second;
	left = left_;
	right = right_;
	empty = false;
}

KDNode::~KDNode() = default;

DBL KDNode::coord(const int &idx) { return x[idx]; }
DBL KDNode::getX() { return x.x(); }
DBL KDNode::getY() { return x.y(); }
DBL KDNode::getZ() { return x.z(); }
KDNode::operator bool() { return (!empty); }
KDNode::operator Vector3d() { return x; }
KDNode::operator int() { return index; }
KDNode::operator pointIndex() { return pointIndex(x, index); }

KDNodePtr NewKDNodePtr() {
	KDNodePtr mynode = std::make_shared< KDNode >();
	return mynode;
}

inline DBL dist2(const Vector3d &a, const Vector3d &b) {
	return (b - a).lengthSqr();
}

inline DBL dist2(const KDNodePtr &a, const KDNodePtr &b) {
	return dist2(a->x, b->x);
}

inline DBL dist(const Vector3d &a, const Vector3d &b) {
	return (b - a).length();
}

inline DBL dist(const KDNodePtr &a, const KDNodePtr &b) {
	return dist(a->x, b->x);
}

comparer::comparer(int idx_) : idx{ idx_ } {};

inline bool comparer::compare_idx(const pointIndex &a,  //
	const pointIndex &b   //
) {
	return (a.first[idx] < b.first[idx]);  //
}

inline void sort_on_idx(const pointIndexArr::iterator &begin,  //
	const pointIndexArr::iterator &end,    //
	int idx) {
	comparer comp(idx);
	comp.idx = idx;

	using std::placeholders::_1;
	using std::placeholders::_2;

	std::nth_element(begin, begin + std::distance(begin, end) / 2,
		end, std::bind(&comparer::compare_idx, comp, _1, _2));
}

using pointVec = std::vector< Vector3d >;

KDNodePtr KDTree::make_tree(const pointIndexArr::iterator &begin,  //
	const pointIndexArr::iterator &end,    //
	const size_t &length,                  //
	const int &level                    //
) {
	if (begin == end) {
		return NewKDNodePtr();  // empty tree
	}

	int dim = 3; // only 3d vectors

	if (length > 1) {
		sort_on_idx(begin, end, level);
	}

	auto middle = begin + (length / 2);

	auto l_begin = begin;
	auto l_end = middle;
	auto r_begin = middle + 1;
	auto r_end = end;

	size_t l_len = length / 2;
	size_t r_len = length - l_len - 1;

	KDNodePtr left;
	if (l_len > 0 && dim > 0) {
		left = make_tree(l_begin, l_end, l_len, (level + 1) % dim);
	}
	else {
		left = leaf;
	}
	KDNodePtr right;
	if (r_len > 0 && dim > 0) {
		right = make_tree(r_begin, r_end, r_len, (level + 1) % dim);
	}
	else {
		right = leaf;
	}

	// KDNode result = KDNode();
	return std::make_shared< KDNode >(*middle, left, right);
}

KDTree::KDTree(pointVec point_array) {
	leaf = std::make_shared< KDNode >();
	// iterators
	pointIndexArr arr;
	for (int i = 0; i < point_array.size(); i++) {
		arr.push_back(pointIndex(point_array.at(i), i));
	}

	auto begin = arr.begin();
	auto end = arr.end();

	size_t length = arr.size();
	int level = 0;  // starting

	root = KDTree::make_tree(begin, end, length, level);
}

KDNodePtr KDTree::nearest_(   //
	const KDNodePtr &branch,  //
	const Vector3d &pt,        //
	const int &level,      //
	const KDNodePtr &best,    //
	const DBL &best_dist   //
) {
	DBL d, dx, dx2;

	if (!bool(*branch)) {
		return NewKDNodePtr();  // basically, null
	}

	Vector3d branch_pt(*branch);
	int dim = 3;  // 3d vectors only

	d = dist2(branch_pt, pt);
	dx = branch_pt[level] - pt[level];
	dx2 = dx * dx;

	KDNodePtr best_l = best;
	DBL best_dist_l = best_dist;

	if (d < best_dist) {
		best_dist_l = d;
		best_l = branch;
	}

	int next_lv = (level + 1) % dim;
	KDNodePtr section;
	KDNodePtr other;

	// select which branch makes sense to check
	if (dx > 0) {
		section = branch->left;
		other = branch->right;
	}
	else {
		section = branch->right;
		other = branch->left;
	}

	// keep nearest neighbor from further down the tree
	KDNodePtr further = nearest_(section, pt, next_lv, best_l, best_dist_l);
	if (!further->empty) {
		DBL dl = dist2(further->x, pt);
		if (dl < best_dist_l) {
			best_dist_l = dl;
			best_l = further;
		}
	}
	// only check the other branch if it makes sense to do so
	if (dx2 < best_dist_l) {
		further = nearest_(other, pt, next_lv, best_l, best_dist_l);
		if (!further->empty) {
			DBL dl = dist2(further->x, pt);
			if (dl < best_dist_l) {
				best_dist_l = dl;
				best_l = further;
			}
		}
	}

	return best_l;
};

// default caller
KDNodePtr KDTree::nearest_(const Vector3d &pt) {
	int level = 0;
	// KDNodePtr best = branch;
	DBL branch_dist = dist2(Vector3d(*root), pt);
	return nearest_(root,          // beginning of tree
		pt,            // point we are querying
		level,         // start from level 0
		root,          // best is the root
		branch_dist);  // best_dist = branch_dist
};

Vector3d KDTree::nearest_point(const Vector3d &pt) {
	return Vector3d(*nearest_(pt));
};

int KDTree::nearest_index(const Vector3d &pt) {
	return int(*nearest_(pt));
};

pointIndex KDTree::nearest_pointIndex(const Vector3d &pt) {
	KDNodePtr Nearest = nearest_(pt);
	return pointIndex(Vector3d(*Nearest), int(*Nearest));
}

pointIndexArr KDTree::neighborhood_(  //
	const KDNodePtr &branch,          //
	const Vector3d &pt,                //
	const DBL &rad,                //
	const int &level               //
) {
	DBL d, dx, dx2;

	if (!bool(*branch)) {
		// branch has no point, means it is a leaf,
		// no points to add
		return pointIndexArr();
	}

	int dim = 3;  // only 3d vectors

	DBL r2 = rad * rad;

	d = dist2(Vector3d(*branch), pt);
	dx = Vector3d(*branch)[level] - pt[level];
	dx2 = dx * dx;

	pointIndexArr nbh, nbh_s, nbh_o;
	if (d <= r2) {
		nbh.push_back(pointIndex(*branch));
	}

	//
	KDNodePtr section;
	KDNodePtr other;
	if (dx > 0) {
		section = branch->left;
		other = branch->right;
	}
	else {
		section = branch->right;
		other = branch->left;
	}

	nbh_s = neighborhood_(section, pt, rad, (level + 1) % dim);
	nbh.insert(nbh.end(), nbh_s.begin(), nbh_s.end());
	if (dx2 < r2) {
		nbh_o = neighborhood_(other, pt, rad, (level + 1) % dim);
		nbh.insert(nbh.end(), nbh_o.begin(), nbh_o.end());
	}

	return nbh;
};

pointIndexArr KDTree::neighborhood(  //
	const Vector3d &pt,               //
	const DBL &rad) {
	int level = 0;
	return neighborhood_(root, pt, rad, level);
}

pointVec KDTree::neighborhood_points(  //
	const Vector3d &pt,                 //
	const DBL &rad) {
	int level = 0;
	pointIndexArr nbh = neighborhood_(root, pt, rad, level);
	pointVec nbhp;
	nbhp.resize(nbh.size());
	std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
		[](pointIndex x) { return x.first; });
	return nbhp;
}

indexArr KDTree::neighborhood_indices(  //
	const Vector3d &pt,                  //
	const DBL &rad) {
	int level = 0;
	pointIndexArr nbh = neighborhood_(root, pt, rad, level);
	indexArr nbhi;
	nbhi.resize(nbh.size());
	std::transform(nbh.begin(), nbh.end(), nbhi.begin(),
		[](pointIndex x) { return x.second; });
	return nbhi;
}

}
// end of namespace pov
