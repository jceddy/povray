//******************************************************************************
///
/// @file core/shape/triangle.h
///
/// Declarations related to the triangle geometric primitive.
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

#ifndef POVRAY_CORE_TRIANGLE_H
#define POVRAY_CORE_TRIANGLE_H

// Module config header file must be the first file included within POV-Ray unit header files
#include "core/configcore.h"

// C++ variants of C standard header files
// C++ standard header files
//  (none at the moment)

// POV-Ray header files (base module)
//  (none at the moment)

// POV-Ray header files (core module)
#include "core/scene/object.h"

namespace pov
{

//##############################################################################
///
/// @addtogroup PovCoreShape
///
/// @{

//******************************************************************************
///
/// @name Object Types
///
/// @{

#define TRIANGLE_OBJECT        (PATCH_OBJECT)
#define SMOOTH_TRIANGLE_OBJECT (PATCH_OBJECT)

/// @}
///
//******************************************************************************

class Triangle : public NonsolidObject
{
    public:
        Vector3d        P1, P2, P3;
        Vector3d        Normal_Vector;
        DBL             Distance;
        unsigned int    Dominant_Axis:2;
        unsigned int    vAxis:2;  /* used only for smooth triangles */
        bool            mPointOrderSwapped:1; ///< Whether ordering of points had been swapped

        Triangle();
        Triangle(int t);
        virtual ~Triangle() override;

        virtual ObjectPtr Copy() override;

        virtual bool All_Intersections(const Ray&, IStack&, TraceThreadData *) override;
        virtual bool Inside(const Vector3d&, TraceThreadData *) const override;
        virtual void Normal(Vector3d&, Intersection *, TraceThreadData *) const override;
        // virtual void UVCoord(Vector2d&, const Intersection *) const override; // TODO FIXME - why is there no UV-mapping for this trivial object? [trf]
        virtual void Translate(const Vector3d&, const TRANSFORM *) override;
        virtual void Rotate(const Vector3d&, const TRANSFORM *) override;
        virtual void Scale(const Vector3d&, const TRANSFORM *) override;
        virtual void Transform(const TRANSFORM *) override;
        virtual void Compute_BBox() override;
        virtual bool Intersect_BBox(BBoxDirection, const BBoxVector3d&, const BBoxVector3d&, BBoxScalar) const override;

        virtual bool Compute_Triangle();

		/// Get the proximity of a point to the triangle.
		/// pointOnObject will be populated with the nearest point on the object's surface to the samplePoint.
		/// The method returns the proximity (distance), which is the length of the vector from samplePoint to pointOnObject.
		virtual DBL Proximity(Vector3d &pointOnObject, const Vector3d &samplePoint, TraceThreadData *threaddata) override;
    protected:
        bool Intersect(const BasicRay& ray, DBL *Depth) const;
        void find_triangle_dominant_axis();
};

class SmoothTriangle final : public Triangle
{
    public:
        Vector3d  N1, N2, N3, Perp;

        SmoothTriangle();

        virtual ObjectPtr Copy() override;

        virtual void Normal(Vector3d&, Intersection *, TraceThreadData *) const override;
        virtual void Translate(const Vector3d&, const TRANSFORM *) override;
        virtual void Rotate(const Vector3d&, const TRANSFORM *) override;
        virtual void Scale(const Vector3d&, const TRANSFORM *) override;
        virtual void Transform(const TRANSFORM *) override;

        virtual bool Compute_Triangle() override;

        static DBL Calculate_Smooth_T(const Vector3d& IPoint, const Vector3d& P1, const Vector3d& P2, const Vector3d& P3);
    protected:
        bool Compute_Smooth_Triangle();
};

/// @}
///
//##############################################################################

}
// end of namespace pov

#endif // POVRAY_CORE_TRIANGLE_H
