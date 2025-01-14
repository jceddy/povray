//******************************************************************************
///
/// @file core/shape/mesh.cpp
///
/// Implementation of the mesh geometric primitive.
///
/// @author Dieter Bayer
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

/****************************************************************************
*
*  Explanation:
*
*    -
*
*  Syntax:
*
*    mesh
*    {
*      triangle { <CORNER1>, <CORNER2>, <CORNER3>, texture { NAME } }
*      smooth_triangle { <CORNER1>, <NORMAL1>, <CORNER2>, <NORMAL2>, <CORNER3>, <NORMAL3>, texture { NAME } }
*      ...
*      [ hierarchy FLAG ]
*    }
*
*  ---
*
*  Feb 1995 : Creation. [DB]
*
*****************************************************************************/

// Unit header file must be the first file included within POV-Ray *.cpp files (pulls in config)
#include "core/shape/mesh.h"

// C++ variants of C standard header files
//  (none at the moment)

// C++ standard header files
#include <algorithm>
#include <limits>
#include <array>

// POV-Ray header files (base module)
#include "base/pov_err.h"

// POV-Ray header files (core module)
#include "core/bounding/boundingbox.h"
#include "core/material/texture.h"
#include "core/math/matrix.h"
#include "core/render/ray.h"
#include "core/scene/tracethreaddata.h"
#include "core/shape/triangle.h"
#include "core/support/statistics.h"

// this must be the last file included
#include "base/povdebug.h"

namespace pov
{

/*****************************************************************************
* Local preprocessor defines
******************************************************************************/

const DBL DEPTH_TOLERANCE = 1e-6;

#define max3_coordinate(x,y,z) ((x > y) ? ((x > z) ? X : Z) : ((y > z) ? Y : Z))

const int HASH_SIZE = 1000;

const int INITIAL_NUMBER_OF_ENTRIES = 256;

const DBL MAX_PROXIMITY_DISTANCE = 20000000000.0;
const DBL ZERO = 0.0;
const DBL ONE = 1.0;
const DBL TINY_TOLERANCE = 1e-8;



/*****************************************************************************
* Local variables
******************************************************************************/

HASH_TABLE **Mesh::Vertex_Hash_Table;
HASH_TABLE **Mesh::Normal_Hash_Table;
UV_HASH_TABLE **Mesh::UV_Hash_Table;

thread_local std::unique_ptr<BBoxPriorityQueue> Mesh::mtpQueue(new BBoxPriorityQueue());

/*****************************************************************************
*
* FUNCTION
*
*   All_Mesh_Intersections
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

bool Mesh::All_Intersections(const Ray& ray, IStack& Depth_Stack, TraceThreadData *Thread)
{
    Thread->Stats()[Ray_Mesh_Tests]++;

    if (Intersect(ray, Depth_Stack, Thread))
    {
        Thread->Stats()[Ray_Mesh_Tests_Succeeded]++;
        return(true);
    }

    return(false);
}



/*****************************************************************************
*
* FUNCTION
*
*   Intersect_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

bool Mesh::Intersect(const BasicRay& ray, IStack& Depth_Stack, TraceThreadData *Thread)
{
    MeshIndex i;
    bool found;
    DBL len, t;
    BasicRay New_Ray;

    /* Transform the ray into mesh space. */

    if (Trans != nullptr)
    {
        MInvTransRay(New_Ray, ray, Trans);

        len = New_Ray.Direction.length();
        New_Ray.Direction /= len;
    }
    else
    {
        New_Ray = ray;

        len = 1.0;
    }

    found = false;

    if (Data->Tree == nullptr)
    {
        /* There's no bounding hierarchy so just step through all elements. */

        for (i = 0; i < Data->Number_Of_Triangles; i++)
        {
            if (intersect_mesh_triangle(New_Ray, &Data->Triangles[i], &t))
            {
                if (test_hit(&Data->Triangles[i], ray, t, len, Depth_Stack, Thread))
                {
                    found = true;
                }
            }
        }
    }
    else
    {
        /* Use the mesh's bounding hierarchy. */

        return(intersect_bbox_tree(New_Ray, ray, len, Depth_Stack, Thread));
    }

    return(found);
}



/*****************************************************************************
*
* FUNCTION
*
*   Inside_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Nathan Kopp & Dieter Bayer (adapted from Intersect_Mesh by Dieter Bayer)
*
* DESCRIPTION
*
*   Shoot a ray out from this point, if the ray hits an odd number of
*     triangles, it is inside the object.  If it hits an even number
*     than it is outside the object.
*   The triangle mesh should be closed, otherwise results are unpredictable.
*
* CHANGES
*
*   October, 1998 : Creation.
*
******************************************************************************/

bool Mesh::Inside(const Vector3d& IPoint, TraceThreadData *Thread) const
{
    bool inside;
    MeshIndex i;
    unsigned int found;
    DBL t;
    BasicRay ray;

    if (has_inside_vector==false)
        return false;

    ray.Direction = Data->Inside_Vect;
    ray.Origin = IPoint;

    /* Transform the ray into mesh space. */
    if (Trans != nullptr)
    {
        MInvTransRay(ray, ray, Trans);

        ray.Direction.normalize();
    }

    found = 0;

    if (Data->Tree == nullptr)
    {
        /* just step through all elements. */
        for (i = 0; i < Data->Number_Of_Triangles; i++)
        {
            if (intersect_mesh_triangle(ray, &Data->Triangles[i], &t))
            {
                /* actually, this should push onto a local depth stack and
                   make sure that we don't have the same intersection point from
                   two (or three) different triangles!!!!! */
                found++;
            }
        }
        /* odd number = inside, even number = outside */
        inside = ((found & 1) != 0);
    }
    else
    {
        /* Use the mesh's bounding hierarchy. */
        inside = inside_bbox_tree(ray, Thread->Stats());
    }

    if (Test_Flag(this, INVERTED_FLAG))
    {
        inside = !inside;
    }
    return (inside);
}




/*****************************************************************************
*
* FUNCTION
*
*   Mesh_Normal
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Return the normalized normal in the given point.
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Normal(Vector3d& Result, Intersection *Inter, TraceThreadData *Thread) const
{
    Vector3d IPoint;
    const MESH_TRIANGLE *Triangle;

    Triangle = reinterpret_cast<const MESH_TRIANGLE *>(Inter->Pointer);

    if (Triangle->Smooth)
    {
        if (Trans != nullptr)
        {
            MInvTransPoint(IPoint, Inter->IPoint, Trans);
        }
        else
        {
            IPoint = Inter->IPoint;
        }

        Smooth_Mesh_Normal(Result, Triangle, IPoint);

        if (Trans != nullptr)
        {
            MTransNormal(Result, Result, Trans);
        }

        Result.normalize();
    }
    else
    {
        Result = Vector3d(Data->Normals[Triangle->Normal_Ind]);

        if (Trans != nullptr)
        {
            MTransNormal(Result, Result, Trans);

            Result.normalize();
        }
    }
}



/*****************************************************************************
*
* FUNCTION
*
*   Smooth_Mesh_Normal
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Remove the un-normalized normal of a smoothed triangle.
*
* CHANGES
*
*   Feb 1995 : Creation. (Derived from TRIANGLE.C)
*
******************************************************************************/

void Mesh::Smooth_Mesh_Normal(Vector3d& Result, const MESH_TRIANGLE *Triangle, const Vector3d& IPoint) const
{
    int axis;
    DBL u, v;
    DBL k1, k2, k3;
    Vector3d PIMinusP1, N1, N2, N3;

    get_triangle_normals(Triangle, N1, N2, N3);

    PIMinusP1 = IPoint - Vector3d(Data->Vertices[Triangle->P1]);

    u = dot(PIMinusP1, Vector3d(Triangle->Perp));

    if (u < EPSILON)
    {
        Result = N1;
    }
    else
    {
        axis = Triangle->vAxis;

        k1 = Data->Vertices[Triangle->P1][axis];
        k2 = Data->Vertices[Triangle->P2][axis];
        k3 = Data->Vertices[Triangle->P3][axis];

        v = (PIMinusP1[axis] / u + k1 - k2) / (k3 - k2);

        Result = N1 + u * (N2 - N1 + v * (N3 - N2));
    }
}



/*****************************************************************************
*
* FUNCTION
*
*   Translate_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Translate(const Vector3d&, const TRANSFORM *tr)
{
    Transform(tr);
}



/*****************************************************************************
*
* FUNCTION
*
*   Rotate_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Rotate(const Vector3d&, const TRANSFORM *tr)
{
    Transform(tr);
}



/*****************************************************************************
*
* FUNCTION
*
*   Scale_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Scale(const Vector3d&, const TRANSFORM *tr)
{
    Transform(tr);
}



/*****************************************************************************
*
* FUNCTION
*
*   Transfrom_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Transform(const TRANSFORM *tr)
{
    MeshIndex i;

    if (Trans == nullptr)
    {
        Trans = Create_Transform();
    }

    Recompute_BBox(&BBox, tr);

    Compose_Transforms(Trans, tr);

    /* NK 1998 added if */
    if (!Test_Flag(this, UV_FLAG))
        for (i=0; i<Number_Of_Textures; i++)
            Transform_Textures(Textures[i], tr);
}



/*****************************************************************************
*
* FUNCTION
*
*   Create_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

Mesh::Mesh() : ObjectBase(MESH_OBJECT)
{
    Set_Flag(this, HIERARCHY_FLAG);

    Trans = nullptr;

    Data = nullptr;

    has_inside_vector=false;

    Number_Of_Textures=0; /* [LSK] these were uninitialized */
    Textures = nullptr;
}



/*****************************************************************************
*
* FUNCTION
*
*   Copy_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Copy a mesh.
*
*   NOTE: The components are not copied, only the number of references is
*         counted, so that Destroy_Mesh() knows if they can be destroyed.
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

ObjectPtr Mesh::Copy()
{
    Mesh *New = new Mesh();
    MeshIndex i;

    Destroy_Transform(New->Trans);
    *New = *this;
    New->Trans = Copy_Transform(Trans);

    New->Data = Data;
    New->Data->References++;

    /* NK 1999 copy textures */
    if (Textures != nullptr)
    {
        New->Textures = reinterpret_cast<TEXTURE **>(POV_MALLOC(Number_Of_Textures*sizeof(TEXTURE *), "triangle mesh data"));
        for (i = 0; i < Number_Of_Textures; i++)
            New->Textures[i] = Copy_Textures(Textures[i]);
    }

    return(New);
}



/*****************************************************************************
*
* FUNCTION
*
*   Destroy_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

Mesh::~Mesh()
{
    MeshIndex i;

    /* NK 1999 move texture outside of data block */
    if (Textures != nullptr)
    {
        for (i = 0; i < Number_Of_Textures; i++)
        {
            Destroy_Textures(Textures[i]);
        }

        POV_FREE(Textures);
    }

    if (--(Data->References) == 0)
    {
        Destroy_BBox_Tree(Data->Tree);

        if (Data->Normals != nullptr)
        {
            POV_FREE(Data->Normals);
        }

        /* NK 1998 */
        if (Data->UVCoords != nullptr)
        {
            POV_FREE(Data->UVCoords);
        }
        /* NK ---- */

        if (Data->Vertices != nullptr)
        {
            POV_FREE(Data->Vertices);
        }

        if (Data->Triangles != nullptr)
        {
            POV_FREE(Data->Triangles);
        }

		if (Data->kdTree != nullptr) {
			delete(Data->kdTree);
			delete(Data->Triangles_For_Vertex);
		}

        POV_FREE(Data);
    }
}



/*****************************************************************************
*
* FUNCTION
*
*   Compute_Mesh_BBox
*
* INPUT
*
*   Mesh - Mesh
*
* OUTPUT
*
*   Mesh
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Calculate the bounding box of a triangle.
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Compute_BBox()
{
    MeshIndex i;
    Vector3d P1, P2, P3;
    Vector3d mins, maxs;

    mins = Vector3d(BOUND_HUGE);
    maxs = Vector3d(-BOUND_HUGE);

    for (i = 0; i < Data->Number_Of_Triangles; i++)
    {
        get_triangle_vertices(&Data->Triangles[i], P1, P2, P3);

        mins = min(mins, P1, P2, P3);
        maxs = max(maxs, P1, P2, P3);
    }

    Make_BBox_from_min_max(BBox, mins, maxs);
}



/*****************************************************************************
*
* FUNCTION
*
*   Compute_Mesh
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

bool Mesh::Compute_Mesh_Triangle(MESH_TRIANGLE *Triangle, bool Smooth, const Vector3d& P1, const Vector3d& P2, const Vector3d& P3, Vector3d& S_Normal) const
{
    MeshIndex temp;
    bool swap;
    DBL x, y, z;
    Vector3d V1, V2;
    DBL Length;

    const Vector3d *pP1;
    const Vector3d *pP2;

    V1 = P2 - P1;
    V2 = P3 - P1;

    S_Normal = cross(V2, V1);

    Length = S_Normal.length();

    /* Set up a flag so we can ignore degenerate triangles */

    if (Length == 0.0)
    {
        return(false);
    }

    /* Normalize the normal vector. */

    S_Normal /= Length;

    Triangle->Distance = dot(S_Normal, P1);

    Triangle->Distance *= -1.0;

    /* Find triangle's dominant axis. */

    x = fabs(S_Normal[X]);
    y = fabs(S_Normal[Y]);
    z = fabs(S_Normal[Z]);

    Triangle->Dominant_Axis = max3_coordinate(x, y, z);

    swap = false;

    switch (Triangle->Dominant_Axis)
    {
        case X:

            if ((P2[Y] - P3[Y])*(P2[Z] - P1[Z]) < (P2[Z] - P3[Z])*(P2[Y] - P1[Y]))
            {
                swap = true;
            }

            break;

        case Y:

            if ((P2[X] - P3[X])*(P2[Z] - P1[Z]) < (P2[Z] - P3[Z])*(P2[X] - P1[X]))
            {
                swap = true;
            }

            break;

        case Z:

            if ((P2[X] - P3[X])*(P2[Y] - P1[Y]) < (P2[Y] - P3[Y])*(P2[X] - P1[X]))
            {
                swap = true;
            }

            break;
    }

    if (swap)
    {
        temp = Triangle->P2;
        Triangle->P2 = Triangle->P1;
        Triangle->P1 = temp;

        /* NK 1998 */
        temp = Triangle->UV2;
        Triangle->UV2 = Triangle->UV1;
        Triangle->UV1 = temp;

        if (Triangle->ThreeTex)
        {
            temp = Triangle->Texture2;
            Triangle->Texture2 = Triangle->Texture;
            Triangle->Texture = temp;
        }

        pP1 = &P2;
        pP2 = &P1;

        if (Smooth)
        {
            temp = Triangle->N2;
            Triangle->N2 = Triangle->N1;
            Triangle->N1 = temp;
        }
    }
    else
    {
        pP1 = &P1;
        pP2 = &P2;
    }

    if (Smooth)
    {
    //  compute_smooth_triangle(Triangle, *pP1, *pP2, P3);
        Triangle->Smooth = true;
    }

    compute_smooth_triangle(Triangle, *pP1, *pP2, P3);

    return(true);
}



/*****************************************************************************
*
* FUNCTION
*
*   compute_smooth_triangle
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::compute_smooth_triangle(MESH_TRIANGLE *Triangle, const Vector3d& P1, const Vector3d& P2, const Vector3d& P3) const
{
    Vector3d P3MinusP2, VTemp1, VTemp2;
    DBL x, y, z, uDenominator, Proj;

    P3MinusP2 = P3 - P2;

    x = fabs(P3MinusP2[X]);
    y = fabs(P3MinusP2[Y]);
    z = fabs(P3MinusP2[Z]);

    Triangle->vAxis = max3_coordinate(x, y, z);

    VTemp1 = (P2 - P3).normalized();

    VTemp2 = P1 - P3;

    Proj = dot(VTemp2, VTemp1);

    VTemp1 *= Proj;

    Triangle->Perp = MeshVector((VTemp1 - VTemp2).normalized());

    uDenominator = -dot(VTemp2, Vector3d(Triangle->Perp));

    Triangle->Perp /= uDenominator;
}



/*****************************************************************************
*
* FUNCTION
*
*   intersect_mesh_triangle
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

bool Mesh::intersect_mesh_triangle(const BasicRay &ray, const MESH_TRIANGLE *Triangle, DBL *Depth) const
{
    DBL NormalDotOrigin, NormalDotDirection;
    DBL s, t;
    Vector3d P1, P2, P3, S_Normal;

    S_Normal = Vector3d(Data->Normals[Triangle->Normal_Ind]);

    NormalDotDirection = dot(S_Normal, ray.Direction);

    if (fabs(NormalDotDirection) < EPSILON)
    {
        return(false);
    }

    NormalDotOrigin = dot(S_Normal, ray.Origin);

    *Depth = -(Triangle->Distance + NormalDotOrigin) / NormalDotDirection;

    if ((*Depth < DEPTH_TOLERANCE) || (*Depth > MAX_DISTANCE))
    {
        return(false);
    }

    get_triangle_vertices(Triangle, P1, P2, P3);

    switch (Triangle->Dominant_Axis)
    {
        case X:

            s = ray.Origin[Y] + *Depth * ray.Direction[Y];
            t = ray.Origin[Z] + *Depth * ray.Direction[Z];

            if ((P2[Y] - s) * (P2[Z] - P1[Z]) < (P2[Z] - t) * (P2[Y] - P1[Y]))
            {
                return(false);
            }

            if ((P3[Y] - s) * (P3[Z] - P2[Z]) < (P3[Z] - t) * (P3[Y] - P2[Y]))
            {
                return(false);
            }

            if ((P1[Y] - s) * (P1[Z] - P3[Z]) < (P1[Z] - t) * (P1[Y] - P3[Y]))
            {
                return(false);
            }

            return(true);

        case Y:

            s = ray.Origin[X] + *Depth * ray.Direction[X];
            t = ray.Origin[Z] + *Depth * ray.Direction[Z];

            if ((P2[X] - s) * (P2[Z] - P1[Z]) < (P2[Z] - t) * (P2[X] - P1[X]))
            {
                return(false);
            }

            if ((P3[X] - s) * (P3[Z] - P2[Z]) < (P3[Z] - t) * (P3[X] - P2[X]))
            {
                return(false);
            }

            if ((P1[X] - s) * (P1[Z] - P3[Z]) < (P1[Z] - t) * (P1[X] - P3[X]))
            {
                return(false);
            }

            return(true);

        case Z:

            s = ray.Origin[X] + *Depth * ray.Direction[X];
            t = ray.Origin[Y] + *Depth * ray.Direction[Y];

            if ((P2[X] - s) * (P2[Y] - P1[Y]) < (P2[Y] - t) * (P2[X] - P1[X]))
            {
                return(false);
            }

            if ((P3[X] - s) * (P3[Y] - P2[Y]) < (P3[Y] - t) * (P3[X] - P2[X]))
            {
                return(false);
            }

            if ((P1[X] - s) * (P1[Y] - P3[Y]) < (P1[Y] - t) * (P1[X] - P3[X]))
            {
                return(false);
            }

            return(true);
    }

    return(false);
}

/*
 *  MeshUV - By Xander Enzmann
 *
 *  Currently unused
 *
 */

const DBL BARY_VAL1 = -0.00001;
const DBL BARY_VAL2 =  1.00001;

void Mesh::MeshUV(const Vector3d& P, const MESH_TRIANGLE *Triangle, Vector2d& Result) const
{
    DBL a, b, r;
    Vector3d Q;
    Matrix3x3 B, IB;
    Vector3d P1, P2, P3;
    Vector2d UV1, UV2, UV3;

    get_triangle_vertices(Triangle, P1, P2, P3);
    B[0] = P2 - P1;
    B[1] = P3 - P1;
    B[2] = Vector3d(Data->Normals[Triangle->Normal_Ind]);

    if (!MInvers3(B, IB)) {
        // Failed to invert - that means this is a degenerate triangle
        Result[U] = P[X];
        Result[V] = P[Y];
        return;
    }

    Q = P - P1;
    a = dot(Q, IB[0]);
    b = dot(Q, IB[1]);

    if (a < BARY_VAL1 || b < BARY_VAL1 || a + b > BARY_VAL2)
    {
        // The use of BARY_VAL1 is an attempt to compensate for the
        //  lack of precision in the floating point numbers used in
        //  the matrices B and IB.  Since floats only have around
        //  7 digits of precision, we make sure that we allow any
        //  slop in a and b that is less than that. */
        Result[U] = P[X];
        Result[V] = P[Y];
        return;
    }

    r = 1.0f - a - b;

    get_triangle_uvcoords(Triangle, UV1, UV2, UV3);

    Result = r * UV1 + a * UV2 + b * UV3;
}


/*****************************************************************************
*
* FUNCTION
*
*   test_hit
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Test if a hit is valid and push if on the intersection depth.
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

bool Mesh::test_hit(const MESH_TRIANGLE *Triangle, const BasicRay &OrigRay, DBL Depth, DBL len, IStack& Depth_Stack, TraceThreadData *Thread)
{
    Vector3d IPoint;
    DBL world_dist = Depth / len;

    IPoint = OrigRay.Evaluate(world_dist);

    if (Clip.empty() || Point_In_Clip(IPoint, Clip, Thread))
    {
        /*
        don't bother calling MeshUV because we reclaculate it later (if needed) anyway
        Vector2d uv;
        Vector3d P; // Object coordinates of intersection
        P = ray.Evaluate(Depth);

        MeshUV(P, Triangle, Mesh, uv);

        push_entry_pointer_uv(world_dist, IPoint, uv, Object, Triangle, Depth_Stack);
        */

        Depth_Stack->push(Intersection(world_dist, IPoint, this, Triangle));
        return(true);
    }

    return(false);
}



/*****************************************************************************
*
* FUNCTION
*
*   Init_Mesh_Triangle
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Init_Mesh_Triangle(MESH_TRIANGLE *Triangle)
{
    Triangle->Smooth = false;
    Triangle->ThreeTex = false;
    Triangle->Dominant_Axis = 0;
    Triangle->vAxis         = 0;

    Triangle->P1 =
    Triangle->P2 =
    Triangle->P3 = -1;

    Triangle->Normal_Ind = -1;
    Triangle->Texture2 =
    Triangle->Texture3 = -1;

    Triangle->Texture = -1;

    Triangle->N1 =
    Triangle->N2 =
    Triangle->N3 = -1;

    Triangle->UV1 =
    Triangle->UV2 =
    Triangle->UV3 = -1;

    Triangle->Perp = MeshVector(0.0, 0.0, 0.0);

    Triangle->Distance = 0.0;
}



/*****************************************************************************
*
* FUNCTION
*
*   get_triangle_bbox
*
* INPUT
*
*   Triangle - Pointer to triangle
*
* OUTPUT
*
*   BBox     - Bounding box
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Calculate the bounding box of a triangle.
*
* CHANGES
*
*   Sep 1994 : Creation.
*
******************************************************************************/

void Mesh::get_triangle_bbox(const MESH_TRIANGLE *Triangle, BoundingBox *BBox) const
{
    Vector3d P1, P2, P3;
    Vector3d Min, Max;

    get_triangle_vertices(Triangle, P1, P2, P3);

    Min = min(P1, P2, P3);
    Max = max(P1, P2, P3);

    Make_BBox_from_min_max(*BBox, Min, Max);
}



/*****************************************************************************
*
* FUNCTION
*
*   Build_Mesh_BBox_Tree
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Create the bounding box hierarchy.
*
* CHANGES
*
*   Feb 1995 : Creation. (Derived from the bounding slab creation code)
*
******************************************************************************/

void Mesh::Build_Mesh_BBox_Tree()
{
    MeshIndex i, nElem, maxelements;
    BBOX_TREE **Triangles;

    if (!Test_Flag(this, HIERARCHY_FLAG))
    {
        return;
    }

    nElem = Data->Number_Of_Triangles;

    maxelements = 2 * nElem;

    /* Now allocate an array to hold references to these elements. */

    Triangles = reinterpret_cast<BBOX_TREE **>(POV_MALLOC(maxelements*sizeof(BBOX_TREE *), "mesh bbox tree"));

    /* Init list with mesh elements. */

    for (i = 0; i < nElem; i++)
    {
        Triangles[i] = reinterpret_cast<BBOX_TREE *>(POV_MALLOC(sizeof(BBOX_TREE), "mesh bbox tree"));

        Triangles[i]->Infinite = false;
        Triangles[i]->Entries  = 0;
        Triangles[i]->Node     = reinterpret_cast<BBOX_TREE **>(&Data->Triangles[i]);

        get_triangle_bbox(&Data->Triangles[i], &Triangles[i]->BBox);
    }

    size_t maxfinitecount = 0;
    Build_BBox_Tree(&Data->Tree, nElem, Triangles, 0, nullptr, maxfinitecount);

    /* Get rid of the Triangles array. */

    POV_FREE(Triangles);
}



/*****************************************************************************
*
* FUNCTION
*
*   Build_Mesh_KDTree
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Joseph Eddy
*
* DESCRIPTION
*
*   Create the KDTree from the mesh vertices for nearest neighbor calculations.
*
* CHANGES
*
*   Jul 2022 : Creation.
*
******************************************************************************/

void Mesh::Build_Mesh_KDTree() {
	if (Data->kdTree == nullptr) {  // only build the tree once
		pointVec vertices;
		for (int v = 0; v < Data->Number_Of_Vertices; v++) {
			vertices.push_back(Vector3d(Data->Vertices[v]));
		}
		Data->kdTree = new KDTree(vertices);

		Data->Triangles_For_Vertex = new std::vector<std::vector<MESH_TRIANGLE*>>(Data->Number_Of_Vertices);

		for (int t = 0; t < Data->Number_Of_Triangles; t++) {
			Data->Triangles_For_Vertex->at(Data->Triangles[t].P1).push_back(&Data->Triangles[t]);
			Data->Triangles_For_Vertex->at(Data->Triangles[t].P2).push_back(&Data->Triangles[t]);
			Data->Triangles_For_Vertex->at(Data->Triangles[t].P3).push_back(&Data->Triangles[t]);
		}
	}
}



/*****************************************************************************
*
* FUNCTION
*
*   intersect_bbox_tree
*
* INPUT
*
*   Mesh     - Mesh object
*   Ray      - Current ray
*   Orig_Ray - Original, untransformed ray
*   len      - Length of the transformed ray direction
*
* OUTPUT
*
*   Depth_Stack - Stack of intersections
*
* RETURNS
*
*   bool - true if an intersection was found
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Intersect a ray with the bounding box tree of a mesh.
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

bool Mesh::intersect_bbox_tree(const BasicRay &ray, const BasicRay &Orig_Ray, DBL len, IStack& Depth_Stack, TraceThreadData *Thread)
{
    bool found;
    MeshIndex i;
    DBL Best, Depth;
    const BBOX_TREE *Node, *Root;
    bool OldStyle = has_inside_vector;

    /* Create the direction vectors for this ray. */
    Rayinfo rayinfo(ray);

    /* Start with an empty priority queue. */
    mtpQueue->Clear();
    found = false;

    Best = BOUND_HUGE;

#ifdef BBOX_EXTRA_STATS
    Thread->Stats()[totalQueueResets]++;
#endif

    /* Check top node. */

    Root = Data->Tree;

    /* Set the root object infinite to avoid a test. */

    Check_And_Enqueue(*mtpQueue, Root, &Root->BBox, &rayinfo, Thread->Stats());

    /* Check elements in the priority queue. */

    while (!mtpQueue->IsEmpty())
    {
        mtpQueue->RemoveMin(Depth, Node);

        /*
         * If current intersection is larger than the best intersection found
         * so far our task is finished, because all other bounding boxes in
         * the priority queue are further away.
         */

        /* NK 1999 - had to comment this out for use with CSG
        if (Depth > Best)
        {
            break;
        }
        */
        if ( !OldStyle && Depth > Best)
            break;

        /* Check current node. */

        if (Node->Entries)
        {
            /* This is a node containing leaves to be checked. */

            for (i = 0; i < Node->Entries; i++)
                Check_And_Enqueue(*mtpQueue, Node->Node[i], &Node->Node[i]->BBox, &rayinfo, Thread->Stats());
        }
        else
        {
            /* This is a leaf so test the contained triangle. */

            if (intersect_mesh_triangle(ray, reinterpret_cast<MESH_TRIANGLE *>(Node->Node), &Depth))
            {
                if (test_hit(reinterpret_cast<MESH_TRIANGLE *>(Node->Node), Orig_Ray, Depth, len, Depth_Stack, Thread))
                {
                    found = true;

                    Best = Depth;
                }
            }
        }
    }

    return(found);
}



/*****************************************************************************
*
* FUNCTION
*
*   mesh_hash
*
* INPUT
*
*   aPoint - Normal/Vertex to store
*
* OUTPUT
*
*   Hash_Table - Normal/Vertex hash table
*   Number     - Number of normals/vertices
*   Max        - Max. number of normals/vertices
*   Elements   - List of normals/vertices
*
* RETURNS
*
*   MeshIndex - Index of normal/vertex into the normals/vertices list
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Try to locate a triangle normal/vertex in the normal/vertex list.
*   If the vertex is not found its stored in the normal/vertex list.
*
* CHANGES
*
*   Feb 1995 : Creation. (With help from Steve Anger's RAW2POV code)
*
******************************************************************************/

MeshIndex Mesh::mesh_hash(HASH_TABLE **Hash_Table, MeshIndex *Number, MeshIndex  *Max, MeshVector **Elements, const Vector3d& aPoint)
{
    int hash;
    MeshVector D, P;
    HASH_TABLE *p;

    P = MeshVector(aPoint);

    /* Get hash value. */

    // TODO - This is inefficient for very small meshes. Maybe scale by 2^N (or 2^-N) into the range 1.0..2.0 first.
    hash = (unsigned int)((int)(326.0*P[X])^(int)(694.7*P[Y])^(int)(1423.6*P[Z])) % HASH_SIZE;

    /* Try to find normal/vertex. */

    for (p = Hash_Table[hash]; p != nullptr; p = p->Next)
    {
        D = p->P - P;

        if ((fabs(D[X]) < EPSILON) && (fabs(D[Y]) < EPSILON) && (fabs(D[Z]) < EPSILON))
        {
            break;
        }
    }

    if ((p != nullptr) && (p->Index >= 0))
    {
        return(p->Index);
    }

    /* Add new normal/vertex to the list and hash table. */

    if ((*Number) >= (*Max))
    {
        if ((*Max) >= std::numeric_limits<MeshIndex>::max()/2)
        {
            throw POV_EXCEPTION_STRING("Too many normals/vertices in mesh.");
        }

        (*Max) *= 2;

        (*Elements) = reinterpret_cast<MeshVector *>(POV_REALLOC((*Elements), (*Max)*sizeof(MeshVector), "mesh data"));
    }

    (*Elements)[*Number] = P;

    p = reinterpret_cast<HASH_TABLE *>(POV_MALLOC(sizeof(HASH_TABLE), "mesh data"));

    p->P = P;

    p->Index = *Number;

    p->Next = Hash_Table[hash];

    Hash_Table[hash] = p;

    return((*Number)++);
}



/*****************************************************************************
*
* FUNCTION
*
*   Mesh_Hash_Vertex
*
* INPUT
*
*   Vertex - Vertex to store
*
* OUTPUT
*
*   Number_Of_Vertices - Number of vertices
*   Max_Vertices       - Max. number of vertices
*   Vertices           - List of vertices
*
* RETURNS
*
*   MeshIndex - Index of vertex into the vertices list
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Try to locate a triangle vertex in the vertex list.
*   If the vertex is not found its stored in the vertex list.
*
* CHANGES
*
*   Feb 1995 : Creation. (With help from Steve Anger's RAW2POV code)
*
******************************************************************************/

MeshIndex Mesh::Mesh_Hash_Vertex(MeshIndex *Number_Of_Vertices, MeshIndex *Max_Vertices, MeshVector **Vertices, const Vector3d& Vertex)
{
    return(mesh_hash(Vertex_Hash_Table, Number_Of_Vertices, Max_Vertices, Vertices, Vertex));
}



/*****************************************************************************
*
* FUNCTION
*
*   Mesh_Hash_Normal
*
* INPUT
*
*   Normal - Normal to store
*
* OUTPUT
*
*   Number_Of_Normals - Number of normals
*   Max_Normals       - Max. number of normals
*   Normals           - List of normals
*
* RETURNS
*
*   MeshIndex - Index of normal into the normals list
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Try to locate a triangle normal in the normal list.
*   If the normal is not found its stored in the normal list.
*
* CHANGES
*
*   Feb 1995 : Creation. (With help from Steve Anger's RAW2POV code)
*
******************************************************************************/

MeshIndex Mesh::Mesh_Hash_Normal(MeshIndex *Number_Of_Normals, MeshIndex *Max_Normals, MeshVector **Normals, const Vector3d& S_Normal)
{
    return(mesh_hash(Normal_Hash_Table, Number_Of_Normals, Max_Normals, Normals, S_Normal));
}



/*****************************************************************************
*
* FUNCTION
*
*   Mesh_Hash_Texture
*
* INPUT
*
*   Texture - Texture to store
*
* OUTPUT
*
*   Number_Of_Textures - Number of textures
*   Max_Textures       - Max. number of textures
*   Textures           - List of textures
*
* RETURNS
*
*   MeshIndex - Index of texture into the texture list
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Try to locate a texture in the texture list.
*   If the texture is not found its stored in the texture list.
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

MeshIndex Mesh::Mesh_Hash_Texture(MeshIndex *Number_Of_Textures, MeshIndex *Max_Textures, TEXTURE ***Textures, TEXTURE *Texture)
{
    MeshIndex i;

    if (Texture == nullptr)
    {
        return(-1);
    }

    /* Just do a linear search. */

    for (i = 0; i < *Number_Of_Textures; i++)
    {
        if ((*Textures)[i] == Texture)
        {
            break;
        }
    }

    if (i == *Number_Of_Textures)
    {
        if ((*Number_Of_Textures) >= (*Max_Textures))
        {
            if ((*Max_Textures) >= std::numeric_limits<MeshIndex>::max()/2)
            {
                throw POV_EXCEPTION_STRING("Too many textures in mesh.");
            }

            (*Max_Textures) *= 2;

            (*Textures) = reinterpret_cast<TEXTURE **>(POV_REALLOC((*Textures), (*Max_Textures)*sizeof(TEXTURE *), "mesh data"));
        }

        (*Textures)[(*Number_Of_Textures)++] = Copy_Texture_Pointer(Texture);
    }

    return(i);
}

/*****************************************************************************
*
* FUNCTION
*
*   Mesh_Hash_UV
*
* INPUT
*
*   aPoint - UV vector to store
*
* OUTPUT
*
*   Hash_Table - UV vector hash table
*   Number     - Number of UV vectors
*   Max        - Max. number of UV vectors
*   Elements   - List of UV vectors
*
* RETURNS
*
*   MeshIndex - Index of UV vector into the UV vector list
*
* AUTHOR
*
*   Dieter Bayer / Nathan Kopp
*
* DESCRIPTION
*
*   adapted from mesh_hash
*
* CHANGES
*
*
******************************************************************************/

MeshIndex Mesh::Mesh_Hash_UV(MeshIndex *Number, MeshIndex *Max, MeshUVVector **Elements, const Vector2d& aPoint)
{
    int hash;
    MeshUVVector D, P;
    UV_HASH_TABLE *p;

    P = MeshUVVector(aPoint);

    /* Get hash value. */

    // TODO - This is inefficient for very small meshes. Maybe scale by 2^N (or 2^-N) into the range 1.0..2.0 first.
    hash = (unsigned int)((int)(326.0*P[U])^(int)(694.7*P[V])) % HASH_SIZE;

    /* Try to find normal/vertex. */

    for (p = UV_Hash_Table[hash]; p != nullptr; p = p->Next)
    {
        /* VSub(D, p->P, P); */
        D = p->P - P;

        if ((fabs(D[U]) < EPSILON) && (fabs(D[V]) < EPSILON))
        {
            break;
        }
    }

    if ((p != nullptr) && (p->Index >= 0))
    {
        return(p->Index);
    }

    /* Add new normal/vertex to the list and hash table. */

    if ((*Number) >= (*Max))
    {
        if ((*Max) >= std::numeric_limits<MeshIndex>::max()/2)
        {
            throw POV_EXCEPTION_STRING("Too many normals/vertices in mesh.");
        }

        (*Max) *= 2;

        (*Elements) = reinterpret_cast<MeshUVVector *>(POV_REALLOC((*Elements), (*Max)*sizeof(MeshUVVector), "mesh data"));
    }

    (*Elements)[*Number] = P;

    p = reinterpret_cast<UV_HASH_TABLE *>(POV_MALLOC(sizeof(UV_HASH_TABLE), "mesh data"));

    p->P = P;

    p->Index = *Number;

    p->Next = UV_Hash_Table[hash];

    UV_Hash_Table[hash] = p;

    return((*Number)++);
}



/*****************************************************************************
*
* FUNCTION
*
*   Create_Mesh_Hash_Tables
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Create_Mesh_Hash_Tables()
{
    MeshIndex i;

    Vertex_Hash_Table = reinterpret_cast<HASH_TABLE **>(POV_MALLOC(HASH_SIZE*sizeof(HASH_TABLE *), "mesh hash table"));

    for (i = 0; i < HASH_SIZE; i++)
    {
        Vertex_Hash_Table[i] = nullptr;
    }

    Normal_Hash_Table = reinterpret_cast<HASH_TABLE **>(POV_MALLOC(HASH_SIZE*sizeof(HASH_TABLE *), "mesh hash table"));

    for (i = 0; i < HASH_SIZE; i++)
    {
        Normal_Hash_Table[i] = nullptr;
    }

    /* NK 1998 */
    UV_Hash_Table = reinterpret_cast<UV_HASH_TABLE **>(POV_MALLOC(HASH_SIZE*sizeof(UV_HASH_TABLE *), "mesh hash table"));

    for (i = 0; i < HASH_SIZE; i++)
    {
        UV_Hash_Table[i] = nullptr;
    }
    /* NK ---- */
}



/*****************************************************************************
*
* FUNCTION
*
*   Destroy_Mesh_Hash_Tables
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::Destroy_Mesh_Hash_Tables()
{
    MeshIndex i;
    HASH_TABLE *Temp;
    /* NK 1998 */
    UV_HASH_TABLE *UVTemp;
    /* NK ---- */

    for (i = 0; i < HASH_SIZE; i++)
    {
        while (Vertex_Hash_Table[i] != nullptr)
        {
            Temp = Vertex_Hash_Table[i];

            Vertex_Hash_Table[i] = Temp->Next;

            POV_FREE(Temp);
        }
    }

    POV_FREE(Vertex_Hash_Table);

    for (i = 0; i < HASH_SIZE; i++)
    {
        while (Normal_Hash_Table[i] != nullptr)
        {
            Temp = Normal_Hash_Table[i];

            Normal_Hash_Table[i] = Temp->Next;

            POV_FREE(Temp);
        }
    }

    POV_FREE(Normal_Hash_Table);

    /* NK 1998 */
    for (i = 0; i < HASH_SIZE; i++)
    {
        while (UV_Hash_Table[i] != nullptr)
        {
            UVTemp = UV_Hash_Table[i];

            UV_Hash_Table[i] = UVTemp->Next;

            POV_FREE(UVTemp);
        }
    }

    POV_FREE(UV_Hash_Table);
    /* NK ---- */
}



/*****************************************************************************
*
* FUNCTION
*
*   get_triangle_vertices
*
* INPUT
*
*   Mesh     - Mesh object
*   Triangle - Triangle
*
* OUTPUT
*
* RETURNS
*
*   P1, P2, P3 - Vertices of the triangle
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::get_triangle_vertices(const MESH_TRIANGLE *Triangle, Vector3d& P1, Vector3d& P2, Vector3d& P3) const
{
    P1 = Vector3d(Data->Vertices[Triangle->P1]);
    P2 = Vector3d(Data->Vertices[Triangle->P2]);
    P3 = Vector3d(Data->Vertices[Triangle->P3]);
}



/*****************************************************************************
*
* FUNCTION
*
*   get_triangle_normals
*
* INPUT
*
*   Mesh     - Mesh object
*   Triangle - Triangle
*
* OUTPUT
*
* RETURNS
*
*   N1, N2, N3 - Normals of the triangle
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

void Mesh::get_triangle_normals(const MESH_TRIANGLE *Triangle, Vector3d& N1, Vector3d& N2, Vector3d& N3) const
{
    N1 = Vector3d(Data->Normals[Triangle->N1]);
    N2 = Vector3d(Data->Normals[Triangle->N2]);
    N3 = Vector3d(Data->Normals[Triangle->N3]);
}


/*****************************************************************************
*
* FUNCTION
*
*   get_triangle_uvcoords
*
* INPUT
*
*   Mesh     - Mesh object
*   Triangle - Triangle
*
* OUTPUT
*
* RETURNS
*
*   UV1, UV2, UV3 - UV coordinates of the triangle's corners
*
* AUTHOR
*
*   Nathan Kopp
*
* DESCRIPTION
*
*   adapted from get_triangle_normals
*
* CHANGES
*
******************************************************************************/

void Mesh::get_triangle_uvcoords(const MESH_TRIANGLE *Triangle, Vector2d& UV1, Vector2d& UV2, Vector2d& UV3) const
{
    UV1 = Vector2d(Data->UVCoords[Triangle->UV1]);
    UV2 = Vector2d(Data->UVCoords[Triangle->UV2]);
    UV3 = Vector2d(Data->UVCoords[Triangle->UV3]);
}


/*****************************************************************************
*
* FUNCTION
*
*   Mesh_Degenerate
*
* INPUT
*
*   P1, P2, P3 - Triangle's vertices
*
* OUTPUT
*
* RETURNS
*
*   bool - true if degenerate
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Test if a triangle is degenerate.
*
* CHANGES
*
*   Feb 1995 : Creation.
*
******************************************************************************/

bool Mesh::Degenerate(const Vector3d& P1, const Vector3d& P2, const Vector3d& P3)
{
    Vector3d V1, V2, Temp;
    DBL Length;

    V1 = P1 - P2;
    V2 = P3 - P2;

    Temp = cross(V1, V2);

    Length = Temp.length();

    return(Length == 0.0);
}


/*****************************************************************************
*
* FUNCTION
*
*   Test_Mesh_Opacity
*
* INPUT
*
*   Mesh - Pointer to mesh structure
*
* OUTPUT
*
*   Mesh
*
* RETURNS
*
* AUTHOR
*
*   Dieter Bayer
*
* DESCRIPTION
*
*   Set the opacity flag of the mesh according to the opacity
*   of the mesh's texture(s).
*
* CHANGES
*
*   Apr 1996 : Creation.
*
******************************************************************************/

bool Mesh::IsOpaque() const
{
    if (Test_Flag(this, MULTITEXTURE_FLAG))
    {
        for (MeshIndex i = 0; i < Number_Of_Textures; i++)
        {
            // If component's texture isn't opaque the mesh is neither.
            if ((Textures[i] != nullptr) && !Test_Opacity(Textures[i]))
                return false;
        }
    }

    // Otherwise it's a question of whether the common texture is opaque or not.
    // TODO FIXME - other objects report as non-opaque if Texture == nullptr.
    // TODO FIXME - other objects report as non-opaque if Interior_Texture present and non-opaque.
    // What we probably really want here is `return ObjectBase::IsOpaque()`.
    return (Texture == nullptr) || Test_Opacity(Texture);
}



/*****************************************************************************
*
* FUNCTION
*
*   Mesh_UVCoord
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Nathan Kopp
*
* DESCRIPTION
*
*   computes the UV coordinates of the intersection for a mesh
*
******************************************************************************/

void Mesh::UVCoord(Vector2d& Result, const Intersection *Inter) const
{
    DBL w1, w2, w3, t1, t2;
    Vector3d vA, vB;
    Vector3d Side1, Side2;
    const MESH_TRIANGLE *Triangle;
    Vector3d P;

    if (Trans != nullptr)
        MInvTransPoint(P, Inter->IPoint, Trans);
    else
        P = Inter->IPoint;

    Triangle = reinterpret_cast<const MESH_TRIANGLE *>(Inter->Pointer);

    /* ---------------- this is for P1 ---------------- */
    /* Side1 is opposite side, Side2 is an adjacent side (vector pointing away) */
    Side1 = Vector3d(Data->Vertices[Triangle->P3] - Data->Vertices[Triangle->P2]);
    Side2 = Vector3d(Data->Vertices[Triangle->P3] - Data->Vertices[Triangle->P1]);

    /* find A */
    /* A is a vector from this vertex to the intersection point */
    vA = P - Vector3d(Data->Vertices[Triangle->P1]);

    /* find B */
    /* B is a vector from this intersection to the opposite side (Side1) */
    /*    which is the exact length to get to the side                   */
    /* to do this we split Side2 into two components, but only keep the  */
    /*    one that's perp to Side2                                       */
    t1 = dot(Side2, Side1);
    t2 = dot(Side1, Side1);
    vB = Side1 * (t1/t2) - Side2;

    /* finding the weight is the scale part of a projection of A onto B */
    t1 = dot(vA, vB);
    t2 = dot(vB, vB);
    /* w1 = 1-fabs(t1/t2); */
    w1 = 1+t1/t2;

    /* ---------------- this is for P2 ---------------- */
    Side1 = Vector3d(Data->Vertices[Triangle->P3] - Data->Vertices[Triangle->P1]);
    Side2 = Vector3d(Data->Vertices[Triangle->P3] - Data->Vertices[Triangle->P2]);

    /* find A */
    vA = P - Vector3d(Data->Vertices[Triangle->P2]);

    /* find B */
    t1 = dot(Side2, Side1);
    t2 = dot(Side1, Side1);
    vB = Side1 * (t1/t2) - Side2;

    t1 = dot(vA, vB);
    t2 = dot(vB, vB);
    /* w2 = 1-fabs(t1/t2); */
    w2 = 1+t1/t2;

    /* ---------------- this is for P3 ---------------- */
    Side1 = Vector3d(Data->Vertices[Triangle->P2] - Data->Vertices[Triangle->P1]);
    Side2 = Vector3d(Data->Vertices[Triangle->P2] - Data->Vertices[Triangle->P3]);

    /* find A */
    vA = P - Vector3d(Data->Vertices[Triangle->P3]);

    /* find B */
    t1 = dot(Side2, Side1);
    t2 = dot(Side1, Side1);
    vB = Side1 * (t1/t2) - Side2;

    t1 = dot(vA, vB);
    t2 = dot(vB, vB);
    /* w3 = 1-fabs(t1/t2); */
    w3 = 1+t1/t2;

    Result =  w1 * Vector2d(Data->UVCoords[Triangle->UV1]) +
              w2 * Vector2d(Data->UVCoords[Triangle->UV2]) +
              w3 * Vector2d(Data->UVCoords[Triangle->UV3]);
}


/*****************************************************************************
*
* FUNCTION
*
*   inside_bbox_tree
*
* INPUT
*
*   Mesh     - Mesh object
*   Ray      - Current ray
*
* OUTPUT
*
* RETURNS
*
*   bool - true if inside the object
*
* AUTHOR
*
*   Nathan Kopp & Dieter Bayer
*
* DESCRIPTION
*
*   Check if a point is within the bounding box tree of a mesh.
*
* CHANGES
*
*   Oct 1998 : Creation.
*
******************************************************************************/

bool Mesh::inside_bbox_tree(const BasicRay &ray, RenderStatistics& stats) const
{
    MeshIndex i, found;
    DBL Best, Depth;
    const BBOX_TREE *Node, *Root;

    /* Create the direction vectors for this ray. */
    Rayinfo rayinfo(ray);

    /* Start with an empty priority queue. */
    mtpQueue->Clear();
    found = 0;

    Best = BOUND_HUGE;

#ifdef BBOX_EXTRA_STATS
    stats[totalQueueResets]++;
#endif

    /* Check top node. */
    Root = Data->Tree;

    /* Set the root object infinite to avoid a test. */
    Check_And_Enqueue(*mtpQueue, Root, &Root->BBox, &rayinfo, stats);

    /* Check elements in the priority queue. */
    while (!mtpQueue->IsEmpty())
    {
        mtpQueue->RemoveMin(Depth, Node);

        /* Check current node. */
        if (Node->Entries)
        {
            /* This is a node containing leaves to be checked. */
            for (i = 0; i < Node->Entries; i++)
                Check_And_Enqueue(*mtpQueue, Node->Node[i], &Node->Node[i]->BBox, &rayinfo, stats);
        }
        else
        {
            /* This is a leaf so test the contained triangle. */

            if (intersect_mesh_triangle(ray, reinterpret_cast<MESH_TRIANGLE *>(Node->Node), &Depth))
            {
                /* actually, this should push onto a local depth stack and
                   make sure that we don't have the same intersection point from
                   two (or three) different triangles!!!!! */
                found++;
            }
        }
    }

    /* odd number = inside, even number = outside */
    return ((found & 1) != 0);
}

void Mesh::Determine_Textures(Intersection *isect, bool hitinside, WeightedTextureVector& textures, TraceThreadData *Threaddata)
{
    const MESH_TRIANGLE *tri = reinterpret_cast<const MESH_TRIANGLE *>(isect->Pointer);

    if ((Interior_Texture != nullptr) && (hitinside == true)) // useful feature for checking mesh orientation and other effects [trf]
        textures.push_back(WeightedTexture(1.0, Interior_Texture));
    else if(tri->ThreeTex)
    {
        Vector3d p1, p2, p3;
        Vector3d epoint;
        COLC w1, w2, w3;
        COLC wsum;

        if (Trans != nullptr)
            MInvTransPoint(epoint, isect->IPoint, Trans);
        else
            epoint = isect->IPoint;

        p1 = Vector3d(Data->Vertices[tri->P1]);
        p2 = Vector3d(Data->Vertices[tri->P2]);
        p3 = Vector3d(Data->Vertices[tri->P3]);

        w1 = 1.0 - COLC(SmoothTriangle::Calculate_Smooth_T(epoint, p1, p2, p3));
        w2 = 1.0 - COLC(SmoothTriangle::Calculate_Smooth_T(epoint, p2, p3, p1));
        w3 = 1.0 - COLC(SmoothTriangle::Calculate_Smooth_T(epoint, p3, p1, p2));

        wsum = 1.0 / (w1 + w2 + w3);

        textures.push_back(WeightedTexture(w1 * wsum, Textures[tri->Texture]));
        textures.push_back(WeightedTexture(w2 * wsum, Textures[tri->Texture2]));
        textures.push_back(WeightedTexture(w3 * wsum, Textures[tri->Texture3]));
    }
    else if(tri->Texture >= 0) // TODO FIXME - make sure there always is some valid texture, also for code above! [trf]
        textures.push_back(WeightedTexture(1.0, Textures[tri->Texture]));
    else if (Texture != nullptr)
        textures.push_back(WeightedTexture(1.0, Texture));
}

// helper functions for Proximity()
inline void GetMinEdge02(DBL const& a11, DBL const& b1, std::array<DBL, 2>& p)
{
	p[0] = ZERO;
	if (b1 >= ZERO)
	{
		p[1] = ZERO;
	}
	else if (a11 + b1 <= ZERO)
	{
		p[1] = ONE;
	}
	else
	{
		p[1] = -b1 / a11;
	}
}

inline void GetMinEdge12(DBL const& a01, DBL const& a11, DBL const& b1,
	DBL const& f10, DBL const& f01, std::array<DBL, 2>& p)
{
	DBL h0 = a01 + b1 - f10;
	if (h0 >= ZERO)
	{
		p[1] = ZERO;
	}
	else
	{
		DBL h1 = a11 + b1 - f01;
		if (h1 <= ZERO)
		{
			p[1] = ONE;
		}
		else
		{
			p[1] = h0 / (h0 - h1);
		}
	}
	p[0] = ONE - p[1];
}

inline void GetMinInterior(std::array<DBL, 2> const& p0, DBL const& h0,
	std::array<DBL, 2> const& p1, DBL const& h1, std::array<DBL, 2>& p)
{
	DBL z = h0 / (h0 - h1);
	DBL omz = ONE - z;
	p[0] = omz * p0[0] + z * p1[0];
	p[1] = omz * p0[1] + z * p1[1];
}

DBL Mesh::Proximity(Vector3d &pointOnObject, const Vector3d &samplePoint, TraceThreadData *threaddata) {
	if (Data == nullptr) {
		return MAX_PROXIMITY_DISTANCE;
	}

	// get the nearest mesh vertex point and index
	Vector3d transformedPoint = samplePoint;
	if (Trans != nullptr) {
		MInvTransPoint(transformedPoint, transformedPoint, Trans);
	}
	pointIndex nearestPointIdx = Data->kdTree->nearest_pointIndex(transformedPoint);
	DBL distance_vertex = (transformedPoint - nearestPointIdx.first).length();
	if (distance_vertex <= TINY_TOLERANCE) {
		return 0.0;
	}
	DBL dist = 20000000000.0;

	distance_vertex += TINY_TOLERANCE;
	BoundingBox bounds;
	Make_BBox_from_min_max(bounds, transformedPoint - distance_vertex, transformedPoint + distance_vertex);

	/* find triangles that overlap bounds */
	std::vector<BBOX_TREE*> boxes;
	//Vector3d tempdir;
	if (Intersect_BBox_Tree(boxes, Data->Tree, bounds)) {
		for (int i = 0; i < boxes.size(); i++) {
			MESH_TRIANGLE *triangle = reinterpret_cast<MESH_TRIANGLE *>(boxes[i]->Node);

			Vector3d v0 = Vector3d(Data->Vertices[triangle->P1]);
			Vector3d v1 = Vector3d(Data->Vertices[triangle->P2]);
			Vector3d v2 = Vector3d(Data->Vertices[triangle->P3]);
			Vector3d diff = transformedPoint - v0;
			Vector3d edge0 = v1 - v0;
			Vector3d edge1 = v2 - v0;
			DBL a00 = dot(edge0, edge0);
			DBL a01 = dot(edge0, edge1);
			DBL a11 = dot(edge1, edge1);
			DBL b0 = -dot(diff, edge0);
			DBL b1 = -dot(diff, edge1);

			DBL f00 = b0;
			DBL f10 = b0 + a00;
			DBL f01 = b0 + a01;

			std::array<DBL, 2> p0{}, p1{}, p{};
			DBL dt1{}, h0{}, h1{};

			if (f00 >= ZERO)
			{
				if (f01 >= ZERO)
				{
					// (1) p0 = (0,0), p1 = (0,1), H(z) = G(L(z))
					GetMinEdge02(a11, b1, p);
				}
				else
				{
					// (2) p0 = (0,t10), p1 = (t01,1-t01),
					// H(z) = (t11 - t10)*G(L(z))
					p0[0] = ZERO;
					p0[1] = f00 / (f00 - f01);
					p1[0] = f01 / (f01 - f10);
					p1[1] = ONE - p1[0];
					dt1 = p1[1] - p0[1];
					h0 = dt1 * (a11 * p0[1] + b1);
					if (h0 >= ZERO)
					{
						GetMinEdge02(a11, b1, p);
					}
					else
					{
						h1 = dt1 * (a01 * p1[0] + a11 * p1[1] + b1);
						if (h1 <= ZERO)
						{
							GetMinEdge12(a01, a11, b1, f10, f01, p);
						}
						else
						{
							GetMinInterior(p0, h0, p1, h1, p);
						}
					}
				}
			}
			else if (f01 <= ZERO)
			{
				if (f10 <= ZERO)
				{
					// (3) p0 = (1,0), p1 = (0,1), H(z) = G(L(z)) - F(L(z))
					GetMinEdge12(a01, a11, b1, f10, f01, p);
				}
				else
				{
					// (4) p0 = (t00,0), p1 = (t01,1-t01), H(z) = t11*G(L(z))
					p0[0] = f00 / (f00 - f10);
					p0[1] = ZERO;
					p1[0] = f01 / (f01 - f10);
					p1[1] = ONE - p1[0];
					h0 = p1[1] * (a01 * p0[0] + b1);
					if (h0 >= ZERO)
					{
						p = p0;  // GetMinEdge01
					}
					else
					{
						h1 = p1[1] * (a01 * p1[0] + a11 * p1[1] + b1);
						if (h1 <= ZERO)
						{
							GetMinEdge12(a01, a11, b1, f10, f01, p);
						}
						else
						{
							GetMinInterior(p0, h0, p1, h1, p);
						}
					}
				}
			}
			else if (f10 <= ZERO)
			{
				// (5) p0 = (0,t10), p1 = (t01,1-t01),
				// H(z) = (t11 - t10)*G(L(z))
				p0[0] = ZERO;
				p0[1] = f00 / (f00 - f01);
				p1[0] = f01 / (f01 - f10);
				p1[1] = ONE - p1[0];
				dt1 = p1[1] - p0[1];
				h0 = dt1 * (a11 * p0[1] + b1);
				if (h0 >= ZERO)
				{
					GetMinEdge02(a11, b1, p);
				}
				else
				{
					h1 = dt1 * (a01 * p1[0] + a11 * p1[1] + b1);
					if (h1 <= ZERO)
					{
						GetMinEdge12(a01, a11, b1, f10, f01, p);
					}
					else
					{
						GetMinInterior(p0, h0, p1, h1, p);
					}
				}
			}
			else
			{
				// (6) p0 = (t00,0), p1 = (0,t11), H(z) = t11*G(L(z))
				p0[0] = f00 / (f00 - f10);
				p0[1] = ZERO;
				p1[0] = ZERO;
				p1[1] = f00 / (f00 - f01);
				h0 = p1[1] * (a01 * p0[0] + b1);
				if (h0 >= ZERO)
				{
					p = p0;  // GetMinEdge01
				}
				else
				{
					h1 = p1[1] * (a11 * p1[1] + b1);
					if (h1 <= ZERO)
					{
						GetMinEdge02(a11, b1, p);
					}
					else
					{
						GetMinInterior(p0, h0, p1, h1, p);
					}
				}
			}

			Vector3d closest = v0 + p[0] * edge0 + p[1] * edge1;
			diff = closest - transformedPoint;
			if (Trans != nullptr) {
				MTransDirection(diff, diff, Trans);
			}
			DBL sqrDistance = dot(diff, diff);
			DBL distance = std::sqrt(sqrDistance);

			if (distance < dist) {
				dist = distance;
				pointOnObject = samplePoint + diff;
			}
		}
	}

	if (Inside(samplePoint, threaddata)) {
		return 0.0 - dist;
	}
	else {
		return dist;
	}
}

}
// end of namespace pov
