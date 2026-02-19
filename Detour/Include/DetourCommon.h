//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#pragma once

#include <cstdint>

#include "DetourMath.h"
#include "Vec3.h"
#include <stddef.h>

/**
@defgroup detour Detour

Members in this module are used to create, manipulate, and query navigation 
meshes.

@note This is a summary list of members.  Use the index or search 
feature to find minor members.
*/

/// @name General helper functions
/// @{

/// Used to ignore a function parameter.  VS complains about unused parameters
/// and this silences the warning.
template<class T> void dtIgnoreUnused(const T&) { }

/// Swaps the values of the two parameters.
///  @param[in,out]	a	Value A
///  @param[in,out]	b	Value B
template<class T> inline void dtSwap(T& a, T& b) { T t = a; a = b; b = t; }

/// Returns the minimum of two values.
///  @param[in]		a	Value A
///  @param[in]		b	Value B
///  @return The minimum of the two values.
template<class T> inline constexpr T dtMin(T a, T b) { return a < b ? a : b; }

/// Returns the maximum of two values.
///  @param[in]		a	Value A
///  @param[in]		b	Value B
///  @return The maximum of the two values.
template<class T> inline constexpr T dtMax(T a, T b) { return a > b ? a : b; }

/// Returns the absolute value.
///  @param[in]		a	The value.
///  @return The absolute value of the specified value.
template<class T> inline constexpr T dtAbs(T a) { return a < 0 ? -a : a; }

/// Returns the square of the value.
///  @param[in]		a	The value.
///  @return The square of the value.
template<class T> inline constexpr T dtSqr(T a) { return a*a; }

/// Clamps the value to the specified range.
///  @param[in]		v	The value to clamp.
///  @param[in]		mn	The minimum permitted return value.
///  @param[in]		mx	The maximum permitted return value.
///  @return The value, clamped to the specified range.
template<class T> inline constexpr T dtClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

/// @}
/// @name Computational geometry helper functions.
/// @{

/// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
///  @param[in]		a		Vertex A. [(x, y, z)]
///  @param[in]		b		Vertex B. [(x, y, z)]
///  @param[in]		c		Vertex C. [(x, y, z)]
/// @return The signed xz-plane area of the triangle.
inline float dtTriArea2D(const Vec3& a, const Vec3& b, const Vec3& c)
{
	const float abx = b.x - a.x;
	const float abz = b.z - a.z;
	const float acx = c.x - a.x;
	const float acz = c.z - a.z;
	return acx*abz - abx*acz;
}

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
///  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
///  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
///  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapBounds
[[nodiscard]] inline bool dtOverlapQuantBounds(const uint16_t amin[3], const uint16_t amax[3],
								 const uint16_t bmin[3], const uint16_t bmax[3])
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
///  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
///  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
///  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapQuantBounds
[[nodiscard]] inline bool dtOverlapBounds(const Vec3& amin, const Vec3& amax,
							const Vec3& bmin, const Vec3& bmax)
{
	bool overlap = true;
	overlap = (amin.x > bmax.x || amax.x < bmin.x) ? false : overlap;
	overlap = (amin.y > bmax.y || amax.y < bmin.y) ? false : overlap;
	overlap = (amin.z > bmax.z || amax.z < bmin.z) ? false : overlap;
	return overlap;
}

/// Derives the closest point on a triangle from the specified reference point.
///  @param[out]	closest	The closest point on the triangle.	
///  @param[in]		p		The reference point from which to test. [(x, y, z)]
///  @param[in]		a		Vertex A of triangle ABC. [(x, y, z)]
///  @param[in]		b		Vertex B of triangle ABC. [(x, y, z)]
///  @param[in]		c		Vertex C of triangle ABC. [(x, y, z)]
Vec3 dtClosestPtPointTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c);

/// Derives the y-axis height of the closest point on the triangle from the specified reference point.
///  @param[in]		p		The reference point from which to test. [(x, y, z)]
///  @param[in]		a		Vertex A of triangle ABC. [(x, y, z)]
///  @param[in]		b		Vertex B of triangle ABC. [(x, y, z)]
///  @param[in]		c		Vertex C of triangle ABC. [(x, y, z)]
///  @param[out]	h		The resulting height.
[[nodiscard]] bool dtClosestHeightPointTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c, float& h);

[[nodiscard]] bool dtIntersectSegmentPoly2D(const Vec3& p0, const Vec3& p1,
							  const Vec3* verts, int nverts,
							  float& tmin, float& tmax,
							  int& segMin, int& segMax);

[[nodiscard]] bool dtIntersectSegSeg2D(const Vec3& ap, const Vec3& aq,
						 const Vec3& bp, const Vec3& bq,
						 float& s, float& t);

/// Determines if the specified point is inside the convex polygon on the xz-plane.
///  @param[in]		pt		The point to check. [(x, y, z)]
///  @param[in]		verts	The polygon vertices. [(x, y, z) * @p nverts]
///  @param[in]		nverts	The number of vertices. [Limit: >= 3]
/// @return True if the point is inside the polygon.
[[nodiscard]] bool dtPointInPolygon(const Vec3& pt, const Vec3* verts, const int nverts);

[[nodiscard]] bool dtDistancePtPolyEdgesSqr(const Vec3& pt, const Vec3* verts, const int nverts,
							float* ed, float* et);

float dtDistancePtSegSqr2D(const Vec3& pt, const Vec3& p, const Vec3& q, float& t);

/// Derives the centroid of a convex polygon.
///  @param[out]	tc		The centroid of the polgyon. [(x, y, z)]
///  @param[in]		idx		The polygon indices. [(vertIndex) * @p nidx]
///  @param[in]		nidx	The number of indices in the polygon. [Limit: >= 3]
///  @param[in]		verts	The polygon vertices. [(x, y, z) * vertCount]
Vec3 dtCalcPolyCenter(const uint16_t* idx, int nidx, const Vec3* verts);

/// Determines if the two convex polygons overlap on the xz-plane.
///  @param[in]		polya		Polygon A vertices.	[(x, y, z) * @p npolya]
///  @param[in]		npolya		The number of vertices in polygon A.
///  @param[in]		polyb		Polygon B vertices.	[(x, y, z) * @p npolyb]
///  @param[in]		npolyb		The number of vertices in polygon B.
/// @return True if the two polygons overlap.
[[nodiscard]] bool dtOverlapPolyPoly2D(const Vec3* polya, const int npolya,
						 const Vec3* polyb, const int npolyb);

/// @}
/// @name Miscellanious functions.
/// @{

inline uint32_t dtNextPow2(uint32_t v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline uint32_t dtIlog2(uint32_t v)
{
	uint32_t r;
	uint32_t shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

constexpr inline int dtAlign4(int x) { return (x+3) & ~3; }

constexpr inline int dtOppositeTile(int side) { return (side+4) & 0x7; }

inline void dtSwapByte(uint8_t* a, uint8_t* b)
{
	uint8_t tmp = *a;
	*a = *b;
	*b = tmp;
}

inline void dtSwapEndian(uint16_t* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x+0, x+1);
}

inline void dtSwapEndian(short* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x+0, x+1);
}

inline void dtSwapEndian(uint32_t* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
}

inline void dtSwapEndian(int* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
}

inline void dtSwapEndian(float* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
}

Vec3 dtRandomPointInConvexPoly(const Vec3* pts, const int npts, float* areas,
							   const float s, const float t);

template<typename TypeToRetrieveAs>
TypeToRetrieveAs* dtGetThenAdvanceBufferPointer(const uint8_t*& buffer, const size_t distanceToAdvance)
{
	TypeToRetrieveAs* returnPointer = reinterpret_cast<TypeToRetrieveAs*>(buffer);
	buffer += distanceToAdvance;
	return returnPointer;
}

template<typename TypeToRetrieveAs>
TypeToRetrieveAs* dtGetThenAdvanceBufferPointer(uint8_t*& buffer, const size_t distanceToAdvance)
{
	TypeToRetrieveAs* returnPointer = reinterpret_cast<TypeToRetrieveAs*>(buffer);
	buffer += distanceToAdvance;
	return returnPointer;
}


/// @}


///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@fn float dtTriArea2D(const float* a, const float* b, const float* c)
@par

The vertices are projected onto the xz-plane, so the y-values are ignored.

This is a low cost function than can be used for various purposes.  Its main purpose
is for point/line relationship testing.

In all cases: A value of zero indicates that all vertices are collinear or represent the same point.
(On the xz-plane.)

When used for point/line relationship tests, AB usually represents a line against which
the C point is to be tested.  In this case:

A positive value indicates that point C is to the left of line AB, looking from A toward B.<br/>
A negative value indicates that point C is to the right of lineAB, looking from A toward B.

When used for evaluating a triangle:

The absolute value of the return value is two times the area of the triangle when it is
projected onto the xz-plane.

A positive return value indicates:

<ul>
<li>The vertices are wrapped in the normal Detour wrap direction.</li>
<li>The triangle's 3D face normal is in the general up direction.</li>
</ul>

A negative return value indicates:

<ul>
<li>The vertices are reverse wrapped. (Wrapped opposite the normal Detour wrap direction.)</li>
<li>The triangle's 3D face normal is in the general down direction.</li>
</ul>

*/
