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

#include <cstdint>
#include "DetourCommon.h"
#include "DetourMath.h"

//////////////////////////////////////////////////////////////////////////////////////////

Vec3 dtClosestPtPointTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c)
{
	// Check if P in vertex region outside A
	Vec3 ab = b - a;
	Vec3 ac = c - a;
	Vec3 ap = p - a;
	float d1 = ab.dot(ap);
	float d2 = ac.dot(ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		// barycentric coordinates (1,0,0)
		return a;
	}

	// Check if P in vertex region outside B
	Vec3 bp = p - b;
	float d3 = ab.dot(bp);
	float d4 = ac.dot(bp);
	if (d3 >= 0.0f && d4 <= d3)
	{
		// barycentric coordinates (0,1,0)
		return b;
	}

	// Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// barycentric coordinates (1-v,v,0)
		float v = d1 / (d1 - d3);
		return a + ab * v;
	}

	// Check if P in vertex region outside C
	Vec3 cp = p - c;
	float d5 = ab.dot(cp);
	float d6 = ac.dot(cp);
	if (d6 >= 0.0f && d5 <= d6)
	{
		// barycentric coordinates (0,0,1)
		return c;
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// barycentric coordinates (1-w,0,w)
		float w = d2 / (d2 - d6);
		return a + ac * w;
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	float va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		// barycentric coordinates (0,1-w,w)
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + (c - b) * w;
	}

	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	return a + ab * v + ac * w;
}

bool dtIntersectSegmentPoly2D(const Vec3& p0, const Vec3& p1,
							  const Vec3* verts, int nverts,
							  float& tmin, float& tmax,
							  int& segMin, int& segMax)
{
	static const float EPS = 0.000001f;

	tmin = 0;
	tmax = 1;
	segMin = -1;
	segMax = -1;

	Vec3 dir = p1 - p0;

	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		Vec3 edge = verts[i] - verts[j];
		Vec3 diff = p0 - verts[j];
		const float n = perp2D(edge, diff);
		const float d = perp2D(dir, edge);
		if (fabsf(d) < EPS)
		{
			// S is nearly parallel to this edge
			if (n < 0)
				return false;
			else
				continue;
		}
		const float t = n / d;
		if (d < 0)
		{
			// segment S is entering across this edge
			if (t > tmin)
			{
				tmin = t;
				segMin = j;
				// S enters after leaving polygon
				if (tmin > tmax)
					return false;
			}
		}
		else
		{
			// segment S is leaving across this edge
			if (t < tmax)
			{
				tmax = t;
				segMax = j;
				// S leaves before entering polygon
				if (tmax < tmin)
					return false;
			}
		}
	}

	return true;
}

float dtDistancePtSegSqr2D(const Vec3& pt, const Vec3& p, const Vec3& q, float& t)
{
	float pqx = q.x - p.x;
	float pqz = q.z - p.z;
	float dx = pt.x - p.x;
	float dz = pt.z - p.z;
	float d = pqx*pqx + pqz*pqz;
	t = pqx*dx + pqz*dz;
	if (d > 0) t /= d;
	if (t < 0) t = 0;
	else if (t > 1) t = 1;
	dx = p.x + t*pqx - pt.x;
	dz = p.z + t*pqz - pt.z;
	return dx*dx + dz*dz;
}

Vec3 dtCalcPolyCenter(const uint16_t* idx, int nidx, const Vec3* verts)
{
	Vec3 tc(0.0f, 0.0f, 0.0f);
	for (int j = 0; j < nidx; ++j)
	{
		tc += verts[idx[j]];
	}
	const float s = 1.0f / nidx;
	tc *= s;
	return tc;
}

bool dtClosestHeightPointTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c, float& h)
{
	const float EPS = 1e-6f;
	Vec3 v0 = c - a;
	Vec3 v1 = b - a;
	Vec3 v2 = p - a;

	// Compute scaled barycentric coordinates
	float denom = v0.x * v1.z - v0.z * v1.x;
	if (fabsf(denom) < EPS)
		return false;

	float u = v1.z * v2.x - v1.x * v2.z;
	float v = v0.x * v2.z - v0.z * v2.x;

	if (denom < 0) {
		denom = -denom;
		u = -u;
		v = -v;
	}

	// If point lies inside the triangle, return interpolated ycoord.
	if (u >= 0.0f && v >= 0.0f && (u + v) <= denom) {
		h = a.y + (v0.y * u + v1.y * v) / denom;
		return true;
	}
	return false;
}

/// @par
///
/// All points are projected onto the xz-plane, so the y-values are ignored.
bool dtPointInPolygon(const Vec3& pt, const Vec3* verts, const int nverts)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const Vec3& vi = verts[i];
		const Vec3& vj = verts[j];
		if (((vi.z > pt.z) != (vj.z > pt.z)) &&
			(pt.x < (vj.x-vi.x) * (pt.z-vi.z) / (vj.z-vi.z) + vi.x) )
			c = !c;
	}
	return c;
}

bool dtDistancePtPolyEdgesSqr(const Vec3& pt, const Vec3* verts, const int nverts,
							  float* ed, float* et)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const Vec3& vi = verts[i];
		const Vec3& vj = verts[j];
		if (((vi.z > pt.z) != (vj.z > pt.z)) &&
			(pt.x < (vj.x-vi.x) * (pt.z-vi.z) / (vj.z-vi.z) + vi.x) )
			c = !c;
		ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, et[j]);
	}
	return c;
}

static void projectPoly(const Vec3& axis, const Vec3* poly, const int npoly,
						float& rmin, float& rmax)
{
	rmin = rmax = dot2D(axis, poly[0]);
	for (int i = 1; i < npoly; ++i)
	{
		const float d = dot2D(axis, poly[i]);
		rmin = dtMin(rmin, d);
		rmax = dtMax(rmax, d);
	}
}

inline bool overlapRange(const float amin, const float amax,
						 const float bmin, const float bmax,
						 const float eps)
{
	return ((amin+eps) > bmax || (amax-eps) < bmin) ? false : true;
}

/// @par
///
/// All vertices are projected onto the xz-plane, so the y-values are ignored.
bool dtOverlapPolyPoly2D(const Vec3* polya, const int npolya,
						 const Vec3* polyb, const int npolyb)
{
	const float eps = 1e-4f;

	for (int i = 0, j = npolya-1; i < npolya; j=i++)
	{
		const Vec3& va = polya[j];
		const Vec3& vb = polya[i];
		const Vec3 n(vb.z-va.z, 0.0f, -(vb.x-va.x));
		float amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	for (int i = 0, j = npolyb-1; i < npolyb; j=i++)
	{
		const Vec3& va = polyb[j];
		const Vec3& vb = polyb[i];
		const Vec3 n(vb.z-va.z, 0.0f, -(vb.x-va.x));
		float amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	return true;
}

// Returns a random point in a convex polygon.
// Adapted from Graphics Gems article.
Vec3 dtRandomPointInConvexPoly(const Vec3* pts, const int npts, float* areas,
							   const float s, const float t)
{
	// Calc triangle areas
	float areasum = 0.0f;
	for (int i = 2; i < npts; i++) {
		areas[i] = dtTriArea2D(pts[0], pts[i-1], pts[i]);
		areasum += dtMax(0.001f, areas[i]);
	}
	// Find sub triangle weighted by area.
	const float thr = s*areasum;
	float acc = 0.0f;
	float u = 1.0f;
	int tri = npts - 1;
	for (int i = 2; i < npts; i++) {
		const float dacc = areas[i];
		if (thr >= acc && thr < (acc+dacc))
		{
			u = (thr - acc) / dacc;
			tri = i;
			break;
		}
		acc += dacc;
	}

	float v = dtMathSqrtf(t);

	const float a = 1 - v;
	const float b = (1 - u) * v;
	const float c = u * v;
	const Vec3& pa = pts[0];
	const Vec3& pb = pts[tri-1];
	const Vec3& pc = pts[tri];

	return pa * a + pb * b + pc * c;
}

inline float vperpXZ(const Vec3& a, const Vec3& b) { return a.x*b.z - a.z*b.x; }

bool dtIntersectSegSeg2D(const Vec3& ap, const Vec3& aq,
						 const Vec3& bp, const Vec3& bq,
						 float& s, float& t)
{
	Vec3 u = aq - ap;
	Vec3 v = bq - bp;
	Vec3 w = ap - bp;
	float d = vperpXZ(u,v);
	if (fabsf(d) < 1e-6f) return false;
	s = vperpXZ(v,w) / d;
	t = vperpXZ(u,w) / d;
	return true;
}
