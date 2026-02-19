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
#include <math.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

#if defined(__SSE2__) || defined(_M_X64) || \
    (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#  define RC_USE_SSE2 1
#  include <immintrin.h>
#  if defined(_MSC_VER)
#    include <intrin.h>
static int rcCtz32(unsigned x) { unsigned long i; _BitScanForward(&i, x); return (int)i; }
#  else
static int rcCtz32(unsigned x) { return __builtin_ctz(x); }
#  endif
#endif


/// Check whether two bounding boxes overlap
///
/// @param[in]	aMin	Min axis extents of bounding box A
/// @param[in]	aMax	Max axis extents of bounding box A
/// @param[in]	bMin	Min axis extents of bounding box B
/// @param[in]	bMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
static bool overlapBounds(const Vec3& aMin, const Vec3& aMax, const Vec3& bMin, const Vec3& bMax)
{
	return
		aMin.x <= bMax.x && aMax.x >= bMin.x &&
		aMin.y <= bMax.y && aMax.y >= bMin.y &&
		aMin.z <= bMax.z && aMax.z >= bMin.z;
}

/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 
/// @param[in]	heightfield		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static rcSpan* allocSpan(rcHeightfield& heightfield)
{
	// If necessary, allocate new page and update the freelist.
	if (heightfield.freelist == nullptr || heightfield.freelist->next == nullptr)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* spanPool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (spanPool == nullptr)
		{
			return nullptr;
		}

		// Add the pool into the list of pools.
		spanPool->next = heightfield.pools;
		heightfield.pools = spanPool;
		
		// Add new spans to the free list.
		rcSpan* freeList = heightfield.freelist;
		rcSpan* head = &spanPool->items[0];
		rcSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freeList;
			freeList = it;
		}
		while (it != head);
		heightfield.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* newSpan = heightfield.freelist;
	heightfield.freelist = heightfield.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	heightfield		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(rcHeightfield& heightfield, rcSpan* span)
{
	if (span == nullptr)
	{
		return;
	}
	// Add the span to the front of the free list.
	span->next = heightfield.freelist;
	heightfield.freelist = span;
}

/// Adds a span to the heightfield.  If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
///
/// @param[in]	heightfield					Heightfield to add spans to
/// @param[in]	x					The new span's column cell x index
/// @param[in]	z					The new span's column cell z index
/// @param[in]	min					The new span's minimum cell index
/// @param[in]	max					The new span's maximum cell index
/// @param[in]	areaID				The new span's area type ID
/// @param[in]	flagMergeThreshold	How close two spans maximum extents need to be to merge area type IDs
static bool addSpan(rcHeightfield& heightfield,
                    const int x, const int z,
                    const uint16_t min, const uint16_t max,
                    const uint8_t areaID, const int flagMergeThreshold)
{
	// Create the new span.
	rcSpan* newSpan = allocSpan(heightfield);
	if (newSpan == nullptr)
	{
		return false;
	}
	newSpan->smin = min;
	newSpan->smax = max;
	newSpan->area = areaID;
	newSpan->next = nullptr;
	
	const int columnIndex = x + z * heightfield.width;
	rcSpan* previousSpan = nullptr;
	rcSpan* currentSpan = heightfield.spans[columnIndex];
	
	// Insert the new span, possibly merging it with existing spans.
	while (currentSpan != nullptr)
	{
		if (currentSpan->smin > newSpan->smax)
		{
			// Current span is completely after the new span, break.
			break;
		}
		
		if (currentSpan->smax < newSpan->smin)
		{
			// Current span is completely before the new span.  Keep going.
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{
			// The new span overlaps with an existing span.  Merge them.
			if (currentSpan->smin < newSpan->smin)
			{
				newSpan->smin = currentSpan->smin;
			}
			if (currentSpan->smax > newSpan->smax)
			{
				newSpan->smax = currentSpan->smax;
			}
			
			// Merge flags.
			if (rcAbs((int)newSpan->smax - (int)currentSpan->smax) <= flagMergeThreshold)
			{
				// Higher area ID numbers indicate higher resolution priority.
				newSpan->area = rcMax(newSpan->area, currentSpan->area);
			}
			
			// Remove the current span since it's now merged with newSpan.
			// Keep going because there might be other overlapping spans that also need to be merged.
			rcSpan* next = currentSpan->next;
			freeSpan(heightfield, currentSpan);
			if (previousSpan)
			{
				previousSpan->next = next;
			}
			else
			{
				heightfield.spans[columnIndex] = next;
			}
			currentSpan = next;
		}
	}
	
	// Insert new span after prev
	if (previousSpan != nullptr)
	{
		newSpan->next = previousSpan->next;
		previousSpan->next = newSpan;
	}
	else
	{
		// This span should go before the others in the list
		newSpan->next = heightfield.spans[columnIndex];
		heightfield.spans[columnIndex] = newSpan;
	}

	return true;
}

bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
               const int x, const int z,
               const uint16_t spanMin, const uint16_t spanMax,
               const uint8_t areaID, const int flagMergeThreshold)
{
	rcAssert(context);

	// Span is zero size or inverted size. Ignore.
	if (spanMin >= spanMax)
	{
		context->log(RC_LOG_WARNING, "rcAddSpan: Adding a span with zero or negative size. Ignored.");
		return true;
	}

	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

/// Clips a triangle edge (va→vb) to a cell's XZ box and accumulates the Y range
/// of the clipped portion into [ymin, ymax].  Returns false if no intersection.
static bool edgeClipY(const Vec3& va, const Vec3& vb,
                      const float cx0, const float cx1, const float cz0, const float cz1,
                      float& ymin, float& ymax)
{
	float tmin = 0.0f, tmax = 1.0f;
	const float dx = vb.x - va.x;
	const float dz = vb.z - va.z;

	if (dx > 1e-7f)        tmin = rcMax(tmin, (cx0 - va.x) / dx);
	else if (dx < -1e-7f)  tmax = rcMin(tmax, (cx0 - va.x) / dx);
	else if (va.x < cx0)   return false;

	if (dx > 1e-7f)        tmax = rcMin(tmax, (cx1 - va.x) / dx);
	else if (dx < -1e-7f)  tmin = rcMax(tmin, (cx1 - va.x) / dx);
	else if (va.x > cx1)   return false;

	if (dz > 1e-7f)        tmin = rcMax(tmin, (cz0 - va.z) / dz);
	else if (dz < -1e-7f)  tmax = rcMin(tmax, (cz0 - va.z) / dz);
	else if (va.z < cz0)   return false;

	if (dz > 1e-7f)        tmax = rcMin(tmax, (cz1 - va.z) / dz);
	else if (dz < -1e-7f)  tmin = rcMax(tmin, (cz1 - va.z) / dz);
	else if (va.z > cz1)   return false;

	if (tmin >= tmax) return false;  // zero-length or no intersection — exclude boundary-only touches

	const float y0 = va.y + tmin * (vb.y - va.y);
	const float y1 = va.y + tmax * (vb.y - va.y);
	ymin = rcMin(ymin, rcMin(y0, y1));
	ymax = rcMax(ymax, rcMax(y0, y1));
	return true;
}

/// Rasterize a single triangle to the heightfield using the plane equation.
///
/// For each cell in the triangle's XZ AABB, a 2D separating-axis test (3 edge axes)
/// determines overlap.  The Y span is computed analytically from the plane equation
/// evaluated at the four cell corners — exact for planar triangles, always conservative.
/// Vertical triangles (ny ≈ 0) fall back to the triangle's full Y range.
///
/// This code is extremely hot, so much care should be given to maintaining maximum perf here.
static bool rasterizeTri(const Vec3& v0, const Vec3& v1, const Vec3& v2,
                         const float nx, const float ny, const float nz,
                         const uint8_t areaID, rcHeightfield& heightfield,
                         const Vec3& hfMin, const Vec3& hfMax,
                         const float cs, const float ics, const float ich,
                         const int flagMergeThreshold)
{
	const int w = heightfield.width;
	const int h = heightfield.height;
	const float by = hfMax.y - hfMin.y;

	// Triangle AABB
	const float triMinX = rcMin(rcMin(v0.x, v1.x), v2.x);
	const float triMaxX = rcMax(rcMax(v0.x, v1.x), v2.x);
	const float triMinY = rcMin(rcMin(v0.y, v1.y), v2.y);
	const float triMaxY = rcMax(rcMax(v0.y, v1.y), v2.y);
	const float triMinZ = rcMin(rcMin(v0.z, v1.z), v2.z);
	const float triMaxZ = rcMax(rcMax(v0.z, v1.z), v2.z);

	// Cell range in XZ, clamped to heightfield
	const int x0 = rcClamp((int)((triMinX - hfMin.x) * ics), 0, w - 1);
	const int x1 = rcClamp((int)((triMaxX - hfMin.x) * ics), 0, w - 1);
	const int z0 = rcClamp((int)((triMinZ - hfMin.z) * ics), 0, h - 1);
	const int z1 = rcClamp((int)((triMaxZ - hfMin.z) * ics), 0, h - 1);

	// Plane: d = dot(n, v0).  y(x,z) = (d - nx*x - nz*z) / ny
	const float d = nx * v0.x + ny * v0.y + nz * v0.z;
	const float inv_ny = (rcAbs(ny) > 1e-6f) ? 1.0f / ny : 0.0f;

	// 2D SAT (XZ plane): 3 edge-normal axes.  Box axes are satisfied by AABB iteration.
	// Edge i axis = perp2D(vi+1 - vi) = (-(vi+1.z - vi.z),  vi+1.x - vi.x)
	const float a0x = -(v1.z - v0.z), a0z = v1.x - v0.x;
	const float a1x = -(v2.z - v1.z), a1z = v2.x - v1.x;
	const float a2x = -(v0.z - v2.z), a2z = v0.x - v2.x;

	// Triangle interval on each axis.  v0/v1 project equally on a0, etc.
	const float p00 = a0x * v0.x + a0z * v0.z,  p20 = a0x * v2.x + a0z * v2.z;
	const float p11 = a1x * v1.x + a1z * v1.z,  p01 = a1x * v0.x + a1z * v0.z;
	const float p22 = a2x * v2.x + a2z * v2.z,  p12 = a2x * v1.x + a2z * v1.z;

	const float t0min = rcMin(p00, p20), t0max = rcMax(p00, p20);
	const float t1min = rcMin(p11, p01), t1max = rcMax(p11, p01);
	const float t2min = rcMin(p22, p12), t2max = rcMax(p22, p12);

	// Cell radius projected onto each axis: r = (|ax| + |az|) * cs/2
	const float r0 = (rcAbs(a0x) + rcAbs(a0z)) * cs * 0.5f;
	const float r1 = (rcAbs(a1x) + rcAbs(a1z)) * cs * 0.5f;
	const float r2 = (rcAbs(a2x) + rcAbs(a2z)) * cs * 0.5f;

	for (int iz = z0; iz <= z1; ++iz)
	{
		const float cz0 = hfMin.z + iz * cs;
		const float czc = cz0 + cs * 0.5f;

		for (int ix = x0; ix <= x1; ++ix)
		{
			const float cx0 = hfMin.x + ix * cs;
			const float cx1 = cx0 + cs;
			const float cxc = cx0 + cs * 0.5f;

			// Box-axis checks: exclude cells that only touch the triangle AABB boundary.
			// This is required for degenerate XZ triangles where the edge-normal SAT
			// axes all collapse to the same direction and don't provide X/Z separation.
			if (triMaxX <= cx0 || triMinX >= cx1) continue;
			if (triMaxZ <= cz0 || triMinZ >= cz0 + cs) continue;

			// SAT: skip cell if separated (or only touching) on any edge axis.
			// Using <= / >= matches the polygon-clip approach: degenerate
			// (zero-area) intersections along shared edges are excluded.
			// Skip an axis entirely when its edge has zero XZ length (degenerate edge).
			const float c0 = a0x * cxc + a0z * czc;
			if (r0 > 0.0f && (c0 + r0 <= t0min || c0 - r0 >= t0max)) continue;
			const float c1 = a1x * cxc + a1z * czc;
			if (r1 > 0.0f && (c1 + r1 <= t1min || c1 - r1 >= t1max)) continue;
			const float c2 = a2x * cxc + a2z * czc;
			if (r2 > 0.0f && (c2 + r2 <= t2min || c2 - r2 >= t2max)) continue;

			// Y span: plane equation at 4 cell corners (exact for non-vertical triangles,
			// clamped to the triangle's Y range).  For vertical triangles (ny≈0), clip
			// each triangle edge to the cell XZ box and use the Y of the clipped segments.
			float spanMin, spanMax;
			if (inv_ny != 0.0f)
			{
				const float cz1 = cz0 + cs;
				const float y00 = (d - nx * cx0 - nz * cz0) * inv_ny;
				const float y10 = (d - nx * cx1 - nz * cz0) * inv_ny;
				const float y01 = (d - nx * cx0 - nz * cz1) * inv_ny;
				const float y11 = (d - nx * cx1 - nz * cz1) * inv_ny;
				spanMin = rcMax(rcMin(rcMin(y00, y10), rcMin(y01, y11)), triMinY);
				spanMax = rcMin(rcMax(rcMax(y00, y10), rcMax(y01, y11)), triMaxY);
			}
			else
			{
				// Vertical triangle: clip each edge to this cell's XZ box, track Y range.
				const float cz1 = cz0 + cs;
				spanMin =  1e30f;
				spanMax = -1e30f;
				edgeClipY(v0, v1, cx0, cx1, cz0, cz1, spanMin, spanMax);
				edgeClipY(v1, v2, cx0, cx1, cz0, cz1, spanMin, spanMax);
				edgeClipY(v2, v0, cx0, cx1, cz0, cz1, spanMin, spanMax);
				if (spanMin > spanMax) continue;
			}

			spanMin -= hfMin.y;
			spanMax -= hfMin.y;

			if (spanMax < 0.0f || spanMin > by) continue;
			spanMin = rcMax(spanMin, 0.0f);
			spanMax = rcMin(spanMax, by);

			const uint16_t spanMinIdx = (uint16_t)rcClamp((int)floorf(spanMin * ich), 0, RC_SPAN_MAX_HEIGHT);
			const uint16_t spanMaxIdx = (uint16_t)rcClamp((int)ceilf(spanMax * ich), (int)spanMinIdx + 1, RC_SPAN_MAX_HEIGHT);

			if (!addSpan(heightfield, ix, iz, spanMinIdx, spanMaxIdx, areaID, flagMergeThreshold))
				return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const TriChunk& chunk, const NormalChunk& normals,
                          const int numTris, const uint8_t* triAreaIDs,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != nullptr);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	const float ics = 1.0f / heightfield.cs;
	const float ich = 1.0f / heightfield.ch;
	const Vec3& hfMin = heightfield.bmin;
	const Vec3& hfMax = heightfield.bmax;

	int i = 0;

#ifdef RC_USE_SSE2
	const __m128 bbMinX = _mm_set1_ps(hfMin.x);
	const __m128 bbMinY = _mm_set1_ps(hfMin.y);
	const __m128 bbMinZ = _mm_set1_ps(hfMin.z);
	const __m128 bbMaxX = _mm_set1_ps(hfMax.x);
	const __m128 bbMaxY = _mm_set1_ps(hfMax.y);
	const __m128 bbMaxZ = _mm_set1_ps(hfMax.z);

	for (; i + 4 <= numTris; i += 4)
	{
		const __m128 ax = _mm_loadu_ps(chunk.v0x + i), ay = _mm_loadu_ps(chunk.v0y + i), az = _mm_loadu_ps(chunk.v0z + i);
		const __m128 bx = _mm_loadu_ps(chunk.v1x + i), by = _mm_loadu_ps(chunk.v1y + i), bz = _mm_loadu_ps(chunk.v1z + i);
		const __m128 cx = _mm_loadu_ps(chunk.v2x + i), cy = _mm_loadu_ps(chunk.v2y + i), cz = _mm_loadu_ps(chunk.v2z + i);

		// Compute per-triangle AABB across all three vertices.
		const __m128 triMinX = _mm_min_ps(_mm_min_ps(ax, bx), cx);
		const __m128 triMinY = _mm_min_ps(_mm_min_ps(ay, by), cy);
		const __m128 triMinZ = _mm_min_ps(_mm_min_ps(az, bz), cz);
		const __m128 triMaxX = _mm_max_ps(_mm_max_ps(ax, bx), cx);
		const __m128 triMaxY = _mm_max_ps(_mm_max_ps(ay, by), cy);
		const __m128 triMaxZ = _mm_max_ps(_mm_max_ps(az, bz), cz);

		// Overlap test: triMin <= hfMax && triMax >= hfMin on all axes.
		const __m128 okX = _mm_and_ps(_mm_cmple_ps(triMinX, bbMaxX), _mm_cmpge_ps(triMaxX, bbMinX));
		const __m128 okY = _mm_and_ps(_mm_cmple_ps(triMinY, bbMaxY), _mm_cmpge_ps(triMaxY, bbMinY));
		const __m128 okZ = _mm_and_ps(_mm_cmple_ps(triMinZ, bbMaxZ), _mm_cmpge_ps(triMaxZ, bbMinZ));
		unsigned mask = (unsigned)_mm_movemask_ps(_mm_and_ps(_mm_and_ps(okX, okY), okZ));

		while (mask)
		{
			const int j = rcCtz32(mask);
			if (!rasterizeTri(Vec3(chunk.v0x[i+j], chunk.v0y[i+j], chunk.v0z[i+j]),
			                  Vec3(chunk.v1x[i+j], chunk.v1y[i+j], chunk.v1z[i+j]),
			                  Vec3(chunk.v2x[i+j], chunk.v2y[i+j], chunk.v2z[i+j]),
			                  normals.nx[i+j], normals.ny[i+j], normals.nz[i+j],
			                  triAreaIDs[i+j], heightfield, hfMin, hfMax,
			                  heightfield.cs, ics, ich, flagMergeThreshold))
			{
				context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
				return false;
			}
			mask &= mask - 1;
		}
	}
#endif

	for (; i < numTris; ++i)
	{
		const Vec3 a(chunk.v0x[i], chunk.v0y[i], chunk.v0z[i]);
		const Vec3 b(chunk.v1x[i], chunk.v1y[i], chunk.v1z[i]);
		const Vec3 c(chunk.v2x[i], chunk.v2y[i], chunk.v2z[i]);
		if (!overlapBounds(vmin(vmin(a, b), c), vmax(vmax(a, b), c), hfMin, hfMax))
			continue;
		if (!rasterizeTri(a, b, c, normals.nx[i], normals.ny[i], normals.nz[i],
		                  triAreaIDs[i], heightfield, hfMin, hfMax,
		                  heightfield.cs, ics, ich, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}
