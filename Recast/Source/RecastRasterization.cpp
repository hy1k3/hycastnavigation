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
#include <cstring>
#include <math.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

#if defined(_MSC_VER)
#  include <intrin.h>
static int rcCtz64(unsigned long long x) { unsigned long i; _BitScanForward64(&i, x); return (int)i; }
#else
static int rcCtz64(unsigned long long x) { return __builtin_ctzll(x); }
#endif


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

// ---------------------------------------------------------------------------
// 64×64×64 block voxelization — bit-per-voxel variant
//
// Triangle AABBs are computed once upfront and used to bucket each triangle
// into the tiles it overlaps (CSR format).  Each tile then processes only its
// own triangle subset, so the inner voxelizer loop is O(triangles_in_tile).
//
// Each tile×yBase×area combination accumulates into a dense 32 KB bit block
// (uint64_t[64×64], one 64-bit word per XZ column encoding 64 Y levels).
// The block fits in L1 cache; the area ID is known per group so no per-voxel
// area storage is needed.
// ---------------------------------------------------------------------------

static const int BLOCK_XZ = 64;
static const int BLOCK_Y  = 64;

/// Precomputed per-triangle axis-aligned bounding box.
struct TriBounds
{
	float minX, maxX, minY, maxY, minZ, maxZ;
};

/// Precomputed per-triangle plane + SAT constants.
///
/// Everything that depends only on the triangle geometry and the (fixed)
/// cell size cs is computed once here and reused across every tile the
/// triangle is bucketed into.  v0/v1/v2 are kept for the vertical-triangle
/// edgeClipY fallback path (inv_ny == 0).
struct TriPlane
{
	// Vertex positions — needed only by the vertical-triangle edgeClipY path.
	Vec3 v0, v1, v2;

	// Plane equation: y(x,z) = (d - nx*x - nz*z) * inv_ny
	// nx/nz are embedded in the Y-slope offsets below; only d and inv_ny
	// are stored explicitly because nx/nz would be needed to re-derive them.
	float nx, nz;       // stored for plane eval: y00 = (d - nx*cx0 - nz*cz0)*inv_ny
	float d, inv_ny;

	// 2D SAT edge normals: perp2D(v_{i+1} - v_i) = (-(dz), dx)
	float a0x, a0z;
	float a1x, a1z;
	float a2x, a2z;

	// Triangle interval on each SAT axis
	float t0min, t0max;
	float t1min, t1max;
	float t2min, t2max;

	// Cell half-width projected onto each SAT axis: (|ax| + |az|) * cs/2
	float r0, r1, r2;

	// Y range across one cell corner (cx0,cz0) → opposite corner, used to
	// bound the span from the single corner evaluation y00:
	//   spanMin = y00 + min_off,  spanMax = y00 + max_off
	float min_off, max_off;
};

/// Rasterize one triangle into a 64×64 bit block (one uint64_t column per XZ cell).
///
/// All per-triangle constants come from the precomputed TriBounds and TriPlane;
/// the tile XZ overlap is guaranteed by the CSR bucketing so only the Y-chunk
/// check and per-cell SAT are performed here.
static void voxelizeTriToBitBlock(uint64_t* block,
                                  const TriBounds& b,
                                  const TriPlane&  p,
                                  const Vec3& hfMin, const Vec3& hfMax,
                                  const float cs, const float ics, const float ich,
                                  const int tileX, const int tileZ, const int yBase)
{
	// Y chunk world bounds
	const float ch      = 1.0f / ich;
	const float chunkY0 = hfMin.y + yBase * ch;
	const float chunkY1 = chunkY0 + BLOCK_Y * ch;
	if (b.maxY < chunkY0 || b.minY >= chunkY1)
		return;

	// Cell range clamped to this tile
	const int x0 = rcMax((int)((b.minX - hfMin.x) * ics), tileX);
	const int x1 = rcMin((int)((b.maxX - hfMin.x) * ics), tileX + BLOCK_XZ - 1);
	const int z0 = rcMax((int)((b.minZ - hfMin.z) * ics), tileZ);
	const int z1 = rcMin((int)((b.maxZ - hfMin.z) * ics), tileZ + BLOCK_XZ - 1);

	const float by = hfMax.y - hfMin.y;

	for (int iz = z0; iz <= z1; ++iz)
	{
		const float cz0 = hfMin.z + iz * cs;
		const float czc = cz0 + cs * 0.5f;
		if (b.maxZ <= cz0 || b.minZ >= cz0 + cs) continue;

		for (int ix = x0; ix <= x1; ++ix)
		{
			const float cx0 = hfMin.x + ix * cs;
			const float cx1 = cx0 + cs;
			if (b.maxX <= cx0 || b.minX >= cx1) continue;

			const float cxc = cx0 + 0.5f * cs;
			const float c0 = p.a0x * cxc + p.a0z * czc;
			if (p.r0 > 0.f && (c0 + p.r0 <= p.t0min || c0 - p.r0 >= p.t0max)) continue;
			const float c1 = p.a1x * cxc + p.a1z * czc;
			if (p.r1 > 0.f && (c1 + p.r1 <= p.t1min || c1 - p.r1 >= p.t1max)) continue;
			const float c2 = p.a2x * cxc + p.a2z * czc;
			if (p.r2 > 0.f && (c2 + p.r2 <= p.t2min || c2 - p.r2 >= p.t2max)) continue;

			float spanMin, spanMax;
			if (p.inv_ny != 0.f)
			{
				const float y00 = (p.d - p.nx * cx0 - p.nz * cz0) * p.inv_ny;
				spanMin = rcMax(y00 + p.min_off, b.minY);
				spanMax = rcMin(y00 + p.max_off, b.maxY);
			}
			else
			{
				const float cz1 = cz0 + cs;
				spanMin =  1e30f;
				spanMax = -1e30f;
				edgeClipY(p.v0, p.v1, cx0, cx1, cz0, cz1, spanMin, spanMax);
				edgeClipY(p.v1, p.v2, cx0, cx1, cz0, cz1, spanMin, spanMax);
				edgeClipY(p.v2, p.v0, cx0, cx1, cz0, cz1, spanMin, spanMax);
				if (spanMin > spanMax) continue;
			}

			// Convert world Y to span indices; clamp to heightfield then to Y chunk
			const float smRel = spanMin - hfMin.y;
			const float sxRel = spanMax - hfMin.y;
			if (sxRel < 0.f || smRel > by) continue;

			const int yMin = rcClamp((int)floorf(rcMax(smRel, 0.f) * ich), 0, RC_SPAN_MAX_HEIGHT);
			const int yMax = rcClamp((int)ceilf(rcMin(sxRel, by)  * ich), yMin + 1, RC_SPAN_MAX_HEIGHT);

			const int bMin = rcMax(yMin, yBase);
			const int bMax = rcMin(yMax, yBase + BLOCK_Y);
			if (bMin >= bMax) continue;

			// OR a run of bits [localMin, localMax) into the column word
			const int localMin = bMin - yBase;
			const int localMax = bMax - yBase;
			const int len = localMax - localMin;
			const uint64_t mask = (len >= 64) ? ~0ULL : ((1ULL << len) - 1) << localMin;
			block[(iz - tileZ) * BLOCK_XZ + (ix - tileX)] |= mask;
		}
	}
}

/// Scan a bit block and call addSpan for each run of set bits.
///
/// Each uint64_t column word encodes 64 Y levels as bits; the area ID is
/// supplied by the caller (known per triangle group).  The column word IS
/// the occupancy mask — no conversion step needed.
static bool extractSpansFromBitBlock(const uint64_t* block, rcHeightfield& hf,
                                     const int tileX, const int tileZ,
                                     const int yBase, const int H,
                                     const uint8_t area,
                                     const int flagMergeThreshold)
{
	for (int dz = 0; dz < BLOCK_XZ; ++dz)
	{
		const int iz = tileZ + dz;
		if (iz >= hf.height) break;

		for (int dx = 0; dx < BLOCK_XZ; ++dx)
		{
			const int ix = tileX + dx;
			if (ix >= hf.width) break;

			const uint64_t occ = block[dz * BLOCK_XZ + dx];
			if (!occ) continue;

			unsigned long long m = occ;
			int base = 0;
			while (m)
			{
				const int lo     = rcCtz64(m);
				const int abs_lo = base + lo;
				const unsigned long long from_lo = m >> lo;
				const int run    = (from_lo == ~0ULL) ? (BLOCK_Y - abs_lo)
				                                      : rcCtz64(~from_lo);
				const int abs_hi = abs_lo + run;

				const uint16_t smin = (uint16_t)rcMin(yBase + abs_lo, RC_SPAN_MAX_HEIGHT);
				const uint16_t smax = (uint16_t)rcMin(yBase + abs_hi, RC_SPAN_MAX_HEIGHT);
				if (smin < smax && yBase + abs_lo < H)
				{
					if (!addSpan(hf, ix, iz, smin, smax, area, flagMergeThreshold))
						return false;
				}

				if (lo + run >= 64) { m = 0; break; }
				m >>= lo + run;
				base = abs_hi;
			}
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

	const float cs    = heightfield.cs;
	const float ics   = 1.0f / cs;
	const float ich   = 1.0f / heightfield.ch;
	const Vec3& hfMin = heightfield.bmin;
	const Vec3& hfMax = heightfield.bmax;
	const int   hfW   = heightfield.width;
	const int   hfH   = heightfield.height;

	// Total span-height range.  At least 1 so the Y-chunk loop runs for flat heightfields.
	const int H = rcMax(rcMin((int)ceilf((hfMax.y - hfMin.y) * ich), RC_SPAN_MAX_HEIGHT), 1);

	// --- Collect distinct area IDs ---
	bool seen[256] = {};
	uint8_t distinctAreas[256];
	int numDistinct = 0;
	for (int i = 0; i < numTris; ++i)
	{
		const uint8_t a = triAreaIDs[i];
		if (!seen[a]) { seen[a] = true; distinctAreas[numDistinct++] = a; }
	}

	// --- Precompute per-triangle AABBs and plane/SAT constants ---
	// Both are computed once here and reused across every tile the triangle is bucketed into.
	TriBounds* triAABBs  = (TriBounds*)rcAlloc(numTris * (int)sizeof(TriBounds),  RC_ALLOC_TEMP);
	TriPlane*  triPlanes = (TriPlane* )rcAlloc(numTris * (int)sizeof(TriPlane),   RC_ALLOC_TEMP);
	if (!triAABBs || !triPlanes)
	{
		rcFree(triPlanes); rcFree(triAABBs);
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}
	for (int i = 0; i < numTris; ++i)
	{
		const Vec3 v0(chunk.v0x[i], chunk.v0y[i], chunk.v0z[i]);
		const Vec3 v1(chunk.v1x[i], chunk.v1y[i], chunk.v1z[i]);
		const Vec3 v2(chunk.v2x[i], chunk.v2y[i], chunk.v2z[i]);
		const float nx = normals.nx[i], ny = normals.ny[i], nz = normals.nz[i];

		triAABBs[i] = {
			rcMin(rcMin(v0.x, v1.x), v2.x), rcMax(rcMax(v0.x, v1.x), v2.x),
			rcMin(rcMin(v0.y, v1.y), v2.y), rcMax(rcMax(v0.y, v1.y), v2.y),
			rcMin(rcMin(v0.z, v1.z), v2.z), rcMax(rcMax(v0.z, v1.z), v2.z),
		};

		const float inv_ny = (rcAbs(ny) > 1e-6f) ? 1.0f / ny : 0.0f;

		const float a0x = -(v1.z - v0.z), a0z = v1.x - v0.x;
		const float a1x = -(v2.z - v1.z), a1z = v2.x - v1.x;
		const float a2x = -(v0.z - v2.z), a2z = v0.x - v2.x;

		const float p00 = a0x*v0.x + a0z*v0.z, p20 = a0x*v2.x + a0z*v2.z;
		const float p11 = a1x*v1.x + a1z*v1.z, p01 = a1x*v0.x + a1z*v0.z;
		const float p22 = a2x*v2.x + a2z*v2.z, p12 = a2x*v1.x + a2z*v1.z;

		float min_off = 0.f, max_off = 0.f;
		if (inv_ny != 0.f)
		{
			const float dy_dx = -nx * cs * inv_ny;
			const float dy_dz = -nz * cs * inv_ny;
			min_off = rcMin(rcMin(0.f, dy_dx), rcMin(dy_dz, dy_dx + dy_dz));
			max_off = rcMax(rcMax(0.f, dy_dx), rcMax(dy_dz, dy_dx + dy_dz));
		}

		triPlanes[i] = {
			v0, v1, v2,
			nx, nz,
			nx*v0.x + ny*v0.y + nz*v0.z,  // d
			inv_ny,
			a0x, a0z, a1x, a1z, a2x, a2z,
			rcMin(p00, p20), rcMax(p00, p20),
			rcMin(p11, p01), rcMax(p11, p01),
			rcMin(p22, p12), rcMax(p22, p12),
			(rcAbs(a0x) + rcAbs(a0z)) * cs * 0.5f,
			(rcAbs(a1x) + rcAbs(a1z)) * cs * 0.5f,
			(rcAbs(a2x) + rcAbs(a2z)) * cs * 0.5f,
			min_off, max_off,
		};
	}

	// --- Bucket triangles into tiles (CSR format) ---
	const int numTilesX = (hfW + BLOCK_XZ - 1) / BLOCK_XZ;
	const int numTilesZ = (hfH + BLOCK_XZ - 1) / BLOCK_XZ;
	const int numTiles  = numTilesX * numTilesZ;

	int* tileCounts = (int*)rcAlloc(numTiles * (int)sizeof(int), RC_ALLOC_TEMP);
	if (!tileCounts) { rcFree(triPlanes); rcFree(triAABBs); context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory."); return false; }
	memset(tileCounts, 0, numTiles * sizeof(int));

	// Count pass: how many triangles touch each tile
	for (int i = 0; i < numTris; ++i)
	{
		const TriBounds& b = triAABBs[i];
		const int txMin = rcMax((int)((b.minX - hfMin.x) * ics) / BLOCK_XZ, 0);
		const int txMax = rcMin((int)((b.maxX - hfMin.x) * ics) / BLOCK_XZ, numTilesX - 1);
		const int tzMin = rcMax((int)((b.minZ - hfMin.z) * ics) / BLOCK_XZ, 0);
		const int tzMax = rcMin((int)((b.maxZ - hfMin.z) * ics) / BLOCK_XZ, numTilesZ - 1);
		for (int tz = tzMin; tz <= tzMax; ++tz)
			for (int tx = txMin; tx <= txMax; ++tx)
				++tileCounts[tz * numTilesX + tx];
	}

	// Prefix sum → per-tile start offsets in the flat index array
	int* tileStarts = (int*)rcAlloc((numTiles + 1) * (int)sizeof(int), RC_ALLOC_TEMP);
	if (!tileStarts) { rcFree(tileCounts); rcFree(triPlanes); rcFree(triAABBs); context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory."); return false; }
	tileStarts[0] = 0;
	for (int t = 0; t < numTiles; ++t)
		tileStarts[t + 1] = tileStarts[t] + tileCounts[t];

	const int totalRefs = tileStarts[numTiles];

	// Fill pass: write triangle indices into per-tile slots
	int* tileTriList = nullptr;
	if (totalRefs > 0)
	{
		tileTriList = (int*)rcAlloc(totalRefs * (int)sizeof(int), RC_ALLOC_TEMP);
		if (!tileTriList) { rcFree(tileStarts); rcFree(tileCounts); rcFree(triPlanes); rcFree(triAABBs); context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory."); return false; }
	}
	memset(tileCounts, 0, numTiles * sizeof(int));  // reuse as fill cursors
	for (int i = 0; i < numTris; ++i)
	{
		const TriBounds& b = triAABBs[i];
		const int txMin = rcMax((int)((b.minX - hfMin.x) * ics) / BLOCK_XZ, 0);
		const int txMax = rcMin((int)((b.maxX - hfMin.x) * ics) / BLOCK_XZ, numTilesX - 1);
		const int tzMin = rcMax((int)((b.minZ - hfMin.z) * ics) / BLOCK_XZ, 0);
		const int tzMax = rcMin((int)((b.maxZ - hfMin.z) * ics) / BLOCK_XZ, numTilesZ - 1);
		for (int tz = tzMin; tz <= tzMax; ++tz)
			for (int tx = txMin; tx <= txMax; ++tx)
			{
				const int t = tz * numTilesX + tx;
				tileTriList[tileStarts[t] + tileCounts[t]++] = i;
			}
	}

	// --- Allocate the 32 KB bit block (reused across all tiles) ---
	const int blockBytes = BLOCK_XZ * BLOCK_XZ * (int)sizeof(uint64_t);
	uint64_t* const block = (uint64_t*)rcAlloc(blockBytes, RC_ALLOC_TEMP);
	if (!block) { rcFree(tileTriList); rcFree(tileStarts); rcFree(tileCounts); rcFree(triPlanes); rcFree(triAABBs); context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory."); return false; }

	// --- Main loop: tile × yBase × area group ---
	bool ok = true;
	for (int tz = 0; tz < numTilesZ && ok; ++tz)
	{
		const int tileZ = tz * BLOCK_XZ;
		for (int tx = 0; tx < numTilesX && ok; ++tx)
		{
			const int tileX = tx * BLOCK_XZ;
			const int  t     = tz * numTilesX + tx;
			const int  nTris = tileStarts[t + 1] - tileStarts[t];
			const int* tris  = tileTriList + tileStarts[t];
			if (nTris == 0) continue;

			for (int yBase = 0; yBase < H && ok; yBase += BLOCK_Y)
			{
				for (int a = 0; a < numDistinct && ok; ++a)
				{
					const uint8_t area = distinctAreas[a];

					memset(block, 0, blockBytes);

					for (int ii = 0; ii < nTris; ++ii)
					{
						const int i = tris[ii];
						if (triAreaIDs[i] != area) continue;
						voxelizeTriToBitBlock(block,
						    triAABBs[i], triPlanes[i],
						    hfMin, hfMax, cs, ics, ich,
						    tileX, tileZ, yBase);
					}

					ok = extractSpansFromBitBlock(block, heightfield,
					                              tileX, tileZ, yBase, H,
					                              area, flagMergeThreshold);
				}
			}
		}
	}

	rcFree(block);
	rcFree(tileTriList);
	rcFree(tileStarts);
	rcFree(tileCounts);
	rcFree(triPlanes);
	rcFree(triAABBs);

	if (!ok)
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
	return ok;
}
