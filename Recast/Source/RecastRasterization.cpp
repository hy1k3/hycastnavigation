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

#if defined(__SSE2__) || defined(_M_X64) || \
    (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#  define RC_USE_SSE2 1
#  include <immintrin.h>
#  if defined(_MSC_VER)
#    include <intrin.h>
static int rcCtz32(unsigned x)           { unsigned long i; _BitScanForward(&i, x);   return (int)i; }
static int rcCtz64(unsigned long long x) { unsigned long i; _BitScanForward64(&i, x); return (int)i; }
#  else
static int rcCtz32(unsigned x)           { return __builtin_ctz(x); }
static int rcCtz64(unsigned long long x) { return __builtin_ctzll(x); }
#  endif
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
// 64×64×64 block voxelization
//
// The heightfield is tiled into BLOCK_XZ×BLOCK_XZ XZ tiles.  The full span-
// height range is sliced into BLOCK_Y chunks.  Each (tile, chunk) combination
// is processed as a dense 256 KB byte block: triangles write area+1 bytes via
// element-wise max; the extract pass converts byte runs to linked-list spans.
// This avoids the pointer-chasing of addSpan during the hot rasterization loop.
// ---------------------------------------------------------------------------

static const int BLOCK_XZ = 64;
static const int BLOCK_Y  = 64;

/// Fill col[yMin..yMax) with element-wise max(col[y], val) using SSE2.
/// val = areaID+1 so that 0 remains the "empty" sentinel.
static void fillColumnSIMD(uint8_t* col, const int yMin, const int yMax, const uint8_t val)
{
	int y = yMin;
#ifdef RC_USE_SSE2
	const __m128i v = _mm_set1_epi8((char)val);
	for (; y + 16 <= yMax; y += 16)
	{
		const __m128i cur = _mm_loadu_si128((const __m128i*)(col + y));
		_mm_storeu_si128((__m128i*)(col + y), _mm_max_epu8(cur, v));
	}
#endif
	for (; y < yMax; ++y)
		col[y] = rcMax(col[y], val);
}

/// Rasterize one triangle into a 64×64×64 voxel block.
///
/// The block covers heightfield cells [tileX, tileX+BLOCK_XZ) × [tileZ, tileZ+BLOCK_XZ)
/// and span indices [yBase, yBase+BLOCK_Y).  Each byte stores areaID+1
/// (0 = empty); higher value wins when multiple triangles overlap.
///
/// Uses the same plane-equation + 2D SAT approach as the old rasterizeTri.
static void voxelizeTriToBlock(uint8_t* block,
                               const Vec3& v0, const Vec3& v1, const Vec3& v2,
                               const float nx, const float ny, const float nz,
                               const uint8_t areaID,
                               const Vec3& hfMin, const Vec3& hfMax,
                               const float cs, const float ics, const float ich,
                               const int tileX, const int tileZ, const int yBase)
{
	// Practical area IDs are 0–63; stored as 1–64, no uint8 overflow.
	const uint8_t stored = (uint8_t)(areaID + 1u);

	const float triMinX = rcMin(rcMin(v0.x, v1.x), v2.x);
	const float triMaxX = rcMax(rcMax(v0.x, v1.x), v2.x);
	const float triMinY = rcMin(rcMin(v0.y, v1.y), v2.y);
	const float triMaxY = rcMax(rcMax(v0.y, v1.y), v2.y);
	const float triMinZ = rcMin(rcMin(v0.z, v1.z), v2.z);
	const float triMaxZ = rcMax(rcMax(v0.z, v1.z), v2.z);

	// Tile XZ world bounds
	const float tileX0 = hfMin.x + tileX * cs,  tileX1 = tileX0 + BLOCK_XZ * cs;
	const float tileZ0 = hfMin.z + tileZ * cs,  tileZ1 = tileZ0 + BLOCK_XZ * cs;
	if (triMaxX <= tileX0 || triMinX >= tileX1 ||
	    triMaxZ <= tileZ0 || triMinZ >= tileZ1)
		return;

	// Y chunk world bounds
	const float ch = 1.0f / ich;
	const float chunkY0 = hfMin.y + yBase * ch;
	const float chunkY1 = chunkY0 + BLOCK_Y * ch;
	if (triMaxY < chunkY0 || triMinY >= chunkY1)
		return;

	// Cell range clamped to this tile
	const int x0 = rcMax((int)((triMinX - hfMin.x) * ics), tileX);
	const int x1 = rcMin((int)((triMaxX - hfMin.x) * ics), tileX + BLOCK_XZ - 1);
	const int z0 = rcMax((int)((triMinZ - hfMin.z) * ics), tileZ);
	const int z1 = rcMin((int)((triMaxZ - hfMin.z) * ics), tileZ + BLOCK_XZ - 1);
	if (x0 > x1 || z0 > z1) return;

	// Plane equation and SAT — same as rasterizeTri
	const float d     = nx * v0.x + ny * v0.y + nz * v0.z;
	const float inv_ny = (rcAbs(ny) > 1e-6f) ? 1.0f / ny : 0.0f;

	const float a0x = -(v1.z - v0.z), a0z = v1.x - v0.x;
	const float a1x = -(v2.z - v1.z), a1z = v2.x - v1.x;
	const float a2x = -(v0.z - v2.z), a2z = v0.x - v2.x;

	const float p00 = a0x*v0.x + a0z*v0.z,  p20 = a0x*v2.x + a0z*v2.z;
	const float p11 = a1x*v1.x + a1z*v1.z,  p01 = a1x*v0.x + a1z*v0.z;
	const float p22 = a2x*v2.x + a2z*v2.z,  p12 = a2x*v1.x + a2z*v1.z;

	const float t0min = rcMin(p00, p20), t0max = rcMax(p00, p20);
	const float t1min = rcMin(p11, p01), t1max = rcMax(p11, p01);
	const float t2min = rcMin(p22, p12), t2max = rcMax(p22, p12);

	const float r0 = (rcAbs(a0x) + rcAbs(a0z)) * cs * 0.5f;
	const float r1 = (rcAbs(a1x) + rcAbs(a1z)) * cs * 0.5f;
	const float r2 = (rcAbs(a2x) + rcAbs(a2z)) * cs * 0.5f;

	float dy_dx = 0.f, dy_dz = 0.f, min_off = 0.f, max_off = 0.f;
	if (inv_ny != 0.f)
	{
		dy_dx   = -nx * cs * inv_ny;
		dy_dz   = -nz * cs * inv_ny;
		min_off = rcMin(rcMin(0.f, dy_dx), rcMin(dy_dz, dy_dx + dy_dz));
		max_off = rcMax(rcMax(0.f, dy_dx), rcMax(dy_dz, dy_dx + dy_dz));
	}

	const float by = hfMax.y - hfMin.y;

	for (int iz = z0; iz <= z1; ++iz)
	{
		const float cz0 = hfMin.z + iz * cs;
		const float czc = cz0 + cs * 0.5f;
		if (triMaxZ <= cz0 || triMinZ >= cz0 + cs) continue;

		for (int ix = x0; ix <= x1; ++ix)
		{
			const float cx0 = hfMin.x + ix * cs;
			const float cx1 = cx0 + cs;
			if (triMaxX <= cx0 || triMinX >= cx1) continue;

			const float cxc = cx0 + 0.5f * cs;
			const float c0 = a0x * cxc + a0z * czc;
			if (r0 > 0.f && (c0 + r0 <= t0min || c0 - r0 >= t0max)) continue;
			const float c1 = a1x * cxc + a1z * czc;
			if (r1 > 0.f && (c1 + r1 <= t1min || c1 - r1 >= t1max)) continue;
			const float c2 = a2x * cxc + a2z * czc;
			if (r2 > 0.f && (c2 + r2 <= t2min || c2 - r2 >= t2max)) continue;

			float spanMin, spanMax;
			if (inv_ny != 0.f)
			{
				const float y00 = (d - nx * cx0 - nz * cz0) * inv_ny;
				spanMin = rcMax(y00 + min_off, triMinY);
				spanMax = rcMin(y00 + max_off, triMaxY);
			}
			else
			{
				const float cz1 = cz0 + cs;
				spanMin =  1e30f;
				spanMax = -1e30f;
				edgeClipY(v0, v1, cx0, cx1, cz0, cz1, spanMin, spanMax);
				edgeClipY(v1, v2, cx0, cx1, cz0, cz1, spanMin, spanMax);
				edgeClipY(v2, v0, cx0, cx1, cz0, cz1, spanMin, spanMax);
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

			const int col = (iz - tileZ) * BLOCK_XZ + (ix - tileX);
			fillColumnSIMD(block + col * BLOCK_Y, bMin - yBase, bMax - yBase, stored);
		}
	}
}

/// Scan a filled block and call addSpan for each run of occupied bytes.
///
/// Each byte stores areaID+1; 0 = empty.  Runs of non-zero bytes map directly
/// to spans.  SSE2 builds a 64-bit occupancy mask per column in 4 loads.
static bool extractSpansFromBlock(const uint8_t* block, rcHeightfield& hf,
                                  const int tileX, const int tileZ,
                                  const int yBase, const int H,
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

			const uint8_t* col = block + (dz * BLOCK_XZ + dx) * BLOCK_Y;

			// Build 64-bit occupancy mask: bit y = 1 if col[y] != 0
			unsigned long long occ = 0;
#ifdef RC_USE_SSE2
			const __m128i zero = _mm_setzero_si128();
			const __m128i v0v  = _mm_loadu_si128((const __m128i*)(col +  0));
			const __m128i v1v  = _mm_loadu_si128((const __m128i*)(col + 16));
			const __m128i v2v  = _mm_loadu_si128((const __m128i*)(col + 32));
			const __m128i v3v  = _mm_loadu_si128((const __m128i*)(col + 48));
			// movemask of cmpeq-zero gives 1 for empty bytes; invert for occupied
			occ  = (unsigned long long)(uint16_t)~_mm_movemask_epi8(_mm_cmpeq_epi8(v0v, zero));
			occ |= (unsigned long long)(uint16_t)~_mm_movemask_epi8(_mm_cmpeq_epi8(v1v, zero)) << 16;
			occ |= (unsigned long long)(uint16_t)~_mm_movemask_epi8(_mm_cmpeq_epi8(v2v, zero)) << 32;
			occ |= (unsigned long long)(uint16_t)~_mm_movemask_epi8(_mm_cmpeq_epi8(v3v, zero)) << 48;
#else
			for (int y = 0; y < BLOCK_Y; ++y)
				if (col[y]) occ |= (1ULL << y);
#endif
			if (!occ) continue;

			// Extract runs from the occupancy mask using bit tricks
			unsigned long long m = occ;
			int base = 0;  // absolute Y offset within block for current position of m
			while (m)
			{
				const int lo  = rcCtz64(m);          // first set bit = start of run (relative to base)
				const int abs_lo = base + lo;
				const unsigned long long from_lo = m >> lo;
				// Length of the run of 1s starting at bit 0 of from_lo
				const int run = (from_lo == ~0ULL) ? (BLOCK_Y - abs_lo)
				                                   : rcCtz64(~from_lo);
				const int abs_hi = abs_lo + run;

				// Area = max stored byte in [abs_lo, abs_hi) minus 1
				uint8_t maxStored = 0;
				for (int y = abs_lo; y < abs_hi; ++y)
					maxStored = rcMax(maxStored, col[y]);
				const uint8_t area = (uint8_t)(maxStored - 1u);

				const uint16_t smin = (uint16_t)rcMin(yBase + abs_lo, RC_SPAN_MAX_HEIGHT);
				const uint16_t smax = (uint16_t)rcMin(yBase + abs_hi, RC_SPAN_MAX_HEIGHT);
				if (smin < smax && yBase + abs_lo < H)
				{
					if (!addSpan(hf, ix, iz, smin, smax, area, flagMergeThreshold))
						return false;
				}

				// Advance past this run
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

	// Total span-height range, clamped to the maximum representable index.
	// At least 1 so the Y-chunk loop runs even for flat (zero-height) heightfields;
	// spans still get a minimum height of 1 via the yMax clamp in voxelizeTriToBlock.
	const int H = rcMax(rcMin((int)ceilf((hfMax.y - hfMin.y) * ich), RC_SPAN_MAX_HEIGHT), 1);

	// Heap-allocate the 256 KB block to avoid stack overflow.
	uint8_t* const block = (uint8_t*)rcAlloc(BLOCK_XZ * BLOCK_XZ * BLOCK_Y, RC_ALLOC_TEMP);
	if (!block)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	for (int tileZ = 0; tileZ < hfH; tileZ += BLOCK_XZ)
	{
		for (int tileX = 0; tileX < hfW; tileX += BLOCK_XZ)
		{
			for (int yBase = 0; yBase < H; yBase += BLOCK_Y)
			{
				memset(block, 0, BLOCK_XZ * BLOCK_XZ * BLOCK_Y);

				for (int i = 0; i < numTris; ++i)
				{
					voxelizeTriToBlock(block,
					    Vec3(chunk.v0x[i], chunk.v0y[i], chunk.v0z[i]),
					    Vec3(chunk.v1x[i], chunk.v1y[i], chunk.v1z[i]),
					    Vec3(chunk.v2x[i], chunk.v2y[i], chunk.v2z[i]),
					    normals.nx[i], normals.ny[i], normals.nz[i],
					    triAreaIDs[i], hfMin, hfMax, cs, ics, ich,
					    tileX, tileZ, yBase);
				}

				if (!extractSpansFromBlock(block, heightfield,
				                           tileX, tileZ, yBase, H,
				                           flagMergeThreshold))
				{
					rcFree(block);
					context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
					return false;
				}
			}
		}
	}

	rcFree(block);
	return true;
}
