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
		rcSpanPool* span_pool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (span_pool == nullptr)
		{
			return nullptr;
		}

		// Add the pool into the list of pools.
		span_pool->next = heightfield.pools;
		heightfield.pools = span_pool;

		// Add new spans to the free list.
		rcSpan* free_list = heightfield.freelist;
		rcSpan* head = &span_pool->items[0];
		rcSpan* it = &span_pool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = free_list;
			free_list = it;
		}
		while (it != head);
		heightfield.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* new_span = heightfield.freelist;
	heightfield.freelist = heightfield.freelist->next;
	return new_span;
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
/// @param[in]	flag_merge_threshold	How close two spans maximum extents need to be to merge area type IDs
static bool addSpan(rcHeightfield& heightfield,
                    const int x, const int z,
                    const uint16_t min, const uint16_t max,
                    const uint8_t areaID, const int flag_merge_threshold)
{
	// Create the new span.
	rcSpan* new_span = allocSpan(heightfield);
	if (new_span == nullptr)
	{
		return false;
	}
	new_span->smin = min;
	new_span->smax = max;
	new_span->area = areaID;
	new_span->next = nullptr;

	const int column_index = x + z * heightfield.width;
	rcSpan* previous_span = nullptr;
	rcSpan* current_span = heightfield.spans[column_index];

	// Insert the new span, possibly merging it with existing spans.
	while (current_span != nullptr)
	{
		if (current_span->smin > new_span->smax)
		{
			// Current span is completely after the new span, break.
			break;
		}

		if (current_span->smax < new_span->smin)
		{
			// Current span is completely before the new span.  Keep going.
			previous_span = current_span;
			current_span = current_span->next;
		}
		else
		{
			// The new span overlaps with an existing span.  Merge them.
			if (current_span->smin < new_span->smin)
			{
				new_span->smin = current_span->smin;
			}
			if (current_span->smax > new_span->smax)
			{
				new_span->smax = current_span->smax;
			}

			// Merge flags.
			if (rcAbs((int)new_span->smax - (int)current_span->smax) <= flag_merge_threshold)
			{
				// Higher area ID numbers indicate higher resolution priority.
				new_span->area = rcMax(new_span->area, current_span->area);
			}

			// Remove the current span since it's now merged with new_span.
			// Keep going because there might be other overlapping spans that also need to be merged.
			rcSpan* next = current_span->next;
			freeSpan(heightfield, current_span);
			if (previous_span)
			{
				previous_span->next = next;
			}
			else
			{
				heightfield.spans[column_index] = next;
			}
			current_span = next;
		}
	}

	// Insert new span after prev
	if (previous_span != nullptr)
	{
		new_span->next = previous_span->next;
		previous_span->next = new_span;
	}
	else
	{
		// This span should go before the others in the list
		new_span->next = heightfield.spans[column_index];
		heightfield.spans[column_index] = new_span;
	}

	return true;
}

bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
               const int x, const int z,
               const uint16_t span_min, const uint16_t span_max,
               const uint8_t areaID, const int flag_merge_threshold)
{
	rcAssert(context);

	// Span is zero size or inverted size. Ignore.
	if (span_min >= span_max)
	{
		context->log(RC_LOG_WARNING, "rcAddSpan: Adding a span with zero or negative size. Ignored.");
		return true;
	}

	if (!addSpan(heightfield, x, z, span_min, span_max, areaID, flag_merge_threshold))
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
	float t_min = 0.0f, t_max = 1.0f;
	const float dx = vb.x - va.x;
	const float dz = vb.z - va.z;

	if (dx > 1e-7f)        t_min = rcMax(t_min, (cx0 - va.x) / dx);
	else if (dx < -1e-7f)  t_max = rcMin(t_max, (cx0 - va.x) / dx);
	else if (va.x < cx0)   return false;

	if (dx > 1e-7f)        t_max = rcMin(t_max, (cx1 - va.x) / dx);
	else if (dx < -1e-7f)  t_min = rcMax(t_min, (cx1 - va.x) / dx);
	else if (va.x > cx1)   return false;

	if (dz > 1e-7f)        t_min = rcMax(t_min, (cz0 - va.z) / dz);
	else if (dz < -1e-7f)  t_max = rcMin(t_max, (cz0 - va.z) / dz);
	else if (va.z < cz0)   return false;

	if (dz > 1e-7f)        t_max = rcMin(t_max, (cz1 - va.z) / dz);
	else if (dz < -1e-7f)  t_min = rcMax(t_min, (cz1 - va.z) / dz);
	else if (va.z > cz1)   return false;

	if (t_min >= t_max) return false;  // zero-length or no intersection — exclude boundary-only touches

	const float y0 = va.y + t_min * (vb.y - va.y);
	const float y1 = va.y + t_max * (vb.y - va.y);
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
// Each tile×y_base×area combination accumulates into a dense 32 KB bit block
// (uint64_t[64×64], one 64-bit word per XZ column encoding 64 Y levels).
// The block fits in L1 cache; the area ID is known per group so no per-voxel
// area storage is needed.
// ---------------------------------------------------------------------------

static const int BLOCK_XZ = 64;
static const int BLOCK_Y  = 64;

/// Precomputed per-triangle axis-aligned bounding box.
struct TriBoundsSoA
{
	float minX, maxX, minY, maxY, minZ, maxZ;
};

/// Precomputed per-triangle plane + SAT constants.
///
/// Everything that depends only on the triangle geometry and the (fixed)
/// cell size cs is computed once here and reused across every tile the
/// triangle is bucketed into.  v0/v1/v2 are kept for the vertical-triangle
/// edgeClipY fallback path (inv_ny == 0).
struct TriPlaneSoA
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
	//   span_min = y00 + min_off,  span_max = y00 + max_off
	float min_off, max_off;
};

/// Rasterize one triangle into a 64×64 bit block (one uint64_t column per XZ cell).
///
/// All per-triangle constants come from the precomputed TriBoundsSoA and TriPlaneSoA;
/// the tile XZ overlap is guaranteed by the CSR bucketing so only the Y-chunk
/// check and per-cell SAT are performed here.
static void voxelizeTriToBitBlock(uint64_t* block,
                                  const TriBoundsSoA& b,
                                  const TriPlaneSoA&  p,
                                  const Vec3& hf_min, const Vec3& hf_max,
                                  const float cs, const float inv_cs, const float inv_ch,
                                  const int tile_x, const int tile_z, const int y_base)
{
	// Y chunk world bounds
	const float ch      = 1.0f / inv_ch;
	const float chunk_y0 = hf_min.y + y_base * ch;
	const float chunk_y1 = chunk_y0 + BLOCK_Y * ch;
	if (b.maxY < chunk_y0 || b.minY >= chunk_y1)
		return;

	// Cell range clamped to this tile
	const int x0 = rcMax((int)((b.minX - hf_min.x) * inv_cs), tile_x);
	const int x1 = rcMin((int)((b.maxX - hf_min.x) * inv_cs), tile_x + BLOCK_XZ - 1);
	const int z0 = rcMax((int)((b.minZ - hf_min.z) * inv_cs), tile_z);
	const int z1 = rcMin((int)((b.maxZ - hf_min.z) * inv_cs), tile_z + BLOCK_XZ - 1);

	const float hf_y_range = hf_max.y - hf_min.y;

	for (int iz = z0; iz <= z1; ++iz)
	{
		const float cz0 = hf_min.z + iz * cs;
		const float czc = cz0 + cs * 0.5f;
		if (b.maxZ <= cz0 || b.minZ >= cz0 + cs) continue;

		for (int ix = x0; ix <= x1; ++ix)
		{
			const float cx0 = hf_min.x + ix * cs;
			const float cx1 = cx0 + cs;
			if (b.maxX <= cx0 || b.minX >= cx1) continue;

			const float cxc = cx0 + 0.5f * cs;
			const float c0 = p.a0x * cxc + p.a0z * czc;
			if (p.r0 > 0.f && (c0 + p.r0 <= p.t0min || c0 - p.r0 >= p.t0max)) continue;
			const float c1 = p.a1x * cxc + p.a1z * czc;
			if (p.r1 > 0.f && (c1 + p.r1 <= p.t1min || c1 - p.r1 >= p.t1max)) continue;
			const float c2 = p.a2x * cxc + p.a2z * czc;
			if (p.r2 > 0.f && (c2 + p.r2 <= p.t2min || c2 - p.r2 >= p.t2max)) continue;

			float span_min, span_max;
			if (p.inv_ny != 0.f)
			{
				const float y00 = (p.d - p.nx * cx0 - p.nz * cz0) * p.inv_ny;
				span_min = rcMax(y00 + p.min_off, b.minY);
				span_max = rcMin(y00 + p.max_off, b.maxY);
			}
			else
			{
				const float cz1 = cz0 + cs;
				span_min =  1e30f;
				span_max = -1e30f;
				edgeClipY(p.v0, p.v1, cx0, cx1, cz0, cz1, span_min, span_max);
				edgeClipY(p.v1, p.v2, cx0, cx1, cz0, cz1, span_min, span_max);
				edgeClipY(p.v2, p.v0, cx0, cx1, cz0, cz1, span_min, span_max);
				if (span_min > span_max) continue;
			}

			// Convert world Y to span indices; clamp to heightfield then to Y chunk
			const float span_min_rel = span_min - hf_min.y;
			const float span_max_rel = span_max - hf_min.y;
			if (span_max_rel < 0.f || span_min_rel > hf_y_range) continue;

			const int y_min = rcClamp((int)floorf(rcMax(span_min_rel, 0.f) * inv_ch), 0, RC_SPAN_MAX_HEIGHT);
			const int y_max = rcClamp((int)ceilf(rcMin(span_max_rel, hf_y_range)  * inv_ch), y_min + 1, RC_SPAN_MAX_HEIGHT);

			const int b_min = rcMax(y_min, y_base);
			const int b_max = rcMin(y_max, y_base + BLOCK_Y);
			if (b_min >= b_max) continue;

			// OR a run of bits [local_min, local_max) into the column word
			const int local_min = b_min - y_base;
			const int local_max = b_max - y_base;
			const int len = local_max - local_min;
			const uint64_t mask = (len >= 64) ? ~0ULL : ((1ULL << len) - 1) << local_min;
			block[(iz - tile_z) * BLOCK_XZ + (ix - tile_x)] |= mask;
		}
	}
}

/// Scan a bit block and call addSpan for each run of set bits.
///
/// Each uint64_t column word encodes 64 Y levels as bits; the area ID is
/// supplied by the caller (known per triangle group).  The column word IS
/// the occupancy mask — no conversion step needed.
static bool extractSpansFromBitBlock(const uint64_t* block, rcHeightfield& hf,
                                     const int tile_x, const int tile_z,
                                     const int y_base, const int H,
                                     const uint8_t area,
                                     const int flag_merge_threshold)
{
	for (int dz = 0; dz < BLOCK_XZ; ++dz)
	{
		const int iz = tile_z + dz;
		if (iz >= hf.height) break;

		for (int dx = 0; dx < BLOCK_XZ; ++dx)
		{
			const int ix = tile_x + dx;
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

				const uint16_t s_min = (uint16_t)rcMin(y_base + abs_lo, RC_SPAN_MAX_HEIGHT);
				const uint16_t s_max = (uint16_t)rcMin(y_base + abs_hi, RC_SPAN_MAX_HEIGHT);
				if (s_min < s_max && y_base + abs_lo < H)
				{
					if (!addSpan(hf, ix, iz, s_min, s_max, area, flag_merge_threshold))
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
                          const int num_tris, const uint8_t* tri_area_ids,
                          rcHeightfield& heightfield, const int flag_merge_threshold)
{
	rcAssert(context != nullptr);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Cache heightfield parameters into locals for readability and to avoid
	// repeated pointer dereferences in the inner loops.
	const float cs    = heightfield.cs;   // cell size in XZ (metres per voxel column)
	const float inv_cs   = 1.0f / cs;        // reciprocal: world units → voxel column index
	const float inv_ch   = 1.0f / heightfield.ch;  // reciprocal: world Y → voxel row index
	const Vec3& hf_min = heightfield.bmin;  // world-space minimum corner of the heightfield
	const Vec3& hf_max = heightfield.bmax;  // world-space maximum corner
	const int   hf_width   = heightfield.width;   // number of columns in X
	const int   hf_height   = heightfield.height;  // number of columns in Z

	// Total voxel height of the heightfield, clamped to the maximum span height
	// representable in an rcSpan (RC_SPAN_MAX_HEIGHT).  At least 1 so the Y-chunk
	// loop below always executes at least once, even for a completely flat field.
	const int H = rcMax(rcMin((int)ceilf((hf_max.y - hf_min.y) * inv_ch), RC_SPAN_MAX_HEIGHT), 1);

	// --- Collect distinct area IDs present in this batch ---
	// The main rasterization loop processes one area at a time so that spans from
	// different areas stay separate until addSpan merges them with flag_merge_threshold.
	// Collecting distinct IDs here avoids scanning all triangles once per area in
	// the inner loop.  The array is small (at most 256 entries) so it lives on the
	// stack.
	bool seen[256] = {};
	uint8_t distinct_areas[256];
	int num_distinct = 0;
	for (int i = 0; i < num_tris; ++i)
	{
		const uint8_t a = tri_area_ids[i];
		if (!seen[a]) { seen[a] = true; distinct_areas[num_distinct++] = a; }
	}

	// --- Precompute per-triangle AABBs and plane/SAT constants ---
	// All of these are derived from the triangle vertices and normal once, then
	// reused for every tile the triangle overlaps.  Paying the upfront cost here
	// amortises the per-tile work across however many tiles each triangle touches.
	rcScopedDelete<TriBoundsSoA> tri_aabbs ((TriBoundsSoA*)rcAlloc(num_tris * (int)sizeof(TriBoundsSoA),  RC_ALLOC_TEMP));
	rcScopedDelete<TriPlaneSoA>  tri_planes((TriPlaneSoA* )rcAlloc(num_tris * (int)sizeof(TriPlaneSoA),   RC_ALLOC_TEMP));
	if (tri_aabbs == nullptr || tri_planes == nullptr)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	for (int i = 0; i < num_tris; ++i)
	{
		const Vec3 v0(chunk.v0x[i], chunk.v0y[i], chunk.v0z[i]);
		const Vec3 v1(chunk.v1x[i], chunk.v1y[i], chunk.v1z[i]);
		const Vec3 v2(chunk.v2x[i], chunk.v2y[i], chunk.v2z[i]);
		const float nx = normals.nx[i], ny = normals.ny[i], nz = normals.nz[i];

		// Axis-aligned bounding box: component-wise min/max over the three vertices.
		// Used to cull tiles that can't possibly overlap the triangle.
		tri_aabbs[i] = {
			rcMin(rcMin(v0.x, v1.x), v2.x), rcMax(rcMax(v0.x, v1.x), v2.x),
			rcMin(rcMin(v0.y, v1.y), v2.y), rcMax(rcMax(v0.y, v1.y), v2.y),
			rcMin(rcMin(v0.z, v1.z), v2.z), rcMax(rcMax(v0.z, v1.z), v2.z),
		};

		// Reciprocal of the Y component of the triangle normal.
		// Used to solve the plane equation for Y given an (X, Z) position:
		//   Y = (d - nx*X - nz*Z) * inv_ny
		// Zero when the triangle is nearly vertical (|ny| < epsilon), in which
		// case no Y interpolation is needed (the triangle contributes no solid
		// voxels in the height direction and is skipped during voxelization).
		const float inv_ny = (rcAbs(ny) > 1e-6f) ? 1.0f / ny : 0.0f;

		// 2-D edge normals in the XZ plane, one per edge.
		// Each edge normal (aEx, aEz) points outward from the triangle interior.
		// These are the separating axes used by the SAT overlap test between the
		// triangle and each BLOCK_XZ × BLOCK_XZ voxel column cell.
		//   edge 0: v0→v1,  edge 1: v1→v2,  edge 2: v2→v0
		const float a0x = -(v1.z - v0.z), a0z = v1.x - v0.x;
		const float a1x = -(v2.z - v1.z), a1z = v2.x - v1.x;
		const float a2x = -(v0.z - v2.z), a2z = v0.x - v2.x;

		// Project the triangle vertices onto each edge normal to get the interval
		// [min, max] of the triangle along that separating axis.  Only two of the
		// three vertices are needed per edge because one vertex is always on the
		// edge itself and drops out of the min/max.
		const float p00 = a0x*v0.x + a0z*v0.z, p20 = a0x*v2.x + a0z*v2.z;
		const float p11 = a1x*v1.x + a1z*v1.z, p01 = a1x*v0.x + a1z*v0.z;
		const float p22 = a2x*v2.x + a2z*v2.z, p12 = a2x*v1.x + a2z*v1.z;

		// Y offset range of the triangle plane across one voxel cell in XZ.
		// Because the plane is tilted, the Y value at the +X corner of a cell
		// differs from the Y value at the -X corner by dy_dx = -nx*cs/ny (and
		// similarly dy_dz for the Z axis).  min_off / max_off bound the full Y
		// variation across all four corners of a cell, allowing voxelizeTriToBitBlock
		// to tighten the Y interval tested per cell.  Both are zero for a flat
		// (horizontal) triangle, or when the triangle is vertical (inv_ny == 0).
		float min_off = 0.f, max_off = 0.f;
		if (inv_ny != 0.f)
		{
			const float dy_dx = -nx * cs * inv_ny;
			const float dy_dz = -nz * cs * inv_ny;
			min_off = rcMin(rcMin(0.f, dy_dx), rcMin(dy_dz, dy_dx + dy_dz));
			max_off = rcMax(rcMax(0.f, dy_dx), rcMax(dy_dz, dy_dx + dy_dz));
		}

		tri_planes[i] = {
			v0, v1, v2,
			nx, nz,
			nx*v0.x + ny*v0.y + nz*v0.z,  // d: plane equation constant, dot(n, v0)
			inv_ny,
			a0x, a0z, a1x, a1z, a2x, a2z,
			rcMin(p00, p20), rcMax(p00, p20),  // SAT interval on edge-0 axis
			rcMin(p11, p01), rcMax(p11, p01),  // SAT interval on edge-1 axis
			rcMin(p22, p12), rcMax(p22, p12),  // SAT interval on edge-2 axis
			// Half-widths of a voxel cell projected onto each edge normal.
			// A cell's interval on axis aE is centred at dot(aE, cellCentre) with
			// half-width r = (|aEx| + |aEz|) * cs * 0.5.  SAT overlap iff the
			// triangle interval and cell interval intersect.
			(rcAbs(a0x) + rcAbs(a0z)) * cs * 0.5f,
			(rcAbs(a1x) + rcAbs(a1z)) * cs * 0.5f,
			(rcAbs(a2x) + rcAbs(a2z)) * cs * 0.5f,
			min_off, max_off,
		};
	}

	// --- Bucket triangles into tiles (CSR format) ---
	// The heightfield is divided into BLOCK_XZ × BLOCK_XZ column tiles.  Each
	// triangle is associated with every tile whose XZ footprint overlaps the
	// triangle's AABB.  Storing these associations in CSR (compressed sparse row)
	// format — a flat index array with per-tile start offsets — avoids dynamic
	// allocation per tile and gives cache-friendly sequential access in the main loop.
	const int num_tiles_x = (hf_width + BLOCK_XZ - 1) / BLOCK_XZ;
	const int num_tiles_z = (hf_height + BLOCK_XZ - 1) / BLOCK_XZ;
	const int num_tiles  = num_tiles_x * num_tiles_z;

	// Pass 1: count how many triangles reference each tile.
	rcScopedDelete<int> tile_counts((int*)rcAlloc(num_tiles * (int)sizeof(int), RC_ALLOC_TEMP));
	if (tile_counts == nullptr)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}
	memset(tile_counts, 0, num_tiles * sizeof(int));

	for (int i = 0; i < num_tris; ++i)
	{
		const TriBoundsSoA& b = tri_aabbs[i];
		// Convert the triangle AABB corners from world space to tile indices,
		// clamping to the valid tile range.
		const int tx_min = rcMax((int)((b.minX - hf_min.x) * inv_cs) / BLOCK_XZ, 0);
		const int tx_max = rcMin((int)((b.maxX - hf_min.x) * inv_cs) / BLOCK_XZ, num_tiles_x - 1);
		const int tz_min = rcMax((int)((b.minZ - hf_min.z) * inv_cs) / BLOCK_XZ, 0);
		const int tz_max = rcMin((int)((b.maxZ - hf_min.z) * inv_cs) / BLOCK_XZ, num_tiles_z - 1);
		for (int tz = tz_min; tz <= tz_max; ++tz)
			for (int tx = tx_min; tx <= tx_max; ++tx)
				++tile_counts[tz * num_tiles_x + tx];
	}

	// Prefix sum over tile_counts → per-tile start offsets (tile_starts[t] is the
	// index into tile_tri_list where tile t's triangle list begins).
	// tile_starts has num_tiles + 1 entries so tile_starts[t+1] - tile_starts[t]
	// gives the count for tile t without a special-case for the last tile.
	rcScopedDelete<int> tile_starts((int*)rcAlloc((num_tiles + 1) * (int)sizeof(int), RC_ALLOC_TEMP));
	if (tile_starts == nullptr)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}
	tile_starts[0] = 0;
	for (int t = 0; t < num_tiles; ++t)
		tile_starts[t + 1] = tile_starts[t] + tile_counts[t];

	// Total number of (triangle, tile) pairs across all triangles.
	const int total_refs = tile_starts[num_tiles];

	// Pass 2: fill tile_tri_list with triangle indices, using tile_counts as
	// running write cursors (reset to 0 and incremented as each entry is written).
	rcScopedDelete<int> tile_tri_list(total_refs > 0 ? (int*)rcAlloc(total_refs * (int)sizeof(int), RC_ALLOC_TEMP) : nullptr);
	if (total_refs > 0 && tile_tri_list == nullptr)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}
	memset(tile_counts, 0, num_tiles * sizeof(int));  // reuse as fill cursors
	for (int i = 0; i < num_tris; ++i)
	{
		const TriBoundsSoA& b = tri_aabbs[i];
		const int tx_min = rcMax((int)((b.minX - hf_min.x) * inv_cs) / BLOCK_XZ, 0);
		const int tx_max = rcMin((int)((b.maxX - hf_min.x) * inv_cs) / BLOCK_XZ, num_tiles_x - 1);
		const int tz_min = rcMax((int)((b.minZ - hf_min.z) * inv_cs) / BLOCK_XZ, 0);
		const int tz_max = rcMin((int)((b.maxZ - hf_min.z) * inv_cs) / BLOCK_XZ, num_tiles_z - 1);
		for (int tz = tz_min; tz <= tz_max; ++tz)
			for (int tx = tx_min; tx <= tx_max; ++tx)
			{
				const int t = tz * num_tiles_x + tx;
				tile_tri_list[tile_starts[t] + tile_counts[t]++] = i;
			}
	}

	// --- Allocate the bit block (reused across all tiles and Y-chunks) ---
	// The block is a BLOCK_XZ × BLOCK_XZ grid of uint64_t words.  Each word
	// covers one XZ column over BLOCK_Y (= 64) voxel rows, with bit k set when
	// voxel row (y_base + k) within that column is occupied by solid geometry.
	// A single block therefore covers a BLOCK_XZ × BLOCK_Y × BLOCK_XZ region
	// of the heightfield.  It is zeroed before each (tile, y_base, area) triple
	// and reused to avoid repeated allocations in the innermost loops.
	const int block_bytes = BLOCK_XZ * BLOCK_XZ * (int)sizeof(uint64_t);
	rcScopedDelete<uint64_t> block((uint64_t*)rcAlloc(block_bytes, RC_ALLOC_TEMP));
	if (block == nullptr)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	// --- Main loop: tile × Y-chunk × area ---
	// The three-level nesting keeps working-set sizes small:
	//   Outer (tile): selects the subset of triangles relevant to this XZ region.
	//   Middle (y_base): slices the vertical range into BLOCK_Y-voxel chunks so
	//     the bit block fits in L1/L2 cache regardless of heightfield height.
	//   Inner (area): processes one area ID at a time so that spans written for
	//     different areas are kept separate until addSpan merges touching spans
	//     of the same area with flag_merge_threshold.
	bool ok = true;
	for (int tz = 0; tz < num_tiles_z && ok; ++tz)
	{
		const int tile_z = tz * BLOCK_XZ;  // Z origin of this tile in voxel columns
		for (int tx = 0; tx < num_tiles_x && ok; ++tx)
		{
			const int tile_x = tx * BLOCK_XZ;  // X origin of this tile in voxel columns
			const int  t     = tz * num_tiles_x + tx;
			const int  n_tris = tile_starts[t + 1] - tile_starts[t];  // triangles in this tile
			const int* tris  = tile_tri_list + tile_starts[t];         // their indices
			if (n_tris == 0) continue;  // no geometry in this tile — skip

			for (int y_base = 0; y_base < H && ok; y_base += BLOCK_Y)
			{
				for (int a = 0; a < num_distinct && ok; ++a)
				{
					const uint8_t area = distinct_areas[a];

					// Clear all voxel bits for this (tile, y_base, area) triple.
					memset(block, 0, block_bytes);

					// Voxelize every triangle of this area into the bit block.
					// Triangles of other areas are skipped so their solid volume
					// doesn't bleed into this area's span list.
					for (int ii = 0; ii < n_tris; ++ii)
					{
						const int i = tris[ii];
						if (tri_area_ids[i] != area) continue;
						voxelizeTriToBitBlock(block,
						    tri_aabbs[i], tri_planes[i],
						    hf_min, hf_max, cs, inv_cs, inv_ch,
						    tile_x, tile_z, y_base);
					}

					// Convert the occupied bit runs in the block to rcSpan entries
					// and merge them into the heightfield, tagged with this area ID.
					ok = extractSpansFromBitBlock(block, heightfield,
					                              tile_x, tile_z, y_base, H,
					                              area, flag_merge_threshold);
				}
			}
		}
	}

	if (!ok)
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
	return ok;
}
