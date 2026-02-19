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
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

#include <math.h>
#include <string.h>

#if defined(__SSE2__) || defined(_M_X64) || \
    (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#  define RC_USE_SSE2 1
#  include <immintrin.h>
#endif
#include <stdio.h>
#include <stdarg.h>

namespace
{
/// Allocates and constructs an object of the given type, returning a pointer.
/// @param[in]		allocLifetime	Allocation lifetime hint
template<typename T>
T* rcNew(const rcAllocHint allocLifetime)
{
	T* ptr = (T*)rcAlloc(sizeof(T), allocLifetime);
	::new(rcNewTag(), (void*)ptr) T();
	return ptr;
}

/// Destroys and frees an object allocated with rcNew.
/// @param[in]     ptr    The object pointer to delete.
template<typename T>
void rcDelete(T* ptr)
{
	if (ptr)
	{
		ptr->~T();
		rcFree((void*)ptr);
	}
}
} // anonymous namespace

float rcSqrt(float x)
{
	return sqrtf(x);
}

void rcContext::log(const rcLogCategory category, const char* format, ...)
{
	if (!m_logEnabled)
	{
		return;
	}
	static const int MSG_SIZE = 512;
	char msg[MSG_SIZE];
	va_list argList;
	va_start(argList, format);
	int len = vsnprintf(msg, MSG_SIZE, format, argList);
	if (len >= MSG_SIZE)
	{
		len = MSG_SIZE - 1;
		msg[MSG_SIZE - 1] = '\0';

		const char* errorMessage = "Log message was truncated";
		doLog(RC_LOG_ERROR, errorMessage, (int)strlen(errorMessage));
	}
	va_end(argList);
	doLog(category, msg, len);
}

void rcContext::doResetLog()
{
	// Defined out of line to fix the weak v-tables warning
}

rcHeightfield* rcAllocHeightfield()
{
	return rcNew<rcHeightfield>(RC_ALLOC_PERM);
}

void rcFreeHeightField(rcHeightfield* heightfield)
{
	rcDelete(heightfield);
}

rcHeightfield::rcHeightfield()
: width()
, height()
, bmin()
, bmax()
, cs()
, ch()
, spans()
, pools()
, freelist()
{
}

rcHeightfield::~rcHeightfield()
{
	// Delete span array.
	rcFree(spans);
	// Delete span pools.
	while (pools)
	{
		rcSpanPool* next = pools->next;
		rcFree(pools);
		pools = next;
	}
}

rcCompactHeightfield* rcAllocCompactHeightfield()
{
	return rcNew<rcCompactHeightfield>(RC_ALLOC_PERM);
}

void rcFreeCompactHeightfield(rcCompactHeightfield* compactHeightfield)
{
	rcDelete(compactHeightfield);
}

rcCompactHeightfield::rcCompactHeightfield()
: width()
, height()
, spanCount()
, walkableHeight()
, walkableClimb()
, borderSize()
, maxDistance()
, maxRegions()
, bmin()
, bmax()
, cs()
, ch()
, cells()
, spans()
, dist()
, areas()
{
}

rcCompactHeightfield::~rcCompactHeightfield()
{
	rcFree(cells);
	rcFree(spans);
	rcFree(dist);
	rcFree(areas);
}

rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet()
{
	return rcNew<rcHeightfieldLayerSet>(RC_ALLOC_PERM);
}

void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* layerSet)
{
	rcDelete(layerSet);
}

rcHeightfieldLayerSet::rcHeightfieldLayerSet()
: layers()
, nlayers()
{
}

rcHeightfieldLayerSet::~rcHeightfieldLayerSet()
{
	for (int i = 0; i < nlayers; ++i)
	{
		rcFree(layers[i].heights);
		rcFree(layers[i].areas);
		rcFree(layers[i].cons);
	}
	rcFree(layers);
}


rcContourSet* rcAllocContourSet()
{
	return rcNew<rcContourSet>(RC_ALLOC_PERM);
}

void rcFreeContourSet(rcContourSet* contourSet)
{
	rcDelete(contourSet);
}

rcContourSet::rcContourSet()
: conts()
, nconts()
, bmin()
, bmax()
, cs()
, ch()
, width()
, height()
, borderSize()
, maxError()
{
}

rcContourSet::~rcContourSet()
{
	for (int i = 0; i < nconts; ++i)
	{
		rcFree(conts[i].verts);
		rcFree(conts[i].rverts);
	}
	rcFree(conts);
}

rcPolyMesh* rcAllocPolyMesh()
{
	return rcNew<rcPolyMesh>(RC_ALLOC_PERM);
}

void rcFreePolyMesh(rcPolyMesh* polyMesh)
{
	rcDelete(polyMesh);
}

rcPolyMesh::rcPolyMesh()
: verts()
, polys()
, regs()
, flags()
, areas()
, nverts()
, npolys()
, maxpolys()
, nvp()
, bmin()
, bmax()
, cs()
, ch()
, borderSize()
, maxEdgeError()
{
}

rcPolyMesh::~rcPolyMesh()
{
	rcFree(verts);
	rcFree(polys);
	rcFree(regs);
	rcFree(flags);
	rcFree(areas);
}

rcPolyMeshDetail* rcAllocPolyMeshDetail()
{
	return rcNew<rcPolyMeshDetail>(RC_ALLOC_PERM);
}

void rcFreePolyMeshDetail(rcPolyMeshDetail* detailMesh)
{
	if (detailMesh == nullptr)
	{
		return;
	}
	rcFree(detailMesh->meshes);
	rcFree(detailMesh->verts);
	rcFree(detailMesh->tris);
	rcFree(detailMesh);
}

rcPolyMeshDetail::rcPolyMeshDetail()
: meshes()
, verts()
, tris()
, nmeshes()
, nverts()
, ntris()
{
}

void rcCalcBounds(const Vec3* verts, int numVerts, Vec3& minBounds, Vec3& maxBounds)
{
	// Calculate bounding box.
	minBounds = verts[0];
	maxBounds = verts[0];
	for (int i = 1; i < numVerts; ++i)
	{
		minBounds = vmin(minBounds, verts[i]);
		maxBounds = vmax(maxBounds, verts[i]);
	}
}

void rcCalcGridSize(const Vec3& minBounds, const Vec3& maxBounds, const float cellSize, int* sizeX, int* sizeZ)
{
	*sizeX = (int)((maxBounds.x - minBounds.x) / cellSize + 0.5f);
	*sizeZ = (int)((maxBounds.z - minBounds.z) / cellSize + 0.5f);
}

bool rcCreateHeightfield(rcContext* context, rcHeightfield& heightfield, int sizeX, int sizeZ,
                         const Vec3& minBounds, const Vec3& maxBounds,
                         float cellSize, float cellHeight)
{
	rcIgnoreUnused(context);

	heightfield.width = sizeX;
	heightfield.height = sizeZ;
	heightfield.bmin = minBounds;
	heightfield.bmax = maxBounds;
	heightfield.cs = cellSize;
	heightfield.ch = cellHeight;
	heightfield.spans = (rcSpan**)rcAlloc(sizeof(rcSpan*) * heightfield.width * heightfield.height, RC_ALLOC_PERM);
	if (!heightfield.spans)
	{
		return false;
	}
	memset(heightfield.spans, 0, sizeof(rcSpan*) * heightfield.width * heightfield.height);
	return true;
}

void rcMarkWalkableTriangles(rcContext* context, const TriChunk& chunk, const int numTris,
                             const float walkableSlopeAngle, uint8_t* triAreaIDs)
{
	rcIgnoreUnused(context);

	// walkable if ny/|n| > cos(angle), i.e. ny^2 > cos^2(angle) * |n|^2 (with ny > 0)
	const float thr = cosf(walkableSlopeAngle / 180.0f * RC_PI);
	const float thr2 = thr * thr;

	int i = 0;

#ifdef RC_USE_SSE2
	const __m128 simdThr2 = _mm_set1_ps(thr2);
	const __m128 zero     = _mm_setzero_ps();

	for (; i + 4 <= numTris; i += 4)
	{
		const __m128 ax = _mm_loadu_ps(chunk.v0x + i), ay = _mm_loadu_ps(chunk.v0y + i), az = _mm_loadu_ps(chunk.v0z + i);
		const __m128 bx = _mm_loadu_ps(chunk.v1x + i), by = _mm_loadu_ps(chunk.v1y + i), bz = _mm_loadu_ps(chunk.v1z + i);
		const __m128 cx = _mm_loadu_ps(chunk.v2x + i), cy = _mm_loadu_ps(chunk.v2y + i), cz = _mm_loadu_ps(chunk.v2z + i);

		// Edge vectors e0 = b-a, e1 = c-a
		const __m128 e0x = _mm_sub_ps(bx, ax), e0y = _mm_sub_ps(by, ay), e0z = _mm_sub_ps(bz, az);
		const __m128 e1x = _mm_sub_ps(cx, ax), e1y = _mm_sub_ps(cy, ay), e1z = _mm_sub_ps(cz, az);

		// Cross product n = e0 x e1
		const __m128 nx = _mm_sub_ps(_mm_mul_ps(e0y, e1z), _mm_mul_ps(e0z, e1y));
		const __m128 ny = _mm_sub_ps(_mm_mul_ps(e0z, e1x), _mm_mul_ps(e0x, e1z));
		const __m128 nz = _mm_sub_ps(_mm_mul_ps(e0x, e1y), _mm_mul_ps(e0y, e1x));

		const __m128 lenSq = _mm_add_ps(_mm_add_ps(_mm_mul_ps(nx, nx), _mm_mul_ps(ny, ny)), _mm_mul_ps(nz, nz));

		// walkable: ny > 0 && ny^2 > thr2 * |n|^2
		const __m128 walkable = _mm_and_ps(
			_mm_cmpgt_ps(ny, zero),
			_mm_cmpgt_ps(_mm_mul_ps(ny, ny), _mm_mul_ps(simdThr2, lenSq)));

		const int mask = _mm_movemask_ps(walkable);
		if (mask & 1) triAreaIDs[i+0] = RC_WALKABLE_AREA;
		if (mask & 2) triAreaIDs[i+1] = RC_WALKABLE_AREA;
		if (mask & 4) triAreaIDs[i+2] = RC_WALKABLE_AREA;
		if (mask & 8) triAreaIDs[i+3] = RC_WALKABLE_AREA;
	}
#endif

	for (; i < numTris; ++i)
	{
		const Vec3 a(chunk.v0x[i], chunk.v0y[i], chunk.v0z[i]);
		const Vec3 b(chunk.v1x[i], chunk.v1y[i], chunk.v1z[i]);
		const Vec3 c(chunk.v2x[i], chunk.v2y[i], chunk.v2z[i]);
		const Vec3 n = (b - a).cross(c - a);
		if (n.y > 0.0f && n.y * n.y > thr2 * n.dot(n))
			triAreaIDs[i] = RC_WALKABLE_AREA;
	}
}


int rcGetHeightFieldSpanCount(rcContext* context, const rcHeightfield& heightfield)
{
	rcIgnoreUnused(context);

	const int numCols = heightfield.width * heightfield.height;
	int spanCount = 0;
	for (int columnIndex = 0; columnIndex < numCols; ++columnIndex)
	{
		for (rcSpan* span = heightfield.spans[columnIndex]; span != nullptr; span = span->next)
		{
			if (span->area != RC_NULL_AREA)
			{
				spanCount++;
			}
		}
	}
	return spanCount;
}

bool rcBuildCompactHeightfield(rcContext* context, const int walkableHeight, const int walkableClimb,
                               const rcHeightfield& heightfield, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

	const int width = heightfield.width;
	const int height = heightfield.height;
	const int spanCount = rcGetHeightFieldSpanCount(context, heightfield);

	// Fill in header.
	compactHeightfield.width = width;
	compactHeightfield.height = height;
	compactHeightfield.spanCount = spanCount;
	compactHeightfield.walkableHeight = walkableHeight;
	compactHeightfield.walkableClimb = walkableClimb;
	compactHeightfield.maxRegions = 0;
	compactHeightfield.bmin = heightfield.bmin;
	compactHeightfield.bmax = heightfield.bmax;
	compactHeightfield.bmax.y += walkableHeight * heightfield.ch;
	compactHeightfield.cs = heightfield.cs;
	compactHeightfield.ch = heightfield.ch;
	compactHeightfield.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell) * width * height, RC_ALLOC_PERM);
	if (!compactHeightfield.cells)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", width * height);
		return false;
	}
	memset(compactHeightfield.cells, 0, sizeof(rcCompactCell) * width * height);
	compactHeightfield.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.spans)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.spans, 0, sizeof(rcCompactSpan) * spanCount);
	compactHeightfield.areas = (uint8_t*)rcAlloc(sizeof(uint8_t) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.areas)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.areas, RC_NULL_AREA, sizeof(uint8_t) * spanCount);

	const int MAX_HEIGHT = 0xffff;

	// Fill in cells and spans.
	int currentCellIndex = 0;
	const int numColumns = width * height;
	for (int columnIndex = 0; columnIndex < numColumns; ++columnIndex)
	{
		const rcSpan* span = heightfield.spans[columnIndex];
			
		// If there are no spans at this cell, just leave the data to index=0, count=0.
		if (span == nullptr)
		{
			continue;
		}
			
		rcCompactCell& cell = compactHeightfield.cells[columnIndex];
		cell.index = currentCellIndex;
		cell.count = 0;

		for (; span != nullptr; span = span->next)
		{
			if (span->area != RC_NULL_AREA)
			{
				const int bot = (int)span->smax;
				const int top = span->next ? (int)span->next->smin : MAX_HEIGHT;
				compactHeightfield.spans[currentCellIndex].y = (uint16_t)rcClamp(bot, 0, 0xffff);
				compactHeightfield.spans[currentCellIndex].h = (uint8_t)rcClamp(top - bot, 0, 0xff);
				compactHeightfield.areas[currentCellIndex] = span->area;
				currentCellIndex++;
				cell.count++;
			}
		}
	}
	
	// Find neighbour connections.
	const int MAX_LAYERS = RC_NOT_CONNECTED - 1;
	int maxLayerIndex = 0;
	for (int z = 0; z < height; ++z)
	{
		for (int x = 0; x < width; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * width];
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i)
			{
				rcCompactSpan& span = compactHeightfield.spans[i];

				for (int dir = 0; dir < 4; ++dir)
				{
					rcSetCon(span, dir, RC_NOT_CONNECTED);
					const int neighborX = x + rcGetDirOffsetX(dir);
					const int neighborZ = z + rcGetDirOffsetY(dir);
					// First check that the neighbour cell is in bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= width || neighborZ >= height)
					{
						continue;
					}

					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					const rcCompactCell& neighborCell = compactHeightfield.cells[neighborX + neighborZ * width];
					for (int k = (int)neighborCell.index, nk = (int)(neighborCell.index + neighborCell.count); k < nk; ++k)
					{
						const rcCompactSpan& neighborSpan = compactHeightfield.spans[k];
						const int bot = rcMax(span.y, neighborSpan.y);
						const int top = rcMin(span.y + span.h, neighborSpan.y + neighborSpan.h);

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if ((top - bot) >= walkableHeight && rcAbs((int)neighborSpan.y - (int)span.y) <= walkableClimb)
						{
							// Mark direction as walkable.
							const int layerIndex = k - (int)neighborCell.index;
							if (layerIndex < 0 || layerIndex > MAX_LAYERS)
							{
								maxLayerIndex = rcMax(maxLayerIndex, layerIndex);
								continue;
							}
							rcSetCon(span, dir, layerIndex);
							break;
						}
					}
				}
			}
		}
	}

	if (maxLayerIndex > MAX_LAYERS)
	{
		context->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)",
		         maxLayerIndex, MAX_LAYERS);
	}

	return true;
}
