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

#include <string.h> // for memcpy and memset

/// Sorts the given data in-place using insertion sort.
///
/// @param	data		The data to sort
/// @param	dataLength	The number of elements in @p data
static void insertSort(uint8_t* data, const int dataLength)
{
	for (int valueIndex = 1; valueIndex < dataLength; valueIndex++)
	{
		const uint8_t value = data[valueIndex];
		int insertionIndex;
		for (insertionIndex = valueIndex - 1; insertionIndex >= 0 && data[insertionIndex] > value; insertionIndex--)
		{
			// Shift over values
			data[insertionIndex + 1] = data[insertionIndex];
		}
		
		// Insert the value in sorted order.
		data[insertionIndex + 1] = value;
	}
}

// TODO (graham): This is duplicated in the ConvexVolumeTool in RecastDemo
/// Checks if a point is contained within a polygon
///
/// @param[in]	numVerts	Number of vertices in the polygon
/// @param[in]	verts		The polygon vertices
/// @param[in]	point		The point to check
/// @returns true if the point lies within the polygon, false otherwise.
static bool pointInPoly(int numVerts, const Vec3* verts, const Vec3& point)
{
	bool inPoly = false;
	for (int i = 0, j = numVerts - 1; i < numVerts; j = i++)
	{
		const Vec3& vi = verts[i];
		const Vec3& vj = verts[j];

		if ((vi.z > point.z) == (vj.z > point.z))
		{
			continue;
		}

		if (point.x >= (vj.x - vi.x) * (point.z - vi.z) / (vj.z - vi.z) + vi.x)
		{
			continue;
		}
		inPoly = !inPoly;
	}
	return inPoly;
}

bool rcErodeWalkableArea(rcContext* context, const int erosionRadius, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context != NULL);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int& zStride = xSize; // For readability

	rcScopedTimer timer(context, RC_TIMER_ERODE_AREA);

	uint8_t* distanceToBoundary = (uint8_t*)rcAlloc(sizeof(uint8_t) * compactHeightfield.spanCount,
	                                                            RC_ALLOC_TEMP);
	if (!distanceToBoundary)
	{
		context->log(RC_LOG_ERROR, "erodeWalkableArea: Out of memory 'dist' (%d).", compactHeightfield.spanCount);
		return false;
	}
	memset(distanceToBoundary, 0xff, sizeof(uint8_t) * compactHeightfield.spanCount);
	
	// Mark boundary cells.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			for (int spanIndex = (int)cell.index, maxSpanIndex = (int)(cell.index + cell.count); spanIndex < maxSpanIndex; ++spanIndex)
			{
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					distanceToBoundary[spanIndex] = 0;
					continue;
				}
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Check that there is a non-null adjacent span in each of the 4 cardinal directions.
				int neighborCount = 0;
				for (int direction = 0; direction < 4; ++direction)
				{
					const int neighborConnection = rcGetCon(span, direction);
					if (neighborConnection == RC_NOT_CONNECTED)
					{
						break;
					}
					
					const int neighborX = x + rcGetDirOffsetX(direction);
					const int neighborZ = z + rcGetDirOffsetY(direction);
					const int neighborSpanIndex = (int)compactHeightfield.cells[neighborX + neighborZ * zStride].index + neighborConnection;
					
					if (compactHeightfield.areas[neighborSpanIndex] == RC_NULL_AREA)
					{
						break;
					}
					neighborCount++;
				}
				
				// At least one missing neighbour, so this is a boundary cell.
				if (neighborCount != 4)
				{
					distanceToBoundary[spanIndex] = 0;
				}
			}
		}
	}
	
	uint8_t newDistance;
	
	// Pass 1
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				if (rcGetCon(span, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0)
					const int aX = x + rcGetDirOffsetX(0);
					const int aY = z + rcGetDirOffsetY(0);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 0);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (uint8_t)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (-1,-1)
					if (rcGetCon(aSpan, 3) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(3);
						const int bY = aY + rcGetDirOffsetY(3);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 3);
						newDistance = (uint8_t)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
				if (rcGetCon(span, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1)
					const int aX = x + rcGetDirOffsetX(3);
					const int aY = z + rcGetDirOffsetY(3);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 3);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (uint8_t)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (1,-1)
					if (rcGetCon(aSpan, 2) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(2);
						const int bY = aY + rcGetDirOffsetY(2);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 2);
						newDistance = (uint8_t)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
			}
		}
	}

	// Pass 2
	for (int z = zSize - 1; z >= 0; --z)
	{
		for (int x = xSize - 1; x >= 0; --x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				if (rcGetCon(span, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int aX = x + rcGetDirOffsetX(2);
					const int aY = z + rcGetDirOffsetY(2);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 2);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (uint8_t)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (1,1)
					if (rcGetCon(aSpan, 1) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(1);
						const int bY = aY + rcGetDirOffsetY(1);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 1);
						newDistance = (uint8_t)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
				if (rcGetCon(span, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int aX = x + rcGetDirOffsetX(1);
					const int aY = z + rcGetDirOffsetY(1);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 1);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (uint8_t)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (-1,1)
					if (rcGetCon(aSpan, 0) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(0);
						const int bY = aY + rcGetDirOffsetY(0);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 0);
						newDistance = (uint8_t)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
			}
		}
	}

	const uint8_t minBoundaryDistance = (uint8_t)(erosionRadius * 2);
	for (int spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex)
	{
		if (distanceToBoundary[spanIndex] < minBoundaryDistance)
		{
			compactHeightfield.areas[spanIndex] = RC_NULL_AREA;
		}
	}

	rcFree(distanceToBoundary);
	
	return true;
}

bool rcMedianFilterWalkableArea(rcContext* context, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);
	
	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	rcScopedTimer timer(context, RC_TIMER_MEDIAN_AREA);

	uint8_t* areas = (uint8_t*)rcAlloc(sizeof(uint8_t) * compactHeightfield.spanCount, RC_ALLOC_TEMP);
	if (!areas)
	{
		context->log(RC_LOG_ERROR, "medianFilterWalkableArea: Out of memory 'areas' (%d).",
		             compactHeightfield.spanCount);
		return false;
	}
	memset(areas, 0xff, sizeof(uint8_t) * compactHeightfield.spanCount);

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					areas[spanIndex] = compactHeightfield.areas[spanIndex];
					continue;
				}

				uint8_t neighborAreas[9];
				for (int neighborIndex = 0; neighborIndex < 9; ++neighborIndex)
				{
					neighborAreas[neighborIndex] = compactHeightfield.areas[spanIndex];
				}

				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(span, dir) == RC_NOT_CONNECTED)
					{
						continue;
					}
					
					const int aX = x + rcGetDirOffsetX(dir);
					const int aZ = z + rcGetDirOffsetY(dir);
					const int aIndex = (int)compactHeightfield.cells[aX + aZ * zStride].index + rcGetCon(span, dir);
					if (compactHeightfield.areas[aIndex] != RC_NULL_AREA)
					{
						neighborAreas[dir * 2 + 0] = compactHeightfield.areas[aIndex];
					}

					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					const int dir2 = (dir + 1) & 0x3;
					const int neighborConnection2 = rcGetCon(aSpan, dir2);
					if (neighborConnection2 != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(dir2);
						const int bZ = aZ + rcGetDirOffsetY(dir2);
						const int bIndex = (int)compactHeightfield.cells[bX + bZ * zStride].index + neighborConnection2;
						if (compactHeightfield.areas[bIndex] != RC_NULL_AREA)
						{
							neighborAreas[dir * 2 + 1] = compactHeightfield.areas[bIndex];
						}
					}
				}
				insertSort(neighborAreas, 9);
				areas[spanIndex] = neighborAreas[4];
			}
		}
	}

	memcpy(compactHeightfield.areas, areas, sizeof(uint8_t) * compactHeightfield.spanCount);

	rcFree(areas);

	return true;
}

void rcMarkBoxArea(rcContext* context, const Vec3& boxMinBounds, const Vec3& boxMaxBounds, uint8_t areaId,
                   rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_BOX_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Find the footprint of the box area in grid cell coordinates.
	int minX = (int)((boxMinBounds.x - compactHeightfield.bmin.x) / compactHeightfield.cs);
	int minY = (int)((boxMinBounds.y - compactHeightfield.bmin.y) / compactHeightfield.ch);
	int minZ = (int)((boxMinBounds.z - compactHeightfield.bmin.z) / compactHeightfield.cs);
	int maxX = (int)((boxMaxBounds.x - compactHeightfield.bmin.x) / compactHeightfield.cs);
	int maxY = (int)((boxMaxBounds.y - compactHeightfield.bmin.y) / compactHeightfield.ch);
	int maxZ = (int)((boxMaxBounds.z - compactHeightfield.bmin.z) / compactHeightfield.cs);

	// Early-out if the box is outside the bounds of the grid.
	if (maxX < 0) { return; }
	if (minX >= xSize) { return; }
	if (maxZ < 0) { return; }
	if (minZ >= zSize) { return; }

	// Clamp relevant bound coordinates to the grid.
	if (minX < 0) { minX = 0; }
	if (maxX >= xSize) { maxX = xSize - 1; }
	if (minZ < 0) { minZ = 0; }
	if (maxZ >= zSize) { maxZ = zSize - 1; }

	// Mark relevant cells.
	for (int z = minZ; z <= maxZ; ++z)
	{
		for (int x = minX; x <= maxX; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if the span is outside the box extents.
				if ((int)span.y < minY || (int)span.y > maxY)
				{
					continue;
				}

				// Skip if the span has been removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Mark the span.
				compactHeightfield.areas[spanIndex] = areaId;
			}
		}
	}
}

void rcMarkConvexPolyArea(rcContext* context, const Vec3* verts, const int numVerts,
						  const float minY, const float maxY, uint8_t areaId,
						  rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_CONVEXPOLY_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Compute the bounding box of the polygon
	Vec3 bmin = verts[0];
	Vec3 bmax = verts[0];
	for (int i = 1; i < numVerts; ++i)
	{
		bmin = vmin(bmin, verts[i]);
		bmax = vmax(bmax, verts[i]);
	}
	bmin.y = minY;
	bmax.y = maxY;

	// Compute the grid footprint of the polygon
	int minx = (int)((bmin.x - compactHeightfield.bmin.x) / compactHeightfield.cs);
	int miny = (int)((bmin.y - compactHeightfield.bmin.y) / compactHeightfield.ch);
	int minz = (int)((bmin.z - compactHeightfield.bmin.z) / compactHeightfield.cs);
	int maxx = (int)((bmax.x - compactHeightfield.bmin.x) / compactHeightfield.cs);
	int maxy = (int)((bmax.y - compactHeightfield.bmin.y) / compactHeightfield.ch);
	int maxz = (int)((bmax.z - compactHeightfield.bmin.z) / compactHeightfield.cs);

	// Early-out if the polygon lies entirely outside the grid.
	if (maxx < 0) { return; }
    if (minx >= xSize) { return; }
    if (maxz < 0) { return; }
    if (minz >= zSize) { return; }

	// Clamp the polygon footprint to the grid
    if (minx < 0) { minx = 0; }
    if (maxx >= xSize) { maxx = xSize - 1; }
    if (minz < 0) { minz = 0; }
    if (maxz >= zSize) { maxz = zSize - 1; }

	// TODO: Optimize.
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if span is removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Skip if y extents don't overlap.
				if ((int)span.y < miny || (int)span.y > maxy)
				{
					continue;
				}

				Vec3 point(
					compactHeightfield.bmin.x + ((float)x + 0.5f) * compactHeightfield.cs,
					0,
					compactHeightfield.bmin.z + ((float)z + 0.5f) * compactHeightfield.cs
				);

				if (pointInPoly(numVerts, verts, point))
				{
					compactHeightfield.areas[spanIndex] = areaId;
				}
			}
		}
	}
}

static const float EPSILON = 1e-6f;

/// Normalizes the vector if the length is greater than zero.
/// If the magnitude is zero, the vector is unchanged.
/// @param[in,out]	v	The vector to normalize.
static void rcVsafeNormalize(Vec3& v)
{
	const float sqMag = rcSqr(v.x) + rcSqr(v.y) + rcSqr(v.z);
	if (sqMag > EPSILON)
	{
		const float inverseMag = 1.0f / rcSqrt(sqMag);
		v.x *= inverseMag;
		v.y *= inverseMag;
		v.z *= inverseMag;
	}
}

int rcOffsetPoly(const Vec3* verts, const int numVerts, const float offset, Vec3* outVerts, const int maxOutVerts)
{
	// Defines the limit at which a miter becomes a bevel.
	// Similar in behavior to https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/stroke-miterlimit
	const float MITER_LIMIT = 1.20f;

	int numOutVerts = 0;

	for (int vertIndex = 0; vertIndex < numVerts; vertIndex++)
	{
		// Grab three vertices of the polygon.
		const int vertIndexA = (vertIndex + numVerts - 1) % numVerts;
		const int vertIndexB = vertIndex;
		const int vertIndexC = (vertIndex + 1) % numVerts;
		const Vec3& vertA = verts[vertIndexA];
		const Vec3& vertB = verts[vertIndexB];
		const Vec3& vertC = verts[vertIndexC];

		// From A to B on the x/z plane
		Vec3 prevSegmentDir = vertB - vertA;
		prevSegmentDir.y = 0; // Squash onto x/z plane
		rcVsafeNormalize(prevSegmentDir);

		// From B to C on the x/z plane
		Vec3 currSegmentDir = vertC - vertB;
		currSegmentDir.y = 0; // Squash onto x/z plane
		rcVsafeNormalize(currSegmentDir);

		// The y component of the cross product of the two normalized segment directions.
		// The X and Z components of the cross product are both zero because the two
		// segment direction vectors fall within the x/z plane.
		float cross = currSegmentDir.x * prevSegmentDir.z - prevSegmentDir.x * currSegmentDir.z;

		// CCW perpendicular vector to AB.  The segment normal.
		const float prevSegmentNormX = -prevSegmentDir.z;
		const float prevSegmentNormZ = prevSegmentDir.x;

		// CCW perpendicular vector to BC.  The segment normal.
		const float currSegmentNormX = -currSegmentDir.z;
		const float currSegmentNormZ = currSegmentDir.x;

		// Average the two segment normals to get the proportional miter offset for B.
		// This isn't normalized because it's defining the distance and direction the corner will need to be
		// adjusted proportionally to the edge offsets to properly miter the adjoining edges.
		float cornerMiterX = (prevSegmentNormX + currSegmentNormX) * 0.5f;
		float cornerMiterZ = (prevSegmentNormZ + currSegmentNormZ) * 0.5f;
		const float cornerMiterSqMag = rcSqr(cornerMiterX) + rcSqr(cornerMiterZ);

		// If the magnitude of the segment normal average is less than about .69444,
		// the corner is an acute enough angle that the result should be beveled.
		const bool bevel = cornerMiterSqMag * MITER_LIMIT * MITER_LIMIT < 1.0f;

		// Scale the corner miter so it's proportional to how much the corner should be offset compared to the edges.
		if (cornerMiterSqMag > EPSILON)
		{
			const float scale = 1.0f / cornerMiterSqMag;
			cornerMiterX *= scale;
			cornerMiterZ *= scale;
		}

		if (bevel && cross < 0.0f) // If the corner is convex and an acute enough angle, generate a bevel.
		{
			if (numOutVerts + 2 > maxOutVerts)
			{
				return 0;
			}

			// Generate two bevel vertices at a distances from B proportional to the angle between the two segments.
			// Move each bevel vertex out proportional to the given offset.
			float d = (1.0f - (prevSegmentDir.x * currSegmentDir.x + prevSegmentDir.z * currSegmentDir.z)) * 0.5f;

			outVerts[numOutVerts] = Vec3(vertB.x + (-prevSegmentNormX + prevSegmentDir.x * d) * offset,
			                            vertB.y,
			                            vertB.z + (-prevSegmentNormZ + prevSegmentDir.z * d) * offset);
			numOutVerts++;

			outVerts[numOutVerts] = Vec3(vertB.x + (-currSegmentNormX - currSegmentDir.x * d) * offset,
			                            vertB.y,
			                            vertB.z + (-currSegmentNormZ - currSegmentDir.z * d) * offset);
			numOutVerts++;
		}
		else
		{
			if (numOutVerts + 1 > maxOutVerts)
			{
				return 0;
			}

			// Move B along the miter direction by the specified offset.
			outVerts[numOutVerts] = Vec3(vertB.x - cornerMiterX * offset,
			                            vertB.y,
			                            vertB.z - cornerMiterZ * offset);
			numOutVerts++;
		}
	}

	return numOutVerts;
}

void rcMarkCylinderArea(rcContext* context, const Vec3& position, const float radius, const float height,
                        uint8_t areaId, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_CYLINDER_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Compute the grid footprint of the cylinder
	int minx = (int)((position.x - radius - compactHeightfield.bmin.x) / compactHeightfield.cs);
	int miny = (int)((position.y         - compactHeightfield.bmin.y) / compactHeightfield.ch);
	int minz = (int)((position.z - radius - compactHeightfield.bmin.z) / compactHeightfield.cs);
	int maxx = (int)((position.x + radius - compactHeightfield.bmin.x) / compactHeightfield.cs);
	int maxy = (int)((position.y + height  - compactHeightfield.bmin.y) / compactHeightfield.ch);
	int maxz = (int)((position.z + radius - compactHeightfield.bmin.z) / compactHeightfield.cs);

	// Early-out if the cylinder is completely outside the grid bounds.
    if (maxx < 0) { return; }
    if (minx >= xSize) { return; }
    if (maxz < 0) { return; }
    if (minz >= zSize) { return; }

	// Clamp the cylinder bounds to the grid.
    if (minx < 0) { minx = 0; }
    if (maxx >= xSize) { maxx = xSize - 1; }
    if (minz < 0) { minz = 0; }
    if (maxz >= zSize) { maxz = zSize - 1; }

	const float radiusSq = radius * radius;

	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);

			const float cellX = compactHeightfield.bmin.x + ((float)x + 0.5f) * compactHeightfield.cs;
			const float cellZ = compactHeightfield.bmin.z + ((float)z + 0.5f) * compactHeightfield.cs;
			const float deltaX = cellX - position.x;
		const float deltaZ = cellZ - position.z;

			// Skip this column if it's too far from the center point of the cylinder.
            if (rcSqr(deltaX) + rcSqr(deltaZ) >= radiusSq)
            {
	            continue;
            }

			// Mark all overlapping spans
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if span is removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Mark if y extents overlap.
				if ((int)span.y >= miny && (int)span.y <= maxy)
				{
					compactHeightfield.areas[spanIndex] = areaId;
				}
			}
		}
	}
}
