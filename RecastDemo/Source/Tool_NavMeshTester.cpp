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
#include "Tool_NavMeshTester.h"

#include "Vec3.h"
#include "DetourCommon.h"
#include "DetourDebugDraw.h"
#include "imguiHelpers.h"

#include <imgui.h>

// Uncomment this to dump all the requests in stdout.
//#define DUMP_REQS

namespace
{
// Returns a random number [0..1]
float frand()
{
	return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

bool inRange(const Vec3& v1, const Vec3& v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx * dx + dz * dz) < r * r && fabsf(dy) < h;
}

// Merges the visited polygon list into the start of the path corridor after a moveAlongSurface call.
// Originally from DetourPathCorridor.cpp.
static int dtMergeCorridorStartMoved(dtPolyRef* path, const int npath, const int maxPath,
                                     const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;

	// Find furthest common polygon.
	for (int i = npath - 1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited - 1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}

	// If no intersection found just return current path.
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;

	// Concatenate the visited path so far and the remaining original path.
	const int req = nvisited - furthestVisited;
	const int orig = rcMin(furthestPath + 1, npath);
	int size = rcMax(0, npath - orig);
	if (req + size > maxPath)
		size = maxPath - req;
	if (size)
		memmove(path + req, path + orig, size * sizeof(dtPolyRef));

	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited - 1) - i];

	return req + size;
}

// This function checks if the path has a small U-turn, that is,
// a polygon further in the path is adjacent to the first polygon
// in the path. If that happens, a shortcut is taken.
// This can happen if the target (T) location is at tile boundary,
// and we're (S) approaching it parallel to the tile edge.
// The choice at the vertex can be arbitrary,
//  +---+---+
//  |:::|:::|
//  +-S-+-T-+
//  |:::|   | <-- the step can end up in here, resulting U-turn path.
//  +---+---+
int fixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery)
{
	if (npath < 3) { return npath; }

	// Get connected polygons
	static const int maxNeis = 16;
	dtPolyRef neis[maxNeis];
	int nneis = 0;

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly))) { return npath; }

	for (uint32_t k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
	{
		const dtLink* link = &tile->links[k];
		if (link->ref != 0)
		{
			if (nneis < maxNeis)
			{
				neis[nneis++] = link->ref;
			}
		}
	}

	// If any of the neighbor polygons is within the next few polygons
	// in the path, short cut to that polygon directly.
	static const int maxLookAhead = 6;
	int cut = 0;
	for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--)
	{
		for (int j = 0; j < nneis; j++)
		{
			if (path[i] == neis[j])
			{
				cut = i;
				break;
			}
		}
	}
	if (cut > 1)
	{
		int offset = cut - 1;
		npath -= offset;
		for (int i = 1; i < npath; i++)
		{
			path[i] = path[i + offset];
		}
	}

	return npath;
}

bool getSteerTarget(dtNavMeshQuery* navQuery,
	const Vec3& startPos,
	const Vec3& endPos,
	const float minTargetDist,
	const dtPolyRef* path,
	const int pathSize,
	Vec3& steerPos,
	uint8_t& steerPosFlag,
	dtPolyRef& steerPosRef,
	Vec3* outPoints = 0,
	int* outPointCount = 0)
{
	// Find steer target.
	static const int MAX_STEER_POINTS = 3;
	Vec3 steerPath[MAX_STEER_POINTS];
	uint8_t steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = 0;
	navQuery->findStraightPath(
		startPos,
		endPos,
		path,
		pathSize,
		steerPath,
		steerPathFlags,
		steerPathPolys,
		&nsteerPath,
		MAX_STEER_POINTS);

	if (!nsteerPath) { return false; }

	if (outPoints && outPointCount)
	{
		*outPointCount = nsteerPath;
		for (int i = 0; i < nsteerPath; ++i)
		{
			outPoints[i] = steerPath[i];
		}
	}

	// Find vertex far enough to steer to.
	int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)) { break; }
		if (!inRange(steerPath[ns], startPos, minTargetDist, 1000.0f)) { break; }
		ns++;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath) { return false; }

	steerPos = steerPath[ns];
	steerPos.y = startPos.y;
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];

	return true;
}

void getPolyCenter(dtNavMesh* navMesh, dtPolyRef ref, Vec3& center)
{
	center = Vec3(0, 0, 0);

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	dtStatus status = navMesh->getTileAndPolyByRef(ref, &tile, &poly);
	if (dtStatusFailed(status)) { return; }

	for (int i = 0; i < static_cast<int>(poly->vertCount); ++i)
	{
		const Vec3& v = tile->verts[poly->verts[i]];
		center += v;
	}
	const float s = 1.0f / static_cast<float>(poly->vertCount);
	center *= s;
}
}

NavMeshTesterTool::NavMeshTesterTool()
{
	filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	filter.setExcludeFlags(0);
}

void NavMeshTesterTool::init(Sample* newSample)
{
	sample = newSample;
	recalc();

	if (sample->navQuery)
	{
		// Change costs.
		filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
		filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
		filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
		filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
		filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
		filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);
	}

	neighborhoodRadius = sample->agentRadius * 20.0f;
	randomRadius = sample->agentRadius * 30.0f;
}

void NavMeshTesterTool::drawMenuUI()
{
	if (ImGui::RadioButton("Pathfind Follow", toolMode == ToolMode::PATHFIND_FOLLOW))
	{
		toolMode = ToolMode::PATHFIND_FOLLOW;
		recalc();
	}
	if (ImGui::RadioButton("Pathfind Straight", toolMode == ToolMode::PATHFIND_STRAIGHT))
	{
		toolMode = ToolMode::PATHFIND_STRAIGHT;
		recalc();
	}
	if (toolMode == ToolMode::PATHFIND_STRAIGHT)
	{
		ImGui::Indent();
		ImGui::Text("Vertices at crossings");
		if (ImGui::RadioButton("None", straightPathOptions == 0))
		{
			straightPathOptions = 0;
			recalc();
		}
		if (ImGui::RadioButton("Area", straightPathOptions == DT_STRAIGHTPATH_AREA_CROSSINGS))
		{
			straightPathOptions = DT_STRAIGHTPATH_AREA_CROSSINGS;
			recalc();
		}
		if (ImGui::RadioButton("All", straightPathOptions == DT_STRAIGHTPATH_ALL_CROSSINGS))
		{
			straightPathOptions = DT_STRAIGHTPATH_ALL_CROSSINGS;
			recalc();
		}
		ImGui::Unindent();
	}
	if (ImGui::RadioButton("Pathfind Sliced", toolMode == ToolMode::PATHFIND_SLICED))
	{
		toolMode = ToolMode::PATHFIND_SLICED;
		recalc();
	}
	if (ImGui::RadioButton("Distance to Wall", toolMode == ToolMode::DISTANCE_TO_WALL))
	{
		toolMode = ToolMode::DISTANCE_TO_WALL;
		recalc();
	}
	if (ImGui::RadioButton("Raycast", toolMode == ToolMode::RAYCAST))
	{
		toolMode = ToolMode::RAYCAST;
		recalc();
	}
	if (ImGui::RadioButton("Find Polys in Circle", toolMode == ToolMode::FIND_POLYS_IN_CIRCLE))
	{
		toolMode = ToolMode::FIND_POLYS_IN_CIRCLE;
		recalc();
	}
	if (ImGui::RadioButton("Find Polys in Shape", toolMode == ToolMode::FIND_POLYS_IN_SHAPE))
	{
		toolMode = ToolMode::FIND_POLYS_IN_SHAPE;
		recalc();
	}
	if (ImGui::RadioButton("Find Local Neighborhood", toolMode == ToolMode::FIND_LOCAL_NEIGHBORHOOD))
	{
		toolMode = ToolMode::FIND_LOCAL_NEIGHBORHOOD;
		recalc();
	}

	ImGui::SeparatorText("Generate Random Path");
	if (ImGui::Button("Set Start"))
	{
		dtStatus status = sample->navQuery->findRandomPoint(&filter, frand, &startRef, spos);
		if (dtStatusSucceed(status))
		{
			sposSet = true;
			recalc();
		}
	}

	ImGui::SameLine();

	if (ImGui::Button("Set End"))
	{
		dtStatus status = sample->navQuery->findRandomPoint(&filter, frand, &endRef, epos);
		if (dtStatusSucceed(status))
		{
			eposSet = true;
			recalc();
		}
	}

	ImGui::SeparatorText("Generate Random Points");

	if (ImGui::Button("Random Points"))
	{
		randPointsInCircle = false;
		nrandPoints = 0;
		for (int i = 0; i < MAX_RAND_POINTS; i++)
		{
			Vec3 point;
			dtPolyRef polyRef;
			dtStatus status = sample->navQuery->findRandomPoint(&filter, frand, &polyRef, point);
			if (dtStatusSucceed(status))
			{
				randPoints[nrandPoints] = point;
				nrandPoints++;
			}
		}
	}

	ImGui::BeginDisabled(!sposSet);
	if (ImGui::Button("Random Points Around Path Start") && sposSet)
	{
		nrandPoints = 0;
		randPointsInCircle = true;
		for (int i = 0; i < MAX_RAND_POINTS; i++)
		{
			Vec3 point;
			dtPolyRef polyRef;
			dtStatus status = sample->navQuery->findRandomPointAroundCircle(
				startRef,
				spos,
				randomRadius,
				&filter,
				frand,
				&polyRef,
				point);
			if (dtStatusSucceed(status))
			{
				randPoints[nrandPoints] = point;
				nrandPoints++;
			}
		}
	}
	ImGui::EndDisabled();

	if (ImGui::Button("Clear"))
	{
		nrandPoints = 0;
		randPointsInCircle = false;
	}

	ImGui::SeparatorText("Include Flags");

	ImGui::Indent();
	bool walk = (filter.getIncludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0;
	if (ImGui::Checkbox("Walk##Include", &walk))
	{
		filter.setIncludeFlags(filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	bool swim = (filter.getIncludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0;
	if (ImGui::Checkbox("Swim##Include", &swim))
	{
		filter.setIncludeFlags(filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	bool door = (filter.getIncludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0;
	if (ImGui::Checkbox("Door##Include", &door))
	{
		filter.setIncludeFlags(filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	bool jump = (filter.getIncludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0;
	if (ImGui::Checkbox("Jump##Include", &jump))
	{
		filter.setIncludeFlags(filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	ImGui::Unindent();

	ImGui::Separator();
	ImGui::Text("Exclude Flags");

	ImGui::Indent();
	bool excludeWalk = (filter.getExcludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0;
	if (ImGui::Checkbox("Walk##Exclude", &excludeWalk))
	{
		filter.setExcludeFlags(filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	bool excludeSwim = (filter.getExcludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0;
	if (ImGui::Checkbox("Swim##Exclude", &excludeSwim))
	{
		filter.setExcludeFlags(filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	bool excludeDoor = (filter.getExcludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0;
	if (ImGui::Checkbox("Door##Exclude", &excludeDoor))
	{
		filter.setExcludeFlags(filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	bool excludeJump = (filter.getExcludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0;
	if (ImGui::Checkbox("Jump##Exclude", &excludeJump))
	{
		filter.setExcludeFlags(filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	ImGui::Unindent();

	ImGui::Separator();
}

void NavMeshTesterTool::onClick(const float* /*s*/, const float* p, bool shift)
{
	if (shift)
	{
		sposSet = true;
		spos = Vec3(p);
	}
	else
	{
		eposSet = true;
		epos = Vec3(p);
	}
	recalc();
}

void NavMeshTesterTool::singleStep() {}

void NavMeshTesterTool::onToggle()
{
	// TODO: merge separate to a path iterator. Use same code in recalc() too.
	if (toolMode != ToolMode::PATHFIND_FOLLOW) { return; }

	if (!sposSet || !eposSet || !startRef || !endRef) { return; }

	static const float STEP_SIZE = 0.5f;
	static const float SLOP = 0.01f;

	if (pathIterNum == 0)
	{
		sample->navQuery->findPath(startRef, endRef, spos, epos, &filter, polys, &npolys, MAX_POLYS);
		nsmoothPath = 0;

		pathIterPolyCount = npolys;
		if (pathIterPolyCount)
		{
			memcpy(pathIterPolys, polys, sizeof(dtPolyRef) * pathIterPolyCount);
		}

		if (pathIterPolyCount)
		{
			// Iterate over the path to find smooth path on the detail mesh surface.
			sample->navQuery->closestPointOnPoly(startRef, spos, iterPos, 0);
			sample->navQuery->closestPointOnPoly(pathIterPolys[pathIterPolyCount - 1], epos, targetPos, 0);

			nsmoothPath = 0;

			smoothPath[nsmoothPath] = iterPos;
			nsmoothPath++;
		}
	}

	prevIterPos = iterPos;

	pathIterNum++;

	if (!pathIterPolyCount) { return; }

	if (nsmoothPath >= MAX_SMOOTH) { return; }

	// Move towards target a small advancement at a time until target reached or
	// when ran out of memory to store the path.

	// Find location to steer towards.
	Vec3 localSteerPos;
	uint8_t steerPosFlag;
	dtPolyRef steerPosRef;

	if (!getSteerTarget(
		sample->navQuery,
		iterPos,
		targetPos,
		SLOP,
		pathIterPolys,
		pathIterPolyCount,
		localSteerPos,
		steerPosFlag,
		steerPosRef,
		steerPoints,
		&steerPointCount))
	{
		return;
	}

	steerPos = localSteerPos;

	bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
	bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

	// Find movement delta.
	Vec3 delta = localSteerPos - iterPos;
	float len = delta.length();
	// If the steer target is end of path or off-mesh link, do not move past the location.
	if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
	{
		len = 1;
	}
	else
	{
		len = STEP_SIZE / len;
	}
	Vec3 moveTgt = iterPos + delta * len;

	// Move
	Vec3 result;
	dtPolyRef visited[16];
	int nvisited = 0;
	sample->navQuery->moveAlongSurface(pathIterPolys[0], iterPos, moveTgt, &filter, result, visited, &nvisited, 16);
	pathIterPolyCount = dtMergeCorridorStartMoved(pathIterPolys, pathIterPolyCount, MAX_POLYS, visited, nvisited);
	pathIterPolyCount = fixupShortcuts(pathIterPolys, pathIterPolyCount, sample->navQuery);

	float h = 0;
	sample->navQuery->getPolyHeight(pathIterPolys[0], result, &h);
	result.y = h;
	iterPos = result;

	// Handle end of path and off-mesh links when close enough.
	if (endOfPath && inRange(iterPos, localSteerPos, SLOP, 1.0f))
	{
		// Reached end of path.
		iterPos = targetPos;
		if (nsmoothPath < MAX_SMOOTH)
		{
			smoothPath[nsmoothPath] = iterPos;
			nsmoothPath++;
		}
		return;
	}
	else if (offMeshConnection && inRange(iterPos, localSteerPos, SLOP, 1.0f))
	{
		// Reached off-mesh connection.
		Vec3 startPos, endPos;

		// Advance the path up to and over the off-mesh connection.
		dtPolyRef prevRef = 0, polyRef = pathIterPolys[0];
		int npos = 0;
		while (npos < pathIterPolyCount && polyRef != steerPosRef)
		{
			prevRef = polyRef;
			polyRef = pathIterPolys[npos];
			npos++;
		}
		for (int i = npos; i < pathIterPolyCount; ++i)
		{
			pathIterPolys[i - npos] = pathIterPolys[i];
		}
		pathIterPolyCount -= npos;

		// Handle the connection.
		dtStatus status = sample->navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
		if (dtStatusSucceed(status))
		{
			if (nsmoothPath < MAX_SMOOTH)
			{
				smoothPath[nsmoothPath] = startPos;
				nsmoothPath++;
				// Hack to make the dotted path not visible during off-mesh connection.
				if (nsmoothPath & 1)
				{
					smoothPath[nsmoothPath] = startPos;
					nsmoothPath++;
				}
			}
			// Move position at the other side of the off-mesh link.
			iterPos = endPos;
			float eh = 0.0f;
			sample->navQuery->getPolyHeight(pathIterPolys[0], iterPos, &eh);
			iterPos.y = eh;
		}
	}

	// Store results.
	if (nsmoothPath < MAX_SMOOTH)
	{
		smoothPath[nsmoothPath] = iterPos;
		nsmoothPath++;
	}
}

void NavMeshTesterTool::update(const float /*dt*/)
{
	if (toolMode == ToolMode::PATHFIND_SLICED)
	{
		if (dtStatusInProgress(pathFindStatus))
		{
			pathFindStatus = sample->navQuery->updateSlicedFindPath(1, 0);
		}
		if (dtStatusSucceed(pathFindStatus))
		{
			sample->navQuery->finalizeSlicedFindPath(polys, &npolys, MAX_POLYS);
			nstraightPath = 0;
			if (npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				Vec3 clampedEpos = epos;
				if (polys[npolys - 1] != endRef)
				{
					sample->navQuery->closestPointOnPoly(polys[npolys - 1], clampedEpos, clampedEpos, 0);
				}

				sample->navQuery->findStraightPath(
					spos,
					clampedEpos,
					polys,
					npolys,
					straightPath,
					straightPathFlags,
					straightPathPolys,
					&nstraightPath,
					MAX_POLYS,
					DT_STRAIGHTPATH_ALL_CROSSINGS);
			}

			pathFindStatus = DT_FAILURE;
		}
	}
}

void NavMeshTesterTool::reset()
{
	startRef = 0;
	endRef = 0;
	npolys = 0;
	nstraightPath = 0;
	nsmoothPath = 0;
	hitPos = Vec3(0, 0, 0);
	hitNormal = Vec3(0, 0, 0);
	distanceToWall = 0;
}

void NavMeshTesterTool::recalc()
{
	if (!sample->navMesh) { return; }

	if (sposSet)
	{
		sample->navQuery->findNearestPoly(spos, polyPickExt, &filter, &startRef, 0);
	}
	else
	{
		startRef = 0;
	}

	if (eposSet)
	{
		sample->navQuery->findNearestPoly(epos, polyPickExt, &filter, &endRef, 0);
	}
	else
	{
		endRef = 0;
	}

	pathFindStatus = DT_FAILURE;

	if (toolMode == ToolMode::PATHFIND_FOLLOW)
	{
		pathIterNum = 0;
		if (sposSet && eposSet && startRef && endRef)
		{
#ifdef DUMP_REQS
			printf("pi  %f %f %f  %f %f %f  0x%x 0x%x\n",
				spos[0],
				spos[1],
				spos[2],
				epos[0],
				epos[1],
				epos[2],
				filter.getIncludeFlags(),
				filter.getExcludeFlags());
#endif

			sample->navQuery->findPath(startRef, endRef, spos, epos, &filter, polys, &npolys, MAX_POLYS);

			nsmoothPath = 0;

			if (npolys)
			{
				// Iterate over the path to find smooth path on the detail mesh surface.
				dtPolyRef polys[MAX_POLYS];
				memcpy(polys, this->polys, sizeof(dtPolyRef) * npolys);
				int npolys = this->npolys;

				Vec3 iterPos, targetPos;
				sample->navQuery->closestPointOnPoly(startRef, spos, iterPos, 0);
				sample->navQuery->closestPointOnPoly(polys[npolys - 1], epos, targetPos, 0);

				static const float STEP_SIZE = 0.5f;
				static const float SLOP = 0.01f;

				nsmoothPath = 0;

				smoothPath[nsmoothPath] = iterPos;
				nsmoothPath++;

				// Move towards target a small advancement at a time until target reached or
				// when ran out of memory to store the path.
				while (npolys && nsmoothPath < MAX_SMOOTH)
				{
					// Find location to steer towards.
					Vec3 steerPos;
					uint8_t steerPosFlag;
					dtPolyRef steerPosRef;

					if (!getSteerTarget(
						sample->navQuery,
						iterPos,
						targetPos,
						SLOP,
						polys,
						npolys,
						steerPos,
						steerPosFlag,
						steerPosRef))
					{
						break;
					}

					bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
					bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

					// Find movement delta.
					Vec3 delta = steerPos - iterPos;
					float len = delta.length();
					// If the steer target is end of path or off-mesh link, do not move past the location.
					if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
					{
						len = 1;
					}
					else
					{
						len = STEP_SIZE / len;
					}
					Vec3 moveTgt = iterPos + delta * len;

					// Move
					Vec3 result;
					dtPolyRef visited[16];
					int nvisited = 0;
					sample->navQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &filter, result, visited, &nvisited, 16);

					npolys = dtMergeCorridorStartMoved(polys, npolys, MAX_POLYS, visited, nvisited);
					npolys = fixupShortcuts(polys, npolys, sample->navQuery);

					float h = 0;
					sample->navQuery->getPolyHeight(polys[0], result, &h);
					result.y = h;
					iterPos = result;

					// Handle end of path and off-mesh links when close enough.
					if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached end of path.
						iterPos = targetPos;
						if (nsmoothPath < MAX_SMOOTH)
						{
							smoothPath[nsmoothPath] = iterPos;
							nsmoothPath++;
						}
						break;
					}
					else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached off-mesh connection.
						Vec3 startPos, endPos;

						// Advance the path up to and over the off-mesh connection.
						dtPolyRef prevRef = 0, polyRef = polys[0];
						int npos = 0;
						while (npos < npolys && polyRef != steerPosRef)
						{
							prevRef = polyRef;
							polyRef = polys[npos];
							npos++;
						}
						for (int i = npos; i < npolys; ++i) { polys[i - npos] = polys[i]; }
						npolys -= npos;

						// Handle the connection.
						dtStatus status = sample->navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
						if (dtStatusSucceed(status))
						{
							if (nsmoothPath < MAX_SMOOTH)
							{
								smoothPath[nsmoothPath] = startPos;
								nsmoothPath++;
								// Hack to make the dotted path not visible during off-mesh connection.
								if (nsmoothPath & 1)
								{
									smoothPath[nsmoothPath] = startPos;
									nsmoothPath++;
								}
							}
							// Move position at the other side of the off-mesh link.
							iterPos = endPos;
							float eh = 0.0f;
							sample->navQuery->getPolyHeight(polys[0], iterPos, &eh);
							iterPos.y = eh;
						}
					}

					// Store results.
					if (nsmoothPath < MAX_SMOOTH)
					{
						smoothPath[nsmoothPath] = iterPos;
						nsmoothPath++;
					}
				}
			}
		}
		else
		{
			npolys = 0;
			nsmoothPath = 0;
		}
	}
	else if (toolMode == ToolMode::PATHFIND_STRAIGHT)
	{
		if (sposSet && eposSet && startRef && endRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				spos[0],
				spos[1],
				spos[2],
				epos[0],
				epos[1],
				epos[2],
				filter.getIncludeFlags(),
				filter.getExcludeFlags());
#endif
			sample->navQuery->findPath(startRef, endRef, spos, epos, &filter, polys, &npolys, MAX_POLYS);
			nstraightPath = 0;
			if (npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				Vec3 clampedEpos = epos;
				if (polys[npolys - 1] != endRef)
				{
					sample->navQuery->closestPointOnPoly(polys[npolys - 1], clampedEpos, clampedEpos, 0);
				}

				sample->navQuery->findStraightPath(
					spos,
					clampedEpos,
					polys,
					npolys,
					straightPath,
					straightPathFlags,
					straightPathPolys,
					&nstraightPath,
					MAX_POLYS,
					straightPathOptions);
			}
		}
		else
		{
			npolys = 0;
			nstraightPath = 0;
		}
	}
	else if (toolMode == ToolMode::PATHFIND_SLICED)
	{
		if (sposSet && eposSet && startRef && endRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				spos[0],
				spos[1],
				spos[2],
				epos[0],
				epos[1],
				epos[2],
				filter.getIncludeFlags(),
				filter.getExcludeFlags());
#endif
			npolys = 0;
			nstraightPath = 0;

			pathFindStatus = sample->navQuery->initSlicedFindPath(
				startRef,
				endRef,
				spos,
				epos,
				&filter,
				DT_FINDPATH_ANY_ANGLE);
		}
		else
		{
			npolys = 0;
			nstraightPath = 0;
		}
	}
	else if (toolMode == ToolMode::RAYCAST)
	{
		nstraightPath = 0;
		if (sposSet && eposSet && startRef)
		{
#ifdef DUMP_REQS
			printf("rc  %f %f %f  %f %f %f  0x%x 0x%x\n",
				spos[0],
				spos[1],
				spos[2],
				epos[0],
				epos[1],
				epos[2],
				filter.getIncludeFlags(),
				filter.getExcludeFlags());
#endif
			float t = 0;
			npolys = 0;
			nstraightPath = 2;
			straightPath[0] = spos;
			sample->navQuery->raycast(startRef, spos, epos, &filter, &t, hitNormal, polys, &npolys, MAX_POLYS);
			if (t > 1)
			{
				// No hit
				hitPos = epos;
				hitResult = false;
			}
			else
			{
				// Hit
				hitPos = lerp(spos, epos, t);
				hitResult = true;
			}
			// Adjust height.
			if (npolys > 0)
			{
				float h = 0;
				sample->navQuery->getPolyHeight(polys[npolys - 1], hitPos, &h);
				hitPos.y = h;
			}
			straightPath[1] = hitPos;
		}
	}
	else if (toolMode == ToolMode::DISTANCE_TO_WALL)
	{
		distanceToWall = 0;
		if (sposSet && startRef)
		{
#ifdef DUMP_REQS
			printf("dw  %f %f %f  %f  0x%x 0x%x\n",
				spos[0],
				spos[1],
				spos[2],
				100.0f,
				filter.getIncludeFlags(),
				filter.getExcludeFlags());
#endif
			distanceToWall = 0.0f;
			sample->navQuery->findDistanceToWall(startRef, spos, 100.0f, &filter, &distanceToWall, hitPos, hitNormal);
		}
	}
	else if (toolMode == ToolMode::FIND_POLYS_IN_CIRCLE)
	{
		if (sposSet && startRef && eposSet)
		{
			const float dx = epos[0] - spos[0];
			const float dz = epos[2] - spos[2];
			float dist = sqrtf(dx * dx + dz * dz);
#ifdef DUMP_REQS
			printf("fpc  %f %f %f  %f  0x%x 0x%x\n",
				spos[0],
				spos[1],
				spos[2],
				dist,
				filter.getIncludeFlags(),
				filter.getExcludeFlags());
#endif
			sample->navQuery->findPolysAroundCircle(startRef, spos, dist, &filter, polys, parent, 0, &npolys, MAX_POLYS);
		}
	}
	else if (toolMode == ToolMode::FIND_POLYS_IN_SHAPE)
	{
		if (sposSet && startRef && eposSet)
		{
			const float nx = (epos.z - spos.z) * 0.25f;
			const float nz = -(epos.x - spos.x) * 0.25f;
			const float agentHeight = sample ? sample->agentHeight : 0;

			queryPoly[0] = Vec3(spos.x + nx * 1.2f, spos.y + agentHeight / 2, spos.z + nz * 1.2f);
			queryPoly[1] = Vec3(spos.x - nx * 1.3f, spos.y + agentHeight / 2, spos.z - nz * 1.3f);
			queryPoly[2] = Vec3(epos.x - nx * 0.8f, epos.y + agentHeight / 2, epos.z - nz * 0.8f);
			queryPoly[3] = Vec3(epos.x + nx, epos.y + agentHeight / 2, epos.z + nz);

#ifdef DUMP_REQS
			printf("fpp  %f %f %f  %f %f %f  %f %f %f  %f %f %f  0x%x 0x%x\n",
				queryPoly[0].x, queryPoly[0].y, queryPoly[0].z,
				queryPoly[1].x, queryPoly[1].y, queryPoly[1].z,
				queryPoly[2].x, queryPoly[2].y, queryPoly[2].z,
				queryPoly[3].x, queryPoly[3].y, queryPoly[3].z,
				filter.getIncludeFlags(),
				filter.getExcludeFlags());
#endif
			sample->navQuery->findPolysAroundShape(startRef, queryPoly, 4, &filter, polys, parent, 0, &npolys, MAX_POLYS);
		}
	}
	else if (toolMode == ToolMode::FIND_LOCAL_NEIGHBORHOOD)
	{
		if (sposSet && startRef)
		{
#ifdef DUMP_REQS
			printf("fln  %f %f %f  %f  0x%x 0x%x\n",
				spos[0],
				spos[1],
				spos[2],
				neighborhoodRadius,
				filter.getIncludeFlags(),
				filter.getExcludeFlags());
#endif
			sample->navQuery->findLocalNeighbourhood(
				startRef, spos, neighborhoodRadius, &filter, polys, parent, &npolys, MAX_POLYS);
		}
	}
}

void NavMeshTesterTool::render()
{
	duDebugDraw& dd = sample->debugDraw;

	static const uint32_t startCol = duRGBA(128, 25, 0, 192);
	static const uint32_t endCol = duRGBA(51, 102, 0, 129);
	static const uint32_t pathCol = duRGBA(0, 0, 0, 64);

	const float agentRadius = sample->agentRadius;
	const float agentHeight = sample->agentHeight;
	const float agentClimb = sample->agentMaxClimb;

	dd.depthMask(false);
	if (sposSet)
	{
		drawAgent(spos, agentRadius, agentHeight, agentClimb, startCol);
	}
	if (eposSet)
	{
		drawAgent(epos, agentRadius, agentHeight, agentClimb, endCol);
	}
	dd.depthMask(true);

	if (!sample->navMesh)
	{
		return;
	}

	switch (toolMode)
	{
	case ToolMode::PATHFIND_FOLLOW:
	{
		duDebugDrawNavMeshPoly(&dd, *sample->navMesh, startRef, startCol);
		duDebugDrawNavMeshPoly(&dd, *sample->navMesh, endRef, endCol);

		if (npolys)
		{
			for (int i = 0; i < npolys; ++i)
			{
				if (polys[i] == startRef || polys[i] == endRef)
				{
					continue;
				}
				duDebugDrawNavMeshPoly(&dd, *sample->navMesh, polys[i], pathCol);
			}
		}

		if (nsmoothPath)
		{
			dd.depthMask(false);
			const uint32_t spathCol = duRGBA(0, 0, 0, 220);
			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int i = 0; i < nsmoothPath; ++i)
			{
				dd.vertex(smoothPath[i].x, smoothPath[i].y + 0.1f, smoothPath[i].z, spathCol);
			}
			dd.end();
			dd.depthMask(true);
		}

		if (pathIterNum)
		{
			duDebugDrawNavMeshPoly(&dd, *sample->navMesh, pathIterPolys[0], duRGBA(255, 255, 255, 128));

			dd.depthMask(false);
			dd.begin(DU_DRAW_LINES, 1.0f);

			const uint32_t prevCol = duRGBA(255, 192, 0, 220);
			const uint32_t curCol = duRGBA(255, 255, 255, 220);
			const uint32_t steerCol = duRGBA(0, 192, 255, 220);

			dd.vertex(prevIterPos.x, prevIterPos.y - 0.3f, prevIterPos.z, prevCol);
			dd.vertex(prevIterPos.x, prevIterPos.y + 0.3f, prevIterPos.z, prevCol);

			dd.vertex(iterPos.x, iterPos.y - 0.3f, iterPos.z, curCol);
			dd.vertex(iterPos.x, iterPos.y + 0.3f, iterPos.z, curCol);

			dd.vertex(prevIterPos.x, prevIterPos.y + 0.3f, prevIterPos.z, prevCol);
			dd.vertex(iterPos.x, iterPos.y + 0.3f, iterPos.z, prevCol);

			dd.vertex(prevIterPos.x, prevIterPos.y + 0.3f, prevIterPos.z, steerCol);
			dd.vertex(steerPos.x, steerPos.y + 0.3f, steerPos.z, steerCol);

			for (int i = 0; i < steerPointCount - 1; ++i)
			{
				dd.vertex(steerPoints[i].x, steerPoints[i].y + 0.2f, steerPoints[i].z, duDarkenCol(steerCol));
				dd.vertex(
					steerPoints[i + 1].x,
					steerPoints[i + 1].y + 0.2f,
					steerPoints[i + 1].z,
					duDarkenCol(steerCol));
			}

			dd.end();
			dd.depthMask(true);
		}
	}
	break;
	case ToolMode::PATHFIND_STRAIGHT:
	case ToolMode::PATHFIND_SLICED:
	{
		duDebugDrawNavMeshPoly(&dd, *sample->navMesh, startRef, startCol);
		duDebugDrawNavMeshPoly(&dd, *sample->navMesh, endRef, endCol);

		if (npolys)
		{
			for (int i = 0; i < npolys; ++i)
			{
				if (polys[i] == startRef || polys[i] == endRef)
				{
					continue;
				}
				duDebugDrawNavMeshPoly(&dd, *sample->navMesh, polys[i], pathCol);
			}
		}

		if (nstraightPath)
		{
			dd.depthMask(false);
			const uint32_t spathCol = duRGBA(64, 16, 0, 220);
			const uint32_t offMeshCol = duRGBA(128, 96, 0, 220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < nstraightPath - 1; ++i)
			{
				uint32_t col;
				if (straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
				{
					col = offMeshCol;
				}
				else
				{
					col = spathCol;
				}

				dd.vertex(straightPath[i].x, straightPath[i].y + 0.4f, straightPath[i].z, col);
				dd.vertex(straightPath[i + 1].x, straightPath[i + 1].y + 0.4f, straightPath[i + 1].z, col);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 6.0f);
			for (int i = 0; i < nstraightPath; ++i)
			{
				uint32_t col;
				if (straightPathFlags[i] & DT_STRAIGHTPATH_START)
				{
					col = startCol;
				}
				else if (straightPathFlags[i] & DT_STRAIGHTPATH_END)
				{
					col = endCol;
				}
				else if (straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
				{
					col = offMeshCol;
				}
				else
				{
					col = spathCol;
				}
				dd.vertex(straightPath[i].x, straightPath[i].y + 0.4f, straightPath[i].z, col);
			}
			dd.end();
			dd.depthMask(true);
		}
	}
	break;
	case ToolMode::RAYCAST:
	{
		duDebugDrawNavMeshPoly(&dd, *sample->navMesh, startRef, startCol);

		if (nstraightPath)
		{
			for (int i = 1; i < npolys; ++i)
			{
				duDebugDrawNavMeshPoly(&dd, *sample->navMesh, polys[i], pathCol);
			}

			dd.depthMask(false);
			const uint32_t spathCol = hitResult ? duRGBA(64, 16, 0, 220) : duRGBA(240, 240, 240, 220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < nstraightPath - 1; ++i)
			{
				dd.vertex(straightPath[i].x, straightPath[i].y + 0.4f, straightPath[i].z, spathCol);
				dd.vertex(
					straightPath[i + 1].x,
					straightPath[i + 1].y + 0.4f,
					straightPath[i + 1].z,
					spathCol);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 4.0f);
			for (int i = 0; i < nstraightPath; ++i)
			{
				dd.vertex(straightPath[i].x, straightPath[i].y + 0.4f, straightPath[i].z, spathCol);
			}
			dd.end();

			if (hitResult)
			{
				const uint32_t hitCol = duRGBA(0, 0, 0, 128);
				dd.begin(DU_DRAW_LINES, 2.0f);
				dd.vertex(hitPos[0], hitPos[1] + 0.4f, hitPos[2], hitCol);
				dd.vertex(
					hitPos[0] + hitNormal[0] * agentRadius,
					hitPos[1] + 0.4f + hitNormal[1] * agentRadius,
					hitPos[2] + hitNormal[2] * agentRadius,
					hitCol);
				dd.end();
			}
			dd.depthMask(true);
		}
	}
	break;
	case ToolMode::DISTANCE_TO_WALL:
	{
		duDebugDrawNavMeshPoly(&dd, *sample->navMesh, startRef, startCol);
		dd.depthMask(false);
		duDebugDrawCircle(&dd, spos[0], spos[1] + agentHeight / 2, spos[2], distanceToWall, duRGBA(64, 16, 0, 220), 2.0f);
		dd.begin(DU_DRAW_LINES, 3.0f);
		dd.vertex(hitPos[0], hitPos[1] + 0.02f, hitPos[2], duRGBA(0, 0, 0, 192));
		dd.vertex(hitPos[0], hitPos[1] + agentHeight, hitPos[2], duRGBA(0, 0, 0, 192));
		dd.end();
		dd.depthMask(true);
	}
	break;
	case ToolMode::FIND_POLYS_IN_CIRCLE:
	{
		for (int i = 0; i < npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *sample->navMesh, polys[i], pathCol);
			dd.depthMask(false);
			if (parent[i])
			{
				Vec3 p0, p1;
				dd.depthMask(false);
				getPolyCenter(sample->navMesh, parent[i], p0);
				getPolyCenter(sample->navMesh, polys[i], p1);
				duDebugDrawArc(&dd, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, 0.25f, 0.0f, 0.4f, duRGBA(0, 0, 0, 128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}

		if (sposSet && eposSet)
		{
			dd.depthMask(false);
			const float dx = epos[0] - spos[0];
			const float dz = epos[2] - spos[2];
			const float dist = sqrtf(dx * dx + dz * dz);
			duDebugDrawCircle(&dd, spos[0], spos[1] + agentHeight / 2, spos[2], dist, duRGBA(64, 16, 0, 220), 2.0f);
			dd.depthMask(true);
		}
	}
	break;
	case ToolMode::FIND_POLYS_IN_SHAPE:
	{
		for (int i = 0; i < npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *sample->navMesh, polys[i], pathCol);
			dd.depthMask(false);
			if (parent[i])
			{
				Vec3 p0, p1;
				dd.depthMask(false);
				getPolyCenter(sample->navMesh, parent[i], p0);
				getPolyCenter(sample->navMesh, polys[i], p1);
				duDebugDrawArc(&dd, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, 0.25f, 0.0f, 0.4f, duRGBA(0, 0, 0, 128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}

		if (sposSet && eposSet)
		{
			dd.depthMask(false);
			const uint32_t col = duRGBA(64, 16, 0, 220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0, j = 3; i < 4; j = i++)
			{
				dd.vertex(queryPoly[j].ptr(), col);
				dd.vertex(queryPoly[i].ptr(), col);
			}
			dd.end();
			dd.depthMask(true);
		}
	}
	break;
	case ToolMode::FIND_LOCAL_NEIGHBORHOOD:
	{
		for (int i = 0; i < npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *sample->navMesh, polys[i], pathCol);
			dd.depthMask(false);
			if (parent[i])
			{
				Vec3 p0, p1;
				dd.depthMask(false);
				getPolyCenter(sample->navMesh, parent[i], p0);
				getPolyCenter(sample->navMesh, polys[i], p1);
				duDebugDrawArc(&dd, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, 0.25f, 0.0f, 0.4f, duRGBA(0, 0, 0, 128), 2.0f);
				dd.depthMask(true);
			}

			static const int MAX_SEGS = DT_VERTS_PER_POLYGON * 4;
			Vec3 segs[MAX_SEGS * 2];
			dtPolyRef refs[MAX_SEGS];
			memset(refs, 0, sizeof(dtPolyRef) * MAX_SEGS);
			int nsegs = 0;
			sample->navQuery->getPolyWallSegments(polys[i], &filter, segs, refs, &nsegs, MAX_SEGS);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int j = 0; j < nsegs; ++j)
			{
				const Vec3& segStart = segs[j * 2];
				const Vec3& segEnd = segs[j * 2 + 1];

				// Skip too distant segments.
				float tseg;
				float distSqr = dtDistancePtSegSqr2D(spos, segStart, segEnd, tseg);
				if (distSqr > dtSqr(neighborhoodRadius))
				{
					continue;
				}

				Vec3 delta = segEnd - segStart;
				Vec3 p0 = segStart + delta * 0.5f;
				Vec3 norm(delta.z, 0, -delta.x);
				norm.normalize();
				Vec3 p1 = p0 + norm * agentRadius * 0.5f;

				// Skip backfacing segments.
				if (refs[j])
				{
					uint32_t col = duRGBA(255, 255, 255, 32);
					dd.vertex(segStart.x, segStart.y + agentClimb, segStart.z, col);
					dd.vertex(segEnd.x, segEnd.y + agentClimb, segEnd.z, col);
				}
				else
				{
					uint32_t col = duRGBA(192, 32, 16, 192);
					if (dtTriArea2D(spos, segStart, segEnd) < 0.0f)
					{
						col = duRGBA(96, 32, 16, 192);
					}

					dd.vertex(p0.x, p0.y + agentClimb, p0.z, col);
					dd.vertex(p1.x, p1.y + agentClimb, p1.z, col);

					dd.vertex(segStart.x, segStart.y + agentClimb, segStart.z, col);
					dd.vertex(segEnd.x, segEnd.y + agentClimb, segEnd.z, col);
				}
			}
			dd.end();

			dd.depthMask(true);
		}

		if (sposSet)
		{
			dd.depthMask(false);
			duDebugDrawCircle(
				&dd,
				spos[0],
				spos[1] + agentHeight / 2,
				spos[2],
				neighborhoodRadius,
				duRGBA(64, 16, 0, 220),
				2.0f);
			dd.depthMask(true);
		}
	}
	break;
	}

	if (nrandPoints > 0)
	{
		dd.begin(DU_DRAW_POINTS, 6.0f);
		for (int i = 0; i < nrandPoints; i++)
		{
				dd.vertex(randPoints[i].x, randPoints[i].y + 0.1f, randPoints[i].z, duRGBA(220, 32, 16, 192));
		}
		dd.end();

		if (randPointsInCircle && sposSet)
		{
			duDebugDrawCircle(
				&dd,
				spos[0],
				spos[1] + agentHeight / 2,
				spos[2],
				randomRadius,
				duRGBA(64, 16, 0, 220),
				2.0f);
		}
	}
}

void NavMeshTesterTool::drawOverlayUI()
{
	if (sposSet)
	{
		DrawWorldspaceText(spos[0], spos[1], spos[2], IM_COL32(0, 0, 0, 220), "Start", true);
	}

	if (eposSet)
	{
		DrawWorldspaceText(epos[0], epos[1], epos[2], IM_COL32(0, 0, 0, 220), "End", true);
	}

	// Tool help
	DrawScreenspaceText(280, 40, IM_COL32(255, 255, 255, 192), "LMB+SHIFT: Set start location  LMB: Set end location");
}

void NavMeshTesterTool::drawAgent(const Vec3& pos, float r, float h, float c, const uint32_t col) const
{
	duDebugDraw& draw = sample->debugDraw;

	draw.depthMask(false);

	// Agent dimensions.
	duDebugDrawCylinderWire(&draw, pos[0] - r, pos[1] + 0.02f, pos[2] - r, pos[0] + r, pos[1] + h, pos[2] + r, col, 2.0f);

	duDebugDrawCircle(&draw, pos[0], pos[1] + c, pos[2], r, duRGBA(0, 0, 0, 64), 1.0f);

	const uint32_t color = duRGBA(0, 0, 0, 196);
	draw.begin(DU_DRAW_LINES);
	draw.vertex(pos[0], pos[1] - c, pos[2], color);
	draw.vertex(pos[0], pos[1] + c, pos[2], color);
	draw.vertex(pos[0] - r / 2, pos[1] + 0.02f, pos[2], color);
	draw.vertex(pos[0] + r / 2, pos[1] + 0.02f, pos[2], color);
	draw.vertex(pos[0], pos[1] + 0.02f, pos[2] - r / 2, color);
	draw.vertex(pos[0], pos[1] + 0.02f, pos[2] + r / 2, color);
	draw.end();

	draw.depthMask(true);
}
