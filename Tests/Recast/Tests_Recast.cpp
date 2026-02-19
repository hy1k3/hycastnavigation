#include <cstdint>
#include <stdio.h>
#include <string.h>

#include "catch2/catch_amalgamated.hpp"

#include "Recast.h"
#include "Vec3.h"

TEST_CASE("rcSwap", "[recast]")
{
	SECTION("Swap two values")
	{
		int one = 1;
		int two = 2;
		rcSwap(one, two);
		REQUIRE(one == 2);
		REQUIRE(two == 1);
	}
}

TEST_CASE("rcMin", "[recast]")
{
	SECTION("Min returns the lowest value.")
	{
		REQUIRE(rcMin(1, 2) == 1);
		REQUIRE(rcMin(2, 1) == 1);
	}

	SECTION("Min with equal args")
	{
		REQUIRE(rcMin(1, 1) == 1);
	}
}

TEST_CASE("rcMax", "[recast]")
{
	SECTION("Max returns the greatest value.")
	{
		REQUIRE(rcMax(1, 2) == 2);
		REQUIRE(rcMax(2, 1) == 2);
	}

	SECTION("Max with equal args")
	{
		REQUIRE(rcMax(1, 1) == 1);
	}
}

TEST_CASE("rcAbs", "[recast]")
{
	SECTION("Abs returns the absolute value.")
	{
		REQUIRE(rcAbs(-1) == 1);
		REQUIRE(rcAbs(1) == 1);
		REQUIRE(rcAbs(0) == 0);
	}
}

TEST_CASE("rcSqr", "[recast]")
{
	SECTION("Sqr squares a number")
	{
		REQUIRE(rcSqr(2) == 4);
		REQUIRE(rcSqr(-4) == 16);
		REQUIRE(rcSqr(0) == 0);
	}
}

TEST_CASE("rcClamp", "[recast]")
{
	SECTION("Higher than range")
	{
		REQUIRE(rcClamp(2, 0, 1) == 1);
	}

	SECTION("Within range")
	{
		REQUIRE(rcClamp(1, 0, 2) == 1);
	}

	SECTION("Lower than range")
	{
		REQUIRE(rcClamp(0, 1, 2) == 1);
	}
}

TEST_CASE("rcSqrt", "[recast]")
{
	SECTION("Sqrt gets the sqrt of a number")
	{
		REQUIRE(rcSqrt(4) == Catch::Approx(2));
		REQUIRE(rcSqrt(81) == Catch::Approx(9));
	}
}

TEST_CASE("rcVcross", "[recast]")
{
	SECTION("Computes cross product")
	{
		Vec3 v1(3, -3, 1);
		Vec3 v2(4, 9, 2);
		Vec3 result;
		result = v1.cross(v2);
		REQUIRE(result.x == Catch::Approx(-15));
		REQUIRE(result.y == Catch::Approx(-2));
		REQUIRE(result.z == Catch::Approx(39));
	}

	SECTION("Cross product with itself is zero")
	{
		Vec3 v1(3, -3, 1);
		Vec3 result;
		result = v1.cross(v1);
		REQUIRE(result.x == Catch::Approx(0));
		REQUIRE(result.y == Catch::Approx(0));
		REQUIRE(result.z == Catch::Approx(0));
	}
}

TEST_CASE("rcVdot", "[recast]")
{
	SECTION("Dot normalized vector with itself")
	{
		Vec3 v1( 1, 0, 0 );
		float result = v1.dot(v1);
		REQUIRE(result == Catch::Approx(1));
	}

	SECTION("Dot zero vector with anything is zero")
	{
		Vec3 v1( 1, 2, 3 );
		Vec3 v2( 0, 0, 0 );

		float result = v1.dot(v2);
		REQUIRE(result == Catch::Approx(0));
	}
}

TEST_CASE("rcVmad", "[recast]")
{
	SECTION("scaled add two vectors")
	{
		Vec3 v1(1, 2, 3);
		Vec3 v2(0, 2, 4);
		Vec3 result;
		result = v1 + v2 * 2;
		REQUIRE(result.x == Catch::Approx(1));
		REQUIRE(result.y == Catch::Approx(6));
		REQUIRE(result.z == Catch::Approx(11));
	}

	SECTION("second vector is scaled, first is not")
	{
		Vec3 v1(1, 2, 3);
		Vec3 v2(5, 6, 7);
		Vec3 result;
		result = v1 + v2 * 0;
		REQUIRE(result.x == Catch::Approx(1));
		REQUIRE(result.y == Catch::Approx(2));
		REQUIRE(result.z == Catch::Approx(3));
	}
}

TEST_CASE("rcVadd", "[recast]")
{
	SECTION("add two vectors")
	{
		Vec3 v1(1, 2, 3);
		Vec3 v2(5, 6, 7);
		Vec3 result;
		result = v1 + v2;
		REQUIRE(result.x == Catch::Approx(6));
		REQUIRE(result.y == Catch::Approx(8));
		REQUIRE(result.z == Catch::Approx(10));
	}
}

TEST_CASE("rcVsub", "[recast]")
{
	SECTION("subtract two vectors")
	{
		Vec3 v1(5, 4, 3);
		Vec3 v2(1, 2, 3);
		Vec3 result;
		result = v1 - v2;
		REQUIRE(result.x == Catch::Approx(4));
		REQUIRE(result.y == Catch::Approx(2));
		REQUIRE(result.z == Catch::Approx(0));
	}
}

TEST_CASE("rcVmin", "[recast]")
{
	SECTION("selects the min component from the vectors")
	{
		Vec3 v1(5, 4, 0);
		Vec3 v2(1, 2, 9);
		v1 = vmin(v1, v2);
		REQUIRE(v1.x == Catch::Approx(1));
		REQUIRE(v1.y == Catch::Approx(2));
		REQUIRE(v1.z == Catch::Approx(0));
	}

	SECTION("v1 is min")
	{
		Vec3 v1(1, 2, 3);
		Vec3 v2(4, 5, 6);
		v1 = vmin(v1, v2);
		REQUIRE(v1.x == Catch::Approx(1));
		REQUIRE(v1.y == Catch::Approx(2));
		REQUIRE(v1.z == Catch::Approx(3));
	}

	SECTION("v2 is min")
	{
		Vec3 v1(4, 5, 6);
		Vec3 v2(1, 2, 3);
		v1 = vmin(v1, v2);
		REQUIRE(v1.x == Catch::Approx(1));
		REQUIRE(v1.y == Catch::Approx(2));
		REQUIRE(v1.z == Catch::Approx(3));
	}
}

TEST_CASE("rcVmax", "[recast]")
{
	SECTION("selects the max component from the vectors")
	{
		Vec3 v1(5, 4, 0);
		Vec3 v2(1, 2, 9);
		v1 = vmax(v1, v2);
		REQUIRE(v1.x == Catch::Approx(5));
		REQUIRE(v1.y == Catch::Approx(4));
		REQUIRE(v1.z == Catch::Approx(9));
	}

	SECTION("v2 is max")
	{
		Vec3 v1(1, 2, 3);
		Vec3 v2(4, 5, 6);
		v1 = vmax(v1, v2);
		REQUIRE(v1.x == Catch::Approx(4));
		REQUIRE(v1.y == Catch::Approx(5));
		REQUIRE(v1.z == Catch::Approx(6));
	}

	SECTION("v1 is max")
	{
		Vec3 v1(4, 5, 6);
		Vec3 v2(1, 2, 3);
		v1 = vmax(v1, v2);
		REQUIRE(v1.x == Catch::Approx(4));
		REQUIRE(v1.y == Catch::Approx(5));
		REQUIRE(v1.z == Catch::Approx(6));
	}
}

TEST_CASE("rcVcopy", "[recast]")
{
	SECTION("copies a vector into another vector")
	{
		Vec3 v1(5, 4, 0);
		Vec3 result(1, 2, 9);
		result = v1;
		REQUIRE(result.x == Catch::Approx(5));
		REQUIRE(result.y == Catch::Approx(4));
		REQUIRE(result.z == Catch::Approx(0));
		REQUIRE(v1.x == Catch::Approx(5));
		REQUIRE(v1.y == Catch::Approx(4));
		REQUIRE(v1.z == Catch::Approx(0));
	}
}

TEST_CASE("rcVdist", "[recast]")
{
	SECTION("distance between two vectors")
	{
		Vec3 v1(3, 1, 3);
		Vec3 v2(1, 3, 1);
		float result = dist(v1, v2);

		REQUIRE(result == Catch::Approx(3.4641f));
	}

	SECTION("Distance from zero is magnitude")
	{
		Vec3 v1(3, 1, 3);
		Vec3 v2(0, 0, 0);
		float distance = dist(v1, v2);
		float magnitude = rcSqrt(rcSqr(v1.x) + rcSqr(v1.y) + rcSqr(v1.z));
		REQUIRE(distance == Catch::Approx(magnitude));
	}
}

TEST_CASE("rcVdistSqr", "[recast]")
{
	SECTION("squared distance between two vectors")
	{
		Vec3 v1(3, 1, 3);
		Vec3 v2(1, 3, 1);
		float result = distSqr(v1, v2);

		REQUIRE(result == Catch::Approx(12));
	}

	SECTION("squared distance from zero is squared magnitude")
	{
		Vec3 v1(3, 1, 3);
		Vec3 v2(0, 0, 0);
		float distance = distSqr(v1, v2);
		float magnitude = rcSqr(v1.x) + rcSqr(v1.y) + rcSqr(v1.z);
		REQUIRE(distance == Catch::Approx(magnitude));
	}
}

TEST_CASE("rcVnormalize", "[recast]")
{
	SECTION("normalizing reduces magnitude to 1")
	{
		Vec3 v(3, 3, 3);
		v.normalize();
		REQUIRE(v.x == Catch::Approx(rcSqrt(1.0f / 3.0f)));
		REQUIRE(v.y == Catch::Approx(rcSqrt(1.0f / 3.0f)));
		REQUIRE(v.z == Catch::Approx(rcSqrt(1.0f / 3.0f)));
		float magnitude = rcSqrt(rcSqr(v.x) + rcSqr(v.y) + rcSqr(v.z));
		REQUIRE(magnitude == Catch::Approx(1));
	}
}

TEST_CASE("rcCalcBounds", "[recast]")
{
	SECTION("bounds of one vector")
	{
		Vec3 verts[] = { Vec3(1, 2, 3) };
		Vec3 bmin;
		Vec3 bmax;
		rcCalcBounds(verts, 1, bmin, bmax);

		REQUIRE(bmin.x == Catch::Approx(verts[0].x));
		REQUIRE(bmin.y == Catch::Approx(verts[0].y));
		REQUIRE(bmin.z == Catch::Approx(verts[0].z));

		REQUIRE(bmax.x == Catch::Approx(verts[0].x));
		REQUIRE(bmax.y == Catch::Approx(verts[0].y));
		REQUIRE(bmax.z == Catch::Approx(verts[0].z));
	}

	SECTION("bounds of more than one vector")
	{
		Vec3 verts[] = {
			Vec3(1, 2, 3),
			Vec3(0, 2, 5)
		};
		Vec3 bmin;
		Vec3 bmax;
		rcCalcBounds(verts, 2, bmin, bmax);

		REQUIRE(bmin.x == Catch::Approx(0));
		REQUIRE(bmin.y == Catch::Approx(2));
		REQUIRE(bmin.z == Catch::Approx(3));

		REQUIRE(bmax.x == Catch::Approx(1));
		REQUIRE(bmax.y == Catch::Approx(2));
		REQUIRE(bmax.z == Catch::Approx(5));
	}
}

TEST_CASE("rcCalcGridSize", "[recast]")
{
	SECTION("computes the size of an x & z axis grid")
	{
		Vec3 verts[] = {
			Vec3(1, 2, 3),
			Vec3(0, 2, 6)
		};
		Vec3 bmin;
		Vec3 bmax;
		rcCalcBounds(verts, 2, bmin, bmax);

		float cellSize = 1.5f;

		int width;
		int height;

		rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

		REQUIRE(width == 1);
		REQUIRE(height == 2);
	}
}

TEST_CASE("rcCreateHeightfield", "[recast]")
{
	SECTION("create a heightfield")
	{
		Vec3 verts[] = {
			Vec3(1, 2, 3),
			Vec3(0, 2, 6)
		};
		Vec3 bmin;
		Vec3 bmax;
		rcCalcBounds(verts, 2, bmin, bmax);

		float cellSize = 1.5f;
		float cellHeight = 2;

		int width;
		int height;

		rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

		rcHeightfield heightfield;

		bool result = rcCreateHeightfield(0, heightfield, width, height, bmin, bmax, cellSize, cellHeight);

		REQUIRE(result);

		REQUIRE(heightfield.width == width);
		REQUIRE(heightfield.height == height);

		REQUIRE(heightfield.bmin.x == Catch::Approx(bmin.x));
		REQUIRE(heightfield.bmin.y == Catch::Approx(bmin.y));
		REQUIRE(heightfield.bmin.z == Catch::Approx(bmin.z));

		REQUIRE(heightfield.bmax.x == Catch::Approx(bmax.x));
		REQUIRE(heightfield.bmax.y == Catch::Approx(bmax.y));
		REQUIRE(heightfield.bmax.z == Catch::Approx(bmax.z));

		REQUIRE(heightfield.cs == Catch::Approx(cellSize));
		REQUIRE(heightfield.ch == Catch::Approx(cellHeight));

		REQUIRE(heightfield.spans != 0);
		REQUIRE(heightfield.pools == 0);
		REQUIRE(heightfield.freelist == 0);
	}
}

TEST_CASE("rcMarkWalkableTriangles", "[recast]")
{
	rcContext* ctx = 0;
	float walkableSlopeAngle = 45;
	Vec3 verts[] = {
		Vec3(0, 0, 0),
		Vec3(1, 0, 0),
		Vec3(0, 0, -1)
	};
	uint8_t areas[] = { RC_NULL_AREA };

	TriChunk walkable_chunk;
	walkable_chunk.set(0, verts[0], verts[1], verts[2]);
	NormalChunk walkable_normals;
	rcComputeNormals(walkable_chunk, 1, walkable_normals);

	TriChunk unwalkable_chunk;
	unwalkable_chunk.set(0, verts[0], verts[2], verts[1]);
	NormalChunk unwalkable_normals;
	rcComputeNormals(unwalkable_chunk, 1, unwalkable_normals);

	SECTION("One walkable triangle")
	{
		rcMarkWalkableTriangles(ctx, walkable_normals, 1, walkableSlopeAngle, areas);
		REQUIRE(areas[0] == RC_WALKABLE_AREA);
	}

	SECTION("One non-walkable triangle")
	{
		rcMarkWalkableTriangles(ctx, unwalkable_normals, 1, walkableSlopeAngle, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}

	SECTION("Non-walkable triangle area id's are not modified")
	{
		areas[0] = 42;
		rcMarkWalkableTriangles(ctx, unwalkable_normals, 1, walkableSlopeAngle, areas);
		REQUIRE(areas[0] == 42);
	}

	SECTION("Slopes equal to the max slope are considered unwalkable.")
	{
		walkableSlopeAngle = 0;
		NormalChunk flat_normals;
		rcComputeNormals(walkable_chunk, 1, flat_normals);
		rcMarkWalkableTriangles(ctx, flat_normals, 1, walkableSlopeAngle, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}
}


TEST_CASE("rcRasterizeTriangles", "[recast]")
{
	rcContext ctx;
	Vec3 verts[] = {
		Vec3(0, 0, 0),
		Vec3(1, 0, 0),
		Vec3(0, 0, -1),
		Vec3(0, 0, 1)
	};
	int tris[] = {
		0, 1, 2,
		0, 3, 1
	};
	uint8_t areas[] = {
		1,
		2
	};
	Vec3 bmin;
	Vec3 bmax;
	rcCalcBounds(verts, 4, bmin, bmax);

	float cellSize = .5f;
	float cellHeight = .5f;

	int width;
	int height;

	rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

	rcHeightfield solid; 
	REQUIRE(rcCreateHeightfield(&ctx, solid, width, height, bmin, bmax, cellSize, cellHeight));

	int flagMergeThr = 1;

	SECTION("Rasterize some triangles")
	{
		TriChunk chunk;
		for (int i = 0; i < 2; ++i)
			chunk.set(i, verts[tris[i*3]], verts[tris[i*3+1]], verts[tris[i*3+2]]);
		NormalChunk normals;
		rcComputeNormals(chunk, 2, normals);
		REQUIRE(rcRasterizeTriangles(&ctx, chunk, normals, 2, areas, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}

	SECTION("Unsigned short overload")
	{
		uint16_t utris[] = {
			0, 1, 2,
			0, 3, 1
		};
		TriChunk chunk;
		for (int i = 0; i < 2; ++i)
			chunk.set(i, verts[utris[i*3]], verts[utris[i*3+1]], verts[utris[i*3+2]]);
		NormalChunk normals;
		rcComputeNormals(chunk, 2, normals);
		REQUIRE(rcRasterizeTriangles(&ctx, chunk, normals, 2, areas, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}

	SECTION("Triangle list overload")
	{
		Vec3 vertsList[] = {
			Vec3(0, 0, 0),
			Vec3(1, 0, 0),
			Vec3(0, 0, -1),
			Vec3(0, 0, 0),
			Vec3(0, 0, 1),
			Vec3(1, 0, 0),
		};

		TriChunk chunk;
		for (int i = 0; i < 2; ++i)
			chunk.set(i, vertsList[i*3], vertsList[i*3+1], vertsList[i*3+2]);
		NormalChunk normals;
		rcComputeNormals(chunk, 2, normals);
		REQUIRE(rcRasterizeTriangles(&ctx, chunk, normals, 2, areas, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}
}
