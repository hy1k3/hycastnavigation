#include "DetourCommon.h"
#include "Vec3.h"
#include "catch2/catch_amalgamated.hpp"

TEST_CASE("dtRandomPointInConvexPoly")
{
	SECTION("Properly works when the argument 's' is 1.0f")
	{
		const Vec3 pts[] = {
			Vec3(0, 0, 0),
			Vec3(0, 0, 1),
			Vec3(1, 0, 0),
		};
		const int npts = 3;
		float areas[6];
		Vec3 out;

		out = dtRandomPointInConvexPoly(pts, npts, areas, 0.0f, 1.0f);
		REQUIRE(out.x == Catch::Approx(0));
		REQUIRE(out.y == Catch::Approx(0));
		REQUIRE(out.z == Catch::Approx(1));

		out = dtRandomPointInConvexPoly(pts, npts, areas, 0.5f, 1.0f);
		REQUIRE(out.x == Catch::Approx(1.0f / 2));
		REQUIRE(out.y == Catch::Approx(0));
		REQUIRE(out.z == Catch::Approx(1.0f / 2));

		out = dtRandomPointInConvexPoly(pts, npts, areas, 1.0f, 1.0f);
		REQUIRE(out.x == Catch::Approx(1));
		REQUIRE(out.y == Catch::Approx(0));
		REQUIRE(out.z == Catch::Approx(0));
	}
}
