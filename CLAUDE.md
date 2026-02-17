# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

**CMake (preferred for libraries and tests):**
```bash
mkdir build && cd build
cmake ..
cmake --build . --config Debug
```

**Premake (required for RecastDemo):**
```bash
cd RecastDemo
premake5 xcode4   # macOS
premake5 gmake    # Linux
premake5 vs2022   # Windows
```

CMake build options of note:
- `RECASTNAVIGATION_DT_POLYREF64` — use 64-bit polygon references (large worlds)
- `RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER` — virtual dispatch for `dtQueryFilter`
- `RECASTNAVIGATION_ENABLE_FAST_MATH` — faster but less accurate math

## Tests

Tests use Catch2. Build and run from the CMake build directory:
```bash
cmake --build . --target Tests
./Tests                              # all tests
./Tests "rcAddSpan"                  # single test by name
./Tests "[recast][rasterization]"    # by tag
./Tests --list-tests                 # list available tests
```

CI runs: `./Tests --verbosity high --success`

## Code Style

Formatting is enforced via `.clang-format` (LLVM-based, 4-space tabs, 128-column limit, Allman braces). Static analysis via `.clang-tidy` (all clang-analyzer checks, warnings as errors). RTTI and exceptions are disabled in production code.

## Architecture

Recast Navigation is a navmesh generation + runtime pathfinding system. The modules are independent and composable:

- **Recast** — navmesh generation from arbitrary 3D geometry. The pipeline runs: voxelization (`rcRasterizeTriangles`) → filtering → compaction → distance field → region building → contour tracing → polygon mesh → detail mesh.
- **Detour** — runtime navmesh representation, A\* pathfinding, and spatial queries. This is the only module needed at game runtime.
- **DetourCrowd** — agent movement simulation with local collision avoidance.
- **DetourTileCache** — dynamic navmesh streaming and obstacle support for large/open worlds.
- **DebugUtils** — visualization helpers for editor/debug builds; not needed at runtime.

Data flow: geometry → **Recast** (build time) → navmesh tiles → **Detour** (runtime queries) ← **DetourCrowd** / **DetourTileCache**.

Customization points: memory allocators (`rcAllocSetCustom` / `dtAllocSetCustom`) and assert handlers can be overridden per-module without touching library source.

The core libraries target C++98; the test suite uses C++20.
