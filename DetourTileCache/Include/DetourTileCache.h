#pragma once

#include <cstdint>

#include "DetourStatus.h"
#include "Vec3.h"

typedef uint32_t dtObstacleRef;
typedef uint32_t dtCompressedTileRef;

/// Flags for addTile
enum dtCompressedTileFlags
{
	DT_COMPRESSEDTILE_FREE_DATA = 0x01	///< Navmesh owns the tile memory and should free it.
};

struct dtCompressedTile
{
	uint32_t salt;						///< Counter describing modifications to the tile.
	struct dtTileCacheLayerHeader* header;
	uint8_t* compressed;
	int compressedSize;
	uint8_t* data;
	int dataSize;
	uint32_t flags;
	dtCompressedTile* next;
};

enum ObstacleState
{
	DT_OBSTACLE_EMPTY,
	DT_OBSTACLE_PROCESSING,
	DT_OBSTACLE_PROCESSED,
	DT_OBSTACLE_REMOVING
};

enum ObstacleType
{
	DT_OBSTACLE_CYLINDER,
	DT_OBSTACLE_BOX, // AABB
	DT_OBSTACLE_ORIENTED_BOX // OBB
};

struct dtObstacleCylinder
{
	Vec3 pos;
	float radius;
	float height;
};

struct dtObstacleBox
{
	Vec3 bmin;
	Vec3 bmax;
};

struct dtObstacleOrientedBox
{
	Vec3 center;
	Vec3 halfExtents;
	float rotAux[ 2 ]; //{ cos(0.5f*angle)*sin(-0.5f*angle); cos(0.5f*angle)*cos(0.5f*angle) - 0.5 }
};

static const int DT_MAX_TOUCHED_TILES = 8;
struct dtTileCacheObstacle
{
	union
	{
		dtObstacleCylinder cylinder;
		dtObstacleBox box;
		dtObstacleOrientedBox orientedBox;
	};

	dtCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
	dtCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
	uint16_t salt;
	uint8_t type;
	uint8_t state;
	uint8_t ntouched;
	uint8_t npending;
	dtTileCacheObstacle* next;
};

struct dtTileCacheParams
{
	Vec3 orig;
	float cs, ch;
	int width, height;
	float walkableHeight;
	float walkableRadius;
	float walkableClimb;
	float maxSimplificationError;
	int maxTiles;
	int maxObstacles;
};

struct dtTileCacheMeshProcess
{
	virtual ~dtTileCacheMeshProcess();
	virtual void process(struct dtNavMeshCreateParams* params, uint8_t* polyAreas, uint16_t* polyFlags) = 0;
};

class dtTileCache
{
public:
	dtTileCache();
	~dtTileCache();
	
	struct dtTileCacheAlloc* getAlloc() { return m_talloc; }
	struct dtTileCacheCompressor* getCompressor() { return m_tcomp; }
	const dtTileCacheParams* getParams() const { return &m_params; }
	
	inline int getTileCount() const { return m_params.maxTiles; }
	inline const dtCompressedTile* getTile(const int i) const { return &m_tiles[i]; }
	
	inline int getObstacleCount() const { return m_params.maxObstacles; }
	inline const dtTileCacheObstacle* getObstacle(const int i) const { return &m_obstacles[i]; }
	
	const dtTileCacheObstacle* getObstacleByRef(dtObstacleRef ref);
	
	dtObstacleRef getObstacleRef(const dtTileCacheObstacle* obmin) const;
	
	dtStatus init(const dtTileCacheParams* params,
				  struct dtTileCacheAlloc* talloc,
				  struct dtTileCacheCompressor* tcomp,
				  struct dtTileCacheMeshProcess* tmproc);
	
	int getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const ;
	
	dtCompressedTile* getTileAt(const int tx, const int ty, const int tlayer);
	dtCompressedTileRef getTileRef(const dtCompressedTile* tile) const;
	const dtCompressedTile* getTileByRef(dtCompressedTileRef ref) const;
	
	dtStatus addTile(uint8_t* data, const int dataSize, uint8_t flags, dtCompressedTileRef* result);
	
	dtStatus removeTile(dtCompressedTileRef ref, uint8_t** data, int* dataSize);
	
	// Cylinder obstacle.
	dtStatus addObstacle(const Vec3& pos, const float radius, const float height, dtObstacleRef* result);

	// Aabb obstacle.
	dtStatus addBoxObstacle(const Vec3& bmin, const Vec3& bmax, dtObstacleRef* result);

	// Box obstacle: can be rotated in Y.
	dtStatus addBoxObstacle(const Vec3& center, const Vec3& halfExtents, const float yRadians, dtObstacleRef* result);

	dtStatus removeObstacle(const dtObstacleRef ref);

	dtStatus queryTiles(const Vec3& bmin, const Vec3& bmax,
						dtCompressedTileRef* results, int* resultCount, const int maxResults) const;
	
	/// Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
	///  @param[in]		dt			The time step size. Currently not used.
	///  @param[in]		navmesh		The mesh to affect when rebuilding tiles.
	///  @param[out]	upToDate	Whether the tile cache is fully up to date with obstacle requests and tile rebuilds.
	///  							If the tile cache is up to date another (immediate) call to update will have no effect;
	///  							otherwise another call will continue processing obstacle requests and tile rebuilds.
	dtStatus update(const float dt, class dtNavMesh* navmesh, bool* upToDate = 0);
	
	dtStatus buildNavMeshTilesAt(const int tx, const int ty, class dtNavMesh* navmesh);
	
	dtStatus buildNavMeshTile(const dtCompressedTileRef ref, class dtNavMesh* navmesh);
	
	void calcTightTileBounds(const struct dtTileCacheLayerHeader* header, Vec3& bmin, Vec3& bmax) const;

	void getObstacleBounds(const struct dtTileCacheObstacle* ob, Vec3& bmin, Vec3& bmax) const;
	

	/// Encodes a tile id.
	inline dtCompressedTileRef encodeTileId(uint32_t salt, uint32_t it) const
	{
		return ((dtCompressedTileRef)salt << m_tileBits) | (dtCompressedTileRef)it;
	}
	
	/// Decodes a tile salt.
	inline uint32_t decodeTileIdSalt(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef saltMask = ((dtCompressedTileRef)1<<m_saltBits)-1;
		return (uint32_t)((ref >> m_tileBits) & saltMask);
	}
	
	/// Decodes a tile id.
	inline uint32_t decodeTileIdTile(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef tileMask = ((dtCompressedTileRef)1<<m_tileBits)-1;
		return (uint32_t)(ref & tileMask);
	}

	/// Encodes an obstacle id.
	inline dtObstacleRef encodeObstacleId(uint32_t salt, uint32_t it) const
	{
		return ((dtObstacleRef)salt << 16) | (dtObstacleRef)it;
	}
	
	/// Decodes an obstacle salt.
	inline uint32_t decodeObstacleIdSalt(dtObstacleRef ref) const
	{
		const dtObstacleRef saltMask = ((dtObstacleRef)1<<16)-1;
		return (uint32_t)((ref >> 16) & saltMask);
	}
	
	/// Decodes an obstacle id.
	inline uint32_t decodeObstacleIdObstacle(dtObstacleRef ref) const
	{
		const dtObstacleRef tileMask = ((dtObstacleRef)1<<16)-1;
		return (uint32_t)(ref & tileMask);
	}
	
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtTileCache(const dtTileCache&);
	dtTileCache& operator=(const dtTileCache&);

	enum ObstacleRequestAction
	{
		REQUEST_ADD,
		REQUEST_REMOVE
	};
	
	struct ObstacleRequest
	{
		int action;
		dtObstacleRef ref;
	};
	
	int m_tileLutSize;						///< Tile hash lookup size (must be pot).
	int m_tileLutMask;						///< Tile hash lookup mask.
	
	dtCompressedTile** m_posLookup;			///< Tile hash lookup.
	dtCompressedTile* m_nextFreeTile;		///< Freelist of tiles.
	dtCompressedTile* m_tiles;				///< List of tiles.
	
	uint32_t m_saltBits;				///< Number of salt bits in the tile ID.
	uint32_t m_tileBits;				///< Number of tile bits in the tile ID.
	
	dtTileCacheParams m_params;
	
	dtTileCacheAlloc* m_talloc;
	dtTileCacheCompressor* m_tcomp;
	dtTileCacheMeshProcess* m_tmproc;
	
	dtTileCacheObstacle* m_obstacles;
	dtTileCacheObstacle* m_nextFreeObstacle;
	
	static const int MAX_REQUESTS = 64;
	ObstacleRequest m_reqs[MAX_REQUESTS];
	int m_nreqs;
	
	static const int MAX_UPDATE = 64;
	dtCompressedTileRef m_update[MAX_UPDATE];
	int m_nupdate;
};

dtTileCache* dtAllocTileCache();
void dtFreeTileCache(dtTileCache* tc);

