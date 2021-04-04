#ifndef FF_GRIDMAP_H
#define FF_GRIDMAP_H

#include <array>
#include <map>
#include <vector>
#include <limits>

#include <Godot.hpp>
#include <GridMap.hpp>

struct RasterizedTileInfo {
    std::vector<std::array<int, 3>> occupiedTileCells;

    std::array<int, 3> BBoxMin;
    std::array<int, 3> BBoxMax;
    
    RasterizedTileInfo() {
        BBoxMin = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
	    BBoxMax = {std::numeric_limits<int>::min(), std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};
    };
    ~RasterizedTileInfo() {};

    bool isEmpty() {
        return (BBoxMin[0] == std::numeric_limits<int>::max());
    };

    void updateWithNewCell(int cellX, int cellY, int cellZ) {
        if (cellX < BBoxMin[0])
            BBoxMin[0] = cellX;
        if (cellY < BBoxMin[1])
            BBoxMin[1] = cellY;
        if (cellZ < BBoxMin[2])
            BBoxMin[2] = cellZ;

        if (cellX > BBoxMax[0])
            BBoxMax[0] = cellX;
        if (cellY > BBoxMax[1])
            BBoxMax[1] = cellY;
        if (cellZ > BBoxMax[2])
            BBoxMax[2] = cellZ;
    }
};

namespace godot {

class FlowFieldGridMap : public GridMap {
    GODOT_CLASS(FlowFieldGridMap, GridMap);

    private:
        std::map<std::array<int, 2>, RasterizedTileInfo> tilesInfos;

    public:
        static void _register_methods();

        FlowFieldGridMap();
        ~FlowFieldGridMap();

        void _init() {};

        /*
         * @brief Adds a triangle with given coords from a navmesh tile (x;z)
         */
        void rasterizeTileTriangle(Vector3 v0, Vector3 v1, Vector3 v2, int x, int z);

        /*
         * @brief Cleans a rasterized navmesh tile (x;z) by removing cells on top of others
         */
        void clearSuperposedCellsInTile(int x, int z);

        /*
         * @brief Resets a rasterized tile and its informations
         */
        void clearTile(int x, int z);


        void clear();

        /*
         * @brief Recalculates the BBox of the rasterized navmesh tile (x;z) from its occupied cells
         */
        void recalculateTileBoundingBox(int x, int z);

        RasterizedTileInfo* getTileInfo(int x, int z);
};

}

godot::Vector3 closestEdgePoint(godot::Vector3 v0, godot::Vector3 v1, godot::Vector3 d);
bool pointInTriangle(godot::Vector3 v0, godot::Vector3 v1, godot::Vector3 v2, godot::Vector3 d);
float closestTrianglePointDist_sq(godot::Vector3 v0, godot::Vector3 v1, godot::Vector3 v2, godot::Vector3 d);

#endif