#include "flowFieldGridMap.h"

using namespace godot;

void FlowFieldGridMap::_register_methods() {

}

FlowFieldGridMap::FlowFieldGridMap() {

}

FlowFieldGridMap::~FlowFieldGridMap() {

}

godot::Vector3 closestEdgePoint(godot::Vector3 v0, godot::Vector3 v1, godot::Vector3 d) {
    godot::Vector3 lineDir = v0 - v1;
    float t = (d - v1).dot(lineDir) / lineDir.length_squared();

    if (t <= 0.0f) {
        return v1;
    }
    else if (t >= 1.0f) {
        return v0;
    }
    else {
        return v1 + lineDir * t;
    }
}
	
bool pointInTriangle(godot::Vector3 v0, godot::Vector3 v1, godot::Vector3 v2, godot::Vector3 d) {
    // Compute vectors        
    Vector3 d0 = v2 - v0;
    Vector3 d1 = v1 - v0;
    Vector3 d2 = d - v0;

    // Compute dot products
    float dot00 = d0.dot(d0);
    float dot01 = d0.dot(d1);
    float dot02 = d0.dot(d2);
    float dot11 = d1.dot(d1);
    float dot12 = d1.dot(d2);

    // Compute barycentric coordinates
    float invDenom = 1.0f / ((dot00 * dot11) - (dot01 * dot01));
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if point is in triangle
    return (u >= 0) && (v >= 0) && (u + v < 1);
}

float closestTrianglePointDist_sq(godot::Vector3 v0, godot::Vector3 v1, godot::Vector3 v2, godot::Vector3 d) {
    godot::Vector3 triNormal = ((v0 - v1).cross(v0 - v2)).normalized();
    godot::Vector3 planePoint = d + triNormal * triNormal.dot(v0 - d);

    if (pointInTriangle(v0, v1, v2, planePoint)) {
        return (d - planePoint).length_squared();
    }
    else {
        float edgeDist0 = (closestEdgePoint(v0, v1, d) - d).length_squared();
        float edgeDist1 = (closestEdgePoint(v0, v2, d) - d).length_squared();
        float edgeDist2 = (closestEdgePoint(v1, v2, d) - d).length_squared();

        return std::min(edgeDist0, std::min(edgeDist1, edgeDist2));
    }
}

void FlowFieldGridMap::rasterizeTileTriangle(Vector3 v0, Vector3 v1, Vector3 v2, int x, int z) {
    /*
    * Possible optimizations :
    * 1/Check if all the vertices of the triangle are all adjacent and non-empty. If yes, we can skip it
    * 2/Check if the considered point in the triangle is close enough to the center of the cell, so
    * that the number of cells mostly empty of polygon gets diminished
    * 3/Remove cells on top of other occupied cells
    */
    RasterizedTileInfo* tileInfo = getTileInfo(x,z);
    float minGridCellSize = get_cell_size().x;
    
    Vector3 v0GridCoords = world_to_map(v0);
    Vector3 v1GridCoords = world_to_map(v1);
    Vector3 v2GridCoords = world_to_map(v2);

    Vector3 triangleBBMin = v0GridCoords;
    Vector3 triangleBBMax = v0GridCoords;
    for (int i(0) ; i < 3 ; i++) {
        if (v1GridCoords[i] < triangleBBMin[i])
            triangleBBMin[i] = v1GridCoords[i];
        if (v2GridCoords[i] < triangleBBMin[i])
            triangleBBMin[i] = v2GridCoords[i];
    
        if (v1GridCoords[i] > triangleBBMax[i])
            triangleBBMax[i] = v1GridCoords[i];
        if (v2GridCoords[i] > triangleBBMax[i])
            triangleBBMax[i] = v2GridCoords[i];
    }

    for (int x(triangleBBMin[0]) ; x <= triangleBBMax[0] ; x++) {
        for (int y(triangleBBMin[1]) ; y <= triangleBBMax[1] ; y++) {
            for (int z(triangleBBMin[2]) ; z <= triangleBBMax[2] ; z++) {
                Vector3 currentPosGlobal = map_to_world(x,y,z);

                if (std::sqrt(closestTrianglePointDist_sq(v0, v1, v2, currentPosGlobal)) <= 0.5f * minGridCellSize) {
                    set_cell_item(x, y, z, 0);

                    tileInfo->updateWithNewCell(x, y, z);
                }
            }
        }
    }

    //Todo : memorize cells that were added in this tile (for later reset purposes)
}

void FlowFieldGridMap::clearSuperposedCellsInTile(int x, int z) {
    RasterizedTileInfo* tileInfo = getTileInfo(x, z);

    bool belowCellFilled = false;
    for (float x = tileInfo->BBoxMin[0] ; x <= tileInfo->BBoxMax[0] ; x++) {
        for (float z = tileInfo->BBoxMin[2] ; z <= tileInfo->BBoxMax[2] ; z++) {
            for (float y = tileInfo->BBoxMin[1] ; y <= tileInfo->BBoxMax[1] ; y++) {
                //Empty the tile in a "column" : all full cells under a filled cell are emptied, until an empty cell is found
                //This is to prevent redundant cells, that are just on top of cells where the player can walk

                if (get_cell_item(x, y, z) != -1) {
                    if (belowCellFilled) {
                        set_cell_item(x, y, z, -1);
                    }
                    else {
                        belowCellFilled = true;
                    }
                }
                else {
                    belowCellFilled = false;
                }
            }
            belowCellFilled = false;
        }
    }
}

void FlowFieldGridMap::clearTile(int x, int z) {
    RasterizedTileInfo* tileInfo = getTileInfo(x, z);
    if (!tileInfo->isEmpty()) {

        for (float x = tileInfo->BBoxMin[0] ; x <= tileInfo->BBoxMax[0] ; x++) {
            for (float y = tileInfo->BBoxMin[1] ; y <= tileInfo->BBoxMax[1] ; y++) {
                for (float z = tileInfo->BBoxMin[2] ; z <= tileInfo->BBoxMax[2] ; z++) {
                    set_cell_item(x,y,z,-1);
                }
            }
        }

        tilesInfos[{x,z}] = RasterizedTileInfo();
    }
}

void FlowFieldGridMap::clear() {
    //Emptying the grid cells
    Array usedCells = get_used_cells();

	for (int i(0) ; i < usedCells.size() ; i++) {
		Vector3 curCell = usedCells[i];
		set_cell_item(curCell.x, curCell.y, curCell.z, -1);
	}

    tilesInfos.clear();
}

void FlowFieldGridMap::recalculateTileBoundingBox(int x, int z) {

}

RasterizedTileInfo* FlowFieldGridMap::getTileInfo(int x, int z) {
    std::array<int, 2> tileCoords({x,z});
    auto ret = tilesInfos.find(tileCoords);

    if (ret != tilesInfos.end()) {
        return &(ret->second);
    }
    else {
        //Reaching this code means info for the tile wasn't created yet
        tilesInfos[tileCoords] = RasterizedTileInfo();
        return &(tilesInfos.find(tileCoords)->second);
    }
}