#ifndef NAVIGATION_MESH_H
#define NAVIGATION_MESH_H

#include <list>
#include <vector>
#include <string>
#include <iostream>
#include <string>
#include <sstream>
#include <Godot.hpp>
#include <Spatial.hpp>
#include <Geometry.hpp>
#include <WeakRef.hpp>
#include <Node.hpp>
#include <Ref.hpp>
#include <SpatialMaterial.hpp>
#include <SceneTree.hpp>
#include <MeshInstance.hpp>
#include <MeshLibrary.hpp>
#include <Mesh.hpp>
#include <CubeMesh.hpp>
#include <ArrayMesh.hpp>
#include <CollisionShape.hpp>
#include <GridMap.hpp>
#include <Array.hpp>

#include "flowfield.h"
#include "flowFieldGridMap.h"
#include "helpers.h"
#include "navmesh_parameters.h"
#include "tilecache_helpers.h"
#include "navmesh_generator.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourTileCache.h"
#include "navigation_query.h"
#include "Recast.h"
#include "serializer.h"

#define GRIDMAP_NAME "GridMap"
#define GRIDMAP_CELL_SIZE 3.0

namespace godot
{
class DetourNavigationMesh : public Spatial
{
	GODOT_CLASS(DetourNavigationMesh, Spatial);

private:
	bool _is_being_deleted = false;
	MeshLibrary* gridmapMeshlib = nullptr;
	std::vector<CubeMesh*> meshlibCubes;
	float gridmapCellSize = GRIDMAP_CELL_SIZE;
	bool gridMapVisible = true;

protected:
	int collision_mask;
	Color debug_mesh_color;

public:
	SETGET(input_meshes_storage, Array);
	SETGET(input_transforms_storage, Array);
	SETGET(input_aabbs_storage, Array);
	SETGET(collision_ids_storage, Array);
	SETGET(serialized_navmesh_data, PoolByteArray);
	SETGET(uuid, String);

	FlowField* flowField = nullptr;

	bool debug_navmesh_dirty = true;

	DetourNavigationMesh();
	~DetourNavigationMesh();

	void _init();
	void _enter_tree();
	void _exit_tree();
	void _ready();
	static void _register_methods();
	void fill_pointer_arrays();

	void updateMeshlib();
	/*
	* @brief Function called from the Godot editor to launch a navmesh baking. 
	*/
	void build_navmesh();

	bool alloc();
	void release_navmesh();
	bool unsigned_int();

	bool load_mesh();
	void save_mesh();

	void setGridMapVisibility(bool visible);
	bool getGridMapVisibility() {return gridMapVisible;};

	void removeGridMap();
	void createGridMap();
	void rebuildGridMap();

	void setGridMapCellSize(float new_val);
	float getGridMapCellSize() {return gridmapCellSize;};

	void store_inputs();
	bool load_inputs();

	void build_debug_mesh(bool force_build);
	/*
	* @brief Finds all the lines of given tile within a given navmesh, and adds them to an array 
	*/
	void findTilePolyLines(int tileIdx, const dtNavMesh *navm, std::list<Vector3>* lines);
	Dictionary find_path(Variant from, Variant to);
	void _notification(int p_what);
	DetourNavigationMeshGenerator *init_generator(Transform global_transform);

	Ref<Material> get_debug_navigation_material();
	virtual dtTileCache *get_tile_cache() { return nullptr; };

	int get_collision_mask()
	{
		return collision_mask;
	}

	void set_collision_mask(int cm);

	MeshInstance *debug_mesh_instance = nullptr;

	DetourNavigationQuery *nav_query = nullptr;
	DetourNavigationQueryFilter *query_filter = nullptr;

	DetourNavigationMeshGenerator *generator = nullptr;

	AABB bounding_box;
	dtNavMesh *detour_navmesh = nullptr;
	Ref<ArrayMesh> debug_mesh = nullptr;
	Transform global_transform;
	Ref<NavmeshParameters> navmesh_parameters = nullptr;
	std::string navmesh_name = "default";
	FlowFieldGridMap *gridmap = nullptr;

	void set_generator(DetourNavigationMeshGenerator *g)
	{
		generator = g;
	}

	DetourNavigationMeshGenerator *get_generator()
	{
		return generator;
	}

	dtNavMesh *get_detour_navmesh()
	{
		return detour_navmesh;
	}

	Color get_debug_mesh_color()
	{
		return debug_mesh_color;
	}

	void set_debug_mesh_color(Color dmc)
	{
		debug_mesh_color = dmc;
		if (debug_mesh_instance != nullptr && debug_mesh_instance->get_material_override() != nullptr)
		{
			((Ref<SpatialMaterial>)(debug_mesh_instance->get_material_override()))->set_albedo(debug_mesh_color);
		}
	}

	void init_navigation_mesh_values();

	void clear_debug_mesh();

	void clear_navmesh();

	Ref<ArrayMesh> get_debug_mesh();

	/*==* GDScript exposed functions *==*/

	/*
	 * @brief Builds the flow field & navigation field from either global positions or cell coordinates
	 */
	bool buildFlowFieldToPos(PoolVector3Array targets);
	bool buildFlowFieldToCells(PoolVector3Array cellsCoordinates);

	/*
	 * @brief Returns the best direction towards the target, as defined by the FlowField
	 */
	Vector3 getBestDirectionFromPos(Vector3 currentPos);

	/*
	* @brief Builds a visual debug representation of the FlowField as a GridMap, with colors indicating distance from the goals
	*/
	void buildDebugDistanceField();

	/*
	* @brief Returns the map coordinates of the closest occupied cell to the given position
	*/
	PoolIntArray getClosestCellCoordinates(Vector3 pos);

	/*=* User I/O on the FlowField *=*/
	cellVal getCellBaseCost(PoolIntArray cellPos);
	void setCellBaseCost(PoolIntArray cellPos, cellVal newValue);

	cellVal getCellVariableCost(PoolIntArray cellPos);
	void setCellVariableCost(PoolIntArray cellPos, cellVal newValue);

};

} // namespace godot
#endif