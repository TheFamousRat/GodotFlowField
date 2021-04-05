#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <list>
#include <string>
#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <Godot.hpp>
#include <World.hpp>
#include <PhysicsShapeQueryParameters.hpp>
#include <PhysicsDirectSpaceState.hpp>
#include <Engine.hpp>
#include <Spatial.hpp>
#include <SceneTree.hpp>
#include <MeshInstance.hpp>
#include <BoxShape.hpp>
#include <CapsuleShape.hpp>
#include <CylinderShape.hpp>
#include <SphereShape.hpp>
#include <ConcavePolygonShape.hpp>
#include <ConvexPolygonShape.hpp>
#include <CubeMesh.hpp>
#include <CapsuleMesh.hpp>
#include <CylinderMesh.hpp>
#include <SphereMesh.hpp>
#include <StaticBody.hpp>
#include <CollisionShape.hpp>

#include "Recast.h"
#include "DetourNavMesh.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "navigation_mesh.h"
#include "helpers.h"
#include "navmesh_generator.h"
#include "simulation.hpp"

#define UPDATE_INTERVAL 0.1f //The inverse of this constant is the update frequency

namespace godot
{

class DetourNavigation : public Spatial
{
	GODOT_CLASS(DetourNavigation, Spatial)
	enum ParsedGeometryType
	{
		PARSED_GEOMETRY_STATIC_COLLIDERS = 0,
		PARSED_GEOMETRY_MESH_INSTANCES = 1

	};

private:
	/* geometry_source can be 0 = static bodies, 1 = meshes */
	float aggregated_time_passed = 0.f;

	SETGET(parsed_geometry_type, int);
	SETGET(dynamic_objects, bool);
	SETGET(dynamic_collision_mask, int);
	SETGET(collision_mask, int);

	bool auto_object_management;
	void set_auto_object_management (bool v);
	bool get_auto_object_management();

	void collect_geometry(Array geometries,
						  std::vector<Ref<Mesh>> *meshes, std::vector<Transform> *transforms,
						  std::vector<AABB> *aabbs, std::vector<int64_t> *collision_ids,
						  DetourNavigationMesh *navmesh);

	void convert_collision_shape(CollisionShape *collision_shape,
								 std::vector<Ref<Mesh>> *meshes, std::vector<Transform> *transforms,
								 std::vector<AABB> *aabbs, std::vector<int64_t> *collision_ids);

	std::vector<PhysicsBody *> dyn_bodies_to_add;
	std::vector<StaticBody *> static_bodies_to_add;
	std::vector<int64_t> collisions_to_remove;

	Simulation *sim;

public:
	static void _register_methods();

	DetourNavigation();
	~DetourNavigation();

	void _exit_tree();

	void _on_tree_exiting();

	void _enter_tree();

	void _init();
	void _ready();
	void _process(float passed);

	void registerAgent(Spatial* agentOwner, Area* neighboursDetector);
	void removeAgent(Spatial* agentOwner);

	void recalculate_masks();
	void fill_pointer_arrays();
	void manage_changes();
	void rebuild_dirty_debug_meshes();

	void save_collision_shapes(DetourNavigationMeshGenerator *generator);

	DetourNavigationMesh *create_navmesh(Ref<NavmeshParameters> np);

	std::vector<DetourNavigationMesh *> navmeshes;

	/*
	* @brief Responds to a call from the Godot editor to build a navmesh
	*/
	void build_navmesh(DetourNavigationMesh *navigation);

	void _notification(int p_what);
	void _on_node_renamed(Variant v);

	int process_large_mesh(MeshInstance *mesh_instance, int64_t collision_id,
						   std::vector<Ref<Mesh>> *meshes, std::vector<Transform> *transforms,
						   std::vector<AABB> *aabbs, std::vector<int64_t> *collision_ids);

	void _on_collision_shape_added(Variant node);
	void _on_collision_shape_removed(Variant node);
	void recognize_stored_collision_shapes();
	void collect_mappings(Dictionary &mappings, Array element);
	void map_collision_shapes(DetourNavigationMesh *nm, Dictionary &mappings);
};

} // namespace godot
#endif
