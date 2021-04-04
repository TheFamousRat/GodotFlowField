#include "navigation.h"
#define QUICKHULL_IMPLEMENTATION
#include "quickhull.h"

using namespace godot;

void DetourNavigation::_register_methods()
{
	register_method("_ready", &DetourNavigation::_ready);
	register_method("_process", &DetourNavigation::_process);
	register_method("create_navmesh", &DetourNavigation::create_navmesh);
	register_method("_exit_tree", &DetourNavigation::_exit_tree);
	register_method("_enter_tree", &DetourNavigation::_enter_tree);
	register_method("_on_tree_exiting", &DetourNavigation::_on_tree_exiting);

	register_method("registerAgent", &DetourNavigation::registerAgent);
	register_method("removeAgent", &DetourNavigation::removeAgent);

	register_method("setAgentMaxNeighbors", &DetourNavigation::setAgentMaxNeighbors);
	register_method("setAgentMaxSpeed", &DetourNavigation::setAgentMaxSpeed);
	register_method("setAgentNeighborDist", &DetourNavigation::setAgentNeighborDist);
	register_method("setAgentPosition", &DetourNavigation::setAgentPosition);
	register_method("setAgentPrefVelocity", &DetourNavigation::setAgentPrefVelocity);
	register_method("setAgentRadius", &DetourNavigation::setAgentRadius);
	register_method("setAgentTimeHorizon", &DetourNavigation::setAgentTimeHorizon);
	register_method("setAgentVelocity", &DetourNavigation::setAgentVelocity);
	register_method("setTimeStep", &DetourNavigation::setTimeStep);

	register_method("getAgentMaxNeighbors", &DetourNavigation::getAgentMaxNeighbors);
	register_method("getAgentMaxSpeed", &DetourNavigation::getAgentMaxSpeed);
	register_method("getAgentNeighborDist", &DetourNavigation::getAgentNeighborDist);
	register_method("getAgentPosition", &DetourNavigation::getAgentPosition);
	register_method("getAgentPrefVelocity", &DetourNavigation::getAgentPrefVelocity);
	register_method("getAgentRadius", &DetourNavigation::getAgentRadius);
	register_method("getAgentTimeHorizon", &DetourNavigation::getAgentTimeHorizon);
	register_method("getAgentVelocity", &DetourNavigation::getAgentVelocity);
	register_method("getTimeStep", &DetourNavigation::getTimeStep);
	
	register_property<DetourNavigation, bool>("auto_add_remove_objects", &DetourNavigation::set_auto_object_management, &DetourNavigation::get_auto_object_management, true);
}

/** Exposed functions **/

void DetourNavigation::registerAgent(Spatial* agentOwner) {
	Vector3 ownerGlobalPos = agentOwner->get_global_transform().origin;
	agentOwner->connect("tree_exiting", this, "removeAgent");
	registeredAgents[agentOwner] = sim->addAgent(RVO::Vector3({ownerGlobalPos[0], ownerGlobalPos[1], ownerGlobalPos[2]}));
}

void DetourNavigation::removeAgent(Spatial* agentOwner) {
	if (godot_is_instance_valid(agentOwner)) {
		agentOwner->disconnect("tree_exiting", this, "removeAgent");
	}
	sim->removeAgent(registeredAgents[agentOwner]);
	registeredAgents.erase(agentOwner);
}

DetourNavigation::DetourNavigation()
{
	initializeRVO();
}

DetourNavigation::~DetourNavigation()
{
}

void DetourNavigation::_exit_tree()
{
}

void DetourNavigation::_on_tree_exiting()
{
	if (!Engine::get_singleton()->is_editor_hint() && auto_object_management)
	{
		/*get_tree()->disconnect("node_added", this, "add_collision_shape");
		get_tree()->disconnect("node_removed", this, "remove_collision_shape");*/
	}
}

void DetourNavigation::_enter_tree()
{
	fill_pointer_arrays();
	recalculate_masks();
	if (!Engine::get_singleton()->is_editor_hint() && auto_object_management)
	{
		
	}
}

void DetourNavigation::initializeRVO() {
	sim = new RVO::RVOSimulator();
	sim->setTimeStep(0.02);
	sim->setAgentDefaults(3.0, 4, 0.05, 1.0, 2.0);
}

void DetourNavigation::_init()
{
	if (OS::get_singleton()->is_stdout_verbose())
	{
		Godot::print("Navigation init function called.");
	}

	set_parsed_geometry_type(PARSED_GEOMETRY_STATIC_COLLIDERS);
	auto_object_management = true;
}

/**
 * Ready godot function takes care of initializing child navmeshes 
 * loading them if they are baked and connecting singals
 */
void DetourNavigation::_ready()
{
	if (OS::get_singleton()->is_stdout_verbose())
	{
		Godot::print("Navigation ready function called.");
	}

	for (int i = 0; i < navmeshes.size(); ++i)
	{
		navmeshes[i]->load_mesh();
	}

	recognize_stored_collision_shapes();

	recalculate_masks();
	if (Engine::get_singleton()->is_editor_hint())
	{
		set_process(false);
	}
	else
	{
		/*get_tree()->connect("node_added", this, "add_collision_shape");
		get_tree()->connect("node_removed", this, "remove_collision_shape");*/
	}
}

/*Boring I/O ahead*/
void DetourNavigation::setAgentMaxNeighbors(Spatial* agentOwner, size_t maxNeighbours) {
	sim->setAgentMaxNeighbors(registeredAgents[agentOwner], maxNeighbours);
}

void DetourNavigation::setAgentMaxSpeed(Spatial* agentOwner, float newMaxSpeed) {
	sim->setAgentMaxSpeed(registeredAgents[agentOwner], newMaxSpeed);
}

void DetourNavigation::setAgentNeighborDist(Spatial* agentOwner, float newNeighbDist) {
	sim->setAgentNeighborDist(registeredAgents[agentOwner], newNeighbDist);
}

void DetourNavigation::setAgentPosition(Spatial* agentOwner, Vector3 newPos) {
	sim->setAgentPosition(registeredAgents[agentOwner], godotVecToRvoVec(newPos));
}

void DetourNavigation::setAgentPrefVelocity(Spatial* agentOwner, Vector3 newPrefVel) {
	sim->setAgentPrefVelocity(registeredAgents[agentOwner], godotVecToRvoVec(newPrefVel));
}	

void DetourNavigation::setAgentRadius(Spatial* agentOwner, float newRadius) {
	sim->setAgentRadius(registeredAgents[agentOwner], newRadius);
}

void DetourNavigation::setAgentTimeHorizon(Spatial* agentOwner, float timeHorizon) {
	sim->setAgentTimeHorizon(registeredAgents[agentOwner], timeHorizon);
}

void DetourNavigation::setAgentVelocity(Spatial* agentOwner, Vector3 newVel) {
	sim->setAgentVelocity(registeredAgents[agentOwner], godotVecToRvoVec(newVel));
}

void DetourNavigation::setTimeStep(float newTimeStep) {
	sim->setTimeStep(newTimeStep);
}

size_t DetourNavigation::getAgentMaxNeighbors(Spatial* agentOwner) {
	return sim->getAgentMaxNeighbors(registeredAgents[agentOwner]);
}

float DetourNavigation::getAgentMaxSpeed(Spatial* agentOwner) {
	return sim->getAgentMaxSpeed(registeredAgents[agentOwner]);
}

float DetourNavigation::getAgentNeighborDist(Spatial* agentOwner) {
	return sim->getAgentNeighborDist(registeredAgents[agentOwner]);
}

Vector3 DetourNavigation::getAgentPosition(Spatial* agentOwner) {
	return rvoVecToGodotVec(sim->getAgentPosition(registeredAgents[agentOwner]));
}

Vector3 DetourNavigation::getAgentPrefVelocity(Spatial* agentOwner) {
	return rvoVecToGodotVec(sim->getAgentPrefVelocity(registeredAgents[agentOwner]));
}

float DetourNavigation::getAgentRadius(Spatial* agentOwner) {
	return sim->getAgentRadius(registeredAgents[agentOwner]);
}

float DetourNavigation::getAgentTimeHorizon(Spatial* agentOwner) {
	return sim->getAgentTimeHorizon(registeredAgents[agentOwner]);
}

Vector3 DetourNavigation::getAgentVelocity(Spatial* agentOwner) {
	return rvoVecToGodotVec(sim->getAgentVelocity(registeredAgents[agentOwner]));
}

float DetourNavigation::getTimeStep() {
	return sim->getTimeStep();
}

Vector3 DetourNavigation::rvoVecToGodotVec(RVO::Vector3 vec) {
	return Vector3(vec[0], vec[1], vec[2]);
}

RVO::Vector3 DetourNavigation::godotVecToRvoVec(Vector3 vec) {
	return RVO::Vector3({vec[0], vec[1], vec[2]});
}

/** Non-exposed **/

void DetourNavigation::set_auto_object_management (bool v)
{
	if (auto_object_management != v)
	{
		auto_object_management = v;
		if (auto_object_management)
		{
			/*get_tree()->connect("node_added", this, "add_collision_shape");
			get_tree()->connect("node_removed", this, "remove_collision_shape");*/
		}
		else
		{
			/*get_tree()->disconnect("node_added", this, "add_collision_shape");
			get_tree()->disconnect("node_removed", this, "remove_collision_shape");*/
		}
		
	}
}
bool DetourNavigation::get_auto_object_management()
{
	return auto_object_management;
}


void DetourNavigation::fill_pointer_arrays()
{
	for (int i = 0; i < get_child_count(); i++)
	{
		DetourNavigationMesh *navmesh_p = Object::cast_to<DetourNavigationMesh>(
			get_child(i));
		if (navmesh_p != nullptr)
		{
			navmeshes.push_back(navmesh_p);
		}
	}
}

/**
 * Retrieve all dynamic and static collision masks from children
 * aggregate them and save them to the navigation node for future use
 * This is for performance (eager vs lazy calculation)
 */
void DetourNavigation::recalculate_masks()
{
	int collision_mask = 0;
	int dynamic_collision_mask = 0;

	for (int i = 0; i < navmeshes.size(); ++i)
	{
		collision_mask |= navmeshes[i]->get_collision_mask();
	}
	set_collision_mask(collision_mask);
	set_dynamic_collision_mask(dynamic_collision_mask);
}

void DetourNavigation::save_collision_shapes(
	DetourNavigationMeshGenerator *generator)
{
	if (generator->detour_navmesh == nullptr)
	{
		return;
	}

	int recalculating_start = static_cast<int>(generator->input_aabbs->size());
	for (StaticBody *static_body : static_bodies_to_add)
	{
		for (int j = 0; j < static_body->get_child_count(); ++j)
		{
			CollisionShape *collision_shape = Object::cast_to<CollisionShape>(
				static_body->get_child(j));

			if (collision_shape)
			{
				convert_collision_shape(collision_shape,
										generator->input_meshes, generator->input_transforms,
										generator->input_aabbs, generator->collision_ids);
			}
		}
	}
	// We mark dirty tiles to be recalculated
	generator->mark_dirty(recalculating_start, -1);
}

void DetourNavigation::_process(float passed)
{
	DetourNavigation::manage_changes();
	if (aggregated_time_passed >= UPDATE_INTERVAL)
	{
		aggregated_time_passed = 0.f;
		if (get_tree()->is_debugging_navigation_hint())
		{

			rebuild_dirty_debug_meshes();
		}
	}
	aggregated_time_passed += passed;
}

void DetourNavigation::manage_changes()
{
	if (static_bodies_to_add.size() > 0)
	{
		for (int i = 0; i < navmeshes.size(); ++i)
		{
			save_collision_shapes(navmeshes[i]->generator);
			navmeshes[i]->generator->recalculate_tiles();
			navmeshes[i]->debug_navmesh_dirty = true;
		}
		static_bodies_to_add.clear();
	}

	if (collisions_to_remove.size() > 0)
	{
		for (int i = 0; i < navmeshes.size(); ++i)
		{
			for (int64_t collision_shape_id : collisions_to_remove)
			{
				navmeshes[i]->generator->remove_collision_shape(
					collision_shape_id);
			}
			navmeshes[i]->generator->recalculate_tiles();
			navmeshes[i]->debug_navmesh_dirty = true;
		}
		collisions_to_remove.clear();
	}
}

void DetourNavigation::rebuild_dirty_debug_meshes()
{
	for (int i = 0; i < navmeshes.size(); ++i)
	{
		if (navmeshes[i]->debug_navmesh_dirty)
		{
			navmeshes[i]->build_debug_mesh(true);
		}
	}
}

void DetourNavigation::_on_collision_shape_removed(Variant node)
{
	CollisionShape *collision_shape = Object::cast_to<CollisionShape>(
		node.operator Object *());
	if (collision_shape)
	{
		StaticBody *static_body = Object::cast_to<StaticBody>(
			collision_shape->get_parent());
		if (static_body && (static_body->get_collision_layer() & collision_mask))
		{
			collisions_to_remove.push_back(collision_shape->get_instance_id());
			set_process(true);
		}
	}
}

DetourNavigationMesh *DetourNavigation::create_navmesh(
	Ref<NavmeshParameters> np)
{
	DetourNavigationMesh *navmesh = DetourNavigationMesh::_new();
	navmesh->navmesh_parameters = np;
	add_child(navmesh);
	navmesh->set_owner(get_tree()->get_edited_scene_root());
	navmesh->set_name("DetourNavigationMesh");
	navmeshes.push_back(navmesh);
	return navmesh;
}

void DetourNavigation::build_navmesh(DetourNavigationMesh *navmesh)
{
	if (!navmesh->navmesh_parameters.is_valid())
	{
		return;
	}

	auto start = std::chrono::high_resolution_clock::now();

		DetourNavigationMeshGenerator *dtnavmesh_gen = navmesh->init_generator(get_global_transform());

    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Generator init : " << std::chrono::duration_cast<std::chrono::milliseconds>( end - start).count() << '\n';

	start = std::chrono::high_resolution_clock::now();

		DetourNavigation::collect_geometry(get_children(), dtnavmesh_gen->input_meshes, dtnavmesh_gen->input_transforms,
										dtnavmesh_gen->input_aabbs, dtnavmesh_gen->collision_ids, navmesh);
										
	end = std::chrono::high_resolution_clock::now();
    std::cout << "Collecting geometry : " << std::chrono::duration_cast<std::chrono::milliseconds>( end - start).count() << '\n';

	start = std::chrono::high_resolution_clock::now();

		dtnavmesh_gen->build();

	end = std::chrono::high_resolution_clock::now();
    std::cout << "Building nav mesh : " << std::chrono::duration_cast<std::chrono::milliseconds>( end - start).count() << '\n';

	start = std::chrono::high_resolution_clock::now();

		navmesh->detour_navmesh = dtnavmesh_gen->detour_navmesh;
		if (get_tree()->is_debugging_navigation_hint() || Engine::get_singleton()->is_editor_hint())
		{
			navmesh->build_debug_mesh(true);
			navmesh->debug_mesh_instance->set_owner(navmesh);
		}

	end = std::chrono::high_resolution_clock::now();
    std::cout << "Building debug mesh : " << std::chrono::duration_cast<std::chrono::milliseconds>( end - start).count() << '\n';
}

void DetourNavigation::_notification(int p_what)
{
	switch (p_what)
	{
	case NOTIFICATION_PREDELETE:
	{
		_on_tree_exiting();
	}
	break;
	}
}

/**
 * Processes a mesh and if it's too large it splits it in
 * smaller chunks
 *
 * @return 1 for mesh found and 0 if mesh is too small to be split
 */
int DetourNavigation::process_large_mesh(MeshInstance *mesh_instance,
										 int64_t collision_id, std::vector<Ref<Mesh>> *meshes,
										 std::vector<Transform> *transforms, std::vector<AABB> *aabbs,
										 std::vector<int64_t> *collision_ids)
{
	if (mesh_instance->get_mesh()->get_class() != "ArrayMesh")
	{
		return 0;
	}
	Ref<ArrayMesh> array_mesh = mesh_instance->get_mesh();
	float tile_edge_length = 0;
	tile_edge_length =
		navmeshes[0]->navmesh_parameters->get_tile_edge_length();
	

	//int current_vertex_count = 0;
	int face_count = 0;

	AABB transformed_aabb = mesh_instance->get_transform().xform(
		mesh_instance->get_aabb());

	for (int i = 0; i < array_mesh->get_surface_count(); i++)
	{

		if (array_mesh->surface_get_primitive_type(i) != Mesh::PRIMITIVE_TRIANGLES)
			return 0;

		if (transformed_aabb.get_size().x < tile_edge_length && transformed_aabb.get_size().z < tile_edge_length)
		{
			return 0;
		}
		int index_count = 0;

		bool index_format = array_mesh->surface_get_format(i) & Mesh::ARRAY_FORMAT_INDEX;
		PoolVector3Array mesh_vertices;

		if (index_format)
		{
			index_count = static_cast<int>(array_mesh->surface_get_array_index_len(i));
			return 0; // Temporary continue
		}
		else
		{
			mesh_vertices =
				array_mesh->surface_get_arrays(i)[Mesh::ARRAY_VERTEX];
			index_count = static_cast<int>(array_mesh->surface_get_array_len(i));
			face_count = mesh_vertices.size() / 3;
		}

		if (face_count < 200)
		{
			return 0;
		}

		// We want to split the mesh into "tile edge length" pieces
		Vector3 base_position = transformed_aabb.get_position();
		Vector3 make_positive = Vector3(0.f, 0.f, 0.f);
		make_positive.x -= base_position.x;
		make_positive.z -= base_position.z;

		int x_size = (int)std::ceil(
			(base_position.x + make_positive.x + transformed_aabb.get_size().x) / tile_edge_length);
		int z_size = (int)std::ceil(
			(base_position.z + make_positive.z + transformed_aabb.get_size().z) / tile_edge_length);

		std::vector<std::vector<std::list<Vector3>>> mesh_instance_array(x_size,
																		 std::vector<std::list<Vector3>>(z_size));

		for (int j = 0; j < face_count; j++)
		{
			Vector3 a = mesh_instance->get_transform().xform(
							mesh_vertices[j * 3 + 0]) +
						make_positive;
			Vector3 b = mesh_instance->get_transform().xform(
							mesh_vertices[j * 3 + 1]) +
						make_positive;
			Vector3 c = mesh_instance->get_transform().xform(
							mesh_vertices[j * 3 + 2]) +
						make_positive;

			float min_x = std::min(std::min(a.x, b.x), c.x);
			float min_z = std::min(std::min(a.z, b.z), c.z);

			float max_x = std::max(std::max(a.x, b.x), c.x);
			float max_z = std::max(std::max(a.z, b.z), c.z);

			int tile_min_x = (int)std::floor((min_x) / tile_edge_length);
			int tile_min_z = (int)std::floor((min_z) / tile_edge_length);
			int tile_max_x = (int)std::floor((max_x) / tile_edge_length);
			int tile_max_z = (int)std::floor((max_z) / tile_edge_length);

			for (int x = tile_min_x; x <= tile_max_x; x++)
			{
				for (int z = tile_min_z; z <= tile_max_z; z++)
				{
					mesh_instance_array[x][z].push_back(
						mesh_vertices[j * 3 + 0]);
					mesh_instance_array[x][z].push_back(
						mesh_vertices[j * 3 + 1]);
					mesh_instance_array[x][z].push_back(
						mesh_vertices[j * 3 + 2]);
				}
			}
		}

		for (int i = 0; i < mesh_instance_array.size(); i++)
		{
			for (int j = 0; j < mesh_instance_array[i].size(); j++)
			{
				PoolVector3Array faces;
				faces.resize(static_cast<int>(mesh_instance_array[i][j].size()));
				PoolVector3Array::Write w = faces.write();
				int k = 0;
				for (Vector3 a : mesh_instance_array[i][j])
				{
					w[k] = a;
					k++;
				}

				Ref<ArrayMesh> am;
				am.instance();
				Array arrays;
				arrays.resize(ArrayMesh::ARRAY_MAX);
				arrays[ArrayMesh::ARRAY_VERTEX] = faces;
				am->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

				aabbs->push_back(mesh_instance->get_aabb());
				meshes->push_back(am);
				transforms->push_back(mesh_instance->get_transform());
				collision_ids->push_back(collision_id);
			}
		}
	}

	return 1;
}

/**
 * Loads mesh from file and helper meshes from storage
 * Builds a debug mesh if debug navigation is checked or if 
 * its open in editor
 * 
 * @return true if success
 */
void DetourNavigation::convert_collision_shape(CollisionShape *collision_shape,
											   std::vector<Ref<Mesh>> *meshes, std::vector<Transform> *transforms,
											   std::vector<AABB> *aabbs, std::vector<int64_t> *collision_ids)
{
	Transform transform = collision_shape->get_global_transform();

	Ref<Mesh> mesh;
	Ref<Shape> s = collision_shape->get_shape();

	if (s->get_class() == "BoxShape")
	{
		BoxShape *box = Object::cast_to<BoxShape>(*s);
		Ref<CubeMesh> cube_mesh;
		cube_mesh.instance();
		cube_mesh->set_size(box->get_extents() * 2.0);
		Ref<ArrayMesh> array_mesh;
		array_mesh.instance();
		array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, cube_mesh->get_mesh_arrays());
		cube_mesh.unref();
		mesh = array_mesh;
	}

	else if (s->get_class() == "CapsuleShape")
	{
		CapsuleShape *capsule = Object::cast_to<CapsuleShape>(*s);
		Ref<CapsuleMesh> capsule_mesh;
		capsule_mesh.instance();
		capsule_mesh->set_radius(capsule->get_radius());
		capsule_mesh->set_mid_height(capsule->get_height() / static_cast<real_t>(2.0));
		Ref<ArrayMesh> array_mesh;
		array_mesh.instance();
		array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, capsule_mesh->get_mesh_arrays());
		capsule_mesh.unref();
		mesh = array_mesh;
	}

	else if (s->get_class() == "CylinderShape")
	{
		CylinderShape *cylinder = Object::cast_to<CylinderShape>(*s);
		Ref<CylinderMesh> cylinder_mesh;
		cylinder_mesh.instance();
		cylinder_mesh->set_height(cylinder->get_height());
		cylinder_mesh->set_bottom_radius(cylinder->get_radius());
		cylinder_mesh->set_top_radius(cylinder->get_radius());
		Ref<ArrayMesh> array_mesh;
		array_mesh.instance();
		array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, cylinder_mesh->get_mesh_arrays());
		cylinder_mesh.unref();
		mesh = array_mesh;
	}

	else if (s->get_class() == "SphereShape")
	{
		SphereShape *sphere = Object::cast_to<SphereShape>(*s);
		Ref<SphereMesh> sphere_mesh;
		sphere_mesh.instance();
		sphere_mesh->set_radius(sphere->get_radius());
		sphere_mesh->set_height(sphere->get_radius() * static_cast<real_t>(2.0));
		Ref<ArrayMesh> array_mesh;
		array_mesh.instance();
		array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, sphere_mesh->get_mesh_arrays());
		sphere_mesh.unref();
		mesh = array_mesh;
	}

	else if (s->get_class() == "ConcavePolygonShape")
	{
		ConcavePolygonShape *concave_polygon = Object::cast_to<ConcavePolygonShape>(
			*s);
		PoolVector3Array p_faces = concave_polygon->get_faces();
		Ref<ArrayMesh> array_mesh;
		array_mesh.instance();
		Array arrays;
		arrays.resize(ArrayMesh::ARRAY_MAX);
		arrays[ArrayMesh::ARRAY_VERTEX] = p_faces;

		array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
		mesh = array_mesh;
	}

	else if (s->get_class() == "ConvexPolygonShape")
	{
		Ref<ConvexPolygonShape> convex_polygon = Object::cast_to<ConvexPolygonShape>(*s);
		PoolVector3Array varr = convex_polygon->get_points();
		/* Build a hull here */
		qh_vertex_t *vertices = new qh_vertex_t[varr.size()];

		for (int i = 0; i < varr.size(); ++i)
		{
			vertices[i].z = varr[i].z;
			vertices[i].x = varr[i].x;
			vertices[i].y = varr[i].y;
		}

		qh_mesh_t qh_mesh = qh_quickhull3d(vertices, varr.size());

		PoolVector3Array mesh_vertices;
		PoolIntArray mesh_indices;
		mesh_vertices.resize(qh_mesh.nvertices);
		mesh_indices.resize(qh_mesh.nindices);
		PoolVector3Array::Write w = mesh_vertices.write();
		PoolIntArray::Write wi = mesh_indices.write();
		for (unsigned int j = 0; j < qh_mesh.nvertices; j += 1)
		{
			w[j].x = qh_mesh.vertices[j].x;
			w[j].y = qh_mesh.vertices[j].y;
			w[j].z = qh_mesh.vertices[j].z;
		}
		for (unsigned int j = 0; j < qh_mesh.nindices; j += 1)
		{
			wi[j] = qh_mesh.indices[j];
		}
		qh_free_mesh(qh_mesh);
		Ref<ArrayMesh> array_mesh;
		array_mesh.instance();
		Array arrays;
		arrays.resize(ArrayMesh::ARRAY_MAX);
		arrays[ArrayMesh::ARRAY_VERTEX] = mesh_vertices;
		arrays[ArrayMesh::ARRAY_INDEX] = mesh_indices;

		array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
		mesh = array_mesh;
	}

	if (mesh.is_valid())
	{
		MeshInstance *mi = MeshInstance::_new();
		mi->set_mesh(mesh);
		mi->set_transform(transform);
		if (!process_large_mesh(mi, collision_shape->get_instance_id(), meshes,
								transforms, aabbs, collision_ids))
		{
			aabbs->push_back(mi->get_aabb());
			meshes->push_back(mesh);
			transforms->push_back(transform);
			collision_ids->push_back(collision_shape->get_instance_id());
		}
		mi->set_mesh(NULL);
		mi->queue_free();
		mi->free();
	}
}

/**
 * Recursively collects all necessery geometry for building navmesh
 */
void DetourNavigation::collect_geometry(Array geometries,
										std::vector<Ref<Mesh>> *meshes, std::vector<Transform> *transforms,
										std::vector<AABB> *aabbs, std::vector<int64_t> *collision_ids,
										DetourNavigationMesh *navmesh)
{

	int geom_size = geometries.size();
	for (int i = 0; i < geom_size; i++)
	{
		// If geometry source is MeshInstances, just collect them
		if (parsed_geometry_type == PARSED_GEOMETRY_MESH_INSTANCES)
		{
			MeshInstance *mesh_instance = Object::cast_to<MeshInstance>(
				geometries[i]);
			if (mesh_instance)
			{
				meshes->push_back(mesh_instance->get_mesh());
				transforms->push_back(mesh_instance->get_global_transform());
				aabbs->push_back(mesh_instance->get_aabb());
			}
		}
		// If geometry source is Static bodies, convert them to meshes
		else
		{

			StaticBody *static_body = Object::cast_to<StaticBody>(
				geometries[i]);
			if (static_body && static_body->get_collision_layer() & navmesh->get_collision_mask())
			{
				for (int i = 0; i < static_body->get_child_count(); ++i)
				{
					CollisionShape *collision_shape = Object::cast_to<
						CollisionShape>(static_body->get_child(i));
					if (collision_shape)
					{
						convert_collision_shape(collision_shape, meshes,
												transforms, aabbs, collision_ids);
					}
				}
			}
		}
		Spatial *spatial = Object::cast_to<Spatial>(geometries[i]);
		if (spatial)
		{
			collect_geometry(spatial->get_children(), meshes, transforms,
							 aabbs, collision_ids, navmesh);
		}
	}
}

/**
 * Creates mapping dictionaries from positions to collision ids
 * Applies that mapping dictionary to all navigation meshes
 */
void DetourNavigation::recognize_stored_collision_shapes()
{
	Dictionary mappings;
	collect_mappings(mappings, get_children());

	for (int i = 0; i < navmeshes.size(); ++i)
	{
		map_collision_shapes(navmeshes[i], mappings);
	}
}

/**
 * Recursively gets all children of navigation node and
 * adds them their mapping - position_key : collision_instance_id
 * to mapping dictionary for later use
 */
void DetourNavigation::collect_mappings(Dictionary &mappings, Array elements)
{
	for (int i = 0; i < elements.size(); i++)
	{
		CollisionShape *collision_shape = Object::cast_to<CollisionShape>(elements[i]);
		if (collision_shape)
		{
			Transform gt = collision_shape->get_global_transform();
			std::string key_string = (std::to_string(gt.origin.x) + "x" + std::to_string(gt.origin.y) + "x" + std::to_string(gt.origin.z));
			mappings[key_string.c_str()] = collision_shape->get_instance_id();
		}
		collect_mappings(mappings, ((Spatial *)elements[i])->get_children());
	}
}

/**
 * Maps all input storage positions into corresponding collision shape ids
 */
void DetourNavigation::map_collision_shapes(DetourNavigationMesh *nm, Dictionary &mappings)
{
	// Check if mesh was properly initialized
	if (nm->generator == nullptr || nm->generator->input_transforms == nullptr)
	{
		return;
	}
	for (int j = 0; j < nm->generator->input_transforms->size(); ++j)
	{
		Transform it = nm->generator->input_transforms->at(j);
		std::string key_string = (std::to_string(it.origin.x) + "x" + std::to_string(it.origin.y) + "x" + std::to_string(it.origin.z));
		if (mappings.has(key_string.c_str()))
		{
			nm->generator->collision_ids->at(j) = (int64_t)mappings[key_string.c_str()];
		}
	}
}