# GodotFlowFields
Current version: 0.1 for Godot 3.2.x

This plugin provides an efficient and quick framework for using flow field based pathfinding into your game. Being based on the same technology Godot uses for baking navigation meshes, it provides a familiar framework with the added needs of the flow field.

## What is flow field pathfinding ?

Most pathfinding in game is done today with the A* algorithm. While very efficient, its cost and complexity grows significantly with the number of agents in the game, making it impractical to have a large number of enemies. A known solution to that culprit takes the form of the flow fields, that this project aims to bring into Godot's 3d engine. 

Flow fields are based on Dijkstra maps, who take a "reverse" approach to pathfinding : instead of heuristically going from the agent to the target like in A*, we go from the target to any other possible location. If we discretize the space (in this plugin, by voxelizing the terrain and obstacles), we can then from any location get a rough estimate of how far we are from the goal, and which directions we should take to reach it. The big advantage of Flow field is that we can "bake" that distance to the target : if the target is fixed, then the distance to it from any point will never change. We can thus extremely quickly, for any agent at any position, guide it smoothly to the goal it wants to reach. This means the marginal cost of a new agent is minimal, allowing for a large number of entities at once.

## How does this addon work ?

For my project I ideally need any walkable 3d space to be transformed to a space where a large number of entities can find their path to a moving target. The usual solution to do this is to bake a navigation mesh, which will allow entities to quickly find their path among the level geometry. Since this solution works really well to extract the information of where certain agents can walk on the terrain, I decided the best process for now was to :
	1/Bake a navigation mesh with the terrain, obstacles...
	2/Voxelize this navigation mesh to a GridMap, thus keeping the "walkable" space while providing an easy discrete space to work with
	3/Structure the Flow field from this GridMap : find the neighbours of the cell, their distance from each other etc.
	
Once this is done, the Flow Field is ready to be used, and can quickly build a flow field to a given target, give the next best cell to a target etc.

## Features

Current features :
 - Possibility to add and remove obstacles on the fly
 - Finetuning of the navigation mesh with all the standard recast parameters (agent height, radius, tile size etc.)
 - Quick baking and visualization of the flow field within the Godot editor, guaranteeing very good performances 
 - Handling of "one-way" neighbours : for example, cells to which the agent can jump (but obviously not always jump back to if the height was great)
 - Finetuning of the Flow field with the cell size (ie the resolution of the flow field)
 - Quick (re-)building of the Flow Field towards a given target

Planned features :
 - Variable cost for the cells, allowing the user to change the cost of cells as he wishes (to signify a dangerous zone, traffic jam, or on the contrary attract entities to a point)
 - Addition of "layers" to the Flow field : they could be used to represent different goal of different entities on the same flow field. For example, the layer "player" guides to the player, while the layer "heights" would guide ranged units to find high position to shoot at the player from a distance
 - Flow field detecting if an agent can jump from a cell to another
 
## Installation

Make sure git is installed on your computer. 

First install dependencies :

```
sudo apt-get install libomp-dev
```

Then go to your project's **addons/** folder, and open a terminal. Then run :

```
git clone --recursive https://github.com/TheFamousRat/GodotFlowField.git 
cd GodotFlowField
git clone --recursive https://github.com/godotengine/godot-cpp.git
```

You then need to build the relevant libraries. First godot-cpp (that I advise you to put somewhere else if you intend to use GDNative a lot, it's used very often) :

```
cd godot-cpp
scons generate_bindings=yes -j4
cd ..
```

Then build RVO2-3d :

```
cd RVO2-3D
cmake .
make
```

The dependencies are all built ! Perfect. Now, onto building the library itself. We need the scons utility tool to perform this task. If you don't have it, just open. If not, you can install it with python by simply running 
 
```
python -m pip install scons
```

or, if you use Python 3,

```
python3 -m pip install scons
```

Finally, you need to make sure the godot-headers and godot-cpp files are reachable by Sconstruct (it needs to know their exact location). For this, open the file "SConstruct" in the GodotFlowField folder. What interests us here is the variables  "godot_headers_path" and "cpp_bindings_path". If you don't find them, Ctrl+F them in the file. You need to replace the paths in those variables by the path of your own godot-cpp. If you have only followed the instructions (closely) and haven't changed the godot-cpp folder location (you dirty copy-paster), the location should be simply "godot-cpp" and "godot-cpp/godot-headers".

Then, make sure the build utility "Scons" is installed.

Once that's done, simply run 

```
scons
```

in the folder where the file "SConstruct" is (normally the root of the addon files).

If it builds without errors, life is great ! Otherwise... not so

Then the project is ready to be used ! Simply open your Godot project, go to Project > Project Settings > Plugins, and check "GodotFlowField". Et voilà !

## How to use

The use of the plugin is very similar to that of Navigation meshes in Godot. As such, to build a Flow field, add the new node type called "FlowFieldNavigation". Then, select that node in the editor, and at the top of the screen click on Navigation Manager > create navmesh. And you're now good to go ! The navigation mesh that you just added will contain all the information about the walkable space, navmesh and flow field included. Of course you first need to bake it !

To bake it, first insure you have geometry ! The plugin works by finding **all** the CollisionShapes that are direct/indirect children of the "FlowFieldNavigation" node (and that are on the layer of the navigation meshes). Once you've added geometry in such a way, select the navigation mesh node and click on "Bake navmesh". After some time (expect can take minutes on big maps, but is usually close to a reasonable 5-10 seconds), you should see the baked FlowField ! (in the form here of a GridMap, representing the cells).

I will release a video when I have time covering in detail the process of installation and the use of the plugin on a basic map. 

## Current state 

This addon is actively developped, however it is at the time of writing not ideal to use. The main issue is agents going too fast and falling off ledges/the flow field, which is something I need to actively work on. The performance is also not stellar, which is to be expected at this early stage of the project. There are also a lot of errors popping up in Godot when using the plugin, which are however benign (as far as I experienced while working on this). This will eventually be looked into and fixed.

If the project interests you, I am very open to discussions, suggestions or contributions to the project.
 
## Special thanks

The base of this project largely comes from MilosLukic's Godot-Navigation-Lite https://github.com/MilosLukic/Godot-Navigation-Lite. It is a fantastic project in its own right that aims to provide state of the art navigation meshes to Godot. The navigation mesh baking and the interfaces are mostly from that project, albeit significantly adapted to a Flow field's needs.
