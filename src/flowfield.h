#ifndef FLOWFIELD_H
#define FLOWFIELD_H

#include <chrono>

#include <list>
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <array>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <map>
#include <Godot.hpp>
#include <ArrayMesh.hpp>
#include <GridMap.hpp>
#include <RayCast.hpp>

#include "flowFieldGridMap.h"

#define UNTRAVERSIBLE_CELL_COST 65536
#define CELL_BASE_COST 1
#define LEDGE_COST 3

#define SQRT_2 1.41421356237
#define SQRT_3 1.73205080757

typedef unsigned int uint;
typedef float cellVal;

namespace godot {

//Small helper to hash array<int, 3> objects
struct ArrayHasher {
    std::size_t operator()(const std::array<int, 3>& a) const {
        std::size_t h = 0;

        for (auto e : a) {
            h ^= std::hash<int>{}(e)  + 0x9e3779b9 + (h << 6) + (h >> 2); 
        }
        return h;
    }   
};

struct Cell {
	public:
		/*
		 * @brief The best cost of this cell towards the goal (ie the "time" needed from here to go to the goal, at best)
		 */
		cellVal bestCost; 
		
		/*
		 * @brief The difficulty to traverse that cell, given by the terrain its made up of
		 */
		cellVal baseCost;

		/*
		 * @brief Temporary elements susceptible to change the path (how crowded the cell is, for example)
		 */
		cellVal variableCost;

		/*
		 * @brief Coordinates of the cell within the gridmap. Can be converted to global pos with gridmap->map_to_world
		 */
		std::array<int, 3> cellPos;

		/*
		 * @brief Direction to take to get nearer to the current goal
		 */
		Vector3 bestDirection;

		Cell();
		Cell(std::array<int, 3> pos);
		~Cell();

		Vector3 getWorldPos(GridMap* gridmap) {
			return gridmap->map_to_world(cellPos[0], cellPos[1], cellPos[2]);
		}
};

enum TraversalTypes {WALK, FALL};

struct NeighbourCell {
	/*
	 * Small helper structure representing the neighbour of a cell, as well as its distance from said cell (in gridmap coordinates)
	 */
	public:
		Cell* neighbour;
		float neighbourDist;
		float transferCost;
		TraversalTypes traversal;

		NeighbourCell() {};
		~NeighbourCell() {};
};

class FlowField : public Node {
	GODOT_CLASS(FlowField, Node);

	private:
		//Dictionnary mapping a cell position to its cell
		std::unordered_map<std::array<int, 3>, Cell*, ArrayHasher> allCells;
		std::unordered_map<Cell*, std::vector<NeighbourCell>> cellsNeighbours;

		//Bounding box of the cell coordinates
		std::array<int, 3> bmin;
		std::array<int, 3> bmax;

		FlowFieldGridMap* gridmap = nullptr;
		RayCast* clearanceTestRay = nullptr;
	public:
		FlowField() {};
		~FlowField();

		void _init() {};

		static void _register_methods();

		void _enter_tree();

		/*
		 * @brief Finds the best cost of every cell towards a given target
		 */
		bool createIntegrationField_fromCellsCoords(PoolVector3Array& cellsCoords);
		bool createIntegrationField_fromGlobalPos(PoolVector3Array& targetsPos);
		bool _createIntegrationField(std::vector<Cell*>& targetCells);

		/*
		 * @brief Bakes the direction in each cell towards to goal by looking at the neighbours
		 */
		void bakeCellBestDirections();

		/*
		 * @brief Syntaxic sugar using the below integration field utilities
		 */
		void initializeIntegrationField();

		/*
		 * @brief Resets all the best costs of the field's cells
		 */
		void clearBestCostField();

		/*
		 * @brief Calculates the cost to go from a cell to its neighbour
		 */
		void bakeCellsTransferCosts(); 

		Cell* _findClosestCell_diamond(Vector3 pos); //From pos, returns the first cell it finds in a diamond search
		Cell* _findClosestCell_underneath(Vector3 pos); //From pos, returns the first cell it finds underneath (so under pos.y)

		bool cellWithinBounds(Vector3 cellPos);
		bool cellWithinBounds(std::array<int, 3> cellPos);
		
		/*
		 * @brief Finds the occupied cells a given cell can directly access
		 */
		std::vector<NeighbourCell>* getNeighboringCells(Cell* cell);

		/*
		 * @brief Bakes a cell's directly accessible cells
		 */
		void bakeCellNeighbours(Cell* cell);

		/*
		 * @brief Empties all the currently registered cells of the FlowField
		 */
		void freeCells();

		/*
		 * @brief Creates the flow field cells from a GridMap. Cells will be added from gridmpap->get_used_cells()
		 */
		void cellsFromGridMap(FlowFieldGridMap* newGridMap);

		/*
		 * @brief Considering an agent at pos currentWorldPos, we return the best direction he should take towards the current goal
		 */
		Vector3 getBestDirectionFromPos(Vector3 currentWorldPos);

		/*
		 * @brief Returns the neighbour with the lowest best cost
		 */
		Cell* getBestNeighbour(Cell* cell);

		/*
		 * @brief Utility function to prevent writing over again the same gridmap code
		 */
		void setCellGridMapElement(Cell* cell, int64_t itemIdx);

		/*
		 * @brief Colors the cells of the gridmap according to their distance to the target(s)
		 */
		void buildDebugDistanceField();

		Cell* getCell(std::array<int, 3> cellCoords);

	private:
		/*
		* @brief The usual implementation of integration field building using Dijkstra's algorithm. Only takes one target for now
		*/
		bool _integrationField_Dijkstra(Cell* target);

		/*
		* @brief Building of integration fields using a flood field algorithm, supports many targets (no upper bound tested)
		*/
		bool _integrationField_Lee(std::vector<Cell*>& targets);

		/*
		* @brief Impact a cell has on the direction of an agent situated at global pos worldPos
		*/
		Vector3 getCellInfluenceAtPos(Vector3 worldPos, Cell* cell, float* allWeights);
};

} // namespace godot

template<typename T>
short sign(T val)  {
	return ((val < 0) ? -1 : 1);
};

#endif