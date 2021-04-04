#include "flowfield.h"

using namespace godot;

Cell::Cell() {
	baseCost = 1;
	variableCost = 0;
}

Cell::Cell(std::array<int, 3> pos) {
	baseCost = CELL_BASE_COST;
	variableCost = 0;
    cellPos = pos;
}

Cell::~Cell() {

}

void FlowField::_register_methods()
{
	register_method("_enter_tree", &FlowField::_enter_tree);
}

FlowField::~FlowField() {
	
}

void FlowField::_enter_tree() {
	clearanceTestRay = RayCast::_new();
	add_child(clearanceTestRay);
}

bool FlowField::_integrationField_Dijkstra(Cell* target) {
	target->bestCost = 0;
	std::queue<Cell*> cellsToCheck;
	cellsToCheck.push(target);
	
	while (!cellsToCheck.empty()) {
		Cell* currentCell = cellsToCheck.front();
		cellsToCheck.pop();
		
		std::vector<NeighbourCell>* neighborCells = getNeighboringCells(currentCell);
		
		for (auto it = neighborCells->begin() ; it != neighborCells->end() ; it++) {
			Cell* currentNeighbour = it->neighbour;
			
			if (currentNeighbour->baseCost != UNTRAVERSIBLE_CELL_COST) {
				//Approximate cost to go from the neighbour to the cell
				float cellTransferCost = it->transferCost;
				float neighbourBestCostProposition = cellTransferCost + currentCell->bestCost;

				if (neighbourBestCostProposition < currentNeighbour->bestCost && it->traversal != TraversalTypes::FALL) {
					currentNeighbour->bestCost = neighbourBestCostProposition;
					cellsToCheck.push(currentNeighbour);
				}
				else {
					//It's possible the other way around (neighbour to current cell) would be better. We inspect that possibility
					float currentCellBestCostProposition = cellTransferCost + currentNeighbour->bestCost;

					if (currentCellBestCostProposition < currentCell->bestCost) {
						currentCell->bestCost = currentCellBestCostProposition;
						cellsToCheck.push(currentCell);
					}
				}
			}
		}
	}

	return true;
}

bool FlowField::_integrationField_Lee(std::vector<Cell*>& targets) {
	std::deque<Cell*> progressSources;

	for (auto it = targets.begin() ; it != targets.end() ; it++) {
		(*it)->bestCost = 0;
		progressSources.push_back((*it));
	}

	while (!progressSources.empty()) {
		//Removing duplicate expansion sources
		progressSources.resize(std::distance(progressSources.begin(), std::unique(progressSources.begin(), progressSources.end())));

		int currentNumberOfSources = progressSources.size();

		for (int i(0) ; i < currentNumberOfSources ; i++) {
			Cell* currentCell = progressSources[i];

			std::vector<NeighbourCell>* neighborCells = getNeighboringCells(currentCell);

			for (auto jt = neighborCells->begin() ; jt != neighborCells->end() ; jt++) {
				Cell* currentNeighbour = jt->neighbour;

				if (currentNeighbour->baseCost != UNTRAVERSIBLE_CELL_COST) {
					//Approximate cost to go from the neighbour to the cell
					float cellTransferCost = jt->transferCost;
					float neighbourBestCostProposition = currentCell->bestCost + cellTransferCost;

					if (neighbourBestCostProposition < currentNeighbour->bestCost && jt->traversal != TraversalTypes::FALL) {
						currentNeighbour->bestCost = neighbourBestCostProposition;
						progressSources.push_back(currentNeighbour);
					}
					else {
						//It's possible the other way around (neighbour to current cell) would be better. We inspect that possibility
						float currentCellBestCostProposition = cellTransferCost + currentNeighbour->bestCost;

						if (currentCellBestCostProposition < currentCell->bestCost) {
							currentCell->bestCost = currentCellBestCostProposition;
							progressSources.push_back(currentCell);
							break;
						}
					}
				}				
			}
		}

		//Removing the expansion sources just used
		progressSources.erase(progressSources.begin(), progressSources.begin() + currentNumberOfSources);
	}

	return true;
}

bool FlowField::createIntegrationField_fromCellsCoords(PoolVector3Array& cellsCoords) {
	std::vector<Cell*> targetCells;

	for (int i(0) ; i < cellsCoords.size() ; i++) {
		Vector3 currentCellPos = cellsCoords[i];
		targetCells.push_back(allCells[{static_cast<int>(currentCellPos.x), static_cast<int>(currentCellPos.y), static_cast<int>(currentCellPos.z)}]);
	}

	return _createIntegrationField(targetCells);
}

bool FlowField::createIntegrationField_fromGlobalPos(PoolVector3Array& targetsPos) {
	std::vector<Cell*> targetCells;

	for (int i(0) ; i < targetsPos.size() ; i++) {
		targetCells.push_back(_findClosestCell_diamond(targetsPos[i]));
	}

	return _createIntegrationField(targetCells);
}

bool FlowField::_createIntegrationField(std::vector<Cell*>& targetCells) {
	initializeIntegrationField();

	if (targetCells.size() == 1) {
		//Dijkstra is faster than Lee, but can't work with multiple targets for now
		_integrationField_Dijkstra(targetCells[0]);
	}
	else {
		_integrationField_Lee(targetCells);
	}

	bakeCellBestDirections();

	return true;//TODO : find possible caveats where the building could fail

}

void FlowField::bakeCellBestDirections() {
	for (auto it = allCells.begin() ; it != allCells.end() ; it++) {
		Cell* currentCell = it->second;

		if (currentCell->bestCost == 0) {
			currentCell->bestDirection = Vector3(0.0, 0.0, 0.0);
		}
		else {
			Cell* bestNeighbour = getBestNeighbour(currentCell);
			currentCell->bestDirection = Vector3();

			if (bestNeighbour == nullptr) {
				continue;
			}

			Vector3 cellWorldPos = currentCell->getWorldPos(gridmap);
			Vector3 neighbourWorldPos = bestNeighbour->getWorldPos(gridmap);
			currentCell->bestDirection = (neighbourWorldPos - cellWorldPos).normalized();
		}
	}
}

void FlowField::initializeIntegrationField() {
	clearBestCostField();
	bakeCellsTransferCosts();
}

void FlowField::clearBestCostField() {
	for (auto it = allCells.begin() ; it != allCells.end() ; it++) {
		// We set all the best costs to a really high value, so it's clear to the parsers that this isn't the real best cost of the cell
		it->second->bestCost = std::numeric_limits<int>::max();
	}
}

void FlowField::bakeCellsTransferCosts() {
	for (auto it = allCells.begin() ; it != allCells.end() ; it++) {
		Cell* cell = it->second;

		std::vector<NeighbourCell>* neighborCells = getNeighboringCells(cell);

		for (auto jt = neighborCells->begin() ; jt != neighborCells->end() ; jt++) {
			jt->transferCost = (jt->neighbourDist/2.0) * (
														(cell->baseCost + cell->variableCost) + 
														(jt->neighbour->baseCost + jt->neighbour->variableCost));
		}
	}
}

Cell* FlowField::_findClosestCell_diamond(Vector3 pos) {
	Vector3 mapGridCell = gridmap->world_to_map(pos);

	std::array<int, 3> currentCellPos({
								static_cast<int>(mapGridCell.x), 
								static_cast<int>(mapGridCell.y), 
								static_cast<int>(mapGridCell.z)
								});

	auto foundCellIt = allCells.end();
	int r = 0;
	std::vector<Cell*> foundCells;

	while (foundCells.empty()) {
		//We iterate over the neighbouring cells in a "diamond" fashion
		for (int x(std::max(-r, bmin[0] - static_cast<int>(mapGridCell.x))) ; x <= std::min(r, bmax[0] - static_cast<int>(mapGridCell.x)) ; x++) {

			int yBounds = r - std::abs(x);
			for (int y(std::max(-yBounds, bmin[1] - static_cast<int>(mapGridCell.y))) ; y <= std::min(yBounds, bmax[1] - static_cast<int>(mapGridCell.y)) ; y++) {

				int zBounds = yBounds - std::abs(y);
				for (int z(std::max(-zBounds, bmin[2] - static_cast<int>(mapGridCell.z))) ; z <= std::min(zBounds, bmax[2] - static_cast<int>(mapGridCell.z)) ; z+=2*zBounds) {

					currentCellPos[0] = mapGridCell.x + x;
					currentCellPos[1] = mapGridCell.y + y;
					currentCellPos[2] = mapGridCell.z + z;
					

					foundCellIt = allCells.find(currentCellPos);
					if (foundCellIt != allCells.end()) {
						foundCells.push_back(foundCellIt->second);
					}
						

					if (z == 0) {
						break;
					}
				}
			}
		}

		r++;
	}

	//Once we have found all the occupied cells within a close radius, with return the closest one from the given position
	Cell* currentBestCell = nullptr;
	float currentBestCellDist = INFINITY;
	for (auto it = foundCells.begin() ; it != foundCells.end() ; it++) {
		Cell* currentCell = *it;
		float currentCellDist = (pos - gridmap->map_to_world(currentCell->cellPos[0], currentCell->cellPos[1], currentCell->cellPos[2])).length_squared();

		if (currentCellDist < currentBestCellDist) {
			currentBestCellDist = currentCellDist;
			currentBestCell = currentCell;
		}
	}

	return currentBestCell;
}


Cell* FlowField::_findClosestCell_underneath(Vector3 pos) {
	Vector3 mapGridCell = gridmap->world_to_map(pos);

	std::array<int, 3> currentCellPos({
								static_cast<int>(mapGridCell.x), 
								static_cast<int>(mapGridCell.y), 
								static_cast<int>(mapGridCell.z)
								});

	auto foundCellIt = allCells.end();

	int r = 0;

	while (true) {

		for (int x(std::max(-r, bmin[0] - static_cast<int>(mapGridCell.x))) ; x <= std::min(r, bmax[0] - static_cast<int>(mapGridCell.x)) ; x++) {

			int zBounds = r - std::abs(x);
			for (int z(std::max(-zBounds, bmin[2] - static_cast<int>(mapGridCell.z))) ; z <= std::min(zBounds, bmax[2] - static_cast<int>(mapGridCell.z)) ; z++) {
				/*
				* x and z define the horizontal offset of a search "column", offset from the mapGridCell x and z
				*/
				for (int y(0) ; y <= mapGridCell.y - bmin[1] ; y++) {
					currentCellPos[0] = mapGridCell.x + x;
					currentCellPos[1] = mapGridCell.y - y;
					currentCellPos[2] = mapGridCell.z + z;

					foundCellIt = allCells.find(currentCellPos);
					if (foundCellIt != allCells.end()) {
						return foundCellIt->second;
					}
				}
			}
		}
		
		r++;
	}

	ERR_PRINT("Couldn't find a cell under the given position");
}

bool FlowField::cellWithinBounds(Vector3 cellPos) {
	for (int i(0) ; i < 3 ; i++) {
		if (!(cellPos[i] >= bmin[i] && cellPos[i] <= bmax[i]))
			return false;
	}	

	return true;
}

bool FlowField::cellWithinBounds(std::array<int, 3> cellPos) {
	
	return cellWithinBounds(Vector3(cellPos[0], cellPos[1], cellPos[2]));
}

std::vector<NeighbourCell>* FlowField::getNeighboringCells(Cell* cell) {
	auto retCandidate = cellsNeighbours.find(cell);

	//We first check if we have baked the neighbours already
	if (retCandidate == cellsNeighbours.end()) {
		bakeCellNeighbours(cell);
		return &(cellsNeighbours.find(cell)->second);
	}
	else {
		return &(retCandidate->second);
	}
}

void FlowField::bakeCellNeighbours(Cell* cell) {
	//First cleaning previously baked neighbours
	auto it = cellsNeighbours.find(cell);
	if (it != cellsNeighbours.end()) {
		cellsNeighbours.erase(it);
	}

	std::vector<NeighbourCell> neighbours;
	Vector3 cellPos = Vector3(cell->cellPos[0], cell->cellPos[1], cell->cellPos[2]);
	std::array<int, 3> cellPos_int;

	for (int x=-1 ; x <= 1 ; x++) {
		for (int z=-1 ; z <= 1 ; z++) {
			if (!(x == 0 && z == 0)) {
				bool currentSideUncovered = true;

				for (int y=1 ; y >= -10 ; y--) {	
					//If the current cell isn't the center cell...
					cellPos_int[0] = x + cellPos.x;
					cellPos_int[1] = y + cellPos.y;
					cellPos_int[2] = z + cellPos.z;
					auto cellFound = allCells.find(cellPos_int);

					if (cellFound != allCells.end()) {
						//...And if it exists within the flow field, we add it
						if (y < 0) {
							clearanceTestRay->set_translation(gridmap->map_to_world(cellPos_int[0], cellPos.y, cellPos_int[2]));
							clearanceTestRay->set_cast_to(Vector3(0.0, gridmap->get_cell_size().y * static_cast<float>(y+1), 0.0));
							clearanceTestRay->force_raycast_update();

							if (clearanceTestRay->is_colliding()) {
								break;
							}
						}
						
						NeighbourCell newNeighbour;
						newNeighbour.neighbour = cellFound->second;
						newNeighbour.neighbourDist = std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0) + std::pow(std::abs(z), 2.0));

						if (y >= -1) {
							newNeighbour.traversal = TraversalTypes::WALK;
						}
						else {
							newNeighbour.traversal = TraversalTypes::FALL;
						}

						neighbours.push_back(newNeighbour);

						currentSideUncovered = false;
						break;
					}
				}

				if (currentSideUncovered && (x == 0 || z == 0)) {
					//The current cell was identified as ledge cell
					//setCellGridMapElement(cell, 10);
					cell->baseCost = LEDGE_COST;
				}

			}
		}
	}

	cellsNeighbours[cell] = std::vector<NeighbourCell>(neighbours);
}

void FlowField::freeCells() {
	for (auto it = allCells.begin() ; it != allCells.end() ; it++) {
		delete it->second;
	}	

	allCells.clear();
	cellsNeighbours.clear();

	bmin = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
	bmax = {std::numeric_limits<int>::min(), std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};
}

void FlowField::cellsFromGridMap(FlowFieldGridMap* newGridMap) {
	//Resetting the flow field
	freeCells();

	gridmap = newGridMap;

	//We add new cells based on GridMap occupancy
	Array usedCells = newGridMap->get_used_cells();

	for (int i(0) ; i < usedCells.size() ; i++) {
		Vector3 cellPos = usedCells[i];
		std::array<int, 3> cellPos_int = {static_cast<int>(cellPos.x), static_cast<int>(cellPos.y), static_cast<int>(cellPos.z)};

		allCells[cellPos_int] = new Cell(cellPos_int);

		//Setting the cells bounding box
		for (int j(0) ; j < 3 ; j++) {
			if (cellPos_int[j] < bmin[j]) 
				bmin[j] = cellPos_int[j];

			if (cellPos_int[j] > bmax[j]) 
				bmax[j] = cellPos_int[j];
		}
	}

	//We then bake the neighbours of the cells
	for (auto it = allCells.begin() ; it != allCells.end() ; it++) {
		bakeCellNeighbours(it->second);
	}
}

Vector3 FlowField::getCellInfluenceAtPos(Vector3 worldPos, Cell* cell, float* allWeights) {
	Vector3 cellCenter = gridmap->map_to_world(cell->cellPos[0], cell->cellPos[1], cell->cellPos[2]);
	Vector3 centeredCellPos = (worldPos - cellCenter) * 2.0f / gridmap->get_cell_size().x;

	std::array<float, 3> alphaProposals;
	for (int i(0) ; i < 3 ; i++) {
		alphaProposals[i] = std::abs((sign(cell->bestDirection[i]) - centeredCellPos[i]) / cell->bestDirection[i]);
	}

	float alpha = *(std::min_element(alphaProposals.begin(), alphaProposals.end()));

	float distWeight = 1.0f / (worldPos - cellCenter).length_squared();//std::exp(-alpha*0.05f);
	*allWeights += distWeight;

	return cell->bestDirection * distWeight;
}

Vector3 FlowField::getBestDirectionFromPos(Vector3 currentWorldPos) {
	Cell* closestCell = _findClosestCell_diamond(currentWorldPos);

	return closestCell->bestDirection;
}

Cell* FlowField::getBestNeighbour(Cell* cell) {
	std::vector<NeighbourCell>* neighbours = getNeighboringCells(cell);

	if (neighbours->empty())
		return nullptr;
	
	Cell* closestCell = neighbours->begin()->neighbour;
	for (auto it = neighbours->begin()+1 ; it != neighbours->end() ; it++) {
		Cell* currentNeighbourCell = it->neighbour;

		if (currentNeighbourCell->bestCost < closestCell->bestCost)
			closestCell = currentNeighbourCell;
	}

	return closestCell;
}

void FlowField::setCellGridMapElement(Cell* cell, int64_t itemIdx) {
	gridmap->set_cell_item(
						cell->cellPos[0], 
						cell->cellPos[1], 
						cell->cellPos[2], 
						itemIdx);
}

void FlowField::buildDebugDistanceField() {
	for (auto it = allCells.begin() ; it != allCells.end() ; it++) {
		setCellGridMapElement(it->second, static_cast<int>(std::round(it->second->bestCost)));
	}
}

Cell* FlowField::getCell(std::array<int, 3> cellCoords) {
	return allCells[cellCoords];
}