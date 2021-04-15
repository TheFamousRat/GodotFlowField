
#include "simulation.hpp"

using namespace godot;

Simulation::Simulation() {
    
}

Simulation::~Simulation() {
    
}

void Simulation::_register_methods() {
    
}

void Simulation::createAgent(Spatial* agentOwner, Area* neighboursDetector) {
    if (allAgents.find(agentOwner) != allAgents.end()) {
        WARN_PRINT("Couldn't create agent : already the simulation\n");
        return;
    }

    //agentOwner->connect("tree_exiting", this, "deleteAgent");

    Agent* newAgent = new Agent();
    Vector3 ownerGlobalPos = agentOwner->get_global_transform().origin;
    newAgent->position = ownerGlobalPos;
    newAgent->neighboursDetector = neighboursDetector;

    allAreas[neighboursDetector] = newAgent;
    allAgents[agentOwner] = newAgent;   
}

void Simulation::deleteAgent(Spatial* agentOwner) {
    if (allAgents.find(agentOwner) == allAgents.end()) {
        WARN_PRINT("Couldn't remove agent : not the simulation\n");
        return;
    }

    /*if (godot_is_instance_valid(agentOwner)) {
        agentOwner->disconnect("tree_exiting", this, "removeAgent");
    }*/

    delete allAgents[agentOwner];
    allAgents.erase(agentOwner);
}

Agent* Simulation::getAgent(Spatial* agentOwner) {
	std::cout << agentOwner->get_name().alloc_c_string() << '\n';
    auto agentIt = allAgents.find(agentOwner);
    if (agentIt != allAgents.end()) {
        return agentIt->second;
    } else {
        ERR_PRINT("No agent attributed to the given owner\n");
    }
}

std::vector<Agent*> Simulation::getAgentNeighbours(Agent* agent) {
    std::vector<Agent*> ret;

    godot::Array nearAreas = agent->neighboursDetector->get_overlapping_areas();

    for (int i(0) ; i < nearAreas.size() ; i++) {
        auto it = allAreas.find(nearAreas[i]);

        if (it != allAreas.end()) {
            ret.push_back(it->second);
        }
    }

    return ret;
}

Vector3 Simulation::getAgentNeighboursGradient(Agent* agent) {
    Vector3 ret = Vector3(0.0, 0.0, 0.0);

    std::vector<Agent*> neighbours = getAgentNeighbours(agent);

    for (auto it = neighbours.begin() ; it != neighbours.end() ; it++) {
        ret += agent->neighbourGradient(*it, agent->velocity);
    }

    return ret;
}

Vector3 Simulation::getAgentObstaclesGradient(Agent* agent) {
    Vector3 ret = Vector3(0.0, 0.0, 0.0);

    std::vector<Cell*> obstacleCells = agent->flowField->getCellsNearToPos(agent->position, 10.0, true);

    for (auto it = obstacleCells.begin() ; it != obstacleCells.end() ; it++) {
        Cell* currentObstCell = (*it);
        
        godot::Array cellDirs = currentObstCell->obstacleDirection;
        for (int i(0) ; i < cellDirs.size() ; i++) {
            Vector3 cellCoords = agent->flowField->mapToWorld(Vector3(currentObstCell->cellPos[0], currentObstCell->cellPos[1], currentObstCell->cellPos[2]));
            ret += agent->obstacleGradient(cellCoords, cellDirs[i], agent->velocity);
        }
    }

    return ret;
}

Vector3 Simulation::getAgentSpeedPrefGradient(Agent* agent) {
    Vector3 planeNormal = agent->prefVelocity.cross(agent->prefVelocity.cross(Vector3(0.0,1.0,0.0)));
    return agent->speedPrefGradient(agent->velocity, planeNormal);
}

Vector3 Simulation::getAgentGradient(Agent* agent) {
    Vector3 agentGradient = Vector3(0.0, 0.0, 0.0);

    agentGradient += getAgentNeighboursGradient(agent);
    agentGradient += getAgentObstaclesGradient(agent);
    agentGradient += getAgentSpeedPrefGradient(agent);

    return agentGradient;
}

void Simulation::doStep(float stepTime) {
    for (auto it = allAgents.begin() ; it != allAgents.end() ; it++) {
        Agent* agent = it->second;
        
    } 
}

Vector3 Simulation::getAgentPosition(Spatial* agentOwner) {
    return getAgent(agentOwner)->position;
}

Vector3 Simulation::getAgentPrefVelocity(Spatial* agentOwner) {
    return getAgent(agentOwner)->prefVelocity;
}

Vector3 Simulation::getAgentVelocity(Spatial* agentOwner) {
    return getAgent(agentOwner)->velocity;
}

void Simulation::setAgentPosition(Spatial* agentOwner, Vector3 newPos) {
    getAgent(agentOwner)->position = newPos;
}

void Simulation::setAgentPrefVelocity(Spatial* agentOwner, Vector3 newPrefVel) {
    getAgent(agentOwner)->prefVelocity = newPrefVel;
}

void Simulation::setAgentVelocity(Spatial* agentOwner, Vector3 newVel) {
    getAgent(agentOwner)->velocity = newVel;
}