
#include "agent.hpp"

using namespace godot;

Agent::Agent() {

}

Agent::~Agent() {
    
}

size_t Simulation::generateFreeId() {
    if (freeIds.empty()) {
        return allAgents.size();
    } 
    else {
        size_t ret = freeIds.front();
        freeIds.pop();
        return ret;
    }
}

Simulation::Simulation() {
    
}

Simulation::~Simulation() {
    
}

void Simulation::createAgent(Spatial* agentOwner) {
    
}

void Simulation::deleteAgent(Spatial* agentOwner) {
    
}