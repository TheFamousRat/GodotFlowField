
#include "simulation.hpp"

using namespace godot;

Agent::Agent() {

}

Agent::~Agent() {
    
}

Simulation::Simulation() {
    
}

Simulation::~Simulation() {
    
}

void Simulation::_register_methods() {
    
}

void Simulation::createAgent(Spatial* agentOwner, Area* neighboursDetector) {
    if (allAgents.find(agentOwner) != allAgents.end()) {
        WARN_PRINT("Couldn't create agent : already in the simulation\n");
        return;
    }

    agentOwner->connect("tree_exiting", this, "removeAgent");

    Agent* newAgent = new Agent();
    Vector3 ownerGlobalPos = agentOwner->get_global_transform().origin;
    newAgent->position = ownerGlobalPos;
    allAgents[agentOwner] = newAgent;
}

void Simulation::deleteAgent(Spatial* agentOwner) {
    if (allAgents.find(agentOwner) == allAgents.end()) {
        WARN_PRINT("Couldn't remove agent : not in the simulation\n");
        return;
    }

    if (godot_is_instance_valid(agentOwner)) {
        agentOwner->disconnect("tree_exiting", this, "removeAgent");
    }

    delete allAgents[agentOwner];
    allAgents.erase(agentOwner);
}

Agent* Simulation::getAgent(Spatial* agentOwner) {
    auto agentIt = allAgents.find(agentOwner);
    if (agentIt != allAgents.end()) {
        return agentIt->second;
    } else {
        ERR_PRINT("No agent attributed to the given owner\n");
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