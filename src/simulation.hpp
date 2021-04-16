#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <unordered_map>
#include <map>
#include <iostream>

#include <Godot.hpp>
#include <Object.hpp>
#include <Vector3.hpp>
#include <Spatial.hpp>
#include <Area.hpp>
#include <Plane.hpp>

#include "Agent.hpp"
#include "navigation_mesh.h"

namespace godot {

    class Simulation : public Node {
	    
        GODOT_CLASS(Simulation, Node);

        private:
            std::unordered_map<Spatial*, Agent*> allAgents;
            std::unordered_map<Area*, Agent*> allAreas;
        public:
            Simulation();
            ~Simulation();

            static void _register_methods();

            void createAgent(Spatial* agentOwner, Area* neighboursDetector, DetourNavigationMesh* agentNavmesh);
            void deleteAgent(Spatial* agentOwner);
            Agent* getAgent(Spatial* agentOwner);
            std::vector<Agent*> getAgentNeighbours(Agent* agent);

            //Velocity gradient as influenced by nearby agents
            Vector3 getAgentNeighboursGradient(Agent* agent, float stepTime);
            //Velocity gradient as influenced by nearby static obstacles
            Vector3 getAgentObstaclesGradient(Agent* agent);
            //Velocity gradient as influenced by agent targets
            Vector3 getAgentSpeedPrefGradient(Agent* agent);
            //Sum of the three above
            Vector3 getAgentGradient(Agent* agent, float stepTime);

            void doStep(float stepTime);

            Vector3 getAgentPosition(Spatial* agentOwner);
            Vector3 getAgentPrefVelocity(Spatial* agentOwner);
            Vector3 getAgentVelocity(Spatial* agentOwner);

            void setAgentPosition(Spatial* agentOwner, Vector3 newPos);
            void setAgentPrefVelocity(Spatial* agentOwner, Vector3 newPrefVel);
            void setAgentVelocity(Spatial* agentOwner, Vector3 newVel);
    };

} //namespace godot

#endif