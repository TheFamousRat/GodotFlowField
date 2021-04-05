#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <queue>
#include <unordered_map>

#include <Godot.hpp>
#include <Vector3.hpp>
#include <Spatial.hpp>
#include <Area.hpp>

namespace godot {

    class Agent {
        public:
            Vector3 position; //The last agent position
            Vector3 desiredVelocity; //The last targeted agent velocity
            Vector3 velocity; //The real last velocity of the agent

        public:
            Agent();
            ~Agent();

            Area* nearNeighbours; //Used to quickly detect where the agent's neighbours are
    };

    class Simulation : public Node {
	    
        GODOT_CLASS(Simulation, Node);

        private:
            std::unordered_map<Spatial*, Agent*> allAgents;
            std::unordered_map<Area*, Spatial*> allAreas;
        public:
            Simulation();
            ~Simulation();

            static void _register_methods();

            void createAgent(Spatial* agentOwner, Area* neighboursDetector);
            void deleteAgent(Spatial* agentOwner);
            Agent* getAgent(Spatial* agentOwner);

            Vector3 getAgentPosition(Spatial* agentOwner);
            Vector3 getAgentPrefVelocity(Spatial* agentOwner);
            Vector3 getAgentVelocity(Spatial* agentOwner);

            void setAgentPosition(Spatial* agentOwner, Vector3 newPos);
            void setAgentPrefVelocity(Spatial* agentOwner, Vector3 newPrefVel);
            void setAgentVelocity(Spatial* agentOwner, Vector3 newVel);
    };

} //namespace godot

#endif