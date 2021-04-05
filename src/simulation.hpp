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

namespace godot {

    class Agent {

        public:
            Vector3 position; //The last agent position
            Vector3 prefVelocity; //The last targeted agent velocity
            Vector3 velocity; //The real last velocity of the agent
            float maxSpeed;

        public:
            Agent();
            ~Agent();

            Area* neighboursDetector; //Used to quickly detect where the agent's neighbours are
    };

    class Simulation : public Node {
	    
        GODOT_CLASS(Simulation, Node);

        private:
            std::unordered_map<Spatial*, Agent*> allAgents;
            std::unordered_map<Area*, Agent*> allAreas;
        public:
            Simulation();
            ~Simulation();

            static void _register_methods();

            void createAgent(Spatial* agentOwner, Area* neighboursDetector);
            void deleteAgent(Spatial* agentOwner);
            Agent* getAgent(Spatial* agentOwner);

            void doStep();

            Vector3 getAgentPosition(Spatial* agentOwner);
            Vector3 getAgentPrefVelocity(Spatial* agentOwner);
            Vector3 getAgentVelocity(Spatial* agentOwner);

            void setAgentPosition(Spatial* agentOwner, Vector3 newPos);
            void setAgentPrefVelocity(Spatial* agentOwner, Vector3 newPrefVel);
            void setAgentVelocity(Spatial* agentOwner, Vector3 newVel);
    };

} //namespace godot

#endif