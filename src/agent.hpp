#ifndef AGENT_HPP
#define AGENT_HPP

#include <queue>
#include <unordered_map>

#include <Godot.hpp>
#include <Vector3.hpp>
#include <Spatial.hpp>

namespace godot {

    class Agent {
        public:
            Vector3 position;
            Vector3 desiredVelocity;
            Vector3 velocity;

        public:
            Agent();
            ~Agent();
        
    };

    class Simulation {
        private:
            std::unordered_map<Spatial*, Agent*> allAgents;
            std::queue<size_t> freeIds;
        
            size_t generateFreeId();
        public:
            Simulation();
            ~Simulation();

            void createAgent(Spatial* agentOwner);
            void deleteAgent(Spatial* agentOwner);
    };

} //namespace godot

#endif