#ifndef AGENT_HPP
#define AGENT_HPP

#include <algorithm>
#include <cmath>

#include <Godot.hpp>
#include <Area.hpp>

#define MAX_SPEED_AVOIDANCE 10.0

namespace godot
{
    class Agent {

        public:
            Vector3 position; //The last agent position
            Vector3 prefVelocity; //The last targeted agent velocity
            Vector3 velocity; //The real last velocity of the agent
            
            float maxSpeed;
            float radius;

        public:
            Agent();
            ~Agent();

            float getCostField();
            Vector3 getGradient();

            Vector3 getClosestObstaclePointDir(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVelocity);
            float obstacleCostField(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVelocity);
            Vector3 obstacleGradient(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVelocity);

            Vector3 getNeighbourClosestDir(Agent* neighbour, Vector3 proposedVelocity);
            float neighbourCostField(Agent* neighbour, Vector3 proposedVelocity);
            Vector3 neighbourGradient(Agent* neighbour, Vector3 proposedVelocity);

            float speedPrefCostField(Vector3 proposedVelocity, Vector3 planeNormal);
            Vector3 speedPrefGradient(Vector3 proposedVelocity, Vector3 planeNormal);

            Area* neighboursDetector; //Used to quickly detect where the agent's neighbours are
    };
} // namespace godot


#endif