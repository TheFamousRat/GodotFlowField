#ifndef AGENT_HPP
#define AGENT_HPP

#include <algorithm>
#include <cmath>

#include <Godot.hpp>
#include <Area.hpp>

#include "flowfield.h"

#define SIGMOID_ACCURACY 10.0 //Used in functions using an exponential for a threshold. The higher the value, the "cleaner" the cutoff at the threshold (where value go from close to 0 to near infinity)
#define OBSTACLE_EXP_SCALE 10.0 //Same as previous constant, but for obstacles (whether static or dynamic)
#define SPEED_DIST_FAC 2.0

namespace godot
{
    struct Agent {
            Vector3 position; //The last agent position
            Vector3 prefVelocity; //The last targeted agent velocity
            Vector3 velocity; //The real last velocity of the agent
            
            float maxSpeed;
            float radius; //Distance the agent wants to impose between his center and any object
            float obstacleMinDistance; //Distance the agent wants to impose between his center and a static obstacle

            FlowField* flowField;
            
            Agent();
            ~Agent();

            /*
            * @brief Returns info on an agent's target velocity gradient at his current speed (used to steer while avoiding collisions)
            */
            float getCostField();
            Vector3 getGradient();

            Vector3 getClosestObstaclePointDir(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVel);
            float getDirPossibility(Vector3 cellPos, Vector3 cellDir);
            float obstacleCostField(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVel);
            Vector3 obstacleGradient(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVel);

            Vector3 getNeighbourClosestDir(Agent* neighbour, Vector3 proposedVel);
            float neighbourCostField(Agent* neighbour, Vector3 proposedVel);
            Vector3 neighbourGradient(Agent* neighbour, Vector3 proposedVel);

            float speedPrefCostField(Vector3 proposedVel, Vector3 planeNormal);
            Vector3 speedPrefGradient(Vector3 proposedVel, Vector3 planeNormal);

            Area* neighboursDetector; //Used to quickly detect where the agent's neighbours are
    };
} // namespace godot


#endif