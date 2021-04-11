#include "Agent.hpp"

using namespace godot;

Agent::Agent() {
    maxSpeed = 5.0;
    radius = 0.5;
}

Agent::~Agent() {
    
}

float Agent::getCostField() {

}

Vector3 Agent::getGradient() {

}

/*
 * Obstacle field functions
 */

Vector3 Agent::getClosestObstaclePointDir(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVelocity) {
    Vector3 v_i = proposedVelocity;
    Vector3 tangDir = -cellDir.cross(Vector3(0.0,1.0,0.0));
    
    float w = obstacleWidth;
    float normWSq = pow(w, 2.0);
    Vector3 d = tangDir * w;
    Vector3 v = v_i;
    Vector3 To = cellPos + w * cellDir - d/2.0;
    Vector3 C = To - position;
    
    float dot_dv = d.dot(v);
    float dot_Cd = C.dot(d);
    float dot_Cv = C.dot(v);
    
    float vFun = (normWSq * v.length_squared()) - pow(dot_dv, 2.0);
    float uFun_t = (normWSq*dot_Cv - dot_Cd * dot_dv);
    float uFun_l = (dot_Cv*dot_dv - dot_Cd * v.length_squared());

    float l = std::clamp(uFun_l / vFun, 0.0f, 1.0f);
    float t = std::clamp(uFun_t / vFun, 0.0f, 1.0f);
    
    return C + d * l - v * t;
}

float Agent::obstacleCostField(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVelocity) {
    float dirPossibility = 1.0/(1.0 + exp(-2.0*cellDir.dot(proposedVelocity)));
    
    float ki = 1.0;
    Vector3 closestPointDir = getClosestObstaclePointDir(cellPos, cellDir, proposedVelocity);
    
    return dirPossibility * exp(-ki * closestPointDir.length());
}

Vector3 Agent::obstacleGradient(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVelocity) {
    float dirPossibility = 1.0/(1.0 + exp(-2.0*cellDir.dot(proposedVelocity)));
    
    float ki = 1.0;
    Vector3 closestPointDir = getClosestObstaclePointDir(cellPos, cellDir, proposedVelocity);
    
    return 2.0 * dirPossibility * exp(-(closestPointDir.length() - (ki + radius))) * (closestPointDir + cellDir * dirPossibility);
}

/*
 * Neighbour agents field functions
 */

Vector3 Agent::getNeighbourClosestDir(Agent* neighbour, Vector3 proposedVelocity) {
    Vector3 v_i = proposedVelocity;
    
    Vector3 relNeighbourPos = neighbour->position - position;
    Vector3 relNeighbourVel = neighbour->velocity - v_i;
    
    float t = std::clamp(-relNeighbourPos.dot(relNeighbourVel) / relNeighbourVel.length_squared(), 0.0f, 1.0f);

    return relNeighbourPos + t * relNeighbourVel;
}

float Agent::neighbourCostField(Agent* neighbour, Vector3 proposedVelocity) {
    Vector3 closestVec = getNeighbourClosestDir(neighbour, proposedVelocity);
    
    return exp(-(closestVec.length() - radius - neighbour->radius));
}

Vector3 Agent::neighbourGradient(Agent* neighbour, Vector3 proposedVelocity) {
    Vector3 closestVec = getNeighbourClosestDir(neighbour, proposedVelocity);
    
    return 2.0 * closestVec * exp(-(closestVec.length() - radius - neighbour->radius));
}

/*
 * Speed preference field functions
 */

float Agent::speedPrefCostField(Vector3 proposedVelocity, Vector3 planeNormal) {
    //Steering the agent towards his requested speed
    float speedPref = (proposedVelocity - prefVelocity).length();
    
    //Allowing the agent to take speed below what was requested, but not above
    float velRatio = proposedVelocity.length_squared()/prefVelocity.length_squared();
    float limitPref = exp(MAX_SPEED_AVOIDANCE*(velRatio - 1.0));
    
    //Forcing the agent to stay close to the direction the user gave him
    float dirPref = (proposedVelocity - prefVelocity * (proposedVelocity.dot(prefVelocity))/prefVelocity.length_squared()).length();
    
    //Forcing the velocity to be close to a given velocity plane (so the agent doesn't start flying)
    float planePref = exp(pow(proposedVelocity.dot(planeNormal), 2.0)) - 1.0;
    
    return speedPref + limitPref + dirPref;
}

Vector3 Agent::speedPrefGradient(Vector3 proposedVelocity, Vector3 planeNormal) {
    Vector3 speedGradient = 0.05 * 2.0 * (proposedVelocity - prefVelocity);
    
    float velRatio = proposedVelocity.length_squared()/prefVelocity.length_squared();
    Vector3 limitGradient = MAX_SPEED_AVOIDANCE * 2.0 * (proposedVelocity/prefVelocity.length_squared()) * exp(MAX_SPEED_AVOIDANCE*(velRatio - 1.0));
    
    Vector3 dirGradient = 2.0 * (proposedVelocity - prefVelocity * (proposedVelocity.dot(prefVelocity)/prefVelocity.length_squared()));
    
    Vector3 planeGradient = 2.0 * proposedVelocity.dot(planeNormal) * planeNormal * (exp(pow(proposedVelocity.dot(planeNormal), 2.0)) - 1.0);
    
    return speedGradient + limitGradient + dirGradient + planeGradient;
}