#include "Agent.hpp"

using namespace godot;

Agent::Agent() {
    maxSpeed = 5.0;
    radius = 0.5;
    obstacleMinDistance = 0.5;
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

Vector3 Agent::getClosestObstaclePointDir(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVel) {
    cellDir = cellDir.normalized();
    Vector3 v_i = flowField->worldToMap(proposedVel);
    Vector3 tangDir = -cellDir.cross(Vector3(0.0,1.0,0.0));
    
    Vector3 d = tangDir;
    Vector3 v = v_i;
    Vector3 To = flowField->worldToMap(cellPos) + 2.0 * cellDir - d/2.0;
    Vector3 C = To -flowField->worldToMap(position);
    
    float dot_dv = d.dot(v);
    float dot_Cd = C.dot(d);
    float dot_Cv = C.dot(v);
    
    float vFun = v.length_squared() - pow(dot_dv, 2.0);
    float uFun_t = (dot_Cv - dot_Cd * dot_dv);
    float uFun_l = (dot_Cv * dot_dv - dot_Cd * v.length_squared());
    
    float l = std::clamp(uFun_l / vFun, 0.0f, 1.0f);
    float t = std::clamp(uFun_t / vFun, 0.0f, 1.0f);
    
    return flowField->mapToWorld(C + d * l - t * v);
}

float Agent::getDirPossibility(Vector3 cellPos, Vector3 cellDir) {
    return 1.0 / (1.0 + exp(-SIGMOID_ACCURACY * cellDir.dot(cellPos - position)));
}

float Agent::obstacleCostField(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVel) {
    float dirPossibility = getDirPossibility(cellPos, cellDir);
    
    Vector3 closestPointDir = getClosestObstaclePointDir(cellPos, cellDir, proposedVel);
    
    float u = proposedVel.length_squared();
    float v = exp(OBSTACLE_EXP_SCALE * (radius + obstacleMinDistance - closestPointDir.length()));
    
    return (dirPossibility * u * v);
}

Vector3 Agent::obstacleGradient(Vector3 cellPos, Vector3 cellDir, Vector3 proposedVel) {
    float dirPossibility = getDirPossibility(cellPos, cellDir);
    
    Vector3 closestPointDir = getClosestObstaclePointDir(cellPos, cellDir, proposedVel);
    
    float u = proposedVel.length_squared();
    Vector3 up = 2.0 * proposedVel;
    float v = exp(OBSTACLE_EXP_SCALE * (radius + obstacleMinDistance - closestPointDir.length()));
    Vector3 vp = OBSTACLE_EXP_SCALE * closestPointDir * v;
    
    return (dirPossibility*(up * v + u * vp));
}

/*
 * Neighbour agents field functions
 */

Vector3 Agent::getNeighbourClosestDir(Agent* neighbour, Vector3 proposedVel) {
    Vector3 relNeighbourPos = neighbour->position - position;
    Vector3 relNeighbourVel = neighbour->velocity - proposedVel;
    
    float t = std::clamp(-relNeighbourPos.dot(relNeighbourVel) / relNeighbourVel.length_squared(), 0.0f, 1.0f);

    return relNeighbourPos + t * relNeighbourVel;
}

float Agent::neighbourCostField(Agent* neighbour, Vector3 proposedVel) {
    Vector3 closestVec = getNeighbourClosestDir(neighbour, proposedVel);
    
    return exp(-OBSTACLE_EXP_SCALE * (closestVec.length() - radius - neighbour->radius));
}

Vector3 Agent::neighbourGradient(Agent* neighbour, Vector3 proposedVel) {
    Vector3 closestVec = getNeighbourClosestDir(neighbour, proposedVel);
    
    return OBSTACLE_EXP_SCALE * closestVec.normalized() * exp(-OBSTACLE_EXP_SCALE * (closestVec.length() - radius - neighbour->radius));
}

/*
 * Speed preference field functions
 */

float Agent::speedPrefCostField(Vector3 proposedVel, Vector3 planeNormal) {
    //Steering the agent towards his requested speed
    float speedPref = SPEED_DIST_FAC * (proposedVel - prefVelocity).length_squared();
    
    //Allowing the agent to take speed below what was requested, but not above
    float velRatio = proposedVel.length_squared() / prefVelocity.length_squared();
    float limitPref = exp(SIGMOID_ACCURACY * (velRatio - 1.0));
    
    //Forcing the agent to stay close to the direction the user gave him
    float dirPref = (proposedVel - prefVelocity * proposedVel.dot(prefVelocity)/prefVelocity.length_squared()).length();
    
    //Forcing the proposedVel to be close to a given proposedVel plane (so the agent doesn't start flying)
    float planePref = exp(pow(proposedVel.dot(planeNormal), 2.0)) - 1.0;
    
    return speedPref + limitPref + dirPref + planePref;
}

Vector3 Agent::speedPrefGradient(Vector3 proposedVel, Vector3 planeNormal) {
    float targetSpeedSq = prefVelocity.length_squared();
    
    Vector3 speedGradient = SPEED_DIST_FAC * 2.0 * (proposedVel - prefVelocity);
    
    float velRatio = proposedVel.length_squared()/prefVelocity.length_squared();
    Vector3 limitGradient = SIGMOID_ACCURACY * 2.0 * (proposedVel/targetSpeedSq) * exp(SIGMOID_ACCURACY * (velRatio - 1.0));
    
    Vector3 dirGradient = 2.0 * (proposedVel - prefVelocity * (prefVelocity.dot(proposedVel)/targetSpeedSq));
    
    Vector3 planeGradient = 2.0 * proposedVel.dot(planeNormal) * planeNormal * (exp(pow(proposedVel.dot(planeNormal), 2.0)) - 1.0);
    
    return speedGradient + limitGradient + dirGradient + planeGradient;
}