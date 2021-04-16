#include "Agent.hpp"

using namespace godot;

Agent::Agent() {
    maxAccel = 200.0;
    maxSpeed = 15.0;
    radius = 1.0;
    obstacleMinDistance = 0.2;
}

Agent::~Agent() {
    
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
    float l = 1.0f;
    float t = 1.0f;

    if (!isRoughlyZero(vFun)) {
        float uFun_t = (dot_Cv - dot_Cd * dot_dv);
        float uFun_l = (dot_Cv * dot_dv - dot_Cd * v.length_squared());
        
        float l = std::clamp(uFun_l / vFun, 0.0f, 1.0f);
        float t = std::clamp(uFun_t / vFun, 0.0f, 1.0f);
    }
    
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
    Vector3 vp = OBSTACLE_EXP_SCALE * closestPointDir.normalized() * v;
    
    return (dirPossibility*(up * v + u * vp));
}

/*
 * Neighbour agents field functions
 */

Vector3 Agent::getNeighbourClosestDir(Agent* neighbour, Vector3 proposedVel, float stepTime) {
    Vector3 relNeighbourPos = neighbour->position - position;
    Vector3 relNeighbourVel = neighbour->velocity - proposedVel;
    
    float relVecLenSq = relNeighbourVel.length_squared();
    float t = stepTime;

    if (!isRoughlyZero(relVecLenSq)) {
        float t = std::clamp(-relNeighbourPos.dot(relNeighbourVel) / relVecLenSq, 0.0f, stepTime);
    }

    return relNeighbourPos + t * relNeighbourVel;
}

float Agent::neighbourCostField(Agent* neighbour, Vector3 proposedVel, float stepTime) {
    Vector3 closestVec = getNeighbourClosestDir(neighbour, proposedVel, stepTime);
    
    return std::exp(OBSTACLE_EXP_SCALE * (radius + neighbour->radius - closestVec.length()));
}

Vector3 Agent::neighbourGradient(Agent* neighbour, Vector3 proposedVel, float stepTime) {
    Vector3 closestVec = getNeighbourClosestDir(neighbour, proposedVel, stepTime);
    
    return 20.0 * OBSTACLE_EXP_SCALE * closestVec.normalized() * std::exp(OBSTACLE_EXP_SCALE * (radius + neighbour->radius - closestVec.length()));
}

/*
 * Speed preference field functions
 */

float Agent::speedPrefCostField(Vector3 proposedVel, Vector3 planeNormal) {
    //Steering the agent towards his requested speed
    float speedPref = SPEED_DIST_FAC * (proposedVel - prefVelocity).length_squared();
    
    //Allowing the agent to take speed below what was requested, but not above
    //float velRatio = proposedVel.length_squared() / prefVelocity.length_squared();
    float limitPref = exp(SIGMOID_ACCURACY * (proposedVel.length() - prefVelocity.length()));
    
    //Forcing the agent to stay close to the direction the user gave him
    //float dirPref = (proposedVel - prefVelocity * proposedVel.dot(prefVelocity)/prefVelocity.length_squared()).length();
    
    //Forcing the proposedVel to be close to a given proposedVel plane (so the agent doesn't start flying)
    //float planePref = exp( (proposedVel.length() - proposedVel.dot(planeNormal)) );
    
    return speedPref + limitPref;// + planePref + dirPref;
}

Vector3 Agent::speedPrefGradient(Vector3 proposedVel, Vector3 planeNormal) {    
    Vector3 speedGradient = SPEED_DIST_FAC * 2.0 * (proposedVel - prefVelocity);
    
    //Vector3 limitGradient = SIGMOID_ACCURACY * proposedVel.normalized() * exp(SIGMOID_ACCURACY * (proposedVel.length() - prefVelocity.length()));
    
    /*Vector3 dirGradient(0.0, 0.0, 0.0);
    float targetSpeedSq = prefVelocity.length_squared();
    if (!isRoughlyZero(targetSpeedSq)) {
        Vector3 dirGradient = 2.0 * (proposedVel - prefVelocity * (prefVelocity.dot(proposedVel)/targetSpeedSq));
    }
    
    Vector3 planeGradient = (proposedVel.normalized() - planeNormal) * exp( (proposedVel.length() - proposedVel.dot(planeNormal)) );*/

    return speedGradient;// + limitGradient;// + dirGradient; + planeGradient;
}