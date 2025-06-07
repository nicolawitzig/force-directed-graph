#ifndef NODE_H
#define NODE_H

#include <string>
#include <cmath>
#include <algorithm> // For std::copysign if used for maxStep capping

class Node {
public:
    Node(std::string id, float x = 0.0f, float y = 0.0f, float mass = 1.0f);

    void addForce(float fx, float fy);
    void clearForces(); // reset m_fx, m_fy and update m_fxPrev, m_fyPrev

    void setPosition(float x, float y);
    float getX() const;
    float getY() const;

    void setVelocity(float vx, float vy);
    float getVelocityX() const;
    float getVelocityY() const;

    float getMass() const;
    void setMass(float mass);

    bool isFixed() const;
    void setFixed(bool fixed);

    std::string getNodeId() const;

    void updatePositionPhysics(float dt, float globalDampingFactor);

    float m_fxPrev;
    float m_fyPrev;

private:
    std::string m_id;
    float m_x, m_y;       // pos
    float m_vx, m_vy;     // velocity
    float m_fx, m_fy;     // forces
    float m_mass;
    bool m_isFixed;  
};

#endif