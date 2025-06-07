#include "Node.h"
#include <iostream> // For potential debugging

Node::Node(std::string id, float x, float y, float mass)
    : m_id(id), m_x(x), m_y(y),
      m_vx(0.0f), m_vy(0.0f),      // Initialize velocity
      m_fx(0.0f), m_fy(0.0f),
      m_fxPrev(0.0f), m_fyPrev(0.0f),
      m_mass(mass > 0.001f ? mass : 1.0f), // Ensure mass is positive
      m_isFixed(false) {                 // Initialize as not fixed
}

void Node::addForce(float fx, float fy) {
    if (!m_isFixed) { // Only accumulate forces if not fixed
        m_fx += fx;
        m_fy += fy;
    }
}

void Node::clearForces() {
    m_fxPrev = m_fx; // Store current force as previous for potential blending
    m_fyPrev = m_fy;
    m_fx = 0.0f;
    m_fy = 0.0f;
}

void Node::setPosition(float x, float y) {
    m_x = x;
    m_y = y;
}

float Node::getX() const { return m_x; }
float Node::getY() const { return m_y; }

void Node::setVelocity(float vx, float vy) {
    m_vx = vx;
    m_vy = vy;
}

float Node::getVelocityX() const { return m_vx; }
float Node::getVelocityY() const { return m_vy; }

float Node::getMass() const { return m_mass; }

void Node::setMass(float mass) {
    if (mass > 0.001f) {
        m_mass = mass;
    }
}

bool Node::isFixed() const { return m_isFixed; }

void Node::setFixed(bool fixed) {
    m_isFixed = fixed;
    if (m_isFixed) { // If fixed, stop its motion
        m_vx = 0.0f;
        m_vy = 0.0f;
    }
}

std::string Node::getNodeId() const { return m_id; }

void Node::updatePositionPhysics(float dt, float globalDampingFactor) {
    if (m_isFixed) {
        // Position is set directly by mouse or fixed point, velocity is zeroed by setFixed(true)
        // No need to clear forces here as addForce already checks m_isFixed
        return;
    }

    // --- Optional: Force Blending (can still be used for extra smoothness) ---
    // float blendAlpha = 0.5f; // Weight for previous force
    // float blended_fx = (1.0f - blendAlpha) * m_fx + blendAlpha * m_fxPrev;
    // float blended_fy = (1.0f - blendAlpha) * m_fy + blendAlpha * m_fyPrev;
    // Use m_fx, m_fy directly if not blending:
    float current_fx = m_fx;
    float current_fy = m_fy;
    // --- End Optional: Force Blending ---

    if (current_fx*current_fx + current_fy*current_fy < 100.0f) {
        return;
    }
    // Acceleration (a = F/m)
    float ax = current_fx / m_mass;
    float ay = current_fy / m_mass;

    // Update velocity (v = v0 + a*dt)
    m_vx += ax * dt;
    m_vy += ay * dt;

    // Apply damping to velocity
    m_vx *= globalDampingFactor;
    m_vy *= globalDampingFactor;

    // Optional: Limit maximum speed
    // const float maxSpeed = 50.0f; // Max pixels per second
    // float speed = std::sqrt(m_vx * m_vx + m_vy * m_vy);
    // if (speed > maxSpeed) {
    //     m_vx = (m_vx / speed) * maxSpeed;
    //     m_vy = (m_vy / speed) * maxSpeed;
    // }

    // Update position (p = p0 + v*dt)
    m_x += m_vx * dt;
    m_y += m_vy * dt;

    // Boundary constraints (optional, remove if graph can go out of view)
    // float screenWidth = 800.0f, screenHeight = 600.0f, margin = 10.0f;
    // m_x = std::max(margin, std::min(screenWidth - margin, m_x));
    // m_y = std::max(margin, std::min(screenHeight - margin, m_y));
}