#include "Edge.h"

Edge::Edge(Node* node1, Node* node2, float stiffness, float naturalLength){
    m_node1 = node1;
    m_node2 = node2;

    m_stiffness = stiffness;
    m_naturalLength = naturalLength;
}

Node* Edge::getNode1() const{
        return m_node1;
    };
    
Node* Edge::getNode2() const{
    return m_node2;
}

float Edge::getStiffness() const{
    return m_stiffness;
}

float Edge::getNaturalLength() const{
    return m_naturalLength;
}


void Edge::setNode1(Node* node){
    m_node1 = node;
}

void Edge::setNode2(Node* node){
    m_node2 = node;
}

void Edge::setStiffness(float stiffness){
    m_stiffness = stiffness;
}

void Edge::setNaturalLength(float naturalLength){
    m_naturalLength = naturalLength;
}

