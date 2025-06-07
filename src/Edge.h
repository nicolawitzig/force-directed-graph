#ifndef EDGE_H
#define EDGE_H

#include "Node.h"

class Edge{
    public:
        Edge(Node* node1, Node* node2, float stiffness = 1.0f, float naturalLength = 100.0f);

        Node* getNode1() const;
        Node* getNode2() const;
        float getStiffness() const;
        float getNaturalLength() const;

        void setNode1(Node* node);
        void setNode2(Node* node);
        void setStiffness(float stiffness);
        void setNaturalLength(float naturalLength);
    
    private:
        Node* m_node1;
        Node* m_node2;
        float m_stiffness;
        float m_naturalLength;

};

#endif