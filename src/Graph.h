#ifndef GRAPH_H
#define GRAPH_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <memory> // For std::unique_ptr
#include "Edge.h"
#include "Node.h"
#include <Eigen/Dense> 

class Graph {
public:
    Graph(std::vector<std::unique_ptr<Node>> nodes, std::vector<std::unique_ptr<Edge>> edges);
    Graph(int n, int m, float c, bool spectralInit); // c is for adjusting m_k 

    // Return const references to avoid copying and allow iteration
    const std::vector<std::unique_ptr<Node>>& getNodes() const;
    const std::vector<std::unique_ptr<Edge>>& getEdges() const;
    
    void renderGraph(sf::RenderWindow& window);

    void step(float dt); 
    void resetSimulation();

    void setRepulsionStrength(float repulsionStrength);
    void setAttractionStrength(float attractionStrength);
    void setK(float k); // allows to directly set the optimal distance

    void calculateRepulsiveForces();
    void calculateAttractiveForces();
    void updatePositions(float dt); 

private:
    std::vector<std::unique_ptr<Node>> m_nodes;
    std::vector<std::unique_ptr<Edge>> m_edges;
    Eigen::MatrixXd m_adjacencyMatrix;
    Eigen::MatrixXd m_degreeMatrix;

    float m_area = 800 * 600;           
    float m_k;              // frchterman reingold optimal distance between nodes

    float m_repulsionStrength = 1.0f;
    float m_attractionStrength = 1.0f;

    float m_globalDampingFactor = 0.95f; //global damping for velocities
};

#endif