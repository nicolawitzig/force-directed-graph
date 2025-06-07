#include "Graph.h"
#include <cmath>
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <set>



Graph::Graph(std::vector<std::unique_ptr<Node>> nodes, std::vector<std::unique_ptr<Edge>> edges)
    : m_area(800 * 600){
    m_nodes = std::move(nodes); 
    m_edges = std::move(edges); 

    int n = m_nodes.size();

    if (n > 0) {
        size_t num_nodes = m_nodes.size();
        m_k = 1.0f * std::sqrt(m_area / static_cast<float>(num_nodes)); // Assuming c=1.0f
        std::cout << "Optimal distance (m_k) for pre-defined graph: " << m_k << std::endl;

        
        std::map<Node*, size_t> node_to_index;
        for (size_t i = 0; i < num_nodes; ++i) {
            if (m_nodes[i]) { 
                node_to_index[m_nodes[i].get()] = i;
            }
        }

        m_adjacencyMatrix = Eigen::MatrixXd::Zero(n, n);
        m_degreeMatrix = Eigen::MatrixXd::Zero(n, n);

        for (const auto& edge : m_edges) {
            if (edge && edge->getNode1() && edge->getNode2()) { 
                auto it1 = node_to_index.find(edge->getNode1());
                auto it2 = node_to_index.find(edge->getNode2());
                if (it1 != node_to_index.end() && it2 != node_to_index.end()) {
                    size_t index1 = it1->second;
                    size_t index2 = it2->second;
                    m_adjacencyMatrix(index1, index2) = 1;
                    m_adjacencyMatrix(index2,index1) = 1;
                    m_degreeMatrix(index1, index1) += 1;
                    m_degreeMatrix(index2,index2) += 1;
                }
            }
        }
    }
}

Graph::Graph(int n, int m, float c, bool spectralInit) : m_area(800 * 600) {
    if (n > 0) {
        m_k = c * std::sqrt(m_area / static_cast<float>(n));
    } else {
        m_k = c * std::sqrt(m_area);
    }
    std::cout << "Optimal distance (m_k): " << m_k << std::endl;
    
    // Create nodes with initial random positions
    for(int i = 0; i < n; i++){
        float x_pos = 50 + (rand() % 700); 
        float y_pos = 50 + (rand() % 500);
        m_nodes.push_back(std::make_unique<Node>("Node"+std::to_string(i), x_pos, y_pos, 1.0f));
    }
    
    // random edge generation
    std::set<std::pair<int, int>> edges_added;
    for(int i = 0; i < m && n > 1; i++){
        int first_idx, second_idx;
        do {
            first_idx = rand() % n;
            second_idx = rand() % n;
        } while(first_idx == second_idx || 
                edges_added.count({std::min(first_idx, second_idx), std::max(first_idx, second_idx)}));
        
        edges_added.insert({std::min(first_idx, second_idx), std::max(first_idx, second_idx)});
        m_edges.push_back(std::make_unique<Edge>(m_nodes[first_idx].get(), m_nodes[second_idx].get()));
    }

    // Apply spectral layout only if we have enough nodes and spectralInit is true
    if (n >= 3 && spectralInit) {
        std::map<Node*, size_t> node_to_index;
        for (size_t i = 0; i < m_nodes.size(); ++i) {
            if (m_nodes[i]) {
                node_to_index[m_nodes[i].get()] = i;
            }
        }
        
        m_adjacencyMatrix = Eigen::MatrixXd::Zero(n, n);
        m_degreeMatrix = Eigen::MatrixXd::Zero(n, n);
        
        for (const auto& edge : m_edges) {
            if (edge && edge->getNode1() && edge->getNode2()) {
                auto it1 = node_to_index.find(edge->getNode1());
                auto it2 = node_to_index.find(edge->getNode2());
                if (it1 != node_to_index.end() && it2 != node_to_index.end()) {
                    size_t index1 = it1->second;
                    size_t index2 = it2->second;
                    m_adjacencyMatrix(index1, index2) = 1;
                    m_adjacencyMatrix(index2, index1) = 1;
                    m_degreeMatrix(index1, index1) += 1;
                    m_degreeMatrix(index2, index2) += 1;
                }
            }
        }

        Eigen::MatrixXd laplacian = m_degreeMatrix - m_adjacencyMatrix;

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(laplacian);
        if (solver.info() != Eigen::Success) {
            std::cerr << "Eigen decomposition failed!" << std::endl;
            return;
        }

        Eigen::VectorXd eigenvalues = solver.eigenvalues();
        Eigen::MatrixXd eigenvectors = solver.eigenvectors();

        // Debug output
        std::cout << "First 3 eigenvalues: ";
        for (int i = 0; i < std::min(3, static_cast<int>(eigenvalues.size())); ++i) {
            std::cout << eigenvalues(i) << " ";
        }
        std::cout << std::endl;

       
        if (eigenvectors.cols() >= 3) {
            // EV debug
            std::cout << "\nEigenvector analysis:" << std::endl;
            std::cout << "Second eigenvector (x): " << eigenvectors.col(1).transpose() << std::endl;
            std::cout << "Third eigenvector (y): " << eigenvectors.col(2).transpose() << std::endl;
            
            double min_x = eigenvectors.col(1).minCoeff();
            double max_x = eigenvectors.col(1).maxCoeff();
            double min_y = eigenvectors.col(2).minCoeff();
            double max_y = eigenvectors.col(2).maxCoeff();
            
            std::cout << "X range: [" << min_x << ", " << max_x << "] = " << (max_x - min_x) << std::endl;
            std::cout << "Y range: [" << min_y << ", " << max_y << "] = " << (max_y - min_y) << std::endl;
            
            double range_x = max_x - min_x;
            double range_y = max_y - min_y;
            
            // Check for degenerate ranges
            if (range_x < 1e-10) {
                std::cout << "ERROR: X range is essentially zero - all nodes will have same x-coordinate!" << std::endl;
                return;
            }
            if (range_y < 1e-10) {
                std::cout << "ERROR: Y range is essentially zero - all nodes will have same y-coordinate!" << std::endl;
                return;
            }
            
            double scale_x = 300.0 / range_x;
            double scale_y = 200.0 / range_y;
            
            std::cout << "Scaling factors: x=" << scale_x << ", y=" << scale_y << std::endl;
            
            // Apply positions WITHOUT random offset first to see the pure spectral layout
            for (int i = 0; i < n; i++) {
                float x_pos = static_cast<float>((eigenvectors(i, 1) - min_x) * scale_x) + 100;
                float y_pos = static_cast<float>((eigenvectors(i, 2) - min_y) * scale_y) + 100;
                
                std::cout << "Node " << i << ": eigenvec=(" << eigenvectors(i, 1) << ", " << eigenvectors(i, 2) 
                        << ") -> pos=(" << x_pos << ", " << y_pos << ")" << std::endl;
                
                m_nodes[i]->setPosition(x_pos, y_pos);
            }
            std::cout << "Spectral layout applied" << std::endl;
        } else {
            std::cout << "Not enough eigenvectors for 2D spectral layout" << std::endl;
        }
    } else {
        std::cout << "No spectral layout" << std::endl;
    }
}

const std::vector<std::unique_ptr<Node>>& Graph::getNodes() const {
    return m_nodes;
}

const std::vector<std::unique_ptr<Edge>>& Graph::getEdges() const {
    return m_edges;
}

void Graph::renderGraph(sf::RenderWindow& window){
    float radius = 10.0f; // Default node radius
    
    for(const auto& node : m_nodes){    
        sf::CircleShape nodeShape(radius);
        
        nodeShape.setPosition(sf::Vector2f(node->getX() - radius, node->getY() - radius)); 
        nodeShape.setFillColor(sf::Color::Cyan);
        
        if (node->isFixed()) {
            nodeShape.setFillColor(sf::Color::Red);
        }
        window.draw(nodeShape);
    }

    for(const auto& edge : m_edges){
        if (!edge->getNode1() || !edge->getNode2()) continue;

        float x1 = edge->getNode1()->getX();
        float y1 = edge->getNode1()->getY();
        float x2 = edge->getNode2()->getX();
        float y2 = edge->getNode2()->getY();
        
        float dx = x2 - x1;
        float dy = y2 - y1;
        float length = std::sqrt(dx*dx + dy*dy);
        
        if (length < 0.1f) continue; // Avoid issues with zero-length lines when nodas are on top of each other

        sf::Angle angle = sf::radians(std::atan2(dy, dx));
        
        sf::RectangleShape line(sf::Vector2f(length, 1.0f)); 
        line.setOrigin(sf::Vector2f(0.0f, 0.5f));
        line.setPosition(sf::Vector2f(x1, y1));
        line.setRotation(angle);
        line.setFillColor(sf::Color(100, 100, 100));
        
        window.draw(line);
    }
}

void Graph::step(float dt){

    for (auto& node : m_nodes){
        node->clearForces();
    }

    calculateAttractiveForces();
    calculateRepulsiveForces();
    updatePositions(dt);
}

void Graph::resetSimulation(){
    std::cout << "Resetting simulation." << std::endl;
    
    for (auto& node_ptr : m_nodes) {
        node_ptr->clearForces();
        node_ptr->setVelocity(0.0f, 0.0f); 
        node_ptr->setFixed(false);

        float x_pos = 50 + (rand() % 700);
        float y_pos = 50 + (rand() % 500);
        node_ptr->setPosition(x_pos, y_pos);
    }
}

void Graph::setRepulsionStrength(float repulsionStrength) {
    m_repulsionStrength = repulsionStrength;
}
void Graph::setAttractionStrength(float attractionStrength) {
    m_attractionStrength = attractionStrength;
}
void Graph::setK(float k) {
    if (k > 0) m_k = k;
}


void Graph::calculateRepulsiveForces() {
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        for (size_t j = i + 1; j < m_nodes.size(); ++j) {
            Node* node_i = m_nodes[i].get();
            Node* node_j = m_nodes[j].get();

            if (node_i->isFixed() && node_j->isFixed()) continue; // Optimization: no force between two fixed nodes

            float dx = node_j->getX() - node_i->getX();
            float dy = node_j->getY() - node_i->getY();
            float distanceSquared = dx * dx + dy * dy;

            // avoid division by zero when nodes are placed on top of eachother by spectral layout
            if (distanceSquared < 0.0001f) distanceSquared = 0.0001f;
            
            
            float distance = std::sqrt(distanceSquared);

            // avoid nodes drifting off infinitely
            if (distanceSquared > 400.0f) continue; 
            
            // fruchterman reingold repulsive force
            float force = (m_k * m_k) / distance * m_repulsionStrength;
            
            float fx_component = (dx / distance) * force;
            float fy_component = (dy / distance) * force;
            
            node_i->addForce(-fx_component, -fy_component);
            node_j->addForce(fx_component, fy_component);
        }
    }
}

void Graph::calculateAttractiveForces() {

    for (auto& edge_ptr : m_edges) {
        Node* n1 = edge_ptr->getNode1();
        Node* n2 = edge_ptr->getNode2();

        // nullptr check
        if (!n1 || !n2) continue;
        
        // skip if both nodes are fixed
        if (n1->isFixed() && n2->isFixed()) continue;

        float dx = n2->getX() - n1->getX();
        float dy = n2->getY() - n1->getY();
        float distanceSquared = dx * dx + dy * dy;

        // avoid issues if nodes are on top of each other
        if (distanceSquared < 0.0001f) distanceSquared = 0.0001f; 

        float distance = std::sqrt(distanceSquared);
        
        // fruchterman reingold attractive force
        float force = (distance * distance) / m_k * m_attractionStrength;
        
        float fx_component = (dx / distance) * force;
        float fy_component = (dy / distance) * force;
        
        
        n1->addForce(fx_component, fy_component);
        n2->addForce(-fx_component, -fy_component);
    }
}

void Graph::updatePositions(float dt) {
    for (auto& node_ptr : m_nodes) {
        node_ptr->updatePositionPhysics(dt, m_globalDampingFactor);
    }
}