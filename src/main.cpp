#include <SFML/Graphics.hpp>
#include "Graph.h"
#include <iostream>
#include <vector>
#include <memory>
#include <cstdlib>
#include <ctime>
#include <optional>

int main()
{
    srand(static_cast<unsigned int>(time(0)));

    sf::RenderWindow window(sf::VideoMode({800, 600}), "Mass-Spring-Damper Graph - SFML 3");
    window.setFramerateLimit(60);

    // initalize random gaph 20 nodes, 25 edges, c=0.5f, no spectral initialization
    Graph graph(20, 25, 0.5f, true);

    bool run_simulation = false;

    sf::Clock clock;
    float fixed_dt = 1.0f / 60.0f;

    Node* draggedNode = nullptr;
    sf::Vector2f mouseNodeOffset;

    // View for panning and zooming
    sf::View mainView = window.getDefaultView();

    bool isPanning = false;
    sf::Vector2i lastPanMousePos; // Store pixel position for panning delta

    while (window.isOpen())
    {
        float dt = clock.restart().asSeconds();
        if (run_simulation) { dt = fixed_dt; }

        
        while (const std::optional<sf::Event> eventOpt = window.pollEvent())
        {
            if (!eventOpt)
                continue;

            const sf::Event& event = *eventOpt;

            
            if (event.is<sf::Event::Closed>())
            {
                window.close();
            }

            // reset with r, pause with space, close with esc
            if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>())
            {
                if (keyPressed->code == sf::Keyboard::Key::Space)
                {
                    run_simulation = !run_simulation;
                    std::cout << "Simulation " << (run_simulation ? "running" : "paused") << std::endl;
                }
                if (keyPressed->code == sf::Keyboard::Key::R)
                {
                    graph.resetSimulation();
                    mainView = window.getDefaultView(); // Reset view on graph reset
                    std::cout << "Graph reset" << std::endl;
                }
                if (keyPressed->code == sf::Keyboard::Key::Escape)
                {
                    window.close();
                }
            }

            // Mouse button pressed events
            if (const auto* mouseButtonPressed = event.getIf<sf::Event::MouseButtonPressed>())
            {
                if (mouseButtonPressed->button == sf::Mouse::Button::Left)
                {
                    // Node dragging uses world coordinates, so map pixel to coords using the current view
                    sf::Vector2f mousePos = window.mapPixelToCoords(mouseButtonPressed->position, mainView);
                    for (auto it = graph.getNodes().rbegin(); it != graph.getNodes().rend(); ++it)
                    {
                        const auto& node_ptr = *it;
                        float dx_mouse_node = mousePos.x - node_ptr->getX();
                        float dy_mouse_node = mousePos.y - node_ptr->getY();
                        float dist_mouse_node_sq = dx_mouse_node * dx_mouse_node + dy_mouse_node * dy_mouse_node;
                        float nodeRadius = 10.0f;

                        if (dist_mouse_node_sq < nodeRadius * nodeRadius)
                        {
                            draggedNode = node_ptr.get();
                            if (draggedNode)
                            {
                                draggedNode->setFixed(true);
                                mouseNodeOffset = sf::Vector2f(draggedNode->getX(), draggedNode->getY()) - mousePos;
                            }
                            break;
                        }
                    }
                }
                else if (mouseButtonPressed->button == sf::Mouse::Button::Right) // Panning start
                {
                    isPanning = true;
                    lastPanMousePos = mouseButtonPressed->position; // Store initial pixel position
                }
            }

            // Mouse button released events
            if (const auto* mouseButtonReleased = event.getIf<sf::Event::MouseButtonReleased>())
            {
                if (mouseButtonReleased->button == sf::Mouse::Button::Left && draggedNode)
                {
                    draggedNode->setFixed(false);
                    draggedNode = nullptr;
                }
                else if (mouseButtonReleased->button == sf::Mouse::Button::Right) // Panning end
                {
                    isPanning = false;
                }
            }

            // Mouse moved events
            if (const auto* mouseMoved = event.getIf<sf::Event::MouseMoved>())
            {
                if (draggedNode) // Node dragging
                {
                    // Node dragging uses world coordinates
                    sf::Vector2f mousePos = window.mapPixelToCoords(mouseMoved->position, mainView);
                    draggedNode->setPosition(mousePos.x + mouseNodeOffset.x, mousePos.y + mouseNodeOffset.y);
                    draggedNode->setVelocity(0, 0);
                }
                else if (isPanning) // View panning
                {
                    sf::Vector2i currentMousePos = mouseMoved->position;
                    sf::Vector2f delta(static_cast<float>(lastPanMousePos.x - currentMousePos.x),
                                       static_cast<float>(lastPanMousePos.y - currentMousePos.y));
                    
                    
                    float panSpeedFactor = 1.0f; 
                    if (mainView.getSize().x != window.getDefaultView().getSize().x) {
                        panSpeedFactor = mainView.getSize().x / window.getDefaultView().getSize().x;
                    }
                     delta *= panSpeedFactor;


                    mainView.move(delta);
                    lastPanMousePos = currentMousePos; // Update for next movement
                }
            }
        } // End of while(pollEvent)


        if (run_simulation)
        {
            graph.step(dt);
             
        }

        window.clear(sf::Color::Black);
        window.setView(mainView); // Apply the main view before drawing the graph
        graph.renderGraph(window);
        window.display();
    } // End of while(window.isOpen())

    return 0;
}