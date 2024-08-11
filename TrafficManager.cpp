#include "TrafficManager.hpp"

// Initialize the city with dummy intersections and roads.
void TrafficManager::initializeCity(int numIntersections, int numRoads) {
    // Create dummy intersections
    for (int i = 1; i <= numIntersections; i++) {
        Intersection intersection;
        intersection.id = i;
        intersection.hasTrafficLight = (i % 2 == 0);  // Every second intersection has a traffic light
        intersection.trafficLightStatus = "GREEN";  // Default to green
        intersections[i] = intersection;
    }

    // Create dummy roads connecting the intersections
    for (int i = 1; i <= numRoads; i++) {
        Road road;
        road.id = i;
        road.startIntersection = i;
        road.endIntersection = i % numIntersections + 1;  // Connect to the next intersection
        road.length = rand() % 100 + 1;  // Random length between 1 and 100
        road.trafficDensity = rand() % 10;  // Random traffic density between 0 and 9
        roads[i] = road;
        intersections[road.startIntersection].connectedRoads.push_back(i);
    }

    // Create dummy vehicles
    for (int i = 1; i <= 5; i++) {
        Vehicle vehicle;
        vehicle.id = i;
        vehicle.currentPosition = i;
        vehicle.destination = numIntersections - i + 1;  // Just to create some movement across the city
        vehicles.push_back(vehicle);
    }
}

// Dijkstra's algorithm to calculate the shortest path between two intersections
std::vector<int> TrafficManager::calculateShortestPath(int start, int destination) {
    std::map<int, int> distances;
    std::map<int, int> previous;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;

    for (const auto& intersection : intersections) {
        distances[intersection.first] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (current == destination) break;

        for (int roadId : intersections[current].connectedRoads) {
            Road& road = roads[roadId];
            int neighbor = road.endIntersection;
            int newDist = distances[current] + road.length + road.trafficDensity;

            if (newDist < distances[neighbor]) {
                distances[neighbor] = newDist;
                previous[neighbor] = current;
                pq.push({newDist, neighbor});
            }
        }
    }

    std::vector<int> path;
    for (int at = destination; at != start; at = previous[at]) {
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}

// Control the traffic signals based on the current traffic density
void TrafficManager::controlTrafficSignals() {
    for (auto& intersection : intersections) {
        if (intersection.second.hasTrafficLight) {
            int maxDensity = 0;
            for (int roadId : intersection.second.connectedRoads) {
                if (roads[roadId].trafficDensity > maxDensity) {
                    maxDensity = roads[roadId].trafficDensity;
                }
            }

            // If the maximum traffic density is high, set the light to green
            intersection.second.trafficLightStatus = (maxDensity > 5) ? "GREEN" : "RED";
        }
    }
}

// Simulate updates to traffic density and recalculate routes if necessary
void TrafficManager::updateTraffic() {
    for (auto& road : roads) {
        road.second.trafficDensity = rand() % 10;  // Randomly update traffic density
    }

    for (auto& vehicle : vehicles) {
        vehicle.route = calculateShortestPath(vehicle.currentPosition, vehicle.destination);
    }
}

// Simulate the movement of vehicles along their routes
void TrafficManager::simulateVehicleMovement() {
    for (auto& vehicle : vehicles) {
        if (!vehicle.route.empty()) {
            vehicle.currentPosition = vehicle.route.front();
            vehicle.route.erase(vehicle.route.begin());
        }
    }
}

// Display the current status of the city (roads, intersections, and vehicles)
void TrafficManager::displayCityStatus() {
    std::cout << "City Status:" << std::endl;
    std::cout << "Intersections:" << std::endl;
    for (const auto& intersection : intersections) {
        std::cout << "Intersection " << intersection.first 
                  << " (Traffic Light: " << intersection.second.trafficLightStatus << ")" << std::endl;
    }

    std::cout << "Roads:" << std::endl;
    for (const auto& road : roads) {
        std::cout << "Road " << road.first 
                  << " (From Intersection " << road.second.startIntersection 
                  << " to Intersection " << road.second.endIntersection 
                  << ", Length: " << road.second.length 
                  << ", Traffic Density: " << road.second.trafficDensity << ")" << std::endl;
    }

    std::cout << "Vehicles:" << std::endl;
    for (const auto& vehicle : vehicles) {
        std::cout << "Vehicle " << vehicle.id 
                  << " (Current Position: Intersection " << vehicle.currentPosition 
                  << ", Destination: Intersection " << vehicle.destination << ")" << std::endl;
    }
}
