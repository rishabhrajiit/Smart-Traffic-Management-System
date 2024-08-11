#ifndef TRAFFIC_MANAGER_HPP
#define TRAFFIC_MANAGER_HPP

#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <limits>
#include <algorithm>

struct Intersection {
    int id;
    bool hasTrafficLight;
    std::string trafficLightStatus;
    std::vector<int> connectedRoads;
};

struct Road {
    int id;
    int startIntersection;
    int endIntersection;
    int length;
    int trafficDensity;
};

struct Vehicle {
    int id;
    int currentPosition;
    int destination;
    std::vector<int> route;
};

class TrafficManager {
public:
    std::map<int, Intersection> intersections;
    std::map<int, Road> roads;
    std::vector<Vehicle> vehicles;

    void initializeCity(int numIntersections, int numRoads);
    std::vector<int> calculateShortestPath(int start, int destination);
    void controlTrafficSignals();
    void updateTraffic();
    void simulateVehicleMovement();
    void displayCityStatus();
};

#endif
