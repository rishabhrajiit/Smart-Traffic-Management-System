#ifndef TRAFFIC_MANAGER_HPP
#define TRAFFIC_MANAGER_HPP

#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <limits>
#include <algorithm>
using namespace std;

struct Intersection {
    int id;
    bool hasTrafficLight;
    string trafficLightStatus;
    vector<int> connectedRoads;
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
    vector<int> route;
};

class TrafficManager {
public:
    map<int, Intersection> intersections;
    map<int, Road> roads;
    vector<Vehicle> vehicles;

    void initializeCity(int numIntersections, int numRoads);
    vector<int> calculateShortestPath(int start, int destination);
    void controlTrafficSignals();
    void updateTraffic();
    void simulateVehicleMovement();
    void displayCityStatus();
};

#endif
