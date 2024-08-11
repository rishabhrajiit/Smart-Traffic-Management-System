#include "TrafficManager.hpp"

int main() {
    TrafficManager trafficManager;

    // Initialize the city with 10 intersections and 15 roads
    trafficManager.initializeCity(10, 15);

    // Display initial city status
    trafficManager.displayCityStatus();

    // Simulate traffic management for 5 iterations
    for (int i = 0; i < 5; i++) {
        std::cout << "\nIteration " << i + 1 << ":\n";
        trafficManager.controlTrafficSignals();
        trafficManager.updateTraffic();
        trafficManager.simulateVehicleMovement();
        trafficManager.displayCityStatus();
    }

    return 0;
}
